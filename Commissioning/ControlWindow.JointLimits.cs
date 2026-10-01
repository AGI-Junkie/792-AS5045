using System.Text.RegularExpressions;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private bool supportsJointLimits, limitsAwaitingState;
    private bool protocolMatches;      // the firmware's proto=<n> equals DeviceContract.ProtocolVersion
    private int protocolVersion;       // proto=<n> from the last LINK line, 0 when absent
    private bool supportsMotionRates;
    private MotionProfile? reportedProfile;
    private byte limitType=9;
    private JointLimits? reportedLimits;
    private string limitSyncError = "";
    private int limitsMotionAllowed;
    // One immutable request remains pending through input edits until ACK + STATE.
    private int limitAxis = -1, limitAttempts;
    private JointPose limitBounds;
    private uint limitEpoch, limitSequence;
    private uint cancelledLimitEpoch;
    private byte[] limitPacket = [];
    private bool limitAcknowledged;
    private DateTime limitRequestedAt;
    private static readonly Regex LimitsAckPattern = new(@"^PTP_(?:LIMITS|RATES) epoch=(\d+) seq=(\d+) axis=([01]) min=(-?\d+) max=(-?\d+)$", RegexOptions.CultureInvariant);
    private static readonly Regex RatesPattern = new(@"^MOTION_RATES epoch=(\d+) r1=(\d+) r2=(\d+) a1=(\d+) a2=(\d+)$", RegexOptions.CultureInvariant);
    private bool RatesReady => xyPreview.RequestedProfile is { } p && reportedProfile == p;

    private bool LimitsReady => compatible && framedControl && protocolMatches && xyPreview.RequestedLimits is { } wanted && reportedLimits == wanted &&
        RatesReady && limitAxis < 0 && !limitsAwaitingState;

    private string LimitsStatus => !protocolMatches ? $"固件协议版本不匹配（固件 proto={(protocolVersion == 0 ? "无" : protocolVersion.ToString())}，上位机需要 proto={DeviceContract.ProtocolVersion}）：请更新固件，并使用同一交付里的固件与上位机；运动已锁定。" :
        xyPreview.RequestedProfile is null ? "机构尺寸或全局速度无效，请检查设置。" :
        !supportsMotionRates ? "固件没有声明全局速度支持（motion_rates=1）；请更新固件后重新连接。" :
        xyPreview.RequestedLimits is null ? "关节范围输入无效；请填写 ±180° 内、包含零位的有效范围（最多两位小数）。" :
        limitSyncError.Length > 0 ? limitSyncError :
        reportedLimits is null ? "等待设备确认关节范围；确认范围前不能使能或运动。" :
        enabled && !LimitsReady ? "关节范围或全局速度与设备不一致：先总停止，禁用后自动同步，再重新使能。" :
        !supportsJointLimits && !LimitsReady ? "当前固件的关节范围与输入不一致；请更新支持范围同步的固件。" :
        "正在同步关节范围与全局速度，等待设备确认；确认前不能使能或运动。";

    private void OnLimitsChanged()
    {
        limitSyncError = "";
        lock (wireSync) {
            if (limitAxis >= 0 && limitAttempts >= 3) { limitAttempts = 0; limitRequestedAt = DateTime.MinValue; }
        }
        // Keep an in-flight request: clearing it when the user changes B back to A
        // could enable with stale A feedback just before firmware applies B.
        keyboardMask = mouseMask = 0;
        UpdateControls(); PublishHeartbeatLease(); ForceHeartbeatNow();
    }

    private void CancelLimitSync(bool requireState = false)
    {
        lock (wireSync) {
            if (pendingControlType is 9 or 10) pendingControlType = 0;
            if (requireState && limitSequence != 0) { limitsAwaitingState = true; cancelledLimitEpoch = limitEpoch; }
            limitAxis = -1; limitSequence = 0; limitPacket = [];
            limitAcknowledged = false; limitAttempts = 0;
        }
    }

    private void ObserveJointLimits(PtpState state)
    {
        reportedLimits = state.Limits;
        if (state.Epoch != cancelledLimitEpoch) limitsAwaitingState = false;
        lock (wireSync) {
            if (limitType==9 && limitAxis >= 0 && limitAcknowledged && state.Epoch == limitEpoch &&
                (limitAxis == 0 ? new JointPose(state.Limits.Min1, state.Limits.Max1) :
                                  new JointPose(state.Limits.Min2, state.Limits.Max2)) == limitBounds) {
                CancelLimitSync();
                limitSyncError = "";
            }
        }
    }
    private void ObserveMotionRates(string line)
    {
        var m=RatesPattern.Match(line);
        var v=new uint[5];
        if(!m.Success) return;
        for(int i=0;i<5;i++) if(!uint.TryParse(m.Groups[i+1].Value,out v[i])) return;
        if(v[0]!=reportedEpoch || v[1]<DeviceContract.RateMin || v[1]>DeviceContract.Arm1RateMax || v[2]<DeviceContract.RateMin || v[2]>DeviceContract.Arm2RateMax ||
           v[3]<DeviceContract.AccelMin || v[3]>DeviceContract.Arm1AccelMax || v[4]<DeviceContract.AccelMin || v[4]>DeviceContract.Arm2AccelMax) return;
        reportedProfile=new((int)v[1],(int)v[2],(int)v[3],(int)v[4]);
        lock(wireSync) {
            if(limitType==10 && limitAxis>=0 && limitAcknowledged && v[0]==limitEpoch && reportedProfile.Value.Axis(limitAxis)==limitBounds) {
                CancelLimitSync(); limitSyncError="";
            }
        }
        UpdateControls();
    }

    private void HandleLimitsAck(string line)
    {
        var m = LimitsAckPattern.Match(line);
        if (!m.Success || !uint.TryParse(m.Groups[1].Value, out uint epoch) ||
            !uint.TryParse(m.Groups[2].Value, out uint sequence) ||
            !short.TryParse(m.Groups[4].Value, out short min) || !short.TryParse(m.Groups[5].Value, out short max)) return;
        lock (wireSync) {
            if ((line.StartsWith("PTP_RATES") ? 10 : 9)!=limitType || limitAxis < 0 || limitSequence == 0 || sequence != limitSequence || epoch != limitEpoch ||
                m.Groups[3].Value != limitAxis.ToString() || min != (int)Math.Round(limitBounds.Q1 * 100) ||
                max != (int)Math.Round(limitBounds.Q2 * 100)) return;
            limitAcknowledged = true;
        }
    }

    private void SynchronizeJointLimits(DateTime now)
    {
        var desired = xyPreview.RequestedLimits;
        lock (wireSync)
        {
            if (limitAxis >= 0 && limitEpoch != reportedEpoch) CancelLimitSync(requireState: true);
            if (LimitsReady) { limitSyncError = ""; return; }
            if (!compatible || !protocolMatches || !framedControl || !supportsJointLimits || enabled || homing || stopPending ||
                PointBusy || waitingStopEpoch || reportedEpoch == 0) return;
            if (limitAxis >= 0) {
                if (pendingControlType is 9 or 10 || (now - limitRequestedAt).TotalSeconds < 1) return;
                if (limitAttempts >= 3) {
                    if (limitSyncError.Length > 0) return;
                    limitSyncError = "范围/速度同步未获完整确认；请重新连接，或修改设置后重试。";
                    Append("HOST " + limitSyncError); return;
                }
                limitAttempts++; limitRequestedAt = now; pendingControlType = limitType;
                Append($"HOST joint limits retry axis={limitAxis} seq={limitSequence} attempt={limitAttempts}/3");
                return;
            }
            if (desired is not { } wanted || reportedLimits is null || limitsAwaitingState) return;
            var actual = reportedLimits.Value;
            if(wanted!=actual) {
                limitType=9;
                limitAxis = wanted.Min1 != actual.Min1 || wanted.Max1 != actual.Max1 ? 0 : 1;
                limitBounds = limitAxis == 0 ? new(wanted.Min1, wanted.Max1) : new(wanted.Min2, wanted.Max2);
            } else {
                if(!supportsMotionRates || reportedProfile is not { } profile || xyPreview.RequestedProfile is not { } requested) return;
                limitType=10; limitAxis=requested.R1!=profile.R1 || requested.A1!=profile.A1 ? 0 : 1;
                limitBounds=requested.Axis(limitAxis);
            }
            limitEpoch = reportedEpoch; limitSequence = 0; limitPacket = [];
            limitSyncError = "";
            limitAcknowledged = false; limitAttempts = 1; limitRequestedAt = now;
            pendingControlType = limitType; lastControlHeartbeat = 0;
            Append($"HOST configuration type={limitType} axis={limitAxis} payload1={limitBounds.Q1*100:F0} payload2={limitBounds.Q2*100:F0}; disabled configuration only");
        }
    }

    private static void JointLimitsSyncSelfTest()
    {
        var w = new ControlWindow { selfTesting = true };
        var frames = new List<byte[]>();
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Joint limits sync: " + reason); }
        void Write(byte[] data, int offset, int count) {
            if (offset == 0 && (frames.Count == 0 || !ReferenceEquals(frames[^1], data))) frames.Add(data);
        }
        void State(bool enabled = false) => w.TestLine($"STATE fw=stage2-v7 enabled={(enabled ? 1 : 0)} home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1");
        void Range(JointLimits range, uint epoch = 1) => w.TestLine($"PTP_STATE lock=0 ref=0 active=0 epoch={epoch} seq=0 min1={(int)Math.Round(range.Min1*100)} max1={(int)Math.Round(range.Max1*100)} min2={(int)Math.Round(range.Min2*100)} max2={(int)Math.Round(range.Max2*100)}");
        void Set(JointLimits range) => w.xyPreview.RestoreLimitsForSelfTest(range.Min1.ToString(System.Globalization.CultureInfo.InvariantCulture),
            range.Max1.ToString(System.Globalization.CultureInfo.InvariantCulture), range.Min2.ToString(System.Globalization.CultureInfo.InvariantCulture), range.Max2.ToString(System.Globalization.CultureInfo.InvariantCulture));
        byte[] SendConfig() {
            w.PublishHeartbeatLease();
            for (int i = 0; i < 4; i++) {
                int before = frames.Count;
                w.WireWriterStep(Write);
                if (frames.Count > before && frames[^1].Length == 36 && ((frames[^1][1] >> 1) & 15) == 9) return frames[^1];
            }
            throw new InvalidOperationException("Joint limits sync: config not emitted");
        }
        void Ack() => w.TestLine($"PTP_LIMITS epoch={w.limitEpoch} seq={w.limitSequence} axis={w.limitAxis} min={(int)Math.Round(w.limitBounds.Q1*100)} max={(int)Math.Round(w.limitBounds.Q2*100)}");
        var defaults = XYPreviewPanel.DefaultTrialRange;
        var wide = new JointLimits(-120, 140, -90, 95);
        try {
            State(); Range(defaults);
            Check(!w.LimitsReady && !w.enable.IsEnabled, "matching old firmware range must not authorize relative q2");
            Set(wide);
            Check(!w.LimitsReady && !w.enable.IsEnabled && w.limitAxis < 0 && w.LimitsStatus.Contains("更新"), "unsupported mismatched firmware was accepted");
            w.TestLine("LINK uart=hw baud=115200 fw=stage2-v7-limits1 short_hb=0 joint_limits=1");
            Check(!w.LimitsReady && !w.enable.IsEnabled && w.limitAxis < 0, "legacy firmware accepted new q2 convention");
            w.TestLine($"LINK uart=hw baud=115200 fw=stage2-v7-absolute-q2 short_hb=0 joint_limits=1 q2_absolute=1 proto={DeviceContract.ProtocolVersion}");
            Check(w.limitAxis == 0 && !w.enable.IsEnabled, "first axis was not queued while disabled");
            var first = SendConfig(); uint firstSeq = w.limitSequence;
            Check(firstSeq > 1 && w.controlSequence == firstSeq, "configuration sequence was reserved before its preceding heartbeat");
            Set(defaults);
            Check(w.limitAxis == 0 && !w.LimitsReady && !w.enable.IsEnabled, "A-B-A edit cancelled an in-flight request");
            Set(wide);
            w.TestLine($"PTP_LIMITS epoch=1 seq={firstSeq + 1} axis=0 min=-12000 max=14000");
            Check(!w.limitAcknowledged, "wrong sequence acknowledged configuration");
            State(); Range(new(-120, 140, -60, 60));
            Check(w.limitAxis == 0 && !w.LimitsReady, "STATE alone skipped the pending ACK");
            w.SynchronizeJointLimits(w.limitRequestedAt.AddSeconds(1.1));
            var retry = SendConfig();
            Check(retry.SequenceEqual(first) && w.controlSequence > firstSeq, "retry changed after interleaved full heartbeat");
            Ack(); Check(w.limitAxis == 0, "ACK alone confirmed both ranges");
            State(); Range(new(-120, 140, -60, 60));
            Check(w.limitAxis == 1, "second axis not queued after first ACK + STATE");
            SendConfig(); Ack(); State(); Range(wide);
            Check(w.LimitsReady && w.enable.IsEnabled && w.pendingControl is null && w.pointWire.Count == 0,
                "range sync automatically enabled/moved or failed to unlock explicit enable");
            State(true); Range(wide); Set(defaults);
            Check(w.enabled && !w.stopPending && w.limitAxis < 0 && !w.home.IsEnabled && !w.manualReference.IsEnabled &&
                w.jogButtons.All(b => !b.IsEnabled) && !w.CanPoint && !w.CanJog && w.LimitsStatus.Contains("总停止"),
                "enabled range edit wrote configuration, stopped automatically, or left a motion gate open");
            w.xyPreview.RestoreLimitsForSelfTest("bad", "30", "-60", "60");
            Check(w.xyPreview.RequestedLimits is null && !w.LimitsReady && w.limitAxis < 0, "invalid input reached firmware");
            Set(defaults); State(); Range(wide); SendConfig();
            for (int attempt = 1; attempt < 3; attempt++) { w.SynchronizeJointLimits(w.limitRequestedAt.AddSeconds(1.1)); SendConfig(); }
            w.SynchronizeJointLimits(w.limitRequestedAt.AddSeconds(1.1));
            Check(w.limitAxis == 0 && !w.enable.IsEnabled && !w.stopPending && w.limitSyncError.Length > 0,
                "timeout discarded uncertain in-flight state or stopped automatically");
            // STOP cancels a sent configuration; an old epoch report cannot prove its outcome.
            w.serial = new System.IO.Ports.SerialPort(); w.Stop();
            State(); Range(defaults);
            Check(w.limitsAwaitingState && !w.LimitsReady, "old epoch cleared cancelled-configuration barrier");
            w.TestLine("PTP_EPOCH 2");
            Check(!w.LimitsReady && !w.enable.IsEnabled, "compact epoch announcement confirmed unknown ranges");
            State(); Range(defaults, 2);
            Check(w.LimitsReady && w.enable.IsEnabled, "post-STOP epoch range did not clear the barrier");
            // A reboot can return to epoch 1, including when both old/new boot epochs are 1.
            Set(wide); SendConfig(); Ack(); State(); Range(new(-120, 140, -60, 60), 2);
            SendConfig(); Ack(); State(); Range(wide, 2);
            w.TestLine("PTP_EPOCH 1");
            Check(!w.LimitsReady && w.limitsMotionAllowed == 0, "epoch rollback retained custom range authorization");
            Range(wide, 1); Check(w.LimitsReady, "range report did not restore matching authorization");
            State(); Check(!w.LimitsReady && !w.enable.IsEnabled, "same-epoch disabled/reboot state trusted cached custom ranges");
            Range(defaults, 1); Check(!w.LimitsReady && w.limitAxis == 0, "boot defaults were not synchronized again");
            w.Disconnect("range sync self-test");
            Check(w.reportedLimits is null && w.limitAxis < 0 && !w.supportsJointLimits && !w.LimitsReady, "disconnect retained range authorization");
        }
        finally { w.Close(); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal void RestoreLimitsForSelfTest(string minA, string maxA, string minB, string maxB)
    {
        var fields = ParameterFields.Select(f => f.Text).ToArray();
        fields[6] = minA; fields[7] = maxA; fields[8] = minB; fields[9] = maxB;
        RestoreParameters(System.Text.Json.JsonSerializer.Serialize(new SavedParameters(1, shape.SelectedIndex, branch.SelectedIndex, fields)));
    }
}
