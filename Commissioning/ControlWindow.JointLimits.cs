using System.Text.RegularExpressions;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private bool supportsJointLimits, limitsAwaitingState;
    private bool protocolMatches;      // the firmware's proto=<n> equals DeviceContract.ProtocolVersion
    private int protocolVersion;       // proto=<n> from the last LINK line, 0 when absent
    private bool supportsMotionRates;
    private bool supportsLiveRates;          // LINK rates_live=1: the firmware takes speed/accel while enabled and idle
    private DateTime lastLimitsEditAt;       // when the requested range or speeds last changed
    private const double LiveSyncSettleMs = 400;   // an enabled device waits for the inputs to settle (a slider drag is one request)
    private MotionProfile? reportedProfile;
    private byte limitType=9;
    private JointLimits? reportedLimits;
    private DateTime limitsReportSince;      // a disabled STATE cleared the reports above; the display holds until they return
    private const double LimitsReportHoldMs = 300;
    private bool enableAfterReport;          // 使能 was clicked during that hold
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
    private static readonly Regex LimitsAckPattern = new(@"^PTP_(?:LIMITS|RATES) epoch=(\d+) seq=(\d+) axis=([012]) min=(-?\d+) max=(-?\d+)$", RegexOptions.CultureInvariant);
    private static readonly Regex RatesPattern = new(@"^MOTION_RATES epoch=(\d+) r1=(\d+) r2=(\d+) a1=(\d+) a2=(\d+)$", RegexOptions.CultureInvariant);
    private bool RatesReady => xyPreview.RequestedProfile is { } p && reportedProfile == p;

    // The device has reported the ranges that are typed, and the speeds that are typed: each on its own, so the panel can say which.
    private bool RangeConfirmed => compatible && framedControl && protocolMatches && xyPreview.RequestedLimits is { } wanted && reportedLimits == wanted && limitAxis < 0 && !limitsAwaitingState;
    private bool SpeedConfirmed => compatible && framedControl && protocolMatches && RatesReady && limitAxis < 0 && !limitsAwaitingState;
    private bool LimitsReady => RangeConfirmed && SpeedConfirmed;

    private string LimitsStatus => !protocolMatches ? Lang.T("limits.protoMismatch",
            protocolVersion == 0 ? (Lang.IsEn ? "none" : "无") : protocolVersion.ToString(), DeviceContract.ProtocolVersion.ToString()) :
        xyPreview.RequestedProfile is null ? Lang.T("limits.invalidProfile") :
        !supportsMotionRates ? Lang.T("limits.noRates") :
        xyPreview.RequestedLimits is null ? Lang.T("limits.invalidRange") :
        limitSyncError.Length > 0 ? Lang.T(limitSyncError) :
        reportedLimits is null ? Lang.T("limits.awaitDevice") :
        enabled && !LimitsReady ? Lang.T(!supportsLiveRates ? "limits.mismatch" : xyPreview.RequestedLimits == reportedLimits ? "limits.liveRates" : "limits.rangeMismatch") :
        !supportsJointLimits && !LimitsReady ? Lang.T("limits.fwMismatch") :
        Lang.T("limits.syncing");

    private void OnLimitsChanged()
    {
        limitSyncError = ""; lastLimitsEditAt = DateTime.UtcNow;
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
                state.Limits.Bounds(limitAxis) == limitBounds) {
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
            // Speeds follow an enabled, idle device; the firmware refuses them while anything moves, and ranges need the stop first.
            bool live = enabled && supportsLiveRates && keyboardMask == 0 && mouseMask == 0 && (now - lastLimitsEditAt).TotalMilliseconds >= LiveSyncSettleMs;
            if (!compatible || !protocolMatches || !framedControl || !supportsJointLimits || (enabled && !live) || homing || stopPending ||
                PointBusy || waitingStopEpoch || reportedEpoch == 0) return;
            if (limitAxis >= 0) {
                if (pendingControlType is 9 or 10 || (now - limitRequestedAt).TotalSeconds < 1) return;
                if (limitAttempts >= 3) {
                    if (limitSyncError.Length > 0) return;
                    limitSyncError = "limits.syncTimeout";
                    Append("HOST " + Lang.T(limitSyncError), LogLevel.Error); return;
                }
                limitAttempts++; limitRequestedAt = now; pendingControlType = limitType;
                Append($"HOST joint limits retry axis={limitAxis} seq={limitSequence} attempt={limitAttempts}/3", LogLevel.Warning);
                return;
            }
            if (desired is not { } wanted || reportedLimits is null || limitsAwaitingState) return;
            var actual = reportedLimits.Value;
            if(wanted!=actual) {
                if (enabled) return;   // a range change needs STOP ALL first
                limitType=9;
                limitAxis = Enumerable.Range(0, 3).First(axis => wanted.Bounds(axis) != actual.Bounds(axis));   // q1, q2, then the elbow
                limitBounds = wanted.Bounds(limitAxis);
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
        void Range(JointLimits range, uint epoch = 1) => w.TestLine($"PTP_STATE lock=0 ref=0 active=0 epoch={epoch} seq=0 min1={(int)Math.Round(range.Min1*100)} max1={(int)Math.Round(range.Max1*100)} min2={(int)Math.Round(range.Min2*100)} max2={(int)Math.Round(range.Max2*100)} emin={(int)Math.Round(range.ElbowMin*100)} emax={(int)Math.Round(range.ElbowMax*100)}");
        static string Text(double v) => v.ToString(System.Globalization.CultureInfo.InvariantCulture);
        void Set(JointLimits range) => w.xyPreview.RestoreLimitsForSelfTest(Text(range.Min1), Text(range.Max1), Text(range.Min2), Text(range.Max2), Text(range.ElbowMin), Text(range.ElbowMax));
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
            // The elbow range is the third entry: an elbow-only edit queues axis 2, and (like the boxes) it only counts once
            // the device has acknowledged it AND reported it back.
            var elbowOnly = wide with { ElbowMin = -100, ElbowMax = 120 };
            State(); Range(wide); Set(elbowOnly);
            Check(w.limitAxis == 2 && !w.LimitsReady && w.limitBounds == new JointPose(-100, 120), "an elbow-only edit was not queued as axis 2");
            var elbowFrame = SendConfig();
            static int Payload(byte[] f, int i) => ((f[1 + 2 * i] >> 1) & 15) | (((f[2 + 2 * i] >> 1) & 15) << 4);
            static int U16(byte[] f, int at) => Payload(f, at) | (Payload(f, at + 1) << 8);
            Check(U16(elbowFrame, 13) == 2 && (short)U16(elbowFrame, 9) == -10000 && (short)U16(elbowFrame, 11) == 12000, "elbow request frame carries axis 2 and the elbow bounds in 0.01 deg");
            Ack(); Check(w.limitAxis == 2 && !w.LimitsReady, "ACK alone confirmed the elbow range");
            State(); Range(wide); Check(w.limitAxis == 2 && !w.LimitsReady, "a report of the old elbow range confirmed the new one");
            State(); Range(elbowOnly); Check(w.limitAxis < 0 && w.LimitsReady && w.enable.IsEnabled, "ACK + reported elbow range did not finish the sync");
            // Boxes first, then the elbow, when both differ.
            Set(wide with { Min1 = -100, ElbowMax = 100 });
            Check(w.limitAxis == 0, "a box edit must be queued before the elbow edit");
            w.CancelLimitSync(); Set(elbowOnly);
            Check(w.limitAxis < 0 && w.LimitsReady, "editing back to what the device reports must need no request");
            Set(wide); Check(w.limitAxis == 2, "the elbow range was not synchronized back");
            SendConfig(); Ack(); State(); Range(wide);
            Check(w.limitAxis < 0 && w.LimitsReady, "putting everything back must restore the match");
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
            State(); Check(!w.LimitsReady && w.limitsMotionAllowed == 0, "same-epoch disabled/reboot state trusted cached custom ranges");
            w.RequestEnable(); Check(w.pendingControl is null, "an enable clicked before the reboot report was sent");
            Range(defaults, 1); Check(!w.LimitsReady && w.limitAxis == 0 && !w.enable.IsEnabled && w.pendingControl is null, "boot defaults were not synchronized again, or a held click enabled anyway");
            w.Disconnect("range sync self-test");
            Check(w.reportedLimits is null && w.limitAxis < 0 && !w.supportsJointLimits && !w.LimitsReady, "disconnect retained range authorization");
        }
        finally { w.Close(); }
    }

    // A disabled device repeats STATE, PTP_STATE, MOTION_RATES every 500 ms; the STATE clears the reports until the
    // other two arrive. Between those lines the display must not flash, and a click on 使能 there must wait for them.
    private static void StatusBlockHoldSelfTest()
    {
        var w = new ControlWindow { selfTesting = true };
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Status block hold: " + reason); }
        var d = XYPreviewPanel.DefaultTrialRange;
        string range = $"PTP_STATE lock=0 ref=0 active=0 epoch=1 seq=0 min1={(int)Math.Round(d.Min1*100)} max1={(int)Math.Round(d.Max1*100)} min2={(int)Math.Round(d.Min2*100)} max2={(int)Math.Round(d.Max2*100)} emin={(int)Math.Round(d.ElbowMin*100)} emax={(int)Math.Round(d.ElbowMax*100)}";
        const string state = "STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1";
        try {
            w.TestLine($"LINK uart=hw baud=115200 fw=stage2-v7-test short_hb=1 joint_limits=1 q2_absolute=1 proto={DeviceContract.ProtocolVersion}");
            w.TestLine(state); w.TestLine(range);
            Check(w.LimitsReady && w.enable.IsEnabled, "a disabled device reporting the requested settings must be ready");
            string Shown() => $"{w.enable.IsEnabled}|{w.rangeStatus.Text}|{w.pointBlockReason}";
            string steady = Shown();
            for (int block = 0; block < 3; block++) {
                w.TestLine(state); w.UpdateControls();
                Check(Shown() == steady && w.limitsMotionAllowed == 0, "the display flashed between STATE and the range report, or the gate opened early");
                w.TestLine(range); w.UpdateControls();
                Check(Shown() == steady && w.LimitsReady, "the display changed after a complete unchanged block");
            }
            w.TestLine(state); w.RequestEnable();
            Check(w.pendingControl is null, "enable was sent before the block re-confirmed the settings");
            w.TestLine(range);
            Check(w.pendingControl == "使能", "an enable clicked inside a status block was lost");
            w.Disconnect("status block self-test");
        }
        finally { w.Close(); }
    }

    // Speeds and accelerations follow an enabled, idle device when the firmware says rates_live=1, once the inputs have
    // settled; ranges still wait for STOP ALL, and a firmware without the flag keeps the old rule.
    private static void LiveRatesSyncSelfTest()
    {
        var w = new ControlWindow { selfTesting = true };
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Live rates sync: " + reason); }
        var frames = new List<byte[]>();
        void Write(byte[] data, int offset, int count) {
            if (offset == 0 && (frames.Count == 0 || !ReferenceEquals(frames[^1], data))) frames.Add(data);
        }
        static int Type(byte[] frame) => (frame[1] >> 1) & 15;
        string Link(bool live) => $"LINK uart=hw baud=115200 fw=stage2-v7-test short_hb=1 joint_limits=1 motion_rates=1 q2_absolute=1 proto={DeviceContract.ProtocolVersion}" + (live ? " rates_live=1" : "");
        void Device(MotionProfile rates, int axisRange = 6000) {
            w.HandleLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            w.HandleLine($"PTP_STATE lock=0 ref=1 active=0 epoch=7 seq=1 min1=-3000 max1=3000 min2=-{axisRange} max2={axisRange} emin=-15500 emax=15500");
            w.HandleLine($"MOTION_RATES epoch=7 r1={rates.R1} r2={rates.R2} a1={rates.A1} a2={rates.A2}");
        }
        byte[] SendConfig() {
            w.PublishHeartbeatLease();
            for (int i = 0; i < 6; i++) {
                w.WireWriterStep(Write);
                if (frames.Count > 0 && frames[^1].Length == 36 && Type(frames[^1]) == 10) return frames[^1];
            }
            throw new InvalidOperationException("Live rates sync: speed configuration not emitted");
        }
        void Ack() => w.HandleLine($"PTP_RATES epoch=7 seq={w.limitSequence} axis={w.limitAxis} min={(int)Math.Round(w.limitBounds.Q1 * 100)} max={(int)Math.Round(w.limitBounds.Q2 * 100)}");
        var later = TimeSpan.FromSeconds(2);
        try {
            w.HandleLine(Link(true));
            var before = w.xyPreview.RequestedProfile!.Value;
            Device(before);
            Check(w.enabled && w.LimitsReady, "an enabled device that reports the requested ranges and speeds must be ready");
            // A speed edit: nothing is sent while the inputs are still moving, then one request, no stop, no enable change.
            w.xyPreview.RestoreRatesForSelfTest("8", "3", "10", "6");
            var wanted = w.xyPreview.RequestedProfile!.Value;
            Check(wanted.R1 != before.R1 && !w.LimitsReady && w.LimitsStatus.Contains("自动同步"), "an edited speed must lock motion and say it syncs by itself");
            w.SynchronizeJointLimits(DateTime.UtcNow);
            Check(w.limitAxis < 0 && w.pendingControlType == 0, "a request left before the input settled");
            w.SynchronizeJointLimits(DateTime.UtcNow + later);
            Check(w.limitType == 10 && w.limitAxis == 0 && w.pendingControlType == 10 && w.enabled, "the settled speed was not queued as axis-0 type 10");
            SendConfig(); Ack();
            Check(!w.LimitsReady, "ACK alone confirmed the speeds");
            Device(wanted);
            Check(w.LimitsReady && w.enabled && w.limitAxis < 0, "the reported speeds did not finish the sync");
            Check(!frames.Any(f => f.Length == 36 && Type(f) == 9), "a speed sync wrote a range");
            // A range edit on an enabled device still waits for STOP ALL, and says so.
            w.xyPreview.RestoreLimitsForSelfTest("-20", "20", "-60", "60");
            w.SynchronizeJointLimits(DateTime.UtcNow + later);
            Check(w.limitAxis < 0 && w.pendingControlType == 0 && !w.LimitsReady && w.LimitsStatus.Contains("先总停止"), "an enabled range change must wait for STOP ALL");
            w.xyPreview.RestoreLimitsForSelfTest("-30", "30", "-60", "60");
            Check(w.LimitsReady, "putting the range back must restore the match");
            // A held key, or a move in progress, keeps the sync waiting.
            w.xyPreview.RestoreRatesForSelfTest("9", "3", "10", "6");
            w.keyboardMask = 1; w.SynchronizeJointLimits(DateTime.UtcNow + later);
            Check(w.limitAxis < 0, "a held jog key must keep the speed sync waiting");
            w.keyboardMask = 0; w.executing = true; w.SynchronizeJointLimits(DateTime.UtcNow + later); w.executing = false;
            Check(w.limitAxis < 0, "a running shape must keep the speed sync waiting");
            w.SynchronizeJointLimits(DateTime.UtcNow + later);
            Check(w.limitAxis == 0, "an idle device did not get the speed after the key and the run ended");
            // A held plan the device drops is forgotten too.
            w.HandleLine("TRAJ_STATE state=0 count=0 total=0 index=0 seq=1 epoch=7");
            w.xyPreview.SelectShapeForSelfTest(3); w.xyPreview.LoadExampleForSelfTest();
            var plan = w.xyPreview.Plan!;
            w.trajectoryEpoch = 7; w.uploaded = plan;
            w.HandleLine($"TRAJ_STATE state=2 count={plan.Nodes.Length} total={plan.Nodes.Length} index=0 seq=2 epoch=7");
            Check(w.uploaded == plan, "a held plan must survive its own status line");
            w.HandleLine("TRAJ_STATE state=0 count=0 total=0 index=0 seq=2 epoch=7");
            Check(w.uploaded is null, "the device dropped the plan but the host still offers it");
            // Firmware without rates_live: the old rule, speeds wait for STOP ALL.
            w.CancelLimitSync(); w.HandleLine(Link(false));
            w.xyPreview.RestoreRatesForSelfTest("7", "3", "10", "6");
            w.SynchronizeJointLimits(DateTime.UtcNow + later);
            Check(w.limitAxis < 0 && w.pendingControlType == 0 && w.LimitsStatus.Contains("先总停止"), "firmware without rates_live must not get speeds while enabled");
            w.Disconnect("live rates self-test");
            Check(!w.supportsLiveRates, "disconnect kept the live-rates capability");
        }
        finally { w.Close(); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal void RestoreLimitsForSelfTest(string minA, string maxA, string minB, string maxB, string elbowLow = "", string elbowHigh = "")
    {
        var fields = ParameterFields.Select(f => f.Text).ToArray();
        fields[6] = minA; fields[7] = maxA; fields[8] = minB; fields[9] = maxB; fields[15] = elbowLow; fields[16] = elbowHigh;
        RestoreParameters(System.Text.Json.JsonSerializer.Serialize(new SavedParameters(1, shape.SelectedIndex, branch.SelectedIndex, fields)));
    }
    internal void RestoreRatesForSelfTest(string v1Text, string v2Text, string a1Text, string a2Text)
    {
        var fields = ParameterFields.Select(f => f.Text).ToArray();
        fields[11] = v1Text; fields[12] = v2Text; fields[13] = a1Text; fields[14] = a2Text;
        RestoreParameters(System.Text.Json.JsonSerializer.Serialize(new SavedParameters(1, shape.SelectedIndex, branch.SelectedIndex, fields)));
    }
}
