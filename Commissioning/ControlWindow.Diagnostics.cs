using System.Globalization;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Controls;

namespace TwoLinkCommissioning;

// What the firmware prints when it stops: one key=value line, captured before the stop resets anything.
// It states what is known and no more. A driver nFAULT can come from over-current, over-temperature or
// under-voltage and the pin does not say which, so Driver stays "nfault_undetermined" and the UI says so.
public sealed record FaultSnapshot(uint Milliseconds, uint Epoch, uint Sequence, string Phase, int Home, int TrajState,
    int TrajIndex, int TrajCount, bool Referenced, int Steps1, int Steps2, int Encoder1, int Slip, int Inputs,
    uint HeartbeatAgeMs, uint HeartbeatMaxGapMs, string Driver, string Reason)
{
    private static readonly Regex Pair = new(@"(\w+)=(""[^""]*""?|\S+)", RegexOptions.CultureInvariant);
    public bool DriverFault => Driver != "ok";
    private TelemetrySample AsSample => new(Milliseconds, Epoch, Sequence, Referenced, Referenced ? 2 : 0, Steps1, Steps2, Encoder1, Slip, Inputs, 0);
    // Joint angles as the host computes them from commanded steps (arm2 has no encoder), and arm1's encoder error.
    public JointPose Command => AsSample.Command;
    public double EncoderErrorDegrees => AsSample.Error1;
    public static FaultSnapshot? Parse(string line)
    {
        if (!line.StartsWith("FAULT_SNAPSHOT ", StringComparison.Ordinal)) return null;
        var values = new Dictionary<string, string>();
        foreach (Match m in Pair.Matches(line[15..])) values[m.Groups[1].Value] = m.Groups[2].Value.Trim('"');
        string[] required = ["t", "epoch", "seq", "phase", "home", "traj", "idx", "ref", "p1", "p2", "e1", "err", "inputs", "hb_age_ms", "hb_max_gap_ms", "drv", "reason"];
        if (required.Any(k => !values.ContainsKey(k))) return null;
        var index = values["idx"].Split('/');
        if (index.Length != 2 || !uint.TryParse(values["t"], out uint t) || !uint.TryParse(values["epoch"], out uint epoch) || !uint.TryParse(values["seq"], out uint seq) ||
            !int.TryParse(values["home"], out int home) || !int.TryParse(values["traj"], out int traj) || !int.TryParse(index[0], out int at) || !int.TryParse(index[1], out int count) ||
            !int.TryParse(values["p1"], out int p1) || !int.TryParse(values["p2"], out int p2) || !int.TryParse(values["e1"], out int e1) || !int.TryParse(values["err"], out int err) ||
            !int.TryParse(values["inputs"], out int inputs) || !uint.TryParse(values["hb_age_ms"], out uint age) || !uint.TryParse(values["hb_max_gap_ms"], out uint gap) ||
            values["ref"] is not ("0" or "1") || inputs is < 0 or > 15) return null;
        return new(t, epoch, seq, values["phase"], home, traj, at, count, values["ref"] == "1", p1, p2, e1, err, inputs, age, gap, values["drv"], values["reason"]);
    }
}

public sealed partial class ControlWindow
{
    // Optional features the device declares in its LINK line. A firmware without a token simply lacks the feature.
    private bool supportsTrajRepeat, supportsFaultSnapshot, supportsTelemRate;
    private int reportedTelemHz = -1;                       // telem_hz= from LINK, confirmed by TELEM_RATE; -1 = unknown
    private int desiredTelemHz = DefaultTelemHz;
    private int telemRequestedHz, telemAttempts;
    private DateTime telemRequestedAt;
    private string lastTelemDiag = "";
    private const int DefaultTelemHz = 10;
    private const int TelemRetryLimit = 5;
    private static readonly Regex TelemRateAck = new(@"^TELEM_RATE epoch=(\d+) seq=(\d+) hz=(\d+)$", RegexOptions.CultureInvariant);
    private static readonly Regex TelemDiagPattern = new(@"^TELEM_DIAG hz=(\d+) sent=(\d+) skipped=(\d+) max_us=(\d+)$", RegexOptions.CultureInvariant);

    // The last seconds of telemetry, kept so a stop can be looked at from before it happened.
    private const double PreFaultSeconds = 15;
    private readonly Queue<(DateTime At, TelemetrySample Sample)> recentTelemetry = new();
    private FaultSnapshot? lastFault;
    private readonly TextBlock faultDetail = new() { TextWrapping = TextWrapping.Wrap, FontSize = 11, Foreground = WorkbenchTheme.Muted, Margin = new Thickness(10, 2, 10, 2) };
    internal static string? FaultRootOverride;               // tests write here instead of the real data folder
    private static string FaultRoot => FaultRootOverride ?? Path.Combine(ExperimentRoot, "faults");
    private DateTime lastFeedbackPaintAt;
    private const double StreamPaintMs = 150;                // with a streamed rate the panel repaints at ~7 Hz, the recorder still sees every sample

    private void ReadCapabilities(string[] words)
    {
        supportsTrajRepeat = words.Contains("traj_repeat=1");
        supportsFaultSnapshot = words.Contains("fault_snapshot=1");
        string? hz = words.FirstOrDefault(w => w.StartsWith("telem_hz=", StringComparison.Ordinal));
        supportsTelemRate = hz is not null;
        reportedTelemHz = hz is not null && int.TryParse(hz.AsSpan(9), out int value) ? value : -1;
    }

    private void ResetDiagnostics()
    {
        supportsTrajRepeat = supportsFaultSnapshot = supportsTelemRate = false;
        reportedTelemHz = -1; telemAttempts = 0; telemRequestedAt = default; lastTelemDiag = "";
        recentTelemetry.Clear(); lastFault = null;
    }

    // Asks the device for the chosen stream rate once the link is up, and again if it ever reports another one.
    // Output only, so it never waits for a quiet machine; a refusal is retried a few times and then reported.
    private void SynchronizeTelemetryRate(DateTime now)
    {
        if (!compatible || !framedControl || !protocolMatches || !supportsTelemRate || reportedEpoch == 0 || waitingStopEpoch || stopPending) return;
        if (reportedTelemHz == desiredTelemHz) { telemAttempts = 0; return; }
        lock (wireSync)
        {
            if (pendingControlType != 0 || (now - telemRequestedAt).TotalSeconds < 1) return;
            if (telemAttempts >= TelemRetryLimit)
            {
                if (telemAttempts == TelemRetryLimit) { telemAttempts++; Append("HOST telemetry rate: the device did not confirm " + desiredTelemHz + " Hz; leaving it at " + reportedTelemHz + " Hz"); }
                return;
            }
            telemAttempts++; telemRequestedHz = desiredTelemHz; telemRequestedAt = now; pendingControlType = 11;
        }
        Append($"HOST telemetry rate request {desiredTelemHz} Hz (attempt {telemAttempts}/{TelemRetryLimit})");
    }

    private void ObserveTelemetryRate(string line)
    {
        var m = TelemRateAck.Match(line);
        if (!m.Success || !int.TryParse(m.Groups[3].Value, out int hz) || !uint.TryParse(m.Groups[1].Value, out uint epoch) || epoch != reportedEpoch) return;
        reportedTelemHz = hz;
        if (hz == desiredTelemHz) telemAttempts = 0;
    }

    private void ObserveTelemetryDiagnostics(string line)
    {
        var m = TelemDiagPattern.Match(line);
        if (m.Success) lastTelemDiag = $"telemetry hz={m.Groups[1].Value} sent={m.Groups[2].Value} skipped={m.Groups[3].Value} max_us={m.Groups[4].Value}";
    }

    private void RememberTelemetry(TelemetrySample sample)
    {
        var now = DateTime.UtcNow;
        recentTelemetry.Enqueue((now, sample));
        while (recentTelemetry.Count > 0 && ((now - recentTelemetry.Peek().At).TotalSeconds > PreFaultSeconds || recentTelemetry.Count > 1200)) recentTelemetry.Dequeue();
    }

    // The device's own account of a stop: show it, and keep it with the seconds of telemetry that led up to it.
    private void HandleFaultSnapshot(string line)
    {
        var snapshot = FaultSnapshot.Parse(line);
        if (snapshot is null) { Append("HOST could not read a FAULT_SNAPSHOT line; kept in the log only"); return; }
        lastFault = snapshot;
        Lang.Text(faultDetail, () => DescribeFault(snapshot));
        try { string? where = SaveFaultBundle(snapshot, line); if (where is not null) Append("HOST fault snapshot saved: " + where); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { Append("HOST fault snapshot not saved: " + ex.Message); }
    }

    private static string DescribeFault(FaultSnapshot f)
    {
        var command = f.Command;
        string phase = Lang.T("snap.phase." + (f.Phase is "home" or "ptp" or "traj_run" or "traj_upload" or "jog" or "idle" or "disabled" ? f.Phase : "other"));
        return Lang.T("snap.summary", phase, f.Referenced ? $"{command.Q1:F2}°/{command.Q2:F2}°" : Lang.T("snap.noRef"),
                   f.Referenced ? $"{f.EncoderErrorDegrees:+0.000;-0.000;0.000}°" : "—", f.Inputs, f.HeartbeatAgeMs, f.HeartbeatMaxGapMs) + "\n" +
               (f.DriverFault ? Lang.T("snap.drvUndetermined") : Lang.T("snap.drvOk"));
    }

    // fault.json next to the recording of the run that stopped (or in the faults folder when nothing was recording),
    // and pre_fault.csv with the telemetry of the seconds before it. Nothing here is a diagnosis: see FaultSnapshot.
    private string? SaveFaultBundle(FaultSnapshot f, string rawLine)
    {
        if (selfTesting && FaultRootOverride is null) return null;
        string directory = experiment?.DirectoryPath ?? Path.Combine(FaultRoot, $"{DateTime.Now:yyyyMMdd-HHmmss-fff}-{f.Phase}");
        Directory.CreateDirectory(directory);
        File.WriteAllText(Path.Combine(directory, "fault.json"), JsonSerializer.Serialize(new {
            schema = 1, host_utc = DateTime.UtcNow, firmware = lastFirmwareBuild, link = lastLinkLine, raw = rawLine, snapshot = f,
            note = f.DriverFault ? "nFAULT can be over-current, over-temperature or under-voltage; the pin does not say which. Cause not determined." : "see snapshot.reason",
            pre_fault_seconds = PreFaultSeconds, pre_fault_samples = recentTelemetry.Count
        }, new JsonSerializerOptions { WriteIndented = true }), new UTF8Encoding(false));
        var csv = new StringBuilder("host_utc,device_ms,epoch,seq,q1_command_deg,q1_encoder_deg,q1_error_deg,q2_command_deg,slip_motor_lsb,inputs_active_mask\n");
        foreach (var (at, s) in recentTelemetry)
            csv.AppendLine(FormattableString.Invariant($"{at:O},{s.Milliseconds},{s.Epoch},{s.Sequence},{s.Command.Q1:F6},{s.Measured1:F6},{s.Error1:F6},{s.Command.Q2:F6},{s.Slip},{s.Inputs}"));
        File.WriteAllText(Path.Combine(directory, "pre_fault.csv"), csv.ToString(), new UTF8Encoding(true));
        return directory;
    }

    // Capabilities, the stream rate and the snapshot, against a firmware that has them and one that does not.
    private static void DiagnosticsSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Diagnostics: " + reason); }
        string temp = Path.Combine(Path.GetTempPath(), "arm-faults-" + Guid.NewGuid().ToString("N"));
        FaultRootOverride = temp;
        var w = new ControlWindow { selfTesting = true };
        var frames = new List<byte[]>();
        void Write(byte[] data, int offset, int count) { if (offset == 0 && (frames.Count == 0 || !ReferenceEquals(frames[^1], data))) frames.Add(data); }
        static int Type(byte[] frame) => (frame[1] >> 1) & 15;
        static int Payload(byte[] f, int i) => ((f[1 + 2 * i] >> 1) & 15) | (((f[2 + 2 * i] >> 1) & 15) << 4);
        byte[]? Next11() {
            w.PublishHeartbeatLease();
            for (int i = 0; i < 6; i++) { w.WireWriterStep(Write); if (frames.Count > 0 && frames[^1].Length == 36 && Type(frames[^1]) == 11) return frames[^1]; }
            return null;
        }
        string Link(string extra) => $"LINK uart=hw baud=115200 fw=stage2-v7-test short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1{extra} proto={DeviceContract.ProtocolVersion}";
        try
        {
            // An old firmware declares none of it: nothing is asked of it, and a finished plan is not offered again.
            w.HandleLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            w.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=3 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            w.HandleLine(Link(""));
            Check(!w.supportsTrajRepeat && !w.supportsFaultSnapshot && !w.supportsTelemRate && w.reportedTelemHz == -1, "an old LINK line declared a feature");
            w.SynchronizeTelemetryRate(DateTime.UtcNow + TimeSpan.FromSeconds(2));
            Check(w.pendingControlType == 0, "asked an old firmware for a stream rate");
            // A new one declares all three, and says its current rate.
            w.HandleLine(Link(" traj_repeat=1 fault_snapshot=1 telem_hz=0"));
            Check(w.supportsTrajRepeat && w.supportsFaultSnapshot && w.supportsTelemRate && w.reportedTelemHz == 0, "the new capabilities were not read");
            w.SynchronizeTelemetryRate(DateTime.UtcNow + TimeSpan.FromSeconds(2));
            Check(w.pendingControlType == 11 && w.telemRequestedHz == DefaultTelemHz, "the default stream rate was not requested");
            var frame = Next11();
            Check(frame is not null && Payload(frame, 13) == DefaultTelemHz && Payload(frame, 14) == 0 && Payload(frame, 9) == 0 && Payload(frame, 11) == 0, "the rate frame has the wrong payload");
            Check(w.pendingControlType == 0, "the rate frame stayed queued");
            // No ACK yet: the request is repeated after a second, not before.
            w.SynchronizeTelemetryRate(w.telemRequestedAt.AddMilliseconds(500)); Check(w.pendingControlType == 0, "retried too early");
            w.SynchronizeTelemetryRate(w.telemRequestedAt.AddSeconds(1.1)); Check(w.pendingControlType == 11 && w.telemAttempts == 2, "an unanswered request was not repeated");
            Next11();
            w.HandleLine("TELEM_RATE epoch=2 seq=9 hz=10"); Check(w.reportedTelemHz == 0, "an ACK for another epoch was taken");
            w.HandleLine("TELEM_RATE epoch=3 seq=9 hz=10"); Check(w.reportedTelemHz == 10 && w.telemAttempts == 0, "the ACK was not taken");
            w.SynchronizeTelemetryRate(DateTime.UtcNow + TimeSpan.FromSeconds(9)); Check(w.pendingControlType == 0, "kept asking after the ACK");
            w.desiredTelemHz = 20; w.SynchronizeTelemetryRate(DateTime.UtcNow + TimeSpan.FromSeconds(9));
            Check(w.pendingControlType == 11 && w.telemRequestedHz == 20, "a changed choice was not sent");
            Next11(); w.HandleLine("TELEM_RATE epoch=3 seq=10 hz=20");
            // A device that never confirms is asked a few times, then left alone and the operator is told.
            w.desiredTelemHz = 5; var at = DateTime.UtcNow + TimeSpan.FromSeconds(30);
            for (int i = 0; i < TelemRetryLimit + 3; i++) { w.SynchronizeTelemetryRate(at); Next11(); at = at.AddSeconds(1.1); }
            Check(w.telemAttempts > TelemRetryLimit && w.pendingControlType == 0, "kept retrying a rate the device does not confirm");
            w.desiredTelemHz = DefaultTelemHz;
            // The status block carries the device's cost numbers; they end up in the session summary.
            w.HandleLine("TELEM_DIAG hz=10 sent=120 skipped=3 max_us=410");
            Check(w.lastTelemDiag == "telemetry hz=10 sent=120 skipped=3 max_us=410", "TELEM_DIAG was not read");

            // The snapshot: parsed, shown, and saved with the telemetry before it. A driver fault is never explained.
            w.RememberTelemetry(TelemetrySample.Parse("TELEM t=1000 epoch=3 seq=0 ref=1 source=2 p1=100 p2=-50 e1=200 err=1 inputs=0 drops=0")!);
            string line = "FAULT_SNAPSHOT t=1500 epoch=3 seq=4 phase=traj_run home=0 traj=3 idx=120/383 ref=1 p1=2000 p2=-3000 e1=3500 err=-7 inputs=8 hb_age_ms=23 hb_max_gap_ms=41 drv=nfault_undetermined reason=\"DRV8825 nFAULT; drivers OFF, arm is NOT held\"";
            var parsed = FaultSnapshot.Parse(line);
            Check(parsed is { Phase: "traj_run", TrajIndex: 120, TrajCount: 383, Referenced: true, Steps1: 2000, Inputs: 8, HeartbeatAgeMs: 23, DriverFault: true } && parsed.Reason.StartsWith("DRV8825 nFAULT"), "the snapshot was not read");
            foreach (string bad in new[] { "FAULT_SNAPSHOT t=1 epoch=3", line.Replace(" ref=1", " ref=2"), line.Replace(" idx=120/383", " idx=120"), line.Replace("inputs=8", "inputs=99"), "STOP: x" })
                Check(FaultSnapshot.Parse(bad) is null, "a malformed snapshot was accepted: " + bad);
            Check(FaultSnapshot.Parse(line.Replace("\"DRV8825 nFAULT; drivers OFF, arm is NOT held\"", "\"cut off")) is { Reason: "cut off" }, "a snapshot cut by the debug buffer must still be readable");
            w.HandleLine(line);
            string[] saved = Directory.GetDirectories(temp);
            Check(saved.Length == 1 && saved[0].EndsWith("-traj_run") && File.Exists(Path.Combine(saved[0], "fault.json")) && File.Exists(Path.Combine(saved[0], "pre_fault.csv")), "the snapshot bundle was not written");
            string json = File.ReadAllText(Path.Combine(saved[0], "fault.json"));
            Check(json.Contains("Cause not determined") && File.ReadAllLines(Path.Combine(saved[0], "pre_fault.csv")).Length == 2, "the bundle must state the cause is undetermined and hold the telemetry before the stop");
            Check(w.faultDetail.Text.Contains("原因待诊断") && w.faultDetail.Text.Contains("图形执行") && w.faultDetail.Text.Contains("不能据此认定"), "the panel must show the phase and say the driver cause is undetermined: " + w.faultDetail.Text);
            w.HandleLine(line.Replace("drv=nfault_undetermined", "drv=ok").Replace("phase=traj_run", "phase=idle").Replace("hb_age_ms=23", "hb_age_ms=300"));
            Check(w.faultDetail.Text.Contains("心跳") && !w.faultDetail.Text.Contains("原因待诊断"), "a non-driver stop must not talk about the driver");
            // The ring keeps only what is recent.
            for (int i = 0; i < 1300; i++) w.RememberTelemetry(TelemetrySample.Parse("TELEM t=1 epoch=3 seq=0 ref=1 source=2 p1=0 p2=0 e1=0 err=0 inputs=0 drops=0")!);
            Check(w.recentTelemetry.Count <= 1200, "the telemetry ring is unbounded");
            w.Disconnect("diagnostics self-test");
            Check(!w.supportsTrajRepeat && !w.supportsTelemRate && w.reportedTelemHz == -1 && w.lastFault is null && w.recentTelemetry.Count == 0, "disconnect kept the device's capabilities or snapshot");
        }
        finally { w.Close(); FaultRootOverride = null; if (Directory.Exists(temp)) Directory.Delete(temp, true); }
    }
}
