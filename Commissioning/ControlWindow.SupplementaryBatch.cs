using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text.Json;
using System.Windows.Controls;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private CancellationTokenSource? supplementaryBatch;
    private int supplementaryRun;
    private string? supplementaryDrop, supplementaryStop;
    internal static bool SupplementaryStartValid(int first) => first is >= 1 and <= 39 && first % 2 == 1;
    private sealed record SupplementaryResult(int Run, string Shape, string Outcome, string? Drop, string? Stop, string? Directory);

    private static (int Axis, int Steps) SupplementaryInjection(int run)
    {
        int k = (run / 2 - 1) % 6;
        return (k % 2 + 1, 8 << (k / 2));
    }

    private void ObserveSupplementaryLine(string line)
    {
        if (supplementaryBatch is null) return;
        if (line.StartsWith("TEST_DROP ", StringComparison.Ordinal)) {
            supplementaryDrop = line;
            var expected = SupplementaryInjection(supplementaryRun);
            if (supplementaryRun <= 0 || supplementaryRun % 2 != 0 ||
                line != $"TEST_DROP run={supplementaryRun} axis={expected.Axis} usteps={expected.Steps}") {
                supplementaryBatch.Cancel(); Stop();
            }
        }
        if (line.StartsWith("STOP:", StringComparison.Ordinal)) {
            // The host sends its own stop after the encoder fault; a manual stop already cancels at the Stop entry point.
            if (supplementaryStop?.StartsWith("STOP: PTP encoder following error", StringComparison.Ordinal) == true &&
                line.Contains("operator stop", StringComparison.Ordinal)) return;
            supplementaryStop = line;
            if (supplementaryRun == 0 || !line.StartsWith("STOP: PTP encoder following error", StringComparison.Ordinal))
                supplementaryBatch.Cancel();
        }
        if (line.StartsWith("ENABLE refused:", StringComparison.Ordinal) || line.StartsWith("HOME refused:", StringComparison.Ordinal) ||
            line.StartsWith("PTP refused:", StringComparison.Ordinal)) supplementaryBatch.Cancel();
    }

    internal async Task RunSupplementaryBatch(string port, int first)
    {
        if (!SupplementaryStartValid(first) || supplementaryBatch is not null) throw new ArgumentOutOfRangeException(nameof(first));
        using var cancellation = supplementaryBatch = new CancellationTokenSource();
        var token = cancellation.Token;
        var results = new List<SupplementaryResult>();
        string report = Path.Combine(ExperimentRoot, $"supplementary-{DateTime.Now:yyyyMMdd-HHmmss}.json");
        Directory.CreateDirectory(ExperimentRoot);
        async Task Progress(string phase) {
            await File.WriteAllTextAsync(report, JsonSerializer.Serialize(new { phase, nextRun = supplementaryRun, results,
                updatedUtc = DateTime.UtcNow }, new JsonSerializerOptions { WriteIndented = true }));
        }
        async Task Wait(Func<bool> ready, string phase, double seconds = 30) {
            var clock = Stopwatch.StartNew();
            while (true) {
                token.ThrowIfCancellationRequested();
                if (serial is not { IsOpen: true }) throw new IOException("Supplementary batch: serial link closed");
                if (ready()) return;
                if (clock.Elapsed.TotalSeconds > seconds) throw new TimeoutException("Supplementary batch: " + phase);
                await Task.Delay(50, token);
            }
        }
        async Task Home() {
            supplementaryRun = 0; supplementaryDrop = supplementaryStop = null;
            await Progress("home");
            if (!enabled) {
                await Wait(() => enable.IsEnabled && LimitsReady, "enable ready");
                RequestEnable();
                await Wait(() => enabled && pendingControl is null, "enable");
            }
            await Wait(() => home.IsEnabled && LimitsReady, "home ready");
            RequestHome();
            if (pendingControl != "回零" && !homing) throw new InvalidOperationException("Supplementary batch: home was not accepted");
            await Wait(() => !homing && pendingControl is null && homed && CanPoint, "home complete", 180);
        }
        async Task Approach(JointPose pose) {
            await Wait(() => CanPoint && LimitsReady, "move ready");
            if (AtStart(pose)) return;
            RequestPoint(PtpProtocol.Quantize(pose));
            if (pendingPoint == 0) throw new InvalidOperationException("Supplementary batch: move was not accepted");
            await Wait(() => pendingPoint == 0 && ptp is { Active: 0 } && AtStart(pose) && experiment is null, "move complete", 150);
        }
        try {
            await Progress("connect");
            if (AutomationWarmUp is { } warm) await warm;
            token.ThrowIfCancellationRequested();
            ports.SelectedItem = port;
            if (ports.SelectedItem is not string selected || selected != port) throw new IOException("Supplementary batch: requested port missing");
            Connect();
            await Wait(() => compatible && protocolMatches && LimitsReady && lastFirmwareBuild != "unknown", "link contract");
            if (!lastFirmwareBuild.EndsWith("-enccomp-TESTDROP", StringComparison.Ordinal))
                throw new InvalidOperationException("Supplementary batch requires enccomp-TESTDROP firmware");
            Append("HOST supplementary batch started first=" + first + " progress=" + report);
            await Home();
            int normalFaults = 0;
            for (int run = first; run <= 40; run++) {
                token.ThrowIfCancellationRequested();
                supplementaryRun = 0; supplementaryStop = supplementaryDrop = null;
                xyPreview.ConfigureSupplementary(run);
                await Wait(() => LimitsReady && CanPoint && boardKnown && storeJob is null, "parameters and archive");
                var plan = xyPreview.Plan ?? throw new InvalidOperationException("Supplementary batch: invalid shape");
                string settings = xyPreview.SettingsDescription;
                await Progress("approach");
                await Approach(plan.Nodes[0].Pose);
                if (TrajectoryArchive.SlotFor(boardSlots, plan.Nodes) is null) {
                    int slot = TrajectoryArchive.FreeSlot(boardSlots, storeSlotCount) ??
                        throw new InvalidOperationException("Supplementary batch: no empty board slot; nothing erased");
                    RequestSaveToBoard(plan, slot);
                    await Wait(() => storeJob is null && pendingBoardSave is null && deferredBoardSave is null && boardKnown, "archive save", 90);
                    if (TrajectoryArchive.SlotFor(boardSlots, plan.Nodes) is null) throw new IOException("Supplementary batch: archive save not verified");
                }
                if (uploaded is null || !uploaded.Nodes.SequenceEqual(plan.Nodes)) {
                    RequestTrajectory(plan);
                    if (uploading is null && uploaded is null) throw new InvalidOperationException("Supplementary batch: load was not accepted");
                    await Wait(() => uploaded is not null && uploading is null && pendingPoint == 0, "archive load", 90);
                }
                if (settings != xyPreview.SettingsDescription) throw new InvalidOperationException("Supplementary batch: parameters changed during preparation");
                supplementaryRun = run; supplementaryStop = supplementaryDrop = null;
                await Progress("run");
                RequestExecute();
                if (uploading is null && !executing) throw new InvalidOperationException("Supplementary batch: RUN was not accepted");
                await Wait(() => uploading is null && !executing && pendingPoint == 0 && experiment is null, "run complete", plan.Duration + 40);
                bool fault = supplementaryStop is not null;
                if (run % 2 == 0 && supplementaryDrop is null) throw new IOException("Supplementary batch: injection counter not confirmed");
                if (!fault && lastExperimentOutcome != "complete") throw new IOException("Supplementary batch: run did not complete");
                results.Add(new(run, settings, lastExperimentOutcome ?? "aborted", supplementaryDrop, supplementaryStop, lastExperimentDirectory));
                await Progress("recorded");
                if (run % 2 == 1) {
                    normalFaults = fault ? normalFaults + 1 : 0;
                    if (normalFaults >= 2) throw new IOException("Supplementary batch: two consecutive normal runs stopped");
                }
                if (run % 2 == 0 || fault) await Home();
            }
            // A6: the approach to A is excluded; each tier records the six A/B moves separately.
            for (int tier = 0; tier < 5; tier++) {
                supplementaryRun = 0; supplementaryStop = supplementaryDrop = null;
                xyPreview.ConfigureSupplementaryPoint(tier, false);
                await Wait(() => LimitsReady && CanPoint, "speed tier ready");
                await Approach(xyPreview.PointTargetPose ?? throw new InvalidOperationException("Supplementary batch: invalid point"));
                for (int move = 0; move < 6; move++) {
                    token.ThrowIfCancellationRequested();
                    xyPreview.ConfigureSupplementaryPoint(tier, move % 2 == 0);
                    await Wait(() => LimitsReady && CanPoint, "point ready");
                    await Progress($"V{tier + 1} move {move + 1}/6");
                    await Approach(xyPreview.PointTargetPose ?? throw new InvalidOperationException("Supplementary batch: invalid point"));
                    results.Add(new(0, $"V{tier + 1} move {move + 1}", lastExperimentOutcome ?? "missing", null, supplementaryStop, lastExperimentDirectory));
                    await Progress("recorded");
                }
            }
            xyPreview.ConfigureSupplementary(20);
            await Wait(() => LimitsReady, "restore L3");
            await Progress("complete motion A4-A6; L3 restored; final temperature and release firmware required");
            Append("HOST supplementary batch complete progress=" + report);
            supplementaryBatch = null; Stop(); Disconnect("HOST supplementary batch complete");
        }
        catch (Exception ex) {
            Stop();
            Append("HOST supplementary batch stopped: " + ex.Message, LogLevel.Error);
            await Progress("stopped: " + ex.Message);
        }
        finally { supplementaryRun = 0; supplementaryBatch = null; }
    }

    internal static void SupplementaryBatchSelfTest()
    {
        static void Check(bool ok, string reason) { if (!ok) throw new InvalidOperationException("Supplementary batch: " + reason); }
        foreach (int bad in new[] { -1, 0, 2, 40, 41 }) Check(!SupplementaryStartValid(bad), "unsafe start accepted");
        var w = new ControlWindow { selfTesting = true };
        try {
            w.boardSlots[0] = BoardSlot.Parse("TRAJ_SLOT slot=0 valid=0");
            Check(TrajectoryArchive.FreeSlot(w.boardSlots, 32) == 0, "reported empty slot was rejected");
            w.xyPreview.PrepareDesktopReplay();
            foreach (int run in new[] { 1, 13, 21, 25, 29, 33, 37 }) {
                w.xyPreview.ConfigureSupplementary(run);
                Check(w.xyPreview.Plan is { Nodes.Length: > 1 }, "invalid A4/A5 shape " + run);
            }
            for (int tier = 0; tier < 5; tier++) foreach (bool b in new[] { false, true }) {
                w.xyPreview.ConfigureSupplementaryPoint(tier, b);
                Check(w.xyPreview.PointTargetPose is { } pose && w.xyPreview.RequestedLimits!.Value.Contains(pose), "invalid speed-tier point");
            }
            w.supplementaryBatch = new CancellationTokenSource(); w.supplementaryRun = 14;
            w.ObserveSupplementaryLine("TEST_DROP run=14 axis=1 usteps=8");
            Check(!w.supplementaryBatch.IsCancellationRequested, "correct injection rejected");
            w.Stop(cancelBatch: false); Check(!w.supplementaryBatch.IsCancellationRequested, "device following-error recovery cancelled");
            w.Stop(); Check(w.supplementaryBatch.IsCancellationRequested, "manual STOP did not cancel batch");
            w.supplementaryBatch.Dispose(); w.supplementaryBatch = new();
            w.ObserveSupplementaryLine("TEST_DROP run=16 axis=2 usteps=8");
            Check(w.supplementaryBatch.IsCancellationRequested, "wrong run counter accepted");
            w.supplementaryBatch.Dispose(); w.supplementaryBatch = new(); w.supplementaryRun = 0;
            w.ObserveSupplementaryLine("STOP: DEADMAN host heartbeat timeout");
            Check(w.supplementaryBatch.IsCancellationRequested, "heartbeat fault did not cancel batch");
        }
        finally { w.supplementaryBatch?.Dispose(); w.supplementaryBatch = null; w.Close(); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal void ConfigureSupplementary(int run)
    {
        var p = run <= 20 ? (3, 220d, 0d, 80d, 10d, 0d) : run <= 24 ? (1, 180d, -100d, 300d, 100d, 0d) :
            run <= 28 ? (2, 170d, -60d, 120d, 120d, 0d) : run <= 32 ? (6, 170d, -60d, 120d, 120d, 20d) :
            run <= 36 ? (8, 230d, 0d, 70d, 5d, 0d) : (7, 230d, 0d, 80d, 50d, 2d);
        SetSupplementaryFields(p.Item1, p.Item2, p.Item3, p.Item4, p.Item5, p.Item6, 14.5, 15.1, 10, 8);
    }

    internal void ConfigureSupplementaryPoint(int tier, bool toB)
    {
        (double V1, double V2)[] speeds = [(14.5, 15.1), (18, 17), (21, 19), (24.5, 20.5), (28, 22)];
        var v = speeds[tier];
        SetSupplementaryFields(0, toB ? 34 : 242.7, toB ? 174.9 : -244.1, 5, 10, 0, v.V1, v.V2, 16.9, 13.2);
    }

    private void SetSupplementaryFields(int s, double px, double py, double pa, double pb, double pc, double speed1, double speed2, double a1, double a2)
    {
        updating = true;
        try {
            shape.SelectedIndex = s; branch.SelectedIndex = 0; direction.SelectedIndex = 0;
            TextBox[] fields = [x, y, a, b, c, v1, v2, acc1, acc2, rotation, scale];
            double[] values = [px, py, pa, pb, pc, speed1, speed2, a1, a2, 0, 1];
            for (int i = 0; i < fields.Length; i++) fields[i].Text = values[i].ToString(CultureInfo.InvariantCulture);
            ClampRateFields();
        }
        finally { updating = false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }
}
