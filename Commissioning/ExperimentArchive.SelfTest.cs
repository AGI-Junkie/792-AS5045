using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows;

namespace TwoLinkCommissioning;

public static partial class ExperimentArchiveTests
{
    // A recording written by the real recorder from a real plan, with telemetry synthesised so the right answers are known:
    // the device clock starts the plan at `startMs`, and arm1's encoder differs from its command by 0.02 deg * sin(2 pi t / 3).
    internal static string WriteSyntheticRun(string root, DrawingShape shape, double startMs, string settings, out ExecutionTrajectory plan, int hz = 10, double errorAmplitude = 0.02, bool followPlan = true, string? calibrationId = null)
    {
        var model = new PlanarArm(180, 176.38);
        // Big enough that the planner is limited by the motors' speed and acceleration, not by the 0.15 mm chord check that stretches tiny shapes.
        var path = DrawingTrajectory.Create(model, shape, 320, -15, shape == DrawingShape.Line ? 350 : shape == DrawingShape.Circle ? 20 : 30, shape == DrawingShape.Line ? 10 : shape == DrawingShape.Circle ? 0 : 24, 0, null);
        plan = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(1000, 5, 3, 10, 6)), new JointLimits(-90, 90, -160, 160));
        using var recorder = new ExperimentRecorder(root, "trajectory", 7, 5, model, settings, "人工伸直零位", plan.Nodes, hz, calibrationId);
        var times = new double[plan.Nodes.Length]; double total = 0;
        for (int i = 0; i < plan.Nodes.Length; i++) { total += plan.Nodes[i].Milliseconds / 1000.0; times[i] = total; }
        var q1 = plan.Nodes.Select(n => n.Pose.Q1).ToArray(); var q2 = plan.Nodes.Select(n => n.Pose.Q2).ToArray();
        double Interp(double[] ys, double t) {
            if (t <= 0) return ys[0]; if (t >= times[^1]) return ys[^1];
            int k = Array.FindIndex(times, x => x >= t); double u = (t - times[k - 1]) / (times[k] - times[k - 1]); return ys[k - 1] + (ys[k] - ys[k - 1]) * u;
        }
        var rng = new Random(11);
        double step = 1000.0 / hz;
        for (double ms = startMs - 600; ms < startMs + times[^1] * 1000 + 800; ms += step)
        {
            double tau = (ms - startMs) / 1000.0;
            var pose = followPlan ? new JointPose(Interp(q1, tau), Interp(q2, tau)) : new JointPose(rng.NextDouble() * 40 - 20, rng.NextDouble() * 40 - 20);
            double err = errorAmplitude * Math.Sin(2 * Math.PI * tau / 3);
            int encoder = (int)Math.Round((pose.Q1 + err) / 360.0 * DeviceContract.EncoderCounts * DeviceContract.Arm1GearRatio);
            int encoder2 = (int)Math.Round((pose.Q2-pose.Q1) / 360.0 * DeviceContract.EncoderCounts * (DeviceContract.Arm2GearRatioX100/100.0));
            var sample = new TelemetrySample((uint)ms, 5, 1, true, 2, ExecutionTrajectory.Counts(0, pose), ExecutionTrajectory.Counts(1, pose), encoder, 0, 0, 0, encoder2, 0);
            recorder.Record(sample, tau < -0.3 ? "upload" : tau < 0.06 ? "ready" : "drawing", 0);
        }
        recorder.Finish("complete");
        return recorder.DirectoryPath;
    }

    public static void SelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Experiment archive: " + reason); }
        string root = Path.Combine(Path.GetTempPath(), "arm-archive-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        try
        {
            const string settings = "shape=2; branch=0; L1=180; L2=176.38; x=320; y=-15; a=30; b=24; feed=136.6; v1=5; v2=3; a1=10; a2=6; min1=-90; max1=90; min2=-160; max2=160; emin=-155; emax=155; c=; rot=0; scale=1; dir=ccw; custom_chars=0";
            string dir = WriteSyntheticRun(root, DrawingShape.Rectangle, 8200, settings, out var plan);
            Thread.Sleep(15);
            string circle = WriteSyntheticRun(root, DrawingShape.Circle, 31000, settings.Replace("shape=2", "shape=3").Replace("v1=5", "v1=2.5"), out var circlePlan, hz: 2, calibrationId: "cal-20261002-101500-v3");
            // The list: both recordings, newest first, with what the table shows.
            var (runs, unreadable) = ExperimentArchive.List(root);
            Check(unreadable == 0 && runs.Count == 2 && runs[0].Directory == circle && runs[1].Directory == dir, "newest first, both listed");
            var run = runs[1];
            // The calibration that was current when it ran travels with the recording: its id in the list, its version in the table and the CSV.
            Check(runs[0].CalibrationId == "cal-20261002-101500-v3" && runs[0].CalibrationVersion == "v3" && run.CalibrationId == "" && run.CalibrationVersion == "", "a recording remembers which calibration was current");
            string csvText = ExperimentArchive.ReportCsv(runs, analyse: false); var csvRows = csvText.TrimEnd().Split("\r\n");
            int calColumn = Array.IndexOf(csvRows[0].Split(','), "calibration_id");
            Check(calColumn > 0 && csvRows[1].Split(',')[calColumn] == "cal-20261002-101500-v3" && csvRows[2].Split(',')[calColumn] == "", "the report carries the calibration id");
            Check(run.Kind == "trajectory" && run.Complete && run.Reference == "人工伸直零位" && run.TelemetryHz == 10 && runs[0].TelemetryHz == 2 && run.Samples > 50 && run.MotionSamples > 40 && run.Rmse is > 0.01 and < 0.02, "summary fields");
            Check(run.Settings is { Shape: 2, V1: 5, V2: 3, A1: 10, A2: 6, Limits.Min1: -90, Limits.ElbowMax: 155, Clockwise: false, Scale: 1 }, "settings parsed from the recording");
            Check(runs[0].Settings is { Shape: 3, V1: 2.5 }, "second recording's own settings");
            // Open it: the plan, aligned to the device clock, with its events, error and speed profile.
            var data = ExperimentArchive.Load(run);
            Check(data.Plan.Length == plan.Nodes.Length && data.Feedback.Length > 50 && data.SkippedRows == 0, "plan and feedback loaded");
            var a = RunAnalysis.Create(run, data);
            Check(a.Rmse2 is { } r2 && run.Rmse2 is { } s2 && Math.Abs(r2 - s2) < 1e-3 && a.Peak2 is { } k2 && run.Peak2 is { } sk2 && Math.Abs(k2 - sk2) < 1e-3, $"arm2 statistics from the samples must equal the recorder's summary: {a.Rmse2} vs {run.Rmse2}");
            Check(csvRows[0].Contains("q2_rmse_deg,q2_peak_deg") && csvRows[2].Split(',')[Array.IndexOf(csvRows[0].Split(','), "q2_rmse_deg")] != "", "the report carries arm2's statistics");
            Check(a.Aligned && Math.Abs(a.AlignmentOffsetMs - 8200) <= 10 && a.AlignmentResidualDeg < 0.02, $"the plan must be aligned to the device clock within 10 ms: {a.AlignmentOffsetMs} residual {a.AlignmentResidualDeg}");
            Check(a.T.Length > 40 && a.T.Zip(a.T.Skip(1)).All(p => p.Second > p.First) && a.T[0] > -0.2 && a.T[^1] > a.Duration - 0.3, "time axis starts with the plan");
            Check(a.Events.Length == 5 && a.Events[0] is { Kind: "start", Time: 0 } && a.Events[1..4].All(e => e.Kind == "dwell") && a.Events[4].Kind == "stop" && a.Events.Zip(a.Events.Skip(1)).All(p => p.Second.Time > p.First.Time), "a rectangle has a start, three corner dwells and a stop: " + string.Join(",", a.Events.Select(e => e.Kind)));
            Check(Math.Abs(a.Events[4].Time - a.Duration) < 0.1, "the stop is at the end of the plan");
            Check(a.Segments.Length == 4 && a.Segments.All(s => s.Samples > 3) && a.Segments.All(s => s.Rmse is > 0.001 and < 0.03), "per-stroke error statistics");
            Check(Math.Abs(a.Rmse - 0.02 / Math.Sqrt(2)) < 0.02 / Math.Sqrt(2) * 0.15 && a.Peak is > 0.017 and <= 0.03, $"RMSE and peak of the known error: {a.Rmse} {a.Peak}");
            Check(Math.Abs(a.Rmse - run.Rmse!.Value) < 0.003, "recomputed RMSE agrees with the recorder's summary");
            // The profile: motor 2's speed is the elbow's (q2 - q1), and one motor runs at its cap (the planner stretches time until nothing exceeds it).
            Check(a.Axes.Length == 4 && a.Axes.All(x => x.Utilization <= 1.06 && x.Peak > 0), "no motor exceeds its cap: " + string.Join(",", a.Axes.Select(x => $"{x.Name}={x.Utilization:F2}")));
            // Whether any motor is at its cap depends on the plan: the planner also stretches time for its pulse and chord checks, so a plan can
            // run well under every cap. The list must say exactly what the numbers say.
            Check(a.LimitingAxes.SequenceEqual(a.Axes.Where(x => x.Utilization >= 0.9).Select(x => x.Name)), "the limiting axes are those at 90% of their cap or more");
            int mid = a.PlanVT.Length / 2;
            double elbowSpeed = ((a.PlanQ2[mid + 1] - a.PlanQ1[mid + 1]) - (a.PlanQ2[mid] - a.PlanQ1[mid])) / (a.PlanT[mid + 1] - a.PlanT[mid]);
            Check(Math.Abs(a.PlanV2[mid] - elbowSpeed) < 1e-9 && a.PlanAT.Length == a.PlanVT.Length - 1, "plan speed of motor 2 is the elbow's");
            Check((new Point(a.PlanX[0], a.PlanY[0]) - new Point(320, -15)).Length < 0.05 && a.XCommand.Length == a.T.Length && Math.Abs(a.At(a.Q1Command, 0) - a.PlanAt(a.PlanQ1, 0)) < 0.05, "plan path and sample series line up at the start");
            // Slower plan, more telemetry: the second recording is the circle at 2.5 deg/s for motor 1.
            var b = RunAnalysis.Create(runs[0], ExperimentArchive.Load(runs[0]));
            Check(b.Aligned && Math.Abs(b.AlignmentOffsetMs - 31000) <= 25 && b.Events.Length == 2 && b.Events[0].Kind == "start" && b.Events[1].Kind == "stop", $"a circle has only start and stop (2 Hz telemetry aligns within 25 ms): {b.AlignmentOffsetMs} / {b.Events.Length}");
            // A recording whose samples do not follow its plan is flagged, not trusted.
            string odd = WriteSyntheticRun(root, DrawingShape.Line, 52000, settings, out _, followPlan: false);
            var oddRun = ExperimentArchive.List(root).Runs.First(r => r.Directory == odd);
            Check(RunAnalysis.Create(oddRun, ExperimentArchive.Load(oddRun)).AlignmentResidualDeg > 1, "a recording that does not follow its plan must show a large alignment residual");
            // A plan that holds motor 1 exactly at its cap and nothing else: the limiter is v1, and only v1.
            string cappedDir = Path.Combine(root, "20260102-000000-000-trajectory-2"); Directory.CreateDirectory(cappedDir);
            File.WriteAllText(Path.Combine(cappedDir, "metadata.json"), JsonSerializer.Serialize(new { schema = 1, kind = "trajectory", sequence = 2, epoch = 2, reference = "x", started_utc = DateTime.UtcNow.AddDays(-2), L1 = 180, L2 = 176.38, settings = "shape=3; branch=0; v1=5; v2=3; a1=10; a2=6" }));
            File.WriteAllText(Path.Combine(cappedDir, "plan.csv"), "index,time_ms,q1_target_deg,q2_target_deg,p1_ccw_microsteps,p2_ccw_microsteps" + Environment.NewLine +
                string.Join(Environment.NewLine, Enumerable.Range(0, 9).Select(i => FormattableString.Invariant($"{i},{i * 250},{i * 1.25:F2},{i * 1.25:F2},0,0"))) + Environment.NewLine);
            var capped = ExperimentArchive.List(root).Runs.First(r => r.Directory == cappedDir);
            var cappedAnalysis = RunAnalysis.Create(capped, ExperimentArchive.Load(capped));
            Check(cappedAnalysis.LimitingAxes.SequenceEqual(["v1"]) && cappedAnalysis.Axes[0].Peak is > 4.99 and < 5.01 && cappedAnalysis.Axes[1].Peak == 0 && cappedAnalysis.Axes[2].Peak < 1e-9, "motor 1 at its cap is the only limiter: " + string.Join(",", cappedAnalysis.Axes.Select(x => $"{x.Name}={x.Utilization:F2}")));
            File.WriteAllText(Path.Combine(cappedDir, "metadata.json"), File.ReadAllText(Path.Combine(cappedDir, "metadata.json")).Replace("v1=5", "v1=20"));
            var slow = ExperimentArchive.List(root).Runs.First(r => r.Directory == cappedDir);
            Check(RunAnalysis.Create(slow, ExperimentArchive.Load(slow)).LimitingAxes.Length == 0, "a plan well under every cap has no limiter");
            Directory.Delete(cappedDir, true);
            // A single point: no plan to align, time starts at the first moving sample.
            string pointDir = Path.Combine(root, "20260101-000000-000-point-1"); Directory.CreateDirectory(pointDir);
            File.WriteAllText(Path.Combine(pointDir, "metadata.json"), JsonSerializer.Serialize(new { schema = 1, kind = "point", sequence = 1, epoch = 2, reference = "x", started_utc = DateTime.UtcNow.AddDays(-1), L1 = 180, L2 = 176.38, settings = "shape=0; branch=0; v1=5; v2=3; a1=10; a2=6" }));
            File.WriteAllText(Path.Combine(pointDir, "plan.csv"), "index,time_ms,q1_target_deg,q2_target_deg,p1_ccw_microsteps,p2_ccw_microsteps\n0,0,10.00,20.00,100,200\n");
            File.WriteAllText(Path.Combine(pointDir, "feedback.csv"), "host_utc,device_ms,epoch,seq,phase,segment,q1_command_deg,q1_encoder_deg,q1_error_deg,q2_command_deg,q2_measured_deg,x_command_model_mm,y_command_model_mm,x_hybrid_estimate_mm,y_hybrid_estimate_mm,slip_motor_lsb,inputs_active_mask,rx_drops\n" +
                "2026-01-01T00:00:00.0000000Z,5000,2,1,point,0,1,1.01,0.01,3,,300,10,300,10,0,0,0\n2026-01-01T00:00:00.5000000Z,5500,2,1,point,0,2,2.02,0.02,4,,301,11,301,11,0,0,0\nnot,a,row\n");
            var pointRun = ExperimentArchive.List(root).Runs.First(r => r.Directory == pointDir);
            Check(pointRun.Outcome == "interrupted" && !pointRun.Complete && pointRun.Settings is { Shape: 0 } && pointRun.Settings.Limits is null, "an old recording without summary.json is listed as interrupted");
            var pointData = ExperimentArchive.Load(pointRun);
            var p = RunAnalysis.Create(pointRun, pointData);
            Check(pointData.SkippedRows == 1 && !p.Aligned && p.T.SequenceEqual([0.0, 0.5]) && p.Events.Length == 0 && Math.Abs(p.Rmse - 0.0158) < 0.001, "a point run: time from the first moving sample, a damaged row skipped and counted");
            // Damaged or foreign folders are counted or ignored, never fatal.
            Directory.CreateDirectory(Path.Combine(root, "broken")); File.WriteAllText(Path.Combine(root, "broken", "metadata.json"), "{ not json");
            Directory.CreateDirectory(Path.Combine(root, "no-time")); File.WriteAllText(Path.Combine(root, "no-time", "metadata.json"), "{\"kind\":\"point\"}");
            Directory.CreateDirectory(Path.Combine(root, "faults", "x")); Directory.CreateDirectory(Path.Combine(root, "empty"));
            var (all, bad) = ExperimentArchive.List(root);
            Check(bad == 2 && all.Count == 4 && all.All(r => !r.Directory.EndsWith("faults")), "two damaged folders are counted and the rest still listed");
            try { ExperimentArchive.Load(new ExperimentRun { Directory = Path.Combine(root, "empty") }); } catch (Exception ex) { Check(false, "an empty folder must load as no data: " + ex.Message); }
            File.WriteAllText(Path.Combine(root, "broken", "feedback.csv"), "a,b,c\n1,2,3\n");
            bool refused = false; try { ExperimentArchive.Load(new ExperimentRun { Directory = Path.Combine(root, "broken") }); } catch (FormatException) { refused = true; }
            Check(refused, "a feedback file with the wrong header must be refused, not misread");
            // Settings text from several generations of the recorder.
            Check(RunSettings.Parse(null) is null && RunSettings.Parse("nonsense") is null && RunSettings.Parse("shape=1; v1=1; v2=1; a1=1; a2=1") is { Shape: 1, Scale: 1, Rotation: 0, Limits: null }, "settings text: missing keys take defaults, missing speeds are refused");
            Check(RunSettings.Parse("shape=3; branch=1; v1=5; v2=3; a1=10; a2=6; min1=-30; max1=30; min2=-60; max2=60; rot=15; scale=1.5; dir=cw") is { Branch: 1, Rotation: 15, Scale: 1.5, Clockwise: true, Limits.ElbowMin: -155 }, "an older recording has no elbow keys: the default elbow is used");
        }
        finally { if (Directory.Exists(root)) Directory.Delete(root, true); }
    }
}
