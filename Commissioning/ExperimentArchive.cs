using System.Globalization;
using System.IO;
using System.Text.Json;
using System.Windows;

namespace TwoLinkCommissioning;

// What was set when an experiment ran, read back from the "settings" text every recording carries
// (shape=3; branch=0; L1=180; ...). Older recordings have fewer keys: the optional ones take their defaults.
public sealed record RunSettings(int Shape, int Branch, double L1, double L2, double X, double Y, double A, double B, double C,
    double Rotation, double Scale, bool Clockwise, double Feed, double V1, double V2, double A1, double A2, JointLimits? Limits)
{
    public static RunSettings? Parse(string? text)
    {
        if (string.IsNullOrWhiteSpace(text)) return null;
        var values = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (string part in text.Split(';', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries))
        {
            int eq = part.IndexOf('=');
            if (eq > 0) values[part[..eq].Trim()] = part[(eq + 1)..].Trim();
        }
        bool Number(string key, out double value) { value = 0; return values.TryGetValue(key, out var t) && double.TryParse(t, NumberStyles.Float, CultureInfo.InvariantCulture, out value) && double.IsFinite(value); }
        double Or(string key, double fallback) => Number(key, out double v) ? v : fallback;
        if (!Number("shape", out double shape) || !Number("v1", out double v1) || !Number("v2", out double v2) || !Number("a1", out double a1) || !Number("a2", out double a2)) return null;
        JointLimits? limits = Number("min1", out double min1) && Number("max1", out double max1) && Number("min2", out double min2) && Number("max2", out double max2)
            ? new JointLimits(min1, max1, min2, max2, Or("emin", -JointLimits.DefaultElbow), Or("emax", JointLimits.DefaultElbow)) : null;
        return new((int)shape, (int)Or("branch", 0), Or("L1", 180), Or("L2", 176.38), Or("x", 0), Or("y", 0), Or("a", 0), Or("b", 0), Or("c", 0),
            Or("rot", 0), Or("scale", 1), values.TryGetValue("dir", out var dir) && dir == "cw", Or("feed", 1000), v1, v2, a1, a2, limits);
    }
}

// One recording on disk: the facts shown in the list. The samples are only read when a run is opened.
public sealed class ExperimentRun
{
    public required string Directory { get; init; }
    public string Id => Path.GetFileName(Directory);
    public string Kind { get; init; } = "";
    public DateTime StartedUtc { get; init; }
    public uint Sequence { get; init; }
    public string Reference { get; init; } = "";
    public string Outcome { get; init; } = "interrupted";          // "complete", another reason, or "interrupted" when summary.json is missing
    public bool Complete => Outcome == "complete";
    public int Samples { get; init; }
    public int MotionSamples { get; init; }
    public double? Rmse { get; init; }
    public double? Peak { get; init; }
    public double? Rmse2 { get; init; }                             // arm2's own encoder, null when the run had none
    public double? Peak2 { get; init; }
    public int TelemetryHz { get; init; }                           // 0 = the old ~2 Hz status-block telemetry
    public RunSettings? Settings { get; init; }
    public string SettingsText { get; init; } = "";                 // as recorded, so the fields can be restored exactly
    public double L1 { get; init; } = 180;
    public double L2 { get; init; } = 176.38;
    public bool HasFault { get; init; }
    public string CalibrationId { get; init; } = "";               // the calibration current when it ran ("" = none recorded)
    public string CalibrationVersion => CalibrationId.Contains("-v") ? "v" + CalibrationId[(CalibrationId.LastIndexOf("-v", StringComparison.Ordinal) + 2)..] : "";
}

public sealed record FeedbackRow(DateTime HostUtc, uint DeviceMs, uint Epoch, uint Sequence, string Phase, int Segment, double Q1Command, double Q1Encoder,
    double Q1Error, double Q2Command, double XCommand, double YCommand, double XHybrid, double YHybrid, int Slip, int Inputs, uint Drops, double? Q2Measured = null);
public sealed record PlanNode(int Index, int TimeMs, double Q1, double Q2);
public sealed record PreFaultRow(double DeviceMs, double Q1Command, double Q1Encoder, double Q1Error, DateTime HostUtc = default);
public sealed record RunData(PlanNode[] Plan, FeedbackRow[] Feedback, int SkippedRows, PreFaultRow[]? PreFault = null);

public static class ExperimentArchive
{
    public const int ListLimit = 500;

    // Newest first. A folder that cannot be read is counted, not fatal: one damaged recording must not hide the others.
    // includeFaults also lists the stop bundles saved under "faults" when nothing was recording (kind "fault": fault.json and pre_fault.csv only).
    public static (List<ExperimentRun> Runs, int Unreadable) List(string root, bool includeFaults = false)
    {
        var runs = new List<ExperimentRun>(); int unreadable = 0;
        if (!System.IO.Directory.Exists(root)) return (runs, 0);
        foreach (string directory in System.IO.Directory.EnumerateDirectories(root))
        {
            if (Path.GetFileName(directory).Equals("faults", StringComparison.OrdinalIgnoreCase) || !File.Exists(Path.Combine(directory, "metadata.json"))) continue;
            try { runs.Add(ReadRun(directory)); }
            catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException or FormatException or InvalidOperationException) { unreadable++; }
        }
        string faults = Path.Combine(root, "faults");
        if (includeFaults && System.IO.Directory.Exists(faults))
            foreach (string directory in System.IO.Directory.EnumerateDirectories(faults))
            {
                if (!File.Exists(Path.Combine(directory, "fault.json")) || File.Exists(Path.Combine(directory, "metadata.json"))) continue;
                try { runs.Add(ReadFaultRun(directory)); }
                catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException or FormatException or InvalidOperationException) { unreadable++; }
            }
        runs.Sort((a, b) => b.StartedUtc.CompareTo(a.StartedUtc));
        if (runs.Count > ListLimit) runs.RemoveRange(ListLimit, runs.Count - ListLimit);
        return (runs, unreadable);
    }

    // A stop bundle with no recording: when the host saved it, and how many telemetry samples came with it.
    private static ExperimentRun ReadFaultRun(string directory)
    {
        using var document = JsonDocument.Parse(File.ReadAllText(Path.Combine(directory, "fault.json")));
        var root = document.RootElement;
        if (!root.TryGetProperty("host_utc", out var at) || !at.TryGetDateTime(out var savedUtc)) throw new FormatException("host_utc");
        return new ExperimentRun { Directory = directory, Kind = "fault", StartedUtc = savedUtc.ToUniversalTime(), Outcome = "fault", HasFault = true,
            Samples = root.TryGetProperty("pre_fault_samples", out var n) && n.ValueKind == JsonValueKind.Number ? n.GetInt32() : 0 };
    }

    private static ExperimentRun ReadRun(string directory)
    {
        using var meta = JsonDocument.Parse(File.ReadAllText(Path.Combine(directory, "metadata.json")));
        var root = meta.RootElement;
        string Text(JsonElement e, string name) => e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() ?? "" : "";
        double Number(JsonElement e, string name, double fallback) => e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.Number ? v.GetDouble() : fallback;
        string outcome = "interrupted"; int samples = 0, motion = 0; double? rmse = null, peak = null, rmse2 = null, peak2 = null;
        string summaryPath = Path.Combine(directory, "summary.json");
        if (File.Exists(summaryPath))
        {
            using var summary = JsonDocument.Parse(File.ReadAllText(summaryPath));
            var s = summary.RootElement;
            outcome = Text(s, "outcome") is { Length: > 0 } o ? o : "interrupted";
            samples = (int)Number(s, "samples", 0); motion = (int)Number(s, "motion_samples", 0);
            if (s.TryGetProperty("q1_rmse_deg", out var r) && r.ValueKind == JsonValueKind.Number) rmse = r.GetDouble();
            if (s.TryGetProperty("q1_peak_abs_error_deg", out var p) && p.ValueKind == JsonValueKind.Number) peak = p.GetDouble();
            if (s.TryGetProperty("q2_rmse_deg", out var r2) && r2.ValueKind == JsonValueKind.Number) rmse2 = r2.GetDouble();
            if (s.TryGetProperty("q2_peak_abs_error_deg", out var p2) && p2.ValueKind == JsonValueKind.Number) peak2 = p2.GetDouble();
        }
        if (!root.TryGetProperty("started_utc", out var started) || !started.TryGetDateTime(out var startedUtc)) throw new FormatException("started_utc");
        return new ExperimentRun {
            Directory = directory, Kind = Text(root, "kind"), StartedUtc = startedUtc.ToUniversalTime(), Sequence = (uint)Number(root, "sequence", 0), Reference = Text(root, "reference"),
            Outcome = outcome, Samples = samples, MotionSamples = motion, Rmse = rmse, Peak = peak, Rmse2 = rmse2, Peak2 = peak2, TelemetryHz = (int)Number(root, "telemetry_hz", 0),
            Settings = RunSettings.Parse(Text(root, "settings")), SettingsText = Text(root, "settings"), L1 = Number(root, "L1", 180), L2 = Number(root, "L2", 176.38),
            HasFault = File.Exists(Path.Combine(directory, "fault.json")), CalibrationId = Text(root, "calibration_id") };
    }

    // The table as a CSV (numbers in invariant format), with how well each drawing's plan lines up with what was commanded: the
    // thing to check before trusting its event markers and per-stroke numbers.
    public static string ReportCsv(IEnumerable<ExperimentRun> runs, bool analyse = true)
    {
        static string N(double? v) => v is { } x ? x.ToString("0.######", CultureInfo.InvariantCulture) : "";
        var csv = new System.Text.StringBuilder("id,started_utc,kind,shape_index,branch,v1_deg_s,v2_deg_s,a1_deg_s2,a2_deg_s2,reference,outcome,q1_rmse_deg,q1_peak_deg,q2_rmse_deg,q2_peak_deg,telemetry_hz,samples,motion_samples,has_fault,calibration_id,aligned,align_offset_ms,align_residual_deg,limiting_axes\r\n");
        foreach (var run in runs)
        {
            var s = run.Settings; string aligned = "", offset = "", residual = "", limiting = "";
            if (analyse)
                try { var a = RunAnalysis.Create(run, Load(run)); aligned = a.Aligned ? "1" : "0"; offset = a.Aligned ? N(a.AlignmentOffsetMs) : ""; residual = a.Aligned ? N(a.AlignmentResidualDeg) : ""; limiting = string.Join("+", a.LimitingAxes); }
                catch (Exception ex) when (ex is IOException or FormatException or UnauthorizedAccessException) { aligned = "error"; }
            csv.Append(string.Join(",", run.Id, run.StartedUtc.ToString("O", CultureInfo.InvariantCulture), run.Kind, s?.Shape.ToString() ?? "", s?.Branch.ToString() ?? "", N(s?.V1), N(s?.V2), N(s?.A1), N(s?.A2),
                run.Reference.Replace(',', ';'), run.Outcome.Replace(',', ';').Replace('\r', ' ').Replace('\n', ' '), N(run.Rmse), N(run.Peak), N(run.Rmse2), N(run.Peak2), run.TelemetryHz, run.Samples, run.MotionSamples, run.HasFault ? 1 : 0, run.CalibrationId, aligned, offset, residual, limiting)).Append("\r\n");
        }
        return csv.ToString();
    }

    // The stop snapshot saved next to a recording, when there is one: where on the device clock it happened, and what it said.
    public static (double DeviceMs, string Phase, string Reason, bool Driver)? ReadFault(ExperimentRun run)
    {
        string path = Path.Combine(run.Directory, "fault.json");
        if (!File.Exists(path)) return null;
        try
        {
            using var document = JsonDocument.Parse(File.ReadAllText(path));
            var snapshot = document.RootElement.GetProperty("snapshot");
            return (snapshot.GetProperty("Milliseconds").GetDouble(), snapshot.GetProperty("Phase").GetString() ?? "", snapshot.GetProperty("Reason").GetString() ?? "",
                (snapshot.GetProperty("Driver").GetString() ?? "ok") != "ok");
        }
        catch (Exception ex) when (ex is IOException or JsonException or KeyNotFoundException or InvalidOperationException) { return null; }
    }

    private static string[] SplitCsv(string line) => line.TrimStart('﻿').Split(',');

    public static RunData Load(ExperimentRun run)
    {
        var plan = new List<PlanNode>(); int skipped = 0;
        string planPath = Path.Combine(run.Directory, "plan.csv");
        if (File.Exists(planPath))
            foreach (string line in File.ReadLines(planPath).Skip(1))
            {
                var f = SplitCsv(line);
                if (f.Length >= 4 && int.TryParse(f[0], out int index) && int.TryParse(f[1], out int ms) &&
                    double.TryParse(f[2], NumberStyles.Float, CultureInfo.InvariantCulture, out double q1) && double.TryParse(f[3], NumberStyles.Float, CultureInfo.InvariantCulture, out double q2))
                    plan.Add(new PlanNode(index, ms, q1, q2));
                else if (line.Length > 0) skipped++;
            }
        var rows = new List<FeedbackRow>();
        string feedbackPath = Path.Combine(run.Directory, "feedback.csv");
        if (File.Exists(feedbackPath))
        {
            using var reader = new StreamReader(feedbackPath);
            string? header = reader.ReadLine();
            if (header is not null)
            {
                var names = SplitCsv(header); var at = new Dictionary<string, int>();
                for (int i = 0; i < names.Length; i++) at[names[i]] = i;
                string[] need = ["host_utc", "device_ms", "epoch", "seq", "phase", "segment", "q1_command_deg", "q1_encoder_deg", "q1_error_deg", "q2_command_deg",
                    "x_command_model_mm", "y_command_model_mm", "x_hybrid_estimate_mm", "y_hybrid_estimate_mm", "slip_motor_lsb", "inputs_active_mask", "rx_drops"];
                if (need.Any(n => !at.ContainsKey(n))) throw new FormatException("feedback.csv header");
                string? line;
                while ((line = reader.ReadLine()) is not null)
                {
                    var f = line.Split(',');
                    double D(string name) => double.Parse(f[at[name]], NumberStyles.Float, CultureInfo.InvariantCulture);
                    try
                    {
                        rows.Add(new FeedbackRow(DateTime.Parse(f[at["host_utc"]], CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind).ToUniversalTime(), uint.Parse(f[at["device_ms"]]), uint.Parse(f[at["epoch"]]), uint.Parse(f[at["seq"]]),
                            f[at["phase"]], int.Parse(f[at["segment"]]), D("q1_command_deg"), D("q1_encoder_deg"), D("q1_error_deg"), D("q2_command_deg"), D("x_command_model_mm"), D("y_command_model_mm"),
                            D("x_hybrid_estimate_mm"), D("y_hybrid_estimate_mm"), int.Parse(f[at["slip_motor_lsb"]]), int.Parse(f[at["inputs_active_mask"]]), uint.Parse(f[at["rx_drops"]]), at.ContainsKey("q2_measured_deg") && double.TryParse(f[at["q2_measured_deg"]], NumberStyles.Float, CultureInfo.InvariantCulture, out var measured2) ? measured2 : null));
                    }
                    catch (Exception ex) when (ex is FormatException or IndexOutOfRangeException or OverflowException or KeyNotFoundException) { if (line.Length > 0) skipped++; }
                }
            }
        }
        return new RunData(plan.ToArray(), rows.ToArray(), skipped, LoadPreFault(run));
    }

    // pre_fault.csv: the telemetry of the seconds before a stop, saved next to fault.json. Unreadable rows are dropped, not fatal.
    private static PreFaultRow[] LoadPreFault(ExperimentRun run)
    {
        string path = Path.Combine(run.Directory, "pre_fault.csv");
        if (!File.Exists(path)) return [];
        var lines = File.ReadAllLines(path);
        if (lines.Length < 2) return [];
        var names = SplitCsv(lines[0]); var at = new Dictionary<string, int>();
        for (int i = 0; i < names.Length; i++) at[names[i]] = i;
        string[] need = ["device_ms", "q1_command_deg", "q1_encoder_deg", "q1_error_deg"];
        if (need.Any(n => !at.ContainsKey(n))) return [];
        var rows = new List<PreFaultRow>();
        foreach (string line in lines.Skip(1))
        {
            var f = line.Split(',');
            bool Get(string name, out double v) { v = 0; return at[name] < f.Length && double.TryParse(f[at[name]], NumberStyles.Float, CultureInfo.InvariantCulture, out v); }
            var host = at.TryGetValue("host_utc", out int h) && h < f.Length && DateTime.TryParse(f[h], CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind, out var parsed) ? parsed.ToUniversalTime() : default;
            if (Get("device_ms", out double ms) && Get("q1_command_deg", out double c) && Get("q1_encoder_deg", out double e) && Get("q1_error_deg", out double err)) rows.Add(new PreFaultRow(ms, c, e, err, host));
        }
        return rows.OrderBy(r => r.DeviceMs).ToArray();
    }
}

public sealed record RunEvent(double Time, string Kind);                                  // Kind: start, dwell, stop
public sealed record SegmentStat(int Index, double Start, double End, int Samples, double Rmse, double Peak, double? Rmse2 = null);
public sealed record AxisUse(string Name, double Peak, double Cap, string Unit)
{
    public double Utilization => Cap > 0 ? Peak / Cap : 0;
}

// Everything the curves and the replay need, derived once from a recording. Time zero is the start of the plan (drawings)
// or the first moving sample (a single point). Nothing here is a measurement of the pen; motor encoders do not see gearbox backlash. Old runs may have no arm2 measurement.
public sealed class RunAnalysis
{
    public required ExperimentRun Run { get; init; }
    public required RunData Data { get; init; }
    public PlanarArm Model { get; init; } = new(180, 176.38);
    // Motion samples (phase point / drawing), time-aligned.
    public double[] T { get; init; } = [];
    public DateTime[] HostUtc { get; init; } = [];                    // host clock of each motion sample, to find the matching log lines
    public double[] Q1Command { get; init; } = [];
    public double[] Q1Encoder { get; init; } = [];
    public double[] Q1Error { get; init; } = [];
    public double[] Q2Command { get; init; } = [];
    public double[] Q2Measured { get; init; } = [];
    public double[] Q2Error { get; init; } = [];
    public double[] XCommand { get; init; } = [];
    public double[] YCommand { get; init; } = [];
    public double[] XHybrid { get; init; } = [];
    public double[] YHybrid { get; init; } = [];
    public int[] Segment { get; init; } = [];
    // The plan as nodes (time in seconds), and the speed/acceleration it asks of each motor. Motor 2 turns q2 - q1.
    public double[] PlanT { get; init; } = [];
    public double[] PlanQ1 { get; init; } = [];
    public double[] PlanQ2 { get; init; } = [];
    public double[] PlanX { get; init; } = [];
    public double[] PlanY { get; init; } = [];
    public double[] PlanVT { get; init; } = [];                    // mid-time of each plan segment
    public double[] PlanV1 { get; init; } = [];
    public double[] PlanV2 { get; init; } = [];
    public double[] PlanAT { get; init; } = [];                    // node times with a defined acceleration
    public double[] PlanA1 { get; init; } = [];
    public double[] PlanA2 { get; init; } = [];
    public RunEvent[] Events { get; init; } = [];
    public SegmentStat[] Segments { get; init; } = [];
    // The seconds before a stop (pre_fault.csv) that the recording itself does not hold: before its first moving sample or after its last. Same time axis as T.
    public double[] PreT { get; init; } = [];
    public double[] PreQ1Command { get; init; } = [];
    public DateTime[] PreHostUtc { get; init; } = [];
    public double[] PreQ1Encoder { get; init; } = [];
    public double[] PreQ1Error { get; init; } = [];
    public double FaultTime { get; init; } = double.NaN;            // the stop snapshot on the same axis as T (NaN = no snapshot)
    public AxisUse[] Axes { get; init; } = [];                      // peak speed and acceleration of each motor against its cap
    public string[] LimitingAxes { get; init; } = [];
    public double Rmse { get; init; }
    public double Peak { get; init; }
    public double? Rmse2 { get; init; }                             // arm2 error (its own encoder minus the command); null when the run has no arm2 measurement
    public double? Peak2 { get; init; }
    public double Duration { get; init; }
    public double AlignmentOffsetMs { get; init; }                  // device clock at plan time zero
    public double AlignmentResidualDeg { get; init; }               // how well the plan matches the commanded motion after aligning; large = do not trust the markers
    public bool Aligned { get; init; }
    public bool HasPlan => PlanT.Length >= 2;
    public bool IsDrawing => Run.Kind == "trajectory";

    private static double Lerp(double[] xs, double[] ys, double x)
    {
        if (xs.Length == 0) return double.NaN;
        if (x <= xs[0]) return ys[0];
        if (x >= xs[^1]) return ys[^1];
        int i = Array.BinarySearch(xs, x); if (i >= 0) return ys[i];
        i = ~i; double u = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
        return ys[i - 1] + (ys[i] - ys[i - 1]) * u;
    }
    public double At(double[] series, double time) => Lerp(T, series, time);
    public double PlanAt(double[] series, double time) => Lerp(PlanT, series, time);

    public static RunAnalysis Create(ExperimentRun run, RunData data)
    {
        PlanarArm model;
        try { model = new PlanarArm(run.L1, run.L2); } catch (ArgumentException) { model = new PlanarArm(180, 176.38); }
        bool drawing = run.Kind == "trajectory";
        var motion = data.Feedback.Where(r => r.Phase is "drawing" or "point").OrderBy(r => r.DeviceMs).ToArray();
        // Plan, in seconds.
        double[] planT = data.Plan.Select(n => n.TimeMs / 1000.0).ToArray(), planQ1 = data.Plan.Select(n => n.Q1).ToArray(), planQ2 = data.Plan.Select(n => n.Q2).ToArray();
        double[] planE = planQ2.Zip(planQ1, (q2, q1) => q2 - q1).ToArray();
        // Align the plan to the device clock: the commanded motion IS the plan, shifted. A coarse start (the first sample of the run)
        // is refined by trying shifts and keeping the one where the commanded q1 and elbow best follow the plan.
        double offsetMs = motion.Length > 0 ? motion[0].DeviceMs : 0, residual = 0; bool aligned = false;
        if (drawing && planT.Length >= 2 && motion.Length >= 3)
        {
            double coarse = motion[0].DeviceMs / 1000.0, best = double.MaxValue, bestOffset = coarse;
            for (double o = coarse - 2.0; o <= coarse + 1.0; o += 0.005)
            {
                double cost = 0;
                foreach (var r in motion)
                {
                    double t = r.DeviceMs / 1000.0 - o;
                    double d1 = r.Q1Command - Lerp(planT, planQ1, t), d2 = (r.Q2Command - r.Q1Command) - Lerp(planT, planE, t);
                    cost += d1 * d1 + d2 * d2;
                }
                if (cost < best) { best = cost; bestOffset = o; }
            }
            offsetMs = bestOffset * 1000; residual = Math.Sqrt(best / motion.Length); aligned = true;
        }
        var pre = (data.PreFault ?? []).Where(r => motion.Length == 0 || r.DeviceMs < motion[0].DeviceMs - 1 || r.DeviceMs > motion[^1].DeviceMs + 1).ToArray();
        if (motion.Length == 0 && pre.Length > 0) offsetMs = pre[0].DeviceMs;
        double[] T = motion.Select(r => (r.DeviceMs - offsetMs) / 1000.0).ToArray();
        // Plan profile: speed of each motor over each segment, acceleration at each node between two segments.
        var vt = new List<double>(); var v1 = new List<double>(); var v2 = new List<double>(); var at = new List<double>(); var a1 = new List<double>(); var a2 = new List<double>();
        var segmentDt = new List<double>();
        for (int i = 1; i < planT.Length; i++)
        {
            double dt = planT[i] - planT[i - 1]; if (dt <= 0) continue;
            vt.Add((planT[i] + planT[i - 1]) / 2); v1.Add((planQ1[i] - planQ1[i - 1]) / dt); v2.Add((planE[i] - planE[i - 1]) / dt); segmentDt.Add(dt);
        }
        for (int i = 1; i < vt.Count; i++)
        {
            double dt = (segmentDt[i] + segmentDt[i - 1]) / 2;
            at.Add((vt[i] + vt[i - 1]) / 2); a1.Add((v1[i] - v1[i - 1]) / dt); a2.Add((v2[i] - v2[i - 1]) / dt);
        }
        var xy = planQ1.Zip(planQ2, (q1, q2) => model.Forward(new JointPose(q1, q2)).Tip).ToArray();
        // Events: the plan's own dwell nodes (a rectangle corner, the end of a stroke) and its last node.
        var events = new List<RunEvent>();
        if (planT.Length >= 2)
        {
            events.Add(new RunEvent(0, "start"));
            // The planner holds a still node of the minimum duration at the end of every stroke (a rectangle corner); the
            // last node of the plan is the stop itself.
            for (int i = 1; i < planT.Length - 1; i++)
            {
                bool still = Math.Abs(planQ1[i] - planQ1[i - 1]) < 0.0051 && Math.Abs(planE[i] - planE[i - 1]) < 0.0051 && Math.Abs((planT[i] - planT[i - 1]) * 1000 - DeviceContract.TrajNodeMinMs) <= 1;
                if (still) events.Add(new RunEvent(planT[i - 1], "dwell"));
            }
            events.Add(new RunEvent(planT[^1], "stop"));
        }
        // Error statistics over the moving part, per stroke between events.
        double sum = 0, peak = 0; int moving = 0;
        for (int i = 0; i < motion.Length; i++) { sum += motion[i].Q1Error * motion[i].Q1Error; peak = Math.Max(peak, Math.Abs(motion[i].Q1Error)); moving++; }
        double[] q2Error = motion.Length > 0 && motion.All(r => r.Q2Measured.HasValue) ? motion.Select(r => r.Q2Measured!.Value - r.Q2Command).ToArray() : [];
        var segments = new List<SegmentStat>();
        if (aligned && events.Count >= 2)
            for (int k = 0; k + 1 < events.Count; k++)
            {
                var inside = Enumerable.Range(0, motion.Length).Where(i => T[i] >= events[k].Time && T[i] < events[k + 1].Time).ToArray();
                if (inside.Length == 0) { segments.Add(new SegmentStat(k + 1, events[k].Time, events[k + 1].Time, 0, 0, 0)); continue; }
                segments.Add(new SegmentStat(k + 1, events[k].Time, events[k + 1].Time, inside.Length,
                    Math.Sqrt(inside.Average(i => motion[i].Q1Error * motion[i].Q1Error)), inside.Max(i => Math.Abs(motion[i].Q1Error)),
                    q2Error.Length > 0 ? Math.Sqrt(inside.Average(i => q2Error[i] * q2Error[i])) : null));
            }
        // Which motor limits the drawing speed: the caps are the ones the plan was built with.
        AxisUse[] axes = [];
        if (run.Settings is { } s && v1.Count > 0)
        {
            PlanningRates caps;
            try { caps = MotionProfile.From(new PlanningRates(1000, s.V1, s.V2, s.A1, s.A2)).DrawingRates(1000); } catch (ArgumentException) { caps = new PlanningRates(1000, s.V1, s.V2, s.A1, s.A2); }
            axes = [new AxisUse("v1", v1.Max(Math.Abs), caps.V1, "°/s"), new AxisUse("v2", v2.Max(Math.Abs), caps.V2, "°/s"),
                    new AxisUse("a1", a1.Count > 0 ? a1.Max(Math.Abs) : 0, caps.A1, "°/s²"), new AxisUse("a2", a2.Count > 0 ? a2.Max(Math.Abs) : 0, caps.A2, "°/s²")];
        }
        return new RunAnalysis {
            Run = run, Data = data, Model = model, T = T, HostUtc = motion.Select(r => r.HostUtc).ToArray(),
            Q1Command = motion.Select(r => r.Q1Command).ToArray(), Q1Encoder = motion.Select(r => r.Q1Encoder).ToArray(), Q1Error = motion.Select(r => r.Q1Error).ToArray(),
            Q2Command = motion.Select(r => r.Q2Command).ToArray(),
            Q2Measured = motion.All(r=>r.Q2Measured.HasValue) ? motion.Select(r=>r.Q2Measured!.Value).ToArray() : [],
            Q2Error = q2Error, Rmse2 = q2Error.Length > 0 ? Math.Sqrt(q2Error.Average(e => e * e)) : null, Peak2 = q2Error.Length > 0 ? q2Error.Max(Math.Abs) : null, XCommand = motion.Select(r => r.XCommand).ToArray(), YCommand = motion.Select(r => r.YCommand).ToArray(),
            XHybrid = motion.Select(r => r.XHybrid).ToArray(), YHybrid = motion.Select(r => r.YHybrid).ToArray(), Segment = motion.Select(r => r.Segment).ToArray(),
            PlanT = planT, PlanQ1 = planQ1, PlanQ2 = planQ2, PlanX = xy.Select(p => p.X).ToArray(), PlanY = xy.Select(p => p.Y).ToArray(),
            PlanVT = vt.ToArray(), PlanV1 = v1.ToArray(), PlanV2 = v2.ToArray(), PlanAT = at.ToArray(), PlanA1 = a1.ToArray(), PlanA2 = a2.ToArray(),
            Events = events.ToArray(), Segments = segments.ToArray(), Axes = axes, LimitingAxes = axes.Where(a => a.Utilization >= 0.9).Select(a => a.Name).ToArray(),
            Rmse = moving > 0 ? Math.Sqrt(sum / moving) : 0, Peak = peak, Duration = planT.Length > 0 ? planT[^1] : T.Length > 0 ? T[^1] - T[0] : 0,
            PreT = pre.Select(r => (r.DeviceMs - offsetMs) / 1000.0).ToArray(), PreQ1Command = pre.Select(r => r.Q1Command).ToArray(), PreHostUtc = pre.Select(r => r.HostUtc).ToArray(), PreQ1Encoder = pre.Select(r => r.Q1Encoder).ToArray(), PreQ1Error = pre.Select(r => r.Q1Error).ToArray(),
            FaultTime = ExperimentArchive.ReadFault(run) is { } fault ? (fault.DeviceMs - offsetMs) / 1000.0 : double.NaN,
            AlignmentOffsetMs = offsetMs, AlignmentResidualDeg = residual, Aligned = aligned };
    }
}
