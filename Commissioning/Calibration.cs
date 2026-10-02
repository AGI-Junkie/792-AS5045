using System.Globalization;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows;

namespace TwoLinkCommissioning;

// A calibration is a record of MEASUREMENTS, dated and versioned, written once and never edited. It does not change the machine:
// nothing here is applied automatically, and how to compensate (if at all) is decided from the numbers afterwards.
public sealed record LinkMeasurement(double L1, double L2, string Method, double UncertaintyMm);
public sealed record ZeroMeasurement(string ReferenceMethod, double Q1OffsetDeg, double Q2OffsetDeg, double? TipX, double? TipY, string Notes);
public sealed record StrokeMeasurement(double? ElbowMin, double? ElbowMax, double? Q1Min, double? Q1Max);
// One arrival at the target: where the pen measured (any consistent frame, mm), and what the device reported at that moment.
public sealed record Trial(string Direction, int Index, double? X, double? Y, double? Q1Command, double? Q1Encoder, double? Q2Command);
public sealed record RepeatabilityMeasurement(double TargetQ1, double TargetQ2, double ApproachDeg, List<Trial> Trials);
public sealed record CalibrationRecord(int Schema, string Id, int Version, DateTime CreatedUtc, string Firmware, string Note,
    LinkMeasurement? Links, ZeroMeasurement? Zero, StrokeMeasurement? Stroke, RepeatabilityMeasurement? Repeatability);

public sealed record DirectionStats(int Count, double MeanX, double MeanY, double StdX, double StdY, double Spread);
// Hysteresis is how far apart the two approach directions land, along the direction of travel; Repeatability is the worse one-way spread.
public sealed record RepeatabilityStats(DirectionStats? Plus, DirectionStats? Minus, double? Hysteresis, double? Repeatability, int Used, int Missing)
{
    private static DirectionStats? Of(IReadOnlyList<Trial> trials)
    {
        var points = trials.Where(t => t.X is { } x && t.Y is { } y && double.IsFinite(x) && double.IsFinite(y)).Select(t => (X: t.X!.Value, Y: t.Y!.Value)).ToArray();
        if (points.Length == 0) return null;
        double mx = points.Average(p => p.X), my = points.Average(p => p.Y);
        double Std(IEnumerable<double> v, double mean) => points.Length < 2 ? 0 : Math.Sqrt(v.Sum(x => (x - mean) * (x - mean)) / (points.Length - 1));
        return new(points.Length, mx, my, Std(points.Select(p => p.X), mx), Std(points.Select(p => p.Y), my), points.Max(p => Math.Sqrt((p.X - mx) * (p.X - mx) + (p.Y - my) * (p.Y - my))));
    }
    // `travel` is the unit vector of the plus-side approach (from its pre-point to the target) in the same frame as the measured X, Y.
    public static RepeatabilityStats Compute(IReadOnlyList<Trial> trials, Vector travel)
    {
        var plus = Of(trials.Where(t => t.Direction == "+").ToArray()); var minus = Of(trials.Where(t => t.Direction == "-").ToArray());
        double? hysteresis = plus is not null && minus is not null && travel.Length > 1e-9
            ? Math.Abs(((plus.MeanX - minus.MeanX) * travel.X + (plus.MeanY - minus.MeanY) * travel.Y) / travel.Length) : null;
        double? repeat = plus is null && minus is null ? null : Math.Max(plus?.Spread ?? 0, minus?.Spread ?? 0);
        return new(plus, minus, hysteresis, repeat, trials.Count(t => t.X is not null && t.Y is not null), trials.Count(t => t.X is null || t.Y is null));
    }
}

public static class Calibrations
{
    public static string Root { get; set; } = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "Calibrations");
    private static readonly JsonSerializerOptions Json = new() { WriteIndented = true };
    private const string ActiveFile = "active.txt";

    // The next version number is one more than the highest on disk; a saved calibration is never overwritten.
    public static int NextVersion() => List().Select(c => c.Version).DefaultIfEmpty(0).Max() + 1;

    public static string Validate(CalibrationRecord record)
    {
        static bool Ok(double v) => double.IsFinite(v);
        if (record.Links is { } l && (!Ok(l.L1) || !Ok(l.L2) || l.L1 is <= 0 or > 10000 || l.L2 is <= 0 or > 10000 || !Ok(l.UncertaintyMm) || l.UncertaintyMm < 0 || l.UncertaintyMm > 100)) return "links";
        if (record.Zero is { } z && (!Ok(z.Q1OffsetDeg) || !Ok(z.Q2OffsetDeg) || Math.Abs(z.Q1OffsetDeg) > 180 || Math.Abs(z.Q2OffsetDeg) > 180 || z.TipX is { } x && !Ok(x) || z.TipY is { } y && !Ok(y))) return "zero";
        if (record.Stroke is { } s && (s.ElbowMin is { } e0 && s.ElbowMax is { } e1 && e0 >= e1 || s.Q1Min is { } a0 && s.Q1Max is { } a1 && a0 >= a1 ||
            new[] { s.ElbowMin, s.ElbowMax, s.Q1Min, s.Q1Max }.Any(v => v is { } d && (!Ok(d) || Math.Abs(d) > 360)))) return "stroke";
        if (record.Repeatability is { } r && (!Ok(r.TargetQ1) || !Ok(r.TargetQ2) || !Ok(r.ApproachDeg) || r.ApproachDeg <= 0 || r.ApproachDeg > 90 || r.Trials.Count > 200 ||
            r.Trials.Any(t => t.Direction is not ("+" or "-") || t.X is { } tx && !Ok(tx) || t.Y is { } ty && !Ok(ty)))) return "repeatability";
        return record.Links is null && record.Zero is null && record.Stroke is null && record.Repeatability is null ? "empty" : "";
    }

    // Writes cal-<local date time>-v<version>.json and makes it the current one. Returns the record as saved.
    public static CalibrationRecord Save(CalibrationRecord draft, string firmware, DateTime? now = null)
    {
        string problem = Validate(draft);
        if (problem.Length > 0) throw Lang.Error("cal.err." + problem);
        Directory.CreateDirectory(Root);
        int version = NextVersion(); var created = (now ?? DateTime.UtcNow).ToUniversalTime();
        string id = $"cal-{created.ToLocalTime():yyyyMMdd-HHmmss}-v{version}";
        var record = draft with { Schema = 1, Id = id, Version = version, CreatedUtc = created, Firmware = firmware };
        string path = Path.Combine(Root, id + ".json");
        if (File.Exists(path)) throw new IOException("calibration already exists: " + id);
        File.WriteAllText(path + ".tmp", JsonSerializer.Serialize(record, Json), new UTF8Encoding(false));
        File.Move(path + ".tmp", path);
        SetActive(id);
        return record;
    }

    public static List<CalibrationRecord> List()
    {
        var list = new List<CalibrationRecord>();
        if (!Directory.Exists(Root)) return list;
        foreach (string file in Directory.EnumerateFiles(Root, "cal-*.json"))
        {
            try
            {
                if (new FileInfo(file).Length > 2_000_000) continue;
                var record = JsonSerializer.Deserialize<CalibrationRecord>(File.ReadAllText(file));
                if (record is { Schema: 1 } && record.Id == Path.GetFileNameWithoutExtension(file) && record.Version > 0 && Validate(record) is "") list.Add(record);
            }
            catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException) { }
        }
        list.Sort((a, b) => b.Version.CompareTo(a.Version));
        return list;
    }

    // The calibration new experiments are tagged with: the last one chosen, else the newest; none if nothing was ever saved.
    public static CalibrationRecord? Active()
    {
        var all = List(); if (all.Count == 0) return null;
        try { if (File.Exists(Path.Combine(Root, ActiveFile)) && File.ReadAllText(Path.Combine(Root, ActiveFile)).Trim() is { Length: > 0 } id && all.FirstOrDefault(c => c.Id == id) is { } chosen) return chosen; }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { }
        return all[0];
    }
    public static void SetActive(string id) { Directory.CreateDirectory(Root); File.WriteAllText(Path.Combine(Root, ActiveFile), id, new UTF8Encoding(false)); }

    public static void SelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Calibration: " + reason); }
        string temp = Path.Combine(Path.GetTempPath(), "arm-cal-" + Guid.NewGuid().ToString("N")), previous = Root;
        Root = Path.Combine(temp, "Calibrations");
        try
        {
            Check(Active() is null && List().Count == 0 && NextVersion() == 1, "nothing saved yet");
            // Repeatability: two groups of arrivals with known scatter and a known gap between them (0.30 mm along the travel direction).
            var plus = new[] { (0.10, 0.02), (-0.10, -0.02), (0.00, 0.00), (0.05, 0.01), (-0.05, -0.01) };
            var trials = new List<Trial>();
            for (int i = 0; i < plus.Length; i++)
            {
                trials.Add(new Trial("+", i + 1, 340 + 0.15 + plus[i].Item1, 0 + plus[i].Item2, 5, 5.001, 7));
                trials.Add(new Trial("-", i + 1, 340 - 0.15 + plus[i].Item1 * 0.5, 0 + plus[i].Item2 * 0.5, 5, 5.002, 7));
            }
            var stats = RepeatabilityStats.Compute(trials, new Vector(-1, 0));   // plus-side arrival travels towards -x
            Check(stats.Plus is { Count: 5 } && stats.Minus is { Count: 5 } && Math.Abs(stats.Plus.MeanX - 340.15) < 1e-9 && Math.Abs(stats.Minus.MeanX - 339.85) < 1e-9, "group means");
            Check(Math.Abs(stats.Hysteresis!.Value - 0.30) < 1e-9, "the gap between the two approach directions, along the travel: " + stats.Hysteresis);
            Check(Math.Abs(stats.Plus!.StdX - Math.Sqrt((0.01 + 0.01 + 0 + 0.0025 + 0.0025) / 4)) < 1e-9 && Math.Abs(stats.Minus!.StdX - stats.Plus.StdX / 2) < 1e-9, "sample standard deviation");
            Check(stats.Plus.Spread > stats.Minus!.Spread && Math.Abs(stats.Repeatability!.Value - stats.Plus.Spread) < 1e-12 && stats.Used == 10 && stats.Missing == 0, "spread is the largest distance from the group's centroid; repeatability is the worse direction");
            var partial = RepeatabilityStats.Compute([new Trial("+", 1, 1, 1, null, null, null), new Trial("+", 2, null, null, null, null, null), new Trial("-", 1, 2, 2, null, null, null)], new Vector(1, 0));
            Check(partial.Used == 2 && partial.Missing == 1 && partial.Plus is { Count: 1, StdX: 0 } && partial.Hysteresis is not null && RepeatabilityStats.Compute([], new Vector(1, 0)) is { Repeatability: null, Hysteresis: null }, "unfilled trials are counted, not guessed; one point has no spread");
            Check(RepeatabilityStats.Compute(trials, new Vector(0, 0)).Hysteresis is null, "no travel direction, no hysteresis");
            // Saving: dated, versioned, never overwritten; the newest becomes active, and an older one can be chosen.
            var draft = new CalibrationRecord(0, "", 0, default, "", "first try", new LinkMeasurement(180.4, 176.1, "caliper", 0.2), new ZeroMeasurement("manual", 0.4, -0.2, 356.4, 1.1, "protractor"),
                new StrokeMeasurement(-150, 158, -120, 125), new RepeatabilityMeasurement(5, 7, 4, trials));
            var first = Save(draft, "fw-test", new DateTime(2026, 10, 2, 3, 4, 5, DateTimeKind.Utc));
            Check(first.Version == 1 && first.Id.StartsWith("cal-") && first.Id.EndsWith("-v1") && first.CreatedUtc == new DateTime(2026, 10, 2, 3, 4, 5, DateTimeKind.Utc) && first.Firmware == "fw-test", "first version");
            var second = Save(draft with { Note = "after tightening" }, "fw-test");
            Check(second.Version == 2 && second.Id != first.Id && Active()!.Id == second.Id && List().Select(c => c.Version).SequenceEqual([2, 1]), "second version is newer, active, listed first");
            SetActive(first.Id); Check(Active()!.Id == first.Id, "an older one can be made current");
            var loaded = List().First(c => c.Version == 1);
            Check(loaded.Links == first.Links && loaded.Zero == first.Zero && loaded.Stroke == first.Stroke && loaded.Repeatability!.Trials.Count == 10 && loaded.Note == "first try" && loaded.Repeatability.Trials[0].Q1Encoder == 5.001, "the file reads back what was saved");
            Check(File.ReadAllText(Directory.GetFiles(Root, "*v1.json")[0]).Contains("\"Version\": 1"), "readable JSON on disk");
            // Bad records are refused with the reason, and nothing is written.
            int files = Directory.GetFiles(Root).Length;
            foreach (var bad in new[] { draft with { Links = new LinkMeasurement(0, 176, "x", 0) }, draft with { Links = new LinkMeasurement(180, double.NaN, "x", 0) }, draft with { Zero = new ZeroMeasurement("m", 200, 0, null, null, "") },
                draft with { Stroke = new StrokeMeasurement(10, -10, null, null) }, draft with { Repeatability = new RepeatabilityMeasurement(0, 0, 0, []) }, draft with { Repeatability = new RepeatabilityMeasurement(0, 0, 4, [new Trial("x", 1, 1, 1, null, null, null)]) },
                new CalibrationRecord(0, "", 0, default, "", "", null, null, null, null) })
            {
                bool refused = false; try { Save(bad, "fw"); } catch (ArgumentException) { refused = true; }
                Check(refused, "an invalid calibration was accepted");
            }
            Check(Directory.GetFiles(Root).Length == files, "a refused calibration left nothing behind");
            // A damaged or foreign file never hides the good ones.
            File.WriteAllText(Path.Combine(Root, "cal-20000101-000000-v9.json"), "{ nope"); File.WriteAllText(Path.Combine(Root, "cal-20000101-000001-v8.json"), JsonSerializer.Serialize(loaded with { Id = "cal-other" }));
            Check(List().Count == 2 && Active()!.Id == first.Id, "damaged files and files whose id does not match their name are ignored");
            File.WriteAllText(Path.Combine(Root, ActiveFile), "cal-does-not-exist"); Check(Active()!.Id == second.Id, "an active pointer to nothing falls back to the newest");
        }
        finally { Root = previous; if (Directory.Exists(temp)) Directory.Delete(temp, true); }
    }
}
