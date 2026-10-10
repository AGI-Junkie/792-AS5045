using System.Windows;
using System.Text.Json.Serialization;

namespace TwoLinkCommissioning;

public readonly record struct FreehandPoint([property: JsonRequired] double X, [property: JsonRequired] double Y)
{
    internal Point Point => new(X, Y);
}

internal static class FreehandTrajectory
{
    internal const int MaxPoints = 5000;
    internal const double CaptureSpacing = 0.25, Tolerance = 1;
    private const double SimplifyTolerance = 0.25;
    internal static FreehandPoint[] Example => Enumerable.Range(0, 41)
        .Select(i => new FreehandPoint(330 + i * 0.25, 2 * Math.Sin(i * Math.PI / 20))).ToArray();
    internal static FreehandPoint[] RefinementExample => [new(320, 0), new(340, 0), new(340, 20)];

    internal static void Validate(IReadOnlyList<FreehandPoint> points)
    {
        if (points is null || points.Count > MaxPoints || points.Any(p => !double.IsFinite(p.X) || !double.IsFinite(p.Y) || Math.Abs(p.X) > 20000 || Math.Abs(p.Y) > 20000))
            throw Lang.Error("err.freehandPoints");
    }

    internal static double FitTolerance(int level) => level switch { 1 => 0.75, 2 => 1, 3 => 1.25, _ => Tolerance };
    private static double Simplification(int level) => level switch { 1 or 2 => 0.5, 3 => 0.75, _ => SimplifyTolerance };

    internal static PathPiece Create(IReadOnlyList<FreehandPoint> input, int level = 0)
    {
        Validate(input);
        var raw = new List<Point>();
        foreach (var p in input)
            if (raw.Count == 0 || (p.Point - raw[^1]).Length > 1e-9) raw.Add(p.Point);
        if (raw.Count < 2 || raw.Zip(raw.Skip(1)).Sum(p => (p.Second - p.First).Length) < 1)
            throw Lang.Error("err.freehandShort");
        var sources = Simplify(raw, Simplification(level));
        var points = sources.Select(i => raw[i]).ToList();
        for (int round = 0; round <= 16; round++)
        {
            var spline = new Spline(points.ToArray(), FitTolerance(level) - Simplification(level));
            var failed = Enumerable.Range(0, points.Count - 1).Where(i => !spline.Bounded(i)).ToArray();
            if (failed.Length == 0) return spline.Piece();
            if (round == 16 || points.Count + failed.Length > MaxPoints)
                throw Lang.Error("err.freehandFit", sources[failed[0]] + 1);
            // Split the certified simplified chord so refinement preserves its simplification budget.
            foreach (int i in failed.Reverse()) {
                points.Insert(i + 1, points[i] + (points[i + 1] - points[i]) * 0.5);
                sources.Insert(i + 1, sources[i]);
            }
        }
        throw Lang.Error("err.freehandFit", 1);
    }

    // Passes of a [1 2 1] smoothing filter and the margin (mm; half of it in degrees) kept from every reach/joint bound, mildest first.
    private static readonly (int Passes, double Margin)[] RepairLevels = [(2, 1), (8, 2), (32, 4), (128, 6)];
    internal const double RepairShift = 12;

    // The draft as drawn when the whole plan checks. Otherwise a smoothed copy with its out-of-reach points pulled into the
    // reachable region, the mildest one that checks. Shift: how far (mm) the plan strays from the draft; 0 when untouched.
    // ponytail: a stroke that crosses an unreachable hole is not re-routed, only pulled to the edge, so it still fails.
    internal static (FreehandPoint[] Points, double Shift) Repair(IReadOnlyList<FreehandPoint> draft, PlanarArm model, JointLimits? limits, int branch, PlanningRates rates, int level = 0)
    {
        void Verify(FreehandPoint[] points)
        {
            var path = DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, branch, limits, new ShapeOptions { FreehandPoints = points, FreehandFitLevel = level });
            var timing = new TimedTrajectory(path, rates);
            if (limits is { } range) _ = new ExecutionTrajectory(timing, range);
        }
        var original = draft.ToArray();
        try { Verify(original); return (original, 0); }
        catch (ArgumentException)
        {
            Validate(original);
            var raw = new List<Point>();
            foreach (var p in original) if (raw.Count == 0 || (p.Point - raw[^1]).Length > 1e-9) raw.Add(p.Point);
            if (raw.Count < 2 || raw.Zip(raw.Skip(1)).Sum(s => (s.Second - s.First).Length) < 1) throw;   // too short to repair: keep Create's own message
            var reference = Simplify(raw).Select(i => raw[i]).ToList();
            foreach (var (passes, margin) in RepairLevels)
            {
                var candidate = Reshape(raw, model, limits, branch, passes, margin);
                if (candidate is null) continue;
                // Both ways: the plan must stay near the draft, and every draft vertex (a retraced excursion included) near the plan.
                double Away(IEnumerable<Point> from, List<Point> to) => MaximumDistance(from.ToArray(), to.ToArray());
                double shift = Math.Max(Away(candidate.Select(p => p.Point), reference), Away(reference, candidate.Select(p => p.Point).ToList()));
                if (shift > RepairShift) continue;
                try { Verify(candidate); return (candidate, shift); } catch (ArgumentException) { }
            }
            throw;
        }
    }

    // Reserve fitting error from the 2/3/4 mm total budget; reject an over-budget filter, never clamp individual passes.
    internal static FreehandPoint[] Smooth(IReadOnlyList<FreehandPoint> input, int level)
    {
        Validate(input);
        if (level == 0 || input.Count < 2) return input.ToArray();
        if (level is < 0 or > 3) throw new ArgumentOutOfRangeException(nameof(level));
        var raw = input.Select(p => p.Point).ToList();
        if (raw.Zip(raw.Skip(1)).Sum(s => (s.Second - s.First).Length) < 1) return input.ToArray();
        // Sampling plus half the sampling spacing bounds distances everywhere on each polyline (distance is 1-Lipschitz).
        double Away(Point[] from, Point[] to) {
            double length = from.Zip(from.Skip(1)).Sum(s => (s.Second - s.First).Length);
            var samples = Resample(from.ToList(), 0.25);
            return MaximumDistance(samples, to) + length / (2 * (samples.Length - 1));
        }
        foreach (int passes in new[] { level switch { 1 => 2, 2 => 8, _ => 32 }, 1, 0 }.Distinct()) {
            var candidate = Resample(Filter(raw, passes).ToList());
            double shift = Math.Max(Away(raw.ToArray(), candidate), Away(candidate, raw.ToArray()));
            if (candidate.Zip(candidate.Skip(1)).Sum(s => (s.Second - s.First).Length) >= 1 && shift + FitTolerance(level) <= level + 1)
                return candidate.Select(p => new FreehandPoint(p.X, p.Y)).ToArray();
        }
        return input.ToArray(); // Sharp excursions must survive when no 1 mm resampling meets the budget.
    }

    private static Point[] Resample(List<Point> raw, double spacing = 1)
    {
        double total = raw.Zip(raw.Skip(1)).Sum(s => (s.Second - s.First).Length);
        int n = Math.Max(1, (int)Math.Ceiling(total / Math.Max(spacing, total / 4000)));
        var line = new Point[n + 1]; line[0] = raw[0]; line[n] = raw[^1];
        double done = 0;
        for (int k = 1, seg = 0; k < n; k++)
        {
            double target = total * k / n;
            while (seg < raw.Count - 2 && done + (raw[seg + 1] - raw[seg]).Length < target) { done += (raw[seg + 1] - raw[seg]).Length; seg++; }
            double length = (raw[seg + 1] - raw[seg]).Length;
            line[k] = raw[seg] + (raw[seg + 1] - raw[seg]) * Math.Clamp(length == 0 ? 0 : (target - done) / length, 0, 1);
        }
        return line;
    }

    private static Point[] Filter(List<Point> raw, int passes)
    {
        var line = Resample(raw); int n = line.Length - 1;
        for (int pass = 0; pass < passes; pass++)
        {
            var next = (Point[])line.Clone();
            for (int i = 1; i < n; i++) next[i] = new Point((line[i - 1].X + 2 * line[i].X + line[i + 1].X) / 4, (line[i - 1].Y + 2 * line[i].Y + line[i + 1].Y) / 4);
            line = next;
        }
        return line;
    }

    private static FreehandPoint[]? Reshape(List<Point> raw, PlanarArm model, JointLimits? limits, int branch, int passes, double margin)
    {
        var line = Filter(raw, passes); int n = line.Length - 1;
        double inner = model.InnerRadius + margin, outer = model.OuterRadius - margin, degrees = margin / 2;
        JointLimits? inside = limits is { } l ? new(l.Min1 + degrees, l.Max1 - degrees, l.Min2 + degrees, l.Max2 - degrees, l.ElbowMin + degrees, l.ElbowMax - degrees) : null;
        bool Feasible(Point p)
        {
            double radius = ((Vector)p).Length;
            if (radius < inner || radius > outer) return false;
            var poses = model.Inverse(p.X, p.Y);
            return inside is not { } range || range.Contains(poses[Math.Min(branch, poses.Length - 1)]);
        }
        for (int i = 0; i <= n; i++)
        {
            if (Feasible(line[i])) continue;
            // Nearest reachable point: grow a ring around it, take the ring point closest to the previous one, then walk it back to the edge.
            Point? hit = null;
            for (double d = 0.5; d <= RepairShift && hit is null; d += 0.5)
            {
                double gap = double.MaxValue;
                for (int k = 0; k < 64; k++)
                {
                    var c = new Point(line[i].X + d * Math.Cos(Math.PI * k / 32), line[i].Y + d * Math.Sin(Math.PI * k / 32));
                    double away = i == 0 ? 0 : (c - line[i - 1]).Length;
                    if (away < gap && Feasible(c)) { gap = away; hit = c; }
                }
            }
            if (hit is not { } edge) return null;
            Point low = line[i];
            for (int k = 0; k < 8; k++) { var mid = low + (edge - low) * 0.5; if (Feasible(mid)) edge = mid; else low = mid; }
            line[i] = edge;
        }
        return line.Select(p => new FreehandPoint(p.X, p.Y)).ToArray();
    }

    private static List<int> Simplify(IReadOnlyList<Point> points, double tolerance = SimplifyTolerance)
    {
        var keep = new SortedSet<int> { 0, points.Count - 1 };
        var pending = new Stack<(int First, int Last)>();
        foreach (var pair in keep.Zip(keep.Skip(1))) pending.Push((pair.First, pair.Second));
        while (pending.TryPop(out var pair))
        {
            double worst = tolerance; int at = -1;
            for (int i = pair.First + 1; i < pair.Last; i++)
            {
                double error = Distance(points[i], points[pair.First], points[pair.Last]);
                if (error > worst) { worst = error; at = i; }
            }
            if (at < 0 && pair.Last - pair.First > 1 && (points[pair.Last] - points[pair.First]).Length < 1e-9)
                at = (pair.First + pair.Last) / 2;
            if (at < 0) continue;
            keep.Add(at); pending.Push((pair.First, at)); pending.Push((at, pair.Last));
        }
        return keep.ToList();
    }

    private static double Distance(Point p, Point a, Point b) => Math.Sqrt(DistanceSquared(p, a, b));

    private static double DistanceSquared(Point p, Point a, Point b)
    {
        var d = b - a;
        double u = d.LengthSquared == 0 ? 0 : Math.Clamp(Vector.Multiply(p - a, d) / d.LengthSquared, 0, 1);
        return (p - (a + d * u)).LengthSquared;
    }

    private static double MaximumDistance(Point[] from, Point[] to)
    {
        const int blockSize = 32;
        var bounds = new (double MinX, double MinY, double MaxX, double MaxY)[(to.Length - 2) / blockSize + 1];
        for (int block = 0; block < bounds.Length; block++) {
            var first = to[block * blockSize];
            var box = (MinX: first.X, MinY: first.Y, MaxX: first.X, MaxY: first.Y);
            for (int i = block * blockSize + 1; i <= Math.Min((block + 1) * blockSize, to.Length - 1); i++) {
                box.MinX = Math.Min(box.MinX, to[i].X); box.MaxX = Math.Max(box.MaxX, to[i].X);
                box.MinY = Math.Min(box.MinY, to[i].Y); box.MaxY = Math.Max(box.MaxY, to[i].Y);
            }
            bounds[block] = box;
        }
        double worstSquared = 0; int closestBlock = 0;
        foreach (var p in from) {
            double nearestSquared = double.PositiveInfinity;
            void Check(int block) {
                for (int i = block * blockSize + 1; i <= Math.Min((block + 1) * blockSize, to.Length - 1); i++) {
                    double squared = DistanceSquared(p, to[i - 1], to[i]);
                    if (squared < nearestSquared) { nearestSquared = squared; closestBlock = block; }
                }
            }
            int firstBlock = closestBlock; Check(firstBlock);
            for (int block = 0; block < bounds.Length; block++) {
                if (block == firstBlock) continue;
                var box = bounds[block];
                double dx = Math.Max(0, Math.Max(box.MinX - p.X, p.X - box.MaxX));
                double dy = Math.Max(0, Math.Max(box.MinY - p.Y, p.Y - box.MaxY));
                // The box distance is a lower bound for every segment it encloses.
                if (dx * dx + dy * dy < nearestSquared) Check(block);
            }
            worstSquared = Math.Max(worstSquared, nearestSquared);
        }
        return Math.Sqrt(worstSquared);
    }

    private sealed class Spline
    {
        private readonly Point[] points, c1, c2;
        private readonly double[] knots;
        private readonly double total, curveTolerance;

        internal Spline(Point[] p, double tolerance)
        {
            points = p; curveTolerance = tolerance; int n = p.Length;
            knots = new double[n]; c1 = new Point[n - 1]; c2 = new Point[n - 1];
            for (int i = 1; i < n; i++) knots[i] = knots[i - 1] + (p[i] - p[i - 1]).Length;
            total = knots[^1];
            var second = new Vector[n]; var diagonal = new double[n]; var rhs = new Vector[n];
            for (int i = 1; i < n - 1; i++)
            {
                double left = knots[i] - knots[i - 1], right = knots[i + 1] - knots[i];
                diagonal[i] = 2 * (left + right);
                rhs[i] = 6 * ((p[i + 1] - p[i]) / right - (p[i] - p[i - 1]) / left);
                if (i > 1) { double factor = left / diagonal[i - 1]; diagonal[i] -= factor * left; rhs[i] -= factor * rhs[i - 1]; }
            }
            for (int i = n - 2; i >= 1; i--)
                second[i] = (rhs[i] - (knots[i + 1] - knots[i]) * second[i + 1]) / diagonal[i];
            for (int i = 0; i < n - 1; i++)
            {
                double h = knots[i + 1] - knots[i]; var slope = (p[i + 1] - p[i]) / h;
                c1[i] = p[i] + (slope - h * (2 * second[i] + second[i + 1]) / 6) * (h / 3);
                c2[i] = p[i + 1] - (slope + h * (second[i] + 2 * second[i + 1]) / 6) * (h / 3);
            }
        }

        internal bool Bounded(int i)
        {
            // Segment distance includes rounded end caps; a bounded reversal may pass its endpoint.
            bool Inside(Point p) => Distance(p, points[i], points[i + 1]) <= curveTolerance;
            bool Bound(Point a, Point b, Point c, Point d, int depth) {
                if (!Inside(a) || !Inside(d)) return false;
                if (Inside(b) && Inside(c)) return true;
                if (depth == 12) return false;
                Point Mid(Point x, Point y) => x + (y - x) * 0.5;
                var ab = Mid(a, b); var bc = Mid(b, c); var cd = Mid(c, d);
                var abc = Mid(ab, bc); var bcd = Mid(bc, cd); var mid = Mid(abc, bcd);
                return Bound(a, ab, abc, mid, depth + 1) && Bound(mid, bcd, cd, d, depth + 1);
            }
            // Subdivision tightens the convex-hull certificate without changing the curve.
            return Bound(points[i], c1[i], c2[i], points[i + 1], 0);
        }

        private PathPoint AtSegment(int i, double t)
        {
            double u = 1 - t, factor = total / (knots[i + 1] - knots[i]);
            Vector p0 = (Vector)points[i], p1 = (Vector)c1[i], p2 = (Vector)c2[i], p3 = (Vector)points[i + 1];
            return new((Point)(p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t)),
                3 * ((p1 - p0) * (u * u) + (p2 - p1) * (2 * u * t) + (p3 - p2) * (t * t)) * factor,
                6 * ((p2 - 2 * p1 + p0) * u + (p3 - 2 * p2 + p1) * t) * (factor * factor));
        }

        private PathPoint At(double s)
        {
            double d = Math.Clamp(s, 0, 1) * total;
            int found = Array.BinarySearch(knots, d);
            int i = Math.Clamp(found >= 0 ? found : ~found - 1, 0, points.Length - 2);
            return AtSegment(i, Math.Clamp((d - knots[i]) / (knots[i + 1] - knots[i]), 0, 1));
        }

        internal PathPiece Piece()
        {
            var checks = new List<double> { 0 }; double length = 0;
            for (int i = 0; i < points.Length - 1; i++)
            {
                double bound = 3 * Math.Max((c1[i] - points[i]).Length, Math.Max((c2[i] - c1[i]).Length, (points[i + 1] - c2[i]).Length));
                int count = Math.Max(4, (int)Math.Ceiling(bound / CaptureSpacing));
                if (checks.Count + count > 20000) throw Lang.Error("err.tooManyPoints");
                double sum = 0;
                for (int k = 0; k <= count; k++)
                {
                    double t = (double)k / count;
                    if (k > 0) checks.Add((knots[i] + (knots[i + 1] - knots[i]) * t) / total);
                    sum += AtSegment(i, t).D1.Length * (k == 0 || k == count ? 0.5 : 1);
                }
                length += sum / count * (knots[i + 1] - knots[i]) / total;
            }
            return new PathPiece(At, length, checks.ToArray());
        }
    }

    internal static void SelfTest()
    {
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Freehand: " + why); }
        static void Reject(FreehandPoint[] p) {
            try { Create(p); } catch (ArgumentException) { return; }
            throw new InvalidOperationException("Freehand accepted invalid input");
        }
        Reject([]); Reject([new(0, 0), new(0, 0)]); Reject([new(0, 0), new(0.5, 0)]);
        Reject([new(double.NaN, 0), new(2, 0)]); Reject(new FreehandPoint[MaxPoints + 1]);
        foreach (int count in new[] { 2, 32, 33, 34, 65, 66 }) {
            var contour = Enumerable.Range(0, count).Select(i => new Point(i * 2, Math.Sin(i))).ToArray();
            contour[1] = contour[0]; // A zero-length segment and complete/partial distance blocks.
            var probes = Enumerable.Range(0, 83).Select(i => new Point(i * 1.7 - 8, 3 * Math.Cos(i))).Concat(contour).ToArray();
            double exhaustive = probes.Max(p => contour.Zip(contour.Skip(1)).Min(s => Distance(p, s.First, s.Second)));
            Check(Math.Abs(MaximumDistance(probes, contour) - exhaustive) < 1e-12, "distance pruning skipped the nearest segment");
        }
        var line = Create([new(0, 0), new(5, 0), new(5, 0), new(10, 0)]);
        Check((line.At(0).P - new Point(0, 0)).Length < 1e-9 && (line.At(1).P - new Point(10, 0)).Length < 1e-9, "endpoints");
        Check(Math.Abs(line.Length - 10) < 1e-8 && line.At(0.4).D2.Length < 1e-8, "line derivatives");
        var raw = Enumerable.Range(0, 81).Select(i => new FreehandPoint(i * 0.25, 3 * Math.Sin(i * 0.08))).ToArray();
        var curve = Create(raw);
        foreach (double s in curve.Checks!)
        {
            var p = curve.At(s).P;
            Check(raw.Zip(raw.Skip(1)).Min(v => Distance(p, v.First.Point, v.Second.Point)) <= Tolerance + 1e-8, "curve deviates from draft");
        }
        foreach (var p in raw)
            Check(curve.Checks!.Min(s => (p.Point - curve.At(s).P).Length) <= Tolerance + CaptureSpacing, "draft deviates from curve");
        for (int i = 1; i < curve.Checks!.Length; i++)
            Check((curve.At(curve.Checks[i]).P - curve.At(curve.Checks[i - 1]).P).Length <= CaptureSpacing + 1e-8, "check spacing");
        const double e = 1e-6;
        for (int k = 1; k < 100; k++) {
            double s = k / 100.0; var at = curve.At(s);
            Check(((curve.At(s + e).P - curve.At(s - e).P) / (2 * e) - at.D1).Length < 1e-3, "first derivative");
            Check(((curve.At(s + e).D1 - curve.At(s - e).D1) / (2 * e) - at.D2).Length < 0.01, "second derivative");
        }
        foreach (var shape in new[] {
            new[] { new FreehandPoint(0, 0), new(5, 0), new(5, 5) },
            new[] { new FreehandPoint(0, 0), new(5, 0), new(0, 0) },
            Enumerable.Range(0, 101).Select(i => new FreehandPoint(10 * Math.Sin(i * 2 * Math.PI / 100), 5 * Math.Sin(i * 4 * Math.PI / 100))).ToArray() }) {
            var smooth = Create(shape);
            Check((smooth.At(0).P - shape[0].Point).Length < 1e-8 && (smooth.At(1).P - shape[^1].Point).Length < 1e-8, "turn/return/crossing endpoints");
            foreach (double s in smooth.Checks!)
                Check(shape.Zip(shape.Skip(1)).Min(v => Distance(smooth.At(s).P, v.First.Point, v.Second.Point)) <= Tolerance + 1e-8, "turn/return/crossing deviation");
        }
        foreach (var shape in new[] {
            new[] { new FreehandPoint(0, 0), new(100, 0), new(100, 100) },
            new[] { new FreehandPoint(0, 0), new(100, 0), new(0, 0) },
            new[] { new FreehandPoint(0, 0), new(99.75, 0), new(100, 0), new(100, 0.25), new(100, 100), new(0, 100) },
            new[] { new FreehandPoint(0, 0), new(80, 0), new(0.25, 0.25), new(80, 0.5), new(0, 0.75) } }) {
            var refined = Create(shape);
            Check((refined.At(0).P - shape[0].Point).Length < 1e-8 && (refined.At(1).P - shape[^1].Point).Length < 1e-8, "refined endpoints");
            foreach (double s in refined.Checks!)
                Check(shape.Zip(shape.Skip(1)).Min(v => Distance(refined.At(s).P, v.First.Point, v.Second.Point)) <= Tolerance + 1e-8, "automatic refinement exceeds tolerance");
            foreach (var p in shape)
                Check(refined.Checks!.Min(s => (p.Point - refined.At(s).P).Length) <= Tolerance + CaptureSpacing, "refinement loses draft points");
        }
        var noisy = Enumerable.Range(0, 241).Select(i => new FreehandPoint(320 + i * 0.1, 2 * Math.Sin(i * 0.025) + (i is 0 or 240 ? 0 : 0.6 * Math.Sin(i * 1.7)))).ToArray();
        // Release-time smoothing must reduce both stored points and actual execution duration.
        var noisySmoothed = Smooth(noisy, 2);
        Check(noisySmoothed.Length < noisy.Length && noisySmoothed[0] == noisy[0] && noisySmoothed[^1] == noisy[^1], "smoothing count/endpoints");
        var model = new PlanarArm(180, 176.38); var range = new JointLimits(-90, 90, -160, 160);
        var path = DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, 0, range,
            new ShapeOptions { FreehandPoints = raw.Select(p => new FreehandPoint(320 + p.X, p.Y)).ToArray() });
        var execution = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(10, 5, 3, 10, 6)), range);
        ExecutionTrajectory.Validate(execution.Nodes, range);
        Check(path.Stops.Length == 2 && execution.Timing.At(0).Speed < 1e-8 && execution.Timing.At(execution.Timing.Duration).Speed < 1e-8, "continuous stroke and endpoint rest");
        var refinedPath = DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, 0, range,
            new ShapeOptions { FreehandPoints = RefinementExample });
        var refinedExecution = new ExecutionTrajectory(new TimedTrajectory(refinedPath, new PlanningRates(10, 5, 3, 10, 6)), range);
        ExecutionTrajectory.Validate(refinedExecution.Nodes, range);
        Check(refinedPath.Stops.Length == 2, "refinement added internal stops");
        for (int i = 0; i <= 3000; i++) {
            var state = refinedExecution.Timing.At(refinedExecution.Timing.Duration * i / 3000);
            Check(state.Speed <= 10 && Math.Abs(state.Velocity.Q1) <= 5 && Math.Abs(state.Velocity.Q2 - state.Velocity.Q1) <= 3, "refined speed limit");
            Check(Math.Abs(state.Acceleration.Q1) <= 10 && Math.Abs(state.Acceleration.Q2 - state.Acceleration.Q1) <= 6, "refined acceleration limit");
        }
        try {
            DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, 0, range,
                new ShapeOptions { FreehandPoints = [new(50, 0), new(-50, 0)] });
            throw new InvalidOperationException("Freehand accepted an unreachable interior");
        } catch (ArgumentException) { }
        // Repair: a legal draft is untouched; one overshooting the reach (356.38 mm) or jittery is made legal within RepairShift.
        var rates = new PlanningRates(10, 5, 3, 10, 6);
        double Duration(FreehandPoint[] points, int level) => new ExecutionTrajectory(new TimedTrajectory(
            DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, 0, range,
                new ShapeOptions { FreehandPoints = points, FreehandFitLevel = level }), rates), range).Duration;
        Check(Duration(noisySmoothed, 2) < Duration(noisy, 0), "smoothing did not shorten execution duration");
        Check(Smooth(noisy, 0).SequenceEqual(noisy), "Off rewrote points");
        foreach (int level in new[] { 1, 2, 3 }) {
            var processed = Smooth(noisy, level); var fit = Create(processed, level);
            Check(processed[0] == noisy[0] && processed[^1] == noisy[^1], "level endpoints");
            foreach (double s in fit.Checks!)
                Check(noisy.Zip(noisy.Skip(1)).Min(v => Distance(fit.At(s).P, v.First.Point, v.Second.Point)) <= level + 1 + 1e-8, "total smoothing + fitting budget");
            foreach (var p in noisy)
                Check(fit.Checks!.Zip(fit.Checks.Skip(1)).Min(v => Distance(p.Point, fit.At(v.First).P, fit.At(v.Second).P)) <= level + 1 + CaptureSpacing, "smoothing lost an excursion");
        }
        var legal = Repair(raw.Select(p => new FreehandPoint(320 + p.X, p.Y)).ToArray(), model, range, 0, rates);
        Check(legal.Shift == 0, "repair touched a legal draft");
        var overshoot = Enumerable.Range(0, 81).Select(i => new FreehandPoint(330 + i * 0.4, 3 * Math.Sin(i * 0.2))).ToArray();
        try { DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, 0, range, new ShapeOptions { FreehandPoints = overshoot }); Check(false, "overshoot was legal"); }
        catch (ArgumentException) { }
        var repaired = Repair(overshoot, model, range, 0, rates);
        Check(repaired.Shift > 0 && repaired.Shift <= RepairShift && repaired.Points.All(p => ((Vector)p.Point).Length < model.OuterRadius), "repair left the reach");
        var legalised = new ExecutionTrajectory(new TimedTrajectory(DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, 0, range,
            new ShapeOptions { FreehandPoints = repaired.Points }), rates), range);
        ExecutionTrajectory.Validate(legalised.Nodes, range);
        foreach (var bad in new[] { Array.Empty<FreehandPoint>(), [new(330, 0)], [new(330, 0), new(330, 0)], [new(330, 0), new(380, 0), new(330, 0)] })   // short drafts keep Create's error; a retraced excursion cannot hide its shift
            try { Repair(bad, model, range, 0, rates); Check(false, "repair accepted " + bad.Length + " points"); } catch (ArgumentException) { }
        try { Repair([new(330, 0), new(50, 0), new(-50, 0)], model, range, 0, rates); Check(false, "repair crossed the unreachable hole"); }   // no re-routing
        catch (ArgumentException) { }
        try {
            ExecutionTrajectory.Validate(new ExecutionNode[DeviceContract.TrajCapacity + 1], range);
            throw new InvalidOperationException("Freehand capacity gate missing");
        } catch (ArgumentException) { }
    }
}
