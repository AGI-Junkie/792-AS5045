using System.Globalization;
using System.Text;
using System.Windows;

namespace TwoLinkCommissioning;

// The first three are the original shapes (their saved indices must not move); the rest were added later.
public enum DrawingShape { Line, Rectangle, Circle, Ellipse, Arc, RoundedRectangle, Figure8, Polygon, Spiral, Custom, Freehand }
public readonly record struct DrawingSample(double Distance, Point Target, JointPose Pose);

// What a, b mean depends on the shape; this carries the rest.  Every shape is drawn around its anchor (x, y), then
// rotated about it, scaled about it, and finally traversed in the chosen direction.
//   C: arc sweep (deg) | rounded-rectangle corner radius (mm) | Lissajous y-frequency (2 = a figure 8) | spiral turns
//   Text: the contour of a Custom shape (point list or one SVG path)
public sealed record ShapeOptions
{
    public static readonly ShapeOptions Default = new();
    public double C { get; init; }
    public double Rotation { get; init; }          // degrees, counter-clockwise
    public double Scale { get; init; } = 1;
    public bool Clockwise { get; init; }           // open shapes: reversed stroke order (arc and spiral: sweep clockwise)
    public string Text { get; init; } = "";
    public IReadOnlyList<FreehandPoint> FreehandPoints { get; init; } = [];
}

// One stroke between two dwell points: P(s) with its first and second derivative in s, 0 <= s <= 1.
public readonly record struct PathPoint(Point P, Vector D1, Vector D2);
public sealed class PathPiece
{
    private readonly Func<double, PathPoint> at;
    public double Length { get; }
    internal double[]? Checks { get; }
    public PathPiece(Func<double, PathPoint> at, double? length = null, double[]? checks = null)
    {
        this.at = at; Checks = checks;
        if (length is { } known) { Length = known; return; }
        // Simpson over |P'(s)|; the pieces are smooth or piecewise straight, so 512 steps is far finer than the 1 mm sampling.
        double sum = 0; const int steps = 512;
        for (int i = 0; i <= steps; i++) sum += (i == 0 || i == steps ? 1 : i % 2 == 1 ? 4 : 2) * at((double)i / steps).D1.Length;
        Length = sum / (3 * steps);
    }
    public PathPoint At(double s) => at(s);
    public PathPiece Reversed() => new(s => { var p = at(1 - s); return new PathPoint(p.P, -p.D1, p.D2); }, Length);
    public static PathPiece Line(Point from, Point to) { var d = to - from; return new(s => new PathPoint(from + d * s, d, new Vector()), d.Length); }
    // A polyline run parametrised by arc length; the direction changes at its vertices.
    public static PathPiece Run(IReadOnlyList<Point> points)
    {
        var cumulative = new double[points.Count];
        for (int i = 1; i < points.Count; i++) cumulative[i] = cumulative[i - 1] + (points[i] - points[i - 1]).Length;
        double total = cumulative[^1];
        return new(s => {
            double d = Math.Clamp(s, 0, 1) * total;
            int i = Math.Clamp(Array.BinarySearch(cumulative, d) is var k && k >= 0 ? k : ~k - 1, 0, points.Count - 2);
            var segment = points[i + 1] - points[i]; double length = cumulative[i + 1] - cumulative[i];
            var unit = length == 0 ? new Vector() : segment / length;
            return new PathPoint(points[i] + unit * (d - cumulative[i]), unit * total, new Vector());
        }, total);
    }
}

// Offline geometry only; timing is separate. No pulse generation or serial commands.
public sealed class DrawingTrajectory
{
    public DrawingSample[] Samples { get; }
    public double Length => Samples[^1].Distance;
    public bool NearSingularity { get; }
    public PlanarArm Model { get; }
    public JointLimits? Limits { get; }
    public DrawingShape Shape { get; }
    public int Branch { get; }
    internal int[] Stops { get; }
    // The strokes between Stops, as analytic curves: the timing planner reads exact velocity and acceleration from them.
    internal PathPiece[] Pieces { get; }
    private DrawingTrajectory(PlanarArm model, JointLimits? limits, DrawingShape shape, int branch, DrawingSample[] samples, int[] stops, PathPiece[] pieces, bool nearSingularity)
    { Model = model; Limits = limits; Shape = shape; Branch = branch; Samples = samples; Stops = stops; Pieces = pieces; NearSingularity = nearSingularity; }

    public static DrawingTrajectory Create(PlanarArm model, DrawingShape shape, double x, double y,
        double a, double b, int branch, JointLimits? limits = null, ShapeOptions? options = null)
    {
        options ??= ShapeOptions.Default;
        if (!Enum.IsDefined(shape) || branch is < 0 or > 1 ||
            new[] { x, y, a, b, options.C, options.Rotation, options.Scale }.Any(v => !double.IsFinite(v) || Math.Abs(v) > 20000))
            throw Lang.Error("err.shapeRange");
        if (options.Scale < 0.01 || options.Scale > 100) throw Lang.Error("err.shapeScale");
        if (limits is { } l && (new[] { l.Min1, l.Max1, l.Min2, l.Max2, l.ElbowMin, l.ElbowMax }.Any(v => !double.IsFinite(v)) ||
            l.Min1 < -180 || l.Max1 > 180 || l.Min2 < -180 || l.Max2 > 180 || l.Min1 >= l.Max1 || l.Min2 >= l.Max2 ||
            l.ElbowMin < -180 || l.ElbowMax > 180 || l.ElbowMin >= l.ElbowMax))
            throw Lang.Error("err.jointRange");
        const double Rad = Math.PI / 180;
        var anchor = new Point(x, y);
        double cos = Math.Cos(options.Rotation * Rad), sin = Math.Sin(options.Rotation * Rad), scale = options.Scale;
        bool identity = options.Rotation == 0 && scale == 1;
        // A vector in the shape's own frame, turned into the plane.  With no rotation and scale 1 the original shapes
        // keep their exact coordinates, bit for bit.
        Vector Turn(Vector v) => identity ? v : new Vector((v.X * cos - v.Y * sin) * scale, (v.X * sin + v.Y * cos) * scale);
        Point Place(double dx, double dy) => identity ? new Point(x + dx, y + dy) : anchor + Turn(new Vector(dx, dy));
        bool clockwise = options.Clockwise;
        double direction = clockwise ? -1 : 1;
        var points = new List<Point> { anchor };
        var stops = new List<int> { 0 };
        PathPiece[]? curved = null;      // parametric strokes (the shapes that are not straight lines or the full circle)
        double circleRadius = 0;         // > 0: the full circle with exact extremum checks
        void Segment(Point end)
        {
            Point start = points[^1];
            Vector delta = end - start;
            int count = (int)Math.Ceiling(delta.Length); // at most 1 mm between samples
            if (points.Count + count > 5000) throw Lang.Error("err.tooManyPoints");
            for (int i = 1; i <= count; i++) points.Add(start + delta * ((double)i / count));
            stops.Add(points.Count - 1);
        }
        // Straight strokes between dwell points: the corners of a line, rectangle, polygon or point list.
        void Polyline(IReadOnlyList<Point> corners)
        {
            points[0] = corners[0];
            for (int i = 1; i < corners.Count; i++) Segment(corners[i]);
        }
        // A smooth stroke in the shape's own frame, moved into the plane (rotation, scale, anchor).
        PathPiece Frame(Func<double, (Vector P, Vector D1, Vector D2)> local) => new(s => {
            var (p, d1, d2) = local(s);
            return new PathPoint(identity ? anchor + p : anchor + Turn(p), Turn(d1), Turn(d2));
        });
        double Positive(double value, string key) => value >= 0.001 ? value : throw Lang.Error(key);
        switch (shape)
        {
            case DrawingShape.Freehand:
                curved = [FreehandTrajectory.Create(options.FreehandPoints)]; break;
            case DrawingShape.Line:
            {
                Point end = identity ? new Point(a, b) : anchor + Turn(new Vector(a - x, b - y));
                if ((end - anchor).Length < 0.001) throw Lang.Error("err.lineDegenerate");
                Polyline(clockwise ? [end, anchor] : [anchor, end]); break;
            }
            case DrawingShape.Rectangle:
            {
                if (a < 0.001 || b < 0.001) throw Lang.Error("err.rectSize");
                Point p0 = Place(0, 0), p1 = Place(a, 0), p2 = Place(a, b), p3 = Place(0, b);
                Polyline(clockwise ? [p0, p3, p2, p1, p0] : [p0, p1, p2, p3, p0]); break;
            }
            case DrawingShape.Polygon:
            {
                Positive(a, "err.polygonSize");
                if (b != Math.Round(b) || b < 3 || b > 12) throw Lang.Error("err.polygonSides");
                int sides = (int)b;
                var corners = new List<Point>();
                for (int k = 0; k <= sides; k++) { double angle = direction * 2 * Math.PI * (k % sides) / sides; corners.Add(Place(a * Math.Cos(angle), a * Math.Sin(angle))); }
                Polyline(corners); break;
            }
            case DrawingShape.Circle:
            {
                if (a < 0.001) throw Lang.Error("err.circleRadius");
                circleRadius = a * scale;
                // Check the exact circle as well as the sampled chords, including extrema between samples.
                double centerRadius = Math.Sqrt(x * x + y * y);
                CheckRadius(Math.Abs(centerRadius - circleRadius), centerRadius + circleRadius);
                int count = Math.Max(32, (int)Math.Ceiling(2 * Math.PI * circleRadius));
                if (count + 1 > 5000) throw Lang.Error("err.circleTooLong");
                points.Clear();
                for (int i = 0; i < count; i++)
                {
                    double angle = direction * 2 * Math.PI * i / count;
                    points.Add(new Point(x + circleRadius * Math.Cos(angle), y + circleRadius * Math.Sin(angle)));
                }
                points.Add(points[0]); stops.Add(points.Count - 1); break;
            }
            case DrawingShape.Ellipse:
            {
                double rx = Positive(a, "err.ellipseSize"), ry = Positive(b, "err.ellipseSize");
                curved = [Frame(s => { double t = direction * 2 * Math.PI * s;
                    return (new Vector(rx * Math.Cos(t), ry * Math.Sin(t)),
                            new Vector(-rx * Math.Sin(t), ry * Math.Cos(t)) * (direction * 2 * Math.PI),
                            new Vector(-rx * Math.Cos(t), -ry * Math.Sin(t)) * (4 * Math.PI * Math.PI)); })];
                break;
            }
            case DrawingShape.Arc:
            {
                double radius = Positive(a, "err.arcRadius");
                if (!(options.C > 0 && options.C <= 360)) throw Lang.Error("err.arcSweep");
                double start = b * Rad, sweep = direction * options.C * Rad;
                curved = [Frame(s => { double t = start + sweep * s;
                    return (new Vector(radius * Math.Cos(t), radius * Math.Sin(t)),
                            new Vector(-radius * Math.Sin(t), radius * Math.Cos(t)) * sweep,
                            new Vector(-radius * Math.Cos(t), -radius * Math.Sin(t)) * (sweep * sweep)); })];
                break;
            }
            case DrawingShape.RoundedRectangle:
            {
                double width = Positive(a, "err.rectSize"), height = Positive(b, "err.rectSize"), corner = options.C;
                if (!(corner >= 0.001 && corner <= Math.Min(width, height) / 2)) throw Lang.Error("err.roundRadius");
                curved = [RoundedRectangle(width, height, corner, Frame, clockwise)];
                break;
            }
            case DrawingShape.Figure8:
            {
                double halfWidth = Positive(a, "err.figureSize"), halfHeight = Positive(b, "err.figureSize"), ratio = options.C;
                if (ratio != Math.Round(ratio) || ratio < 2 || ratio > 6) throw Lang.Error("err.figureRatio");
                curved = [Frame(s => { double t = direction * 2 * Math.PI * s, w = direction * 2 * Math.PI;
                    return (new Vector(halfWidth * Math.Sin(t), halfHeight * Math.Sin(ratio * t)),
                            new Vector(halfWidth * Math.Cos(t), halfHeight * ratio * Math.Cos(ratio * t)) * w,
                            new Vector(-halfWidth * Math.Sin(t), -halfHeight * ratio * ratio * Math.Sin(ratio * t)) * (w * w)); })];
                break;
            }
            case DrawingShape.Spiral:
            {
                double r0 = a, r1 = b, turns = options.C;
                if (r0 < 0 || r1 < 0 || Math.Max(r0, r1) < 0.001) throw Lang.Error("err.spiralRadius");
                if (!(turns > 0 && turns <= 20)) throw Lang.Error("err.spiralTurns");
                double w = direction * 2 * Math.PI * turns, dr = r1 - r0;
                curved = [Frame(s => { double t = w * s, r = r0 + dr * s;
                    return (new Vector(r * Math.Cos(t), r * Math.Sin(t)),
                            new Vector(dr * Math.Cos(t) - r * w * Math.Sin(t), dr * Math.Sin(t) + r * w * Math.Cos(t)),
                            new Vector(-2 * dr * w * Math.Sin(t) - r * w * w * Math.Cos(t), 2 * dr * w * Math.Cos(t) - r * w * w * Math.Sin(t))); })];
                break;
            }
            case DrawingShape.Custom:
            {
                var path = CustomPath.Parse(options.Text);
                var vertices = path.Points.Select(v => anchor + Turn(v)).ToList(); var corner = path.Corner.ToList();
                if (clockwise) { vertices.Reverse(); corner.Reverse(); }
                if (corner.All(c => c)) { Polyline(vertices); break; }
                // Curves were flattened into short strokes: one smooth piece between corners, no stop at the joints.
                var runs = new List<PathPiece>(); int from = 0;
                for (int i = 1; i < vertices.Count; i++) if (corner[i]) { runs.Add(PathPiece.Run(vertices.GetRange(from, i - from + 1))); from = i; }
                curved = runs.ToArray(); break;
            }
        }
        if (curved is not null)
        {
            // Samples at most 1 mm apart, uniform in the stroke's parameter (the planner maps time onto the same parameter).
            points[0] = curved[0].At(0).P;
            foreach (var piece in curved)
            {
                double fastest = 0;
                for (int k = 0; k <= 256; k++) fastest = Math.Max(fastest, piece.At(k / 256.0).D1.Length);
                if (piece.Checks is { } probes)
                    foreach (double s in probes) fastest = Math.Max(fastest, piece.At(s).D1.Length);
                int count = Math.Max(2, (int)Math.Ceiling(fastest * 1.02));
                if (points.Count + count > 5000) throw Lang.Error("err.tooManyPoints");
                for (int i = 1; i <= count; i++) points.Add(piece.At((double)i / count).P);
                stops.Add(points.Count - 1);
            }
        }
        void CheckRadius(double min, double max)
        {
            if (min < model.InnerRadius - 1e-8 || max > model.OuterRadius + 1e-8)
                throw Lang.Error("err.geomUnreachable");
            if (min <= model.InnerRadius + 1e-6 || max >= model.OuterRadius - 1e-6)
                throw Lang.Error("err.singularPath");
        }
        var samples = new DrawingSample[points.Count];
        double distance = 0;
        for (int i = 0; i < points.Count; i++)
        {
            Point point = points[i];
            if (i > 0)
            {
                Point start = points[i - 1]; Vector delta = point - start;
                double t = delta.LengthSquared == 0 ? 0 : Math.Clamp(-(start.X * delta.X + start.Y * delta.Y) / delta.LengthSquared, 0, 1);
                Point nearest = start + delta * t;
                CheckRadius(((Vector)nearest).Length, Math.Max(((Vector)start).Length, ((Vector)point).Length));
                distance += delta.Length;
            }
            var poses = model.Inverse(point.X, point.Y);
            JointPose pose = poses[Math.Min(branch, poses.Length - 1)];
            if (i > 0)
            {
                // Keep both absolute link angles continuous through the -180/+180 boundary.
                double previous1 = samples[i - 1].Pose.Q1, previous2 = samples[i - 1].Pose.Q2;
                pose = pose with {
                    Q1 = pose.Q1 + 360 * Math.Round((previous1 - pose.Q1) / 360),
                    Q2 = pose.Q2 + 360 * Math.Round((previous2 - pose.Q2) / 360)
                };
            }
            if (limits is { } range && !range.Contains(pose))
                throw Lang.Error("err.sampleOutOfRange", i + 1, pose.Q1, pose.Q2, range.Min1, range.Max1, range.Min2, range.Max2,
                    JointLimits.Elbow(pose), range.ElbowMin, range.ElbowMax);
            samples[i] = new DrawingSample(distance, point, pose);
        }
        bool nearSingularity = samples.Any(s => Math.Abs(Math.Sin((s.Pose.Q2 - s.Pose.Q1) * Math.PI / 180)) < 0.05);
        // Joint pose of a point lying between samples first..last, unwrapped next to sample `reference`, checked against the limits.
        void CheckPoint(Point target, int reference)
        {
            var poses = model.Inverse(target.X, target.Y);
            var pose = poses[Math.Min(branch, poses.Length - 1)];
            pose = pose with {
                Q1 = pose.Q1 + 360 * Math.Round((samples[reference].Pose.Q1 - pose.Q1) / 360),
                Q2 = pose.Q2 + 360 * Math.Round((samples[reference].Pose.Q2 - pose.Q2) / 360)
            };
            nearSingularity |= Math.Abs(Math.Sin((pose.Q2 - pose.Q1) * Math.PI / 180)) < 0.05;
            if (limits is { } range && !range.Contains(pose))
                throw Lang.Error("err.betweenSamples", pose.Q1, pose.Q2, JointLimits.Elbow(pose));
        }
        void CheckBetween(double t, int first, int last)
        {
            if (t <= 0 || t >= 1) return; // endpoints were checked above
            double around = direction * 2 * Math.PI * t;
            Point target = circleRadius > 0 ? new Point(x + circleRadius * Math.Cos(around), y + circleRadius * Math.Sin(around)) :
                samples[first].Target + (samples[last].Target - samples[first].Target) * t;
            CheckPoint(target, (int)Math.Round(first + (last - first) * t));
        }
        if (circleRadius > 0)
        {
            double centerRadius = Math.Sqrt(x * x + y * y), centerAngle = Math.Atan2(y, x);
            // Sample parameter of the circle point at plane angle `angle` (samples run with `direction`).
            void CheckAngle(double angle) => CheckBetween((direction * angle / (2 * Math.PI) % 1 + 1) % 1, 0, samples.Length - 1);
            // The elbow depends on the tip's distance from the base only, so its extremes sit at the nearest and farthest
            // points of the circle: on the line through the base and the centre.
            if (centerRadius > 0) { CheckAngle(centerAngle); CheckAngle(centerAngle + Math.PI); }
            // Absolute q1/q2 extrema occur when the other link is radial to the circle.
            foreach (double link in new[] { model.L2, model.L1 })
            foreach (double offset in new[] { circleRadius - link, circleRadius + link })
            {
                if (centerRadius == 0 || offset == 0) continue;
                double other = link == model.L2 ? model.L1 : model.L2;
                double cosine = (other * other - centerRadius * centerRadius - offset * offset) / (2 * centerRadius * offset);
                if (cosine < -1 || cosine > 1) continue;
                double angle = Math.Acos(cosine);
                CheckAngle(centerAngle - angle); CheckAngle(centerAngle + angle);
            }
        }
        else if (curved is not null)
        {
            // ponytail: curves are re-checked at three more points per 1 mm interval (0.25 mm apart) instead of at their
            // exact extrema; the lines and the circle above are exact. Raise the count if a limit is ever met within ~0.01 deg.
            for (int piece = 0; piece < curved.Length; piece++)
            {
                int first = stops[piece], last = stops[piece + 1];
                if (curved[piece].Checks is { } probes)
                    foreach (double s in probes) {
                        Point target = curved[piece].At(s).P;
                        double radius = ((Vector)target).Length;
                        CheckRadius(radius, radius);
                        CheckPoint(target, (int)Math.Round(first + (last - first) * s));
                    }
                for (int j = first; j < last; j++)
                    foreach (double q in new[] { 0.25, 0.5, 0.75 })
                    {
                        Point target = curved[piece].At((j - first + q) / (last - first)).P;
                        double radius = ((Vector)target).Length;
                        CheckRadius(radius, radius);
                        CheckPoint(target, j);
                    }
            }
        }
        else for (int piece = 0; piece < stops.Count - 1; piece++)
        {
            int first = stops[piece], last = stops[piece + 1];
            Vector start = (Vector)samples[first].Target, delta = samples[last].Target - samples[first].Target;
            double squared = delta.LengthSquared;
            CheckBetween(-Vector.Multiply(start, delta) / squared, first, last);
            // Each link-angle extremum has the other link perpendicular to the segment.
            foreach (double link in new[] { model.L2, model.L1 })
            {
                double other = link == model.L2 ? model.L1 : model.L2;
                Vector normal = new Vector(-delta.Y, delta.X) * (link / Math.Sqrt(squared));
                foreach (double sign in new[] { -1.0, 1.0 })
                {
                    Vector offset = start + normal * sign;
                    double middle = -Vector.Multiply(offset, delta) / squared;
                    double remainder = other * other - (offset + delta * middle).LengthSquared;
                    if (remainder < 0) continue;
                    double half = Math.Sqrt(remainder / squared);
                    CheckBetween(middle - half, first, last); CheckBetween(middle + half, first, last);
                }
            }
        }
        PathPiece[] pieces = curved ?? (circleRadius > 0
            ? [new PathPiece(s => { double t = direction * 2 * Math.PI * s; var radial = new Vector(circleRadius * Math.Cos(t), circleRadius * Math.Sin(t));
                    return new PathPoint(anchor + radial, new Vector(-radial.Y, radial.X) * (direction * 2 * Math.PI), -radial * (4 * Math.PI * Math.PI)); }, 2 * Math.PI * circleRadius)]
            : Enumerable.Range(0, stops.Count - 1).Select(i => PathPiece.Line(samples[stops[i]].Target, samples[stops[i + 1]].Target)).ToArray());
        return new DrawingTrajectory(model, limits, shape, branch, samples, stops.ToArray(), pieces, nearSingularity);
    }

    // Eight strokes (four straight, four quarter circles) as ONE smooth piece: the pen never dwells at the corners.
    // Parametrised by arc length: the tangent has length L, and the acceleration is curvature * L^2 towards the inside.
    private static PathPiece RoundedRectangle(double w, double h, double r, Func<Func<double, (Vector P, Vector D1, Vector D2)>, PathPiece> frame, bool clockwise)
    {
        double lw = w - 2 * r, lh = h - 2 * r, quarter = Math.PI * r / 2, total = 2 * lw + 2 * lh + 4 * quarter;
        // (length, start point, start angle of the arc or 0, centre, is arc, tangent of a line)
        var parts = new (double Length, Vector From, Vector Center, double Angle, Vector Tangent, bool Arc)[] {
            (lw, new(r, 0), default, 0, new(1, 0), false),
            (quarter, default, new(w - r, r), -Math.PI / 2, default, true),
            (lh, new(w, r), default, 0, new(0, 1), false),
            (quarter, default, new(w - r, h - r), 0, default, true),
            (lw, new(w - r, h), default, 0, new(-1, 0), false),
            (quarter, default, new(r, h - r), Math.PI / 2, default, true),
            (lh, new(0, h - r), default, 0, new(0, -1), false),
            (quarter, default, new(r, r), Math.PI, default, true),
        };
        var piece = frame(s => {
            double d = Math.Clamp(s, 0, 1) * total, done = 0;
            foreach (var part in parts)
            {
                if (d > done + part.Length && part != parts[^1]) { done += part.Length; continue; }
                double along = Math.Clamp(d - done, 0, part.Length);
                if (!part.Arc) return (part.From + part.Tangent * along, part.Tangent * total, new Vector());
                double t = part.Angle + along / r;
                var tangent = new Vector(-Math.Sin(t), Math.Cos(t));
                return (part.Center + new Vector(Math.Cos(t), Math.Sin(t)) * r, tangent * total, new Vector(-Math.Cos(t), -Math.Sin(t)) * (total * total / r));
            }
            throw new InvalidOperationException("unreachable");
        });
        return clockwise ? piece.Reversed() : piece;
    }

    internal const string CsvMetadataHeader = "kind,shape,l1_mm,l2_mm,branch,limits_status,q1_min_deg,q1_max_deg,q2_min_deg,q2_max_deg,elbow_min_deg,elbow_max_deg";
    internal string CsvMetadata
    {
        get
        {
            static string N(double v) => v.ToString("G17", CultureInfo.InvariantCulture);
            string range = Limits is { } l ? $"{N(l.Min1)},{N(l.Max1)},{N(l.Min2)},{N(l.Max2)},{N(l.ElbowMin)},{N(l.ElbowMax)}" : ",,,,,";
            return $"preview_only,{Shape},{N(Model.L1)},{N(Model.L2)},{Branch},{(Limits is null ? "unmeasured" : "samples_checked")},{range}";
        }
    }
    public string ToCsv()
    {
        static string N(double v) => v.ToString("G17", CultureInfo.InvariantCulture);
        var csv = new StringBuilder(CsvMetadataHeader + ",index,path_mm,x_mm,y_mm,q1_deg,q2_deg\r\n");
        for (int i = 0; i < Samples.Length; i++)
        {
            var s = Samples[i];
            csv.Append($"{CsvMetadata},{i},{N(s.Distance)},{N(s.Target.X)},{N(s.Target.Y)},{N(s.Pose.Q1)},{N(s.Pose.Q2)}\r\n");
        }
        return csv.ToString();
    }

    public static void SelfTest()
    {
        static void Check(bool value) { if (!value) throw new InvalidOperationException("Trajectory check failed"); }
        static void Reject(Action action)
        { try { action(); } catch (ArgumentException) { return; } throw new InvalidOperationException("Invalid trajectory accepted"); }
        var model = new PlanarArm(180, 176.38);
        foreach (int branch in new[] { 0, 1 })
        foreach (var shape in new[] { DrawingShape.Line, DrawingShape.Rectangle, DrawingShape.Circle })
        {
            var path = Create(model, shape, 220, 0, shape == DrawingShape.Line ? 250 : 30, 40, branch);
            Check(path.Samples.Length > 2 && path.Length > 0);
            for (int i = 0; i < path.Samples.Length; i++)
            {
                var s = path.Samples[i]; Check((model.Forward(s.Pose).Tip - s.Target).Length < 1e-6);
                if (i > 0) Check(s.Distance > path.Samples[i - 1].Distance && s.Distance - path.Samples[i - 1].Distance <= 1.000001);
            }
            if (shape != DrawingShape.Line) Check(path.Samples[0].Target == path.Samples[^1].Target);
            string[] rows = path.ToCsv().Split("\r\n", StringSplitOptions.RemoveEmptyEntries);
            Check(rows.Length == path.Samples.Length + 1 && rows.All(row => row.Split(',').Length == 18));
        }
        Check(Math.Abs(Create(model, DrawingShape.Rectangle, 200, 0, 30, 40, 0).Length - 140) < 1e-8);
        var wrap = Create(model, DrawingShape.Circle, 0, 0, 200, 0, 0);
        Check(Math.Abs(wrap.Samples[^1].Pose.Q1 - wrap.Samples[0].Pose.Q1 - 360) < 1e-6);
        Check(wrap.Samples.Zip(wrap.Samples.Skip(1)).All(p => Math.Abs(p.Second.Pose.Q1 - p.First.Pose.Q1) < 2 && Math.Abs(p.Second.Pose.Q2 - p.First.Pose.Q2) < 2));
        // Reachable endpoints can hide an unreachable interior, even within one 1 mm segment.
        Reject(() => Create(model, DrawingShape.Line, -20, 0, 20, 0, 0));
        Reject(() => Create(model, DrawingShape.Line, -0.1, 3.6199, 0.1, 3.6199, 0));
        Reject(() => Create(model, DrawingShape.Circle, 350, 0, 10, 0, 0));
        Reject(() => Create(model, DrawingShape.Circle, 20, 0, 20, 0, 0));
        Reject(() => Create(model, DrawingShape.Circle, 0, 0, 356.38, 0, 0));
        Reject(() => Create(model, DrawingShape.Line, 220, 0, 220, 0, 0));
        Reject(() => Create(model, DrawingShape.Rectangle, 220, 0, -1, 10, 0));
        Reject(() => Create(model, DrawingShape.Line, double.NaN, 0, 230, 0, 0));
        Reject(() => Create(model, DrawingShape.Line, 200, 0, 201, 0, 0, new JointLimits(-1, 1, -1, 1)));
        Reject(() => Create(model, DrawingShape.Circle, 0, 0, 200, 0, 0, new JointLimits(-180, 180, -179, 179)));
        var limited = Create(model, DrawingShape.Line, 220, 0, 250, 20, 0, new JointLimits(-90, 90, 0, 160));
        Check(limited.ToCsv().Contains("samples_checked,-90,90,0,160,-155,155"));
        // The elbow bound rejects a path that both q boxes accept, at a sample and between samples:
        // a line near the base needs a large elbow; the same line far out does not.
        var near = model.Inverse(60, 0)[0]; var far = model.Inverse(300, 0)[0];
        Check(Math.Abs(JointLimits.Elbow(near)) > 120 && Math.Abs(JointLimits.Elbow(far)) < 70);
        var boxes = new JointLimits(-180, 180, -180, 180, -100, 100);
        Reject(() => Create(model, DrawingShape.Line, 60, 0, 70, 0, 0, boxes));
        Check(Create(model, DrawingShape.Line, 300, 0, 310, 0, 0, boxes).Samples.Length > 2);
        // The circle's nearest point to the base falls between two samples, whose elbows stay ~9e-4 deg short of the true
        // extreme: only the analytic nearest/farthest check sees it, so a bound just under it must reject.
        var ring = Create(model, DrawingShape.Circle, 200, 0, 50, 0, 0);
        double ringMax = ring.Samples.Max(s => Math.Abs(JointLimits.Elbow(s.Pose)));
        double exactMax = Math.Abs(JointLimits.Elbow(model.Inverse(150, 0)[0]));
        Check(exactMax >= ringMax - 1e-9);
        Reject(() => Create(model, DrawingShape.Circle, 200, 0, 50, 0, 0, new JointLimits(-180, 180, -180, 180, -(exactMax - 1e-4), exactMax - 1e-4)));
        Check(Create(model, DrawingShape.Circle, 200, 0, 50, 0, 0, new JointLimits(-180, 180, -180, 180, -(exactMax + 1e-4), exactMax + 1e-4)).Samples.Length == ring.Samples.Length);
        // Endpoint/sample checks alone miss these interior joint extrema.
        double endQ2 = model.Inverse(-0.4, 200)[0].Q2, middleQ2 = model.Inverse(0, 200)[0].Q2;
        Reject(() => Create(model, DrawingShape.Line, -0.4, 200, 0.4, 200, 0, new JointLimits(-179, 179, 0, (endQ2 + middleQ2) / 2)));
        Reject(() => Create(model, DrawingShape.Rectangle, -0.4, 200, 0.8, 0.2, 0, new JointLimits(-179, 179, 0, (endQ2 + middleQ2) / 2)));
        var circle = Create(model, DrawingShape.Circle, 260, 0, 50, 0, 0);
        double circleMin1 = -Math.Acos((model.L1 * model.L1 + 260 * 260 - Math.Pow(model.L2 + 50, 2)) / (2 * model.L1 * 260)) * 180 / Math.PI;
        double circleMax2 = model.Inverse(210, 0)[0].Q2;
        Reject(() => Create(model, DrawingShape.Circle, 260, 0, 50, 0, 0, new JointLimits((circleMin1 + circle.Samples.Min(s => s.Pose.Q1)) / 2, 135, 0, 155)));
        Reject(() => Create(model, DrawingShape.Circle, 260, 0, 50, 0, 0, new JointLimits(-135, 135, 0, (circleMax2 + circle.Samples.Max(s => s.Pose.Q2)) / 2)));
        Check(Create(model, DrawingShape.Circle, 260, 0, 50, 0, 0, new JointLimits(circleMin1 - 1e-7, 135, 0, circle.Samples.Max(s => s.Pose.Q2) + 1)).Samples.Length == circle.Samples.Length);
        var oldCulture = CultureInfo.CurrentCulture;
        try { CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("de-DE"); Check(limited.ToCsv().Contains(",176.38,")); }
        finally { CultureInfo.CurrentCulture = oldCulture; }
        MoreShapesSelfTest();
    }

    private static void MoreShapesSelfTest()
    {
        static void Check(bool value, string what) { if (!value) throw new InvalidOperationException("Shape check failed: " + what); }
        static void Reject(Action action, string what)
        { try { action(); } catch (ArgumentException) { return; } throw new InvalidOperationException("Invalid shape accepted: " + what); }
        var model = new PlanarArm(180, 176.38);
        // The original shapes are unchanged by the transform fields at their defaults, and obey them otherwise.
        var plain = Create(model, DrawingShape.Rectangle, 300, -10, 20, 10, 0);
        var same = Create(model, DrawingShape.Rectangle, 300, -10, 20, 10, 0, null, new ShapeOptions { Scale = 1, Rotation = 0 });
        Check(plain.Samples.Select(s => s.Target).SequenceEqual(same.Samples.Select(s => s.Target)), "default options keep the rectangle bit for bit");
        var turned = Create(model, DrawingShape.Rectangle, 300, -10, 20, 10, 0, null, new ShapeOptions { Rotation = 90 });
        Check((turned.Samples[turned.Stops[1]].Target - new Point(300, 10)).Length < 1e-9 && (turned.Samples[turned.Stops[2]].Target - new Point(290, 10)).Length < 1e-9, "rotation turns the rectangle about its corner");
        var doubled = Create(model, DrawingShape.Rectangle, 300, -10, 20, 10, 0, null, new ShapeOptions { Scale = 2 });
        Check(Math.Abs(doubled.Length - 2 * plain.Length) < 1e-8, "scale doubles the perimeter");
        var reversed = Create(model, DrawingShape.Rectangle, 300, -10, 20, 10, 0, null, new ShapeOptions { Clockwise = true });
        Check((reversed.Samples[reversed.Stops[1]].Target - new Point(300, 0)).Length < 1e-9 && reversed.Samples[0].Target == plain.Samples[0].Target, "clockwise rectangle goes up first from the same corner");
        var cwCircle = Create(model, DrawingShape.Circle, 300, 0, 20, 0, 0, null, new ShapeOptions { Clockwise = true });
        var ccwCircle = Create(model, DrawingShape.Circle, 300, 0, 20, 0, 0);
        Check(cwCircle.Samples[1].Target.Y < 0 && ccwCircle.Samples[1].Target.Y > 0 && cwCircle.Samples[0].Target == ccwCircle.Samples[0].Target, "clockwise circle starts the other way from the same point");
        Check(Math.Abs(Create(model, DrawingShape.Circle, 300, 0, 10, 0, 0, null, new ShapeOptions { Scale = 2 }).Length - Create(model, DrawingShape.Circle, 300, 0, 20, 0, 0).Length) < 1e-6, "circle scale scales the radius");
        var lineTurn = Create(model, DrawingShape.Line, 300, 0, 320, 0, 0, null, new ShapeOptions { Rotation = 90 });
        Check((lineTurn.Samples[^1].Target - new Point(300, 20)).Length < 1e-9, "a line turns about its start");

        var options = new Dictionary<DrawingShape, (double A, double B, ShapeOptions O)> {
            [DrawingShape.Ellipse] = (30, 15, new ShapeOptions { Rotation = 30 }),
            [DrawingShape.Arc] = (25, -60, new ShapeOptions { C = 150 }),
            [DrawingShape.RoundedRectangle] = (40, 24, new ShapeOptions { C = 6 }),
            [DrawingShape.Figure8] = (25, 12, new ShapeOptions { C = 2, Rotation = 90 }),
            [DrawingShape.Polygon] = (22, 5, new ShapeOptions()),
            [DrawingShape.Spiral] = (4, 30, new ShapeOptions { C = 2.5 }),
            [DrawingShape.Custom] = (0, 0, new ShapeOptions { Text = "M 0 0 C 0 -25 30 -25 30 0 L 50 0 L 50 -20" }),
        };
        foreach (var (shape, (a, b, o)) in options)
        foreach (bool clockwise in new[] { false, true })
        foreach (int branch in new[] { 0, 1 })
        {
            string name = $"{shape} cw={clockwise} branch={branch}";
            var path = Create(model, shape, 250, 0, a, b, branch, null, o with { Clockwise = clockwise });
            Check(path.Samples.Length > 10 && path.Length > 20 && path.Pieces.Length == path.Stops.Length - 1, name + ": has samples and one piece per stroke");
            for (int i = 0; i < path.Samples.Length; i++)
            {
                var s = path.Samples[i];
                Check((model.Forward(s.Pose).Tip - s.Target).Length < 1e-6, name + ": IK round trip");
                if (i > 0) Check(s.Distance - path.Samples[i - 1].Distance <= 1.000001, name + ": samples at most 1 mm apart");
            }
            // The analytic strokes agree with the samples: each piece starts and ends on its sample points.
            for (int p = 0; p < path.Pieces.Length; p++)
                Check((path.Pieces[p].At(0).P - path.Samples[path.Stops[p]].Target).Length < 1e-6 && (path.Pieces[p].At(1).P - path.Samples[path.Stops[p + 1]].Target).Length < 1e-6, name + ": piece ends match samples");
            // ...and their derivatives are the derivatives of the position (central differences).
            foreach (var piece in path.Pieces)
                foreach (double s in new[] { 0.1337, 0.5123, 0.8761 })   // away from every junction, where D2 of the straight/arc pieces jumps
                {
                    const double h = 1e-6; var before = piece.At(s - h); var after = piece.At(s + h); var here = piece.At(s);
                    Check(((after.P - before.P) / (2 * h) - here.D1).Length < 1e-3 * (1 + here.D1.Length), name + ": D1 is dP/ds");
                    Check(((after.D1 - before.D1) / (2 * h) - here.D2).Length < 1e-2 * (1 + here.D2.Length), name + ": D2 is dD1/ds");
                }
            var timed = new TimedTrajectory(path, new PlanningRates(1000, 8, 5, 12, 9));
            Check(timed.Duration > 1 && (timed.At(timed.Duration).Target - path.Samples[^1].Target).Length < 1e-6 && (timed.At(0).Target - path.Samples[0].Target).Length < 1e-6, name + ": timing starts and ends at the path ends");
            _ = new ExecutionTrajectory(timed, new JointLimits(-180, 180, -180, 180, -180, 180));
        }
        // Reversed shapes visit the same points.
        var ell = Create(model, DrawingShape.Ellipse, 250, 0, 30, 15, 0, null, new ShapeOptions { Rotation = 30 });
        var ellCw = Create(model, DrawingShape.Ellipse, 250, 0, 30, 15, 0, null, new ShapeOptions { Rotation = 30, Clockwise = true });
        Check((ell.Samples[0].Target - ellCw.Samples[0].Target).Length < 1e-9 && ell.Samples.Length == ellCw.Samples.Length && (ell.Samples[1].Target - ellCw.Samples[1].Target).Length > 0.1, "clockwise ellipse starts at the same point and leaves the other way");
        // Ellipse: extent along its tilted axis; width at 90 degrees swaps the axes.
        double Extent(DrawingTrajectory t, Func<Point, double> axis) => t.Samples.Max(s => axis(s.Target)) - t.Samples.Min(s => axis(s.Target));
        var flat = Create(model, DrawingShape.Ellipse, 250, 0, 30, 15, 0);
        var upright = Create(model, DrawingShape.Ellipse, 250, 0, 30, 15, 0, null, new ShapeOptions { Rotation = 90 });
        Check(Math.Abs(Extent(flat, p => p.X) - 60) < 0.01 && Math.Abs(Extent(flat, p => p.Y) - 30) < 0.01 && Math.Abs(Extent(upright, p => p.X) - 30) < 0.01 && Math.Abs(Extent(upright, p => p.Y) - 60) < 0.01, "ellipse axes and rotation");
        // Arc: radius, start angle and sweep; a sweep of 90 deg from 0 ends straight above the centre.
        var arc = Create(model, DrawingShape.Arc, 250, 0, 25, 0, 0, null, new ShapeOptions { C = 90 });
        Check((arc.Samples[0].Target - new Point(275, 0)).Length < 1e-9 && (arc.Samples[^1].Target - new Point(250, 25)).Length < 1e-9 && Math.Abs(arc.Length - 25 * Math.PI / 2) < 0.01, "arc start, end and length");
        var arcCw = Create(model, DrawingShape.Arc, 250, 0, 25, 0, 0, null, new ShapeOptions { C = 90, Clockwise = true });
        Check((arcCw.Samples[^1].Target - new Point(250, -25)).Length < 1e-9, "clockwise arc sweeps below the start");
        Reject(() => Create(model, DrawingShape.Arc, 250, 0, 25, 0, 0, null, new ShapeOptions { C = 0 }), "zero sweep");
        Reject(() => Create(model, DrawingShape.Arc, 250, 0, 25, 0, 0, null, new ShapeOptions { C = 361 }), "sweep over 360");
        // Rounded rectangle: one smooth stroke of the exact perimeter; a bigger radius than half the short side is refused.
        var round = Create(model, DrawingShape.RoundedRectangle, 250, -10, 40, 24, 0, null, new ShapeOptions { C = 6 });
        double perimeter = 2 * (40 - 12) + 2 * (24 - 12) + 2 * Math.PI * 6;
        Check(round.Stops.Length == 2 && Math.Abs(round.Pieces[0].Length - perimeter) < 1e-6 && perimeter - round.Length is > 0 and < 0.1 && (round.Samples[0].Target - round.Samples[^1].Target).Length < 1e-9, "rounded rectangle is one closed piece of the right length (chords are a hair shorter than the arcs)");
        Check(round.Samples.All(s => s.Target.X >= 249.999 && s.Target.X <= 290.001 && s.Target.Y >= -10.001 && s.Target.Y <= 14.001), "rounded rectangle stays inside its box");
        Check((round.Samples[0].Target - new Point(256, -10)).Length < 1e-9, "rounded rectangle starts after the first corner");
        Reject(() => Create(model, DrawingShape.RoundedRectangle, 250, -10, 40, 24, 0, null, new ShapeOptions { C = 12.1 }), "corner radius over half the short side");
        Reject(() => Create(model, DrawingShape.RoundedRectangle, 250, -10, 40, 24, 0, null, new ShapeOptions { C = 0 }), "zero corner radius");
        // "8": closed, crosses itself at the anchor; the frequency has to be a whole number of at least two.
        var eight = Create(model, DrawingShape.Figure8, 250, 0, 25, 12, 0, null, new ShapeOptions { C = 2 });
        Check((eight.Samples[0].Target - new Point(250, 0)).Length < 1e-9 && (eight.Samples[^1].Target - eight.Samples[0].Target).Length < 1e-6 && Math.Abs(Extent(eight, p => p.X) - 50) < 0.01 && Math.Abs(Extent(eight, p => p.Y) - 2 * 12 * 0.9999) < 0.2, "figure 8 extent and closure");
        Check(eight.Samples.Count(s => (s.Target - new Point(250, 0)).Length < 0.6) >= 2, "figure 8 passes the centre twice");
        Reject(() => Create(model, DrawingShape.Figure8, 250, 0, 25, 12, 0, null, new ShapeOptions { C = 1 }), "frequency 1");
        Reject(() => Create(model, DrawingShape.Figure8, 250, 0, 25, 12, 0, null, new ShapeOptions { C = 2.5 }), "fractional frequency");
        // Polygon: n strokes with a dwell at each vertex; the vertex count is a whole number from 3 to 12.
        var triangle = Create(model, DrawingShape.Polygon, 250, 0, 22, 3, 0);
        Check(triangle.Stops.Length == 4 && (triangle.Samples[0].Target - new Point(272, 0)).Length < 1e-9 && Math.Abs(triangle.Length - 3 * 22 * Math.Sqrt(3)) < 1e-6, "triangle: three strokes, side R*sqrt(3)");
        Check(Create(model, DrawingShape.Polygon, 250, 0, 22, 6, 0).Stops.Length == 7, "hexagon has six strokes");
        foreach (double sides in new[] { 2, 13, 4.5 }) Reject(() => Create(model, DrawingShape.Polygon, 250, 0, 22, sides, 0), $"{sides} sides");
        // Spiral: radius grows from a to b over the turns.
        var spiral = Create(model, DrawingShape.Spiral, 250, 0, 4, 30, 0, null, new ShapeOptions { C = 2 });
        Check(Math.Abs((spiral.Samples[0].Target - new Point(250, 0)).Length - 4) < 1e-9 && Math.Abs((spiral.Samples[^1].Target - new Point(250, 0)).Length - 30) < 1e-9 && spiral.Length > 2 * Math.PI * 17 * 2 * 0.98, "spiral radius runs 4 to 30 over two turns");
        Reject(() => Create(model, DrawingShape.Spiral, 250, 0, 4, 30, 0, null, new ShapeOptions { C = 0 }), "no turns");
        Reject(() => Create(model, DrawingShape.Spiral, 250, 0, 0, 0, 0, null, new ShapeOptions { C = 2 }), "zero radius");
        // Custom: points are corners; an SVG contour with curves is smooth between its corners; two contours need a pen lift.
        var pointsPath = Create(model, DrawingShape.Custom, 250, 0, 0, 0, 0, null, new ShapeOptions { Text = "0 0\n20 0\n20 10\n0 10\n0 0" });
        Check(pointsPath.Stops.Length == 5 && Math.Abs(pointsPath.Length - 60) < 1e-9, "point list: four strokes, perimeter 60");
        var svg = Create(model, DrawingShape.Custom, 250, 0, 0, 0, 0, null, new ShapeOptions { Text = "M 0 0 C 0 -25 30 -25 30 0 L 50 0 L 50 -20" });
        Check(svg.Stops.Length == 4 && svg.Samples.Max(s => s.Target.Y) > 10 && (svg.Samples[^1].Target - new Point(300, 20)).Length < 1e-9, "svg: y flipped (up is positive), a smooth arch and two corners");
        Reject(() => Create(model, DrawingShape.Custom, 250, 0, 0, 0, 0, null, new ShapeOptions { Text = "M 0 0 L 10 0 M 20 0 L 30 0" }), "two contours");
        Reject(() => Create(model, DrawingShape.Custom, 250, 0, 0, 0, 0, null, new ShapeOptions { Text = "" }), "empty custom path");
        Reject(() => Create(model, (DrawingShape)99, 250, 0, 0, 0, 0), "undefined shape");
        // The elbow/limit checks apply to curves too: bounds just inside the true extreme refuse the ellipse, just outside accept it.
        double fineMax = Enumerable.Range(0, 20000).Max(i => { double t = i * 2 * Math.PI / 20000; return Math.Abs(JointLimits.Elbow(model.Inverse(250 + 30 * Math.Cos(t), 15 * Math.Sin(t))[0])); });
        Reject(() => Create(model, DrawingShape.Ellipse, 250, 0, 30, 15, 0, new JointLimits(-180, 180, -180, 180, -(fineMax - 0.01), fineMax - 0.01)), "elbow bound inside an ellipse");
        Check(Create(model, DrawingShape.Ellipse, 250, 0, 30, 15, 0, new JointLimits(-180, 180, -180, 180, -(fineMax + 0.01), fineMax + 0.01)).Samples.Length > 10, "elbow bound outside an ellipse");
        Reject(() => Create(model, DrawingShape.Ellipse, 250, 0, 30, 15, 0, null, new ShapeOptions { Scale = 0 }), "zero scale");
        Reject(() => Create(model, DrawingShape.Ellipse, 250, 0, 0, 15, 0), "zero semi-axis");
        Reject(() => Create(model, DrawingShape.Ellipse, 250, 0, 300, 15, 0), "ellipse leaving the reach");
        CustomPath.SelfTest();
        FreehandTrajectory.SelfTest();
    }
}
