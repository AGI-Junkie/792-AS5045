using System.Globalization;
using System.Text;
using System.Windows;

namespace TwoLinkCommissioning;

public enum DrawingShape { Line, Rectangle, Circle }
public readonly record struct DrawingSample(double Distance, Point Target, JointPose Pose);

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
    internal Point? CircleCenter { get; }
    private DrawingTrajectory(PlanarArm model, JointLimits? limits, DrawingShape shape, int branch, DrawingSample[] samples, int[] stops, Point? center, bool nearSingularity)
    { Model = model; Limits = limits; Shape = shape; Branch = branch; Samples = samples; Stops = stops; CircleCenter = center; NearSingularity = nearSingularity; }

    public static DrawingTrajectory Create(PlanarArm model, DrawingShape shape, double x, double y,
        double a, double b, int branch, JointLimits? limits = null)
    {
        if (!Enum.IsDefined(shape) || branch is < 0 or > 1 ||
            new[] { x, y, a, b }.Any(v => !double.IsFinite(v) || Math.Abs(v) > 20000))
            throw Lang.Error("err.shapeRange");
        if (limits is { } l && (new[] { l.Min1, l.Max1, l.Min2, l.Max2 }.Any(v => !double.IsFinite(v)) ||
            l.Min1 < -180 || l.Max1 > 180 || l.Min2 < -180 || l.Max2 > 180 || l.Min1 >= l.Max1 || l.Min2 >= l.Max2))
            throw Lang.Error("err.jointRange");
        var points = new List<Point> { new(x, y) };
        var stops = new List<int> { 0 };
        void Segment(Point end)
        {
            Point start = points[^1];
            Vector delta = end - start;
            int count = (int)Math.Ceiling(delta.Length); // at most 1 mm between samples
            if (points.Count + count > 5000) throw Lang.Error("err.tooManyPoints");
            for (int i = 1; i <= count; i++) points.Add(start + delta * ((double)i / count));
            stops.Add(points.Count - 1);
        }
        switch (shape)
        {
            case DrawingShape.Line:
                if ((new Point(a, b) - points[0]).Length < 0.001) throw Lang.Error("err.lineDegenerate");
                Segment(new Point(a, b)); break;
            case DrawingShape.Rectangle:
                if (a < 0.001 || b < 0.001) throw Lang.Error("err.rectSize");
                Segment(new Point(x + a, y)); Segment(new Point(x + a, y + b));
                Segment(new Point(x, y + b)); Segment(new Point(x, y)); break;
            case DrawingShape.Circle:
                if (a < 0.001) throw Lang.Error("err.circleRadius");
                // Check the exact circle as well as the sampled chords, including extrema between samples.
                double centerRadius = Math.Sqrt(x * x + y * y);
                CheckRadius(Math.Abs(centerRadius - a), centerRadius + a);
                int count = Math.Max(32, (int)Math.Ceiling(2 * Math.PI * a));
                if (count + 1 > 5000) throw Lang.Error("err.circleTooLong");
                points.Clear();
                for (int i = 0; i < count; i++)
                {
                    double angle = 2 * Math.PI * i / count;
                    points.Add(new Point(x + a * Math.Cos(angle), y + a * Math.Sin(angle)));
                }
                points.Add(points[0]); stops.Add(points.Count - 1); break;
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
                throw Lang.Error("err.sampleOutOfRange", i + 1, pose.Q1, pose.Q2, range.Min1, range.Max1, range.Min2, range.Max2);
            samples[i] = new DrawingSample(distance, point, pose);
        }
        bool nearSingularity = samples.Any(s => Math.Abs(Math.Sin((s.Pose.Q2 - s.Pose.Q1) * Math.PI / 180)) < 0.05);
        void CheckBetween(double t, int first, int last)
        {
            if (t <= 0 || t >= 1) return; // endpoints were checked above
            Point target = shape == DrawingShape.Circle ? new Point(x + a * Math.Cos(2 * Math.PI * t), y + a * Math.Sin(2 * Math.PI * t)) :
                samples[first].Target + (samples[last].Target - samples[first].Target) * t;
            var poses = model.Inverse(target.X, target.Y);
            var pose = poses[Math.Min(branch, poses.Length - 1)];
            int reference = (int)Math.Round(first + (last - first) * t);
            pose = pose with {
                Q1 = pose.Q1 + 360 * Math.Round((samples[reference].Pose.Q1 - pose.Q1) / 360),
                Q2 = pose.Q2 + 360 * Math.Round((samples[reference].Pose.Q2 - pose.Q2) / 360)
            };
            nearSingularity |= Math.Abs(Math.Sin((pose.Q2 - pose.Q1) * Math.PI / 180)) < 0.05;
            if (limits is { } range && !range.Contains(pose))
                throw Lang.Error("err.betweenSamples", pose.Q1, pose.Q2);
        }
        if (shape == DrawingShape.Circle)
        {
            double centerRadius = Math.Sqrt(x * x + y * y), centerAngle = Math.Atan2(y, x);
            void CheckAngle(double angle) => CheckBetween((angle / (2 * Math.PI) % 1 + 1) % 1, 0, samples.Length - 1);
            // Absolute q1/q2 extrema occur when the other link is radial to the circle.
            foreach (double link in new[] { model.L2, model.L1 })
            foreach (double offset in new[] { a - link, a + link })
            {
                if (centerRadius == 0 || offset == 0) continue;
                double other = link == model.L2 ? model.L1 : model.L2;
                double cosine = (other * other - centerRadius * centerRadius - offset * offset) / (2 * centerRadius * offset);
                if (cosine < -1 || cosine > 1) continue;
                double angle = Math.Acos(cosine);
                CheckAngle(centerAngle - angle); CheckAngle(centerAngle + angle);
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
        return new DrawingTrajectory(model, limits, shape, branch, samples, stops.ToArray(), shape == DrawingShape.Circle ? new Point(x, y) : null, nearSingularity);
    }

    internal const string CsvMetadataHeader = "kind,shape,l1_mm,l2_mm,branch,limits_status,q1_min_deg,q1_max_deg,q2_min_deg,q2_max_deg";
    internal string CsvMetadata
    {
        get
        {
            static string N(double v) => v.ToString("G17", CultureInfo.InvariantCulture);
            string range = Limits is { } l ? $"{N(l.Min1)},{N(l.Max1)},{N(l.Min2)},{N(l.Max2)}" : ",,,";
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
        foreach (var shape in Enum.GetValues<DrawingShape>())
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
            Check(rows.Length == path.Samples.Length + 1 && rows.All(row => row.Split(',').Length == 16));
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
        Check(limited.ToCsv().Contains("samples_checked,-90,90,0,160"));
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
    }
}
