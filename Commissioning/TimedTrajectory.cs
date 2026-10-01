using System.Globalization;
using System.Text;
using System.Windows;

namespace TwoLinkCommissioning;

public readonly record struct PlanningRates(double Feed, double V1, double V2, double A1, double A2);
public readonly record struct TimedPose(double Time, double Distance, Point Target, JointPose Pose,
    JointPose Velocity, JointPose Acceleration, double Speed);

// Offline timing estimate: analytic IK derivatives, sampled peak search, no hardware commands.
public sealed class TimedTrajectory
{
    public DrawingTrajectory Path { get; }
    public PlanningRates Rates { get; }
    public double[] StopTimes { get; }
    public double Duration => StopTimes[^1];
    public double Length => distances[^1];
    private readonly double[] distances;
    private const double Rad = Math.PI / 180;
    internal void Stretch(double factor)
    {
        if (!double.IsFinite(factor) || factor < 1 || Duration * factor > DeviceContract.TrajTotalMaxMs / 1000.0 - 1)
            throw new ArgumentException($"满足设备上限后的规划超过 {DeviceContract.TrajTotalMaxMs / 1000} 秒，请缩短路径或调整参数。");
        for (int i=1; i<StopTimes.Length; i++) StopTimes[i] *= factor;
    }

    public TimedTrajectory(DrawingTrajectory path, PlanningRates rates)
    {
        if (new[] { rates.Feed, rates.V1, rates.V2, rates.A1, rates.A2 }.Any(v => !double.IsFinite(v) || v < 0.01 || v > 1000))
            throw new ArgumentException("规划速度与加速度须为 0.01～1000 内的有限正数（不是驱动实测能力）。");
        Path = path; Rates = rates;
        StopTimes = new double[path.Stops.Length]; distances = new double[path.Stops.Length];
        for (int piece = 0; piece < StopTimes.Length - 1; piece++)
        {
            double duration = 0.02;
            // ponytail: 1001 normalized-time probes plus 5% margin estimate peaks;
            // replace with verified continuous bounds before using this planner for hardware execution.
            for (int i = 0; i <= 1000; i++)
            {
                var state = EvaluatePiece(piece, i / 1000.0, 1);
                duration = Math.Max(duration, state.Speed / rates.Feed);
                duration = Math.Max(duration, Math.Abs(state.Velocity.Q1) / rates.V1);
                duration = Math.Max(duration, Math.Abs(state.Velocity.Q2 - state.Velocity.Q1) / rates.V2);
                duration = Math.Max(duration, Math.Sqrt(Math.Abs(state.Acceleration.Q1) / rates.A1));
                duration = Math.Max(duration, Math.Sqrt(Math.Abs(state.Acceleration.Q2 - state.Acceleration.Q1) / rates.A2));
            }
            if (!double.IsFinite(duration) || StopTimes[piece] + duration * 1.05 > DeviceContract.TrajTotalMaxMs / 1000.0)
                throw new ArgumentException($"离线规划超过 {DeviceContract.TrajTotalMaxMs / 1000} 秒或接近数值奇异点；请调整图形或规划参数。");
            StopTimes[piece + 1] = StopTimes[piece] + duration * 1.05;
            distances[piece + 1] = distances[piece] + PieceLength(piece);
        }
    }

    private double PieceLength(int piece) => Path.CircleCenter is { } center ?
        2 * Math.PI * (Path.Samples[0].Target - center).Length :
        (Path.Samples[Path.Stops[piece + 1]].Target - Path.Samples[Path.Stops[piece]].Target).Length;

    private TimedPose EvaluatePiece(int piece, double z, double duration)
    {
        // Quintic progress: zero velocity and acceleration at both ends of every stroke.
        double u = z * z * z * (10 + z * (-15 + 6 * z));
        double du = 30 * z * z * (1 - z) * (1 - z) / duration;
        double ddu = 60 * z * (1 - z) * (1 - 2 * z) / (duration * duration);
        Point target; Vector dp, ddp;
        if (Path.CircleCenter is { } center)
        {
            double radius = (Path.Samples[0].Target - center).Length;
            double angle = 2 * Math.PI * u;
            Vector radial = new(radius * Math.Cos(angle), radius * Math.Sin(angle));
            target = center + radial;
            dp = new Vector(-radial.Y, radial.X) * (2 * Math.PI);
            ddp = -radial * (4 * Math.PI * Math.PI);
        }
        else
        {
            Point start = Path.Samples[Path.Stops[piece]].Target, end = Path.Samples[Path.Stops[piece + 1]].Target;
            dp = end - start; ddp = new Vector(); target = start + dp * u;
        }
        var poses = Path.Model.Inverse(target.X, target.Y);
        JointPose pose = poses[Math.Min(Path.Branch, poses.Length - 1)];
        int reference = (int)Math.Round(Path.Stops[piece] + (Path.Stops[piece + 1] - Path.Stops[piece]) * u);
        pose = pose with {
            Q1 = pose.Q1 + 360 * Math.Round((Path.Samples[reference].Pose.Q1 - pose.Q1) / 360),
            Q2 = pose.Q2 + 360 * Math.Round((Path.Samples[reference].Pose.Q2 - pose.Q2) / 360)
        };
        if (Path.Limits is { } limits && !limits.Contains(pose))
            throw new ArgumentException("时间规划采样点超出填写的关节范围，请调整图形或分支。");
        double q1 = pose.Q1 * Rad, q2 = pose.Q2 * Rad;
        double c1 = Path.Model.L1 * Math.Cos(q1), s1 = Path.Model.L1 * Math.Sin(q1);
        double c2 = Path.Model.L2 * Math.Cos(q2), s2 = Path.Model.L2 * Math.Sin(q2);
        double determinant = Path.Model.L1 * Path.Model.L2 * Math.Sin(q2 - q1);
        if (Math.Abs(Math.Sin(q2 - q1)) < 1e-6) throw new ArgumentException("时间规划经过奇异位形。");
        Vector Solve(Vector value) => new((value.X * c2 + s2 * value.Y) / determinant,
            (-s1 * value.Y - value.X * c1) / determinant);
        Vector first = Solve(dp);
        Vector second = Solve(ddp + new Vector(c1 * first.X * first.X + c2 * first.Y * first.Y,
            s1 * first.X * first.X + s2 * first.Y * first.Y));
        Vector velocity = first * (du / Rad), acceleration = (second * (du * du) + first * ddu) / Rad;
        return new TimedPose(z * duration, PieceLength(piece) * u, target, pose,
            new JointPose(velocity.X, velocity.Y), new JointPose(acceleration.X, acceleration.Y), dp.Length * du);
    }

    public TimedPose At(double time)
    {
        if (!double.IsFinite(time)) throw new ArgumentException("预览时间须为有限数值。");
        time = Math.Clamp(time, 0, Duration);
        int piece = Array.BinarySearch(StopTimes, time);
        piece = piece < 0 ? ~piece - 1 : Math.Min(piece, StopTimes.Length - 2);
        double duration = StopTimes[piece + 1] - StopTimes[piece];
        var state = EvaluatePiece(piece, (time - StopTimes[piece]) / duration, duration);
        return state with { Time = time, Distance = distances[piece] + state.Distance };
    }

    public string ToCsv()
    {
        static string N(double v) => v.ToString("G17", CultureInfo.InvariantCulture);
        var csv = new StringBuilder(DrawingTrajectory.CsvMetadataHeader +
            ",timing_status,feed_limit_mm_s,motor1_speed_limit_deg_s,motor2_speed_limit_deg_s,motor1_accel_limit_deg_s2,motor2_accel_limit_deg_s2,time_s,path_mm,x_mm,y_mm,q1_deg,q2_deg,q1_speed_deg_s,q2_speed_deg_s,q1_accel_deg_s2,q2_accel_deg_s2,tip_speed_mm_s\r\n");
        string metadata = Path.CsvMetadata + $",sampled_estimate,{N(Rates.Feed)},{N(Rates.V1)},{N(Rates.V2)},{N(Rates.A1)},{N(Rates.A2)},";
        // Include exact corners/endpoints as well as the nominal 20 ms export grid.
        var times = new SortedSet<double>(StopTimes);
        for (int i = 0; i * 0.02 < Duration; i++) times.Add(i * 0.02);
        foreach (double time in times)
        {
            var s = At(time);
            csv.Append(metadata).AppendJoin(',', new[] { s.Time, s.Distance, s.Target.X, s.Target.Y, s.Pose.Q1, s.Pose.Q2,
                s.Velocity.Q1, s.Velocity.Q2, s.Acceleration.Q1, s.Acceleration.Q2, s.Speed }.Select(N)).Append("\r\n");
        }
        return csv.ToString();
    }

    public static void SelfTest()
    {
        static void Check(bool value, string reason) { if (!value) throw new InvalidOperationException("Timing check: " + reason); }
        static void Reject(Action action)
        { try { action(); } catch (ArgumentException) { return; } throw new InvalidOperationException("Invalid timing accepted"); }
        var model = new PlanarArm(180, 176.38);
        var rates = new PlanningRates(20, 5, 3, 10, 6);
        foreach (int branch in new[] { 0, 1 })
        foreach (var shape in Enum.GetValues<DrawingShape>())
        {
            var path = DrawingTrajectory.Create(model, shape, 220, 20, shape == DrawingShape.Line ? 250 : 30, 40, branch);
            var motion = new TimedTrajectory(path, rates);
            Check(motion.Duration > 0 && motion.Duration < 600, "duration");
            foreach (double stop in motion.StopTimes)
            {
                var s = motion.At(stop);
                Check(Math.Abs(s.Velocity.Q1) + Math.Abs(s.Velocity.Q2) + Math.Abs(s.Acceleration.Q1) + Math.Abs(s.Acceleration.Q2) < 1e-8, "stop derivatives");
            }
            // Different, denser sampling than the planner's probes checks the resulting profile.
            for (int i = 0; i <= 3000; i++)
            {
                var s = motion.At(motion.Duration * i / 3000);
                Check((model.Forward(s.Pose).Tip - s.Target).Length < 1e-6, "FK");
                Check(Math.Abs(s.Velocity.Q1) <= rates.V1 && Math.Abs(s.Velocity.Q2 - s.Velocity.Q1) <= rates.V2 && s.Speed <= rates.Feed, "speed limits");
                Check(Math.Abs(s.Acceleration.Q1) <= rates.A1 && Math.Abs(s.Acceleration.Q2 - s.Acceleration.Q1) <= rates.A2, "acceleration limits");
            }
            // Independently differentiate returned joint positions to verify derivative formulas.
            double t = motion.StopTimes[1] * 0.37, dt = 0.0001;
            var before = motion.At(t - dt); var now = motion.At(t); var after = motion.At(t + dt);
            Check(Math.Abs((after.Pose.Q1 - before.Pose.Q1) / (2 * dt) - now.Velocity.Q1) < 1e-5, "q1 velocity derivative");
            Check(Math.Abs((after.Pose.Q2 - before.Pose.Q2) / (2 * dt) - now.Velocity.Q2) < 1e-5, "q2 velocity derivative");
            Check(Math.Abs((after.Velocity.Q1 - before.Velocity.Q1) / (2 * dt) - now.Acceleration.Q1) < 1e-5, "q1 acceleration derivative");
            Check(Math.Abs((after.Velocity.Q2 - before.Velocity.Q2) / (2 * dt) - now.Acceleration.Q2) < 1e-5, "q2 acceleration derivative");
            Check((motion.At(0).Target - path.Samples[0].Target).Length < 1e-8 &&
                (motion.At(motion.Duration).Target - path.Samples[^1].Target).Length < 1e-8, "endpoints");
            var slower = new TimedTrajectory(path, new PlanningRates(10, 2.5, 1.5, 2.5, 1.5));
            Check(Math.Abs(slower.Duration - 2 * motion.Duration) < 1e-7, "time scaling");
            var rows = motion.ToCsv().Split("\r\n", StringSplitOptions.RemoveEmptyEntries);
            Check(rows.All(r => r.Split(',').Length == 27), "CSV columns");
        }
        var circle = DrawingTrajectory.Create(model, DrawingShape.Circle, 0, 0, 100, 0, 0);
        var turn = new TimedTrajectory(circle, rates);
        Check(Math.Abs(turn.At(turn.Duration).Pose.Q1 - turn.At(0).Pose.Q1 - 360) < 1e-7, "whole-turn continuity");
        Check(Math.Abs(turn.Length - 200 * Math.PI) < 1e-8, "analytic circle length");
        Reject(() => new TimedTrajectory(circle, rates with { V1 = 0 }));
        Reject(() => new TimedTrajectory(circle, rates with { Feed = double.NaN }));
        Reject(() => new TimedTrajectory(circle, rates with { Feed = 0.01 }));
        Reject(() => turn.At(double.NaN));
    }
}
