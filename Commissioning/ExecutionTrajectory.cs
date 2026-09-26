namespace TwoLinkCommissioning;

public readonly record struct ExecutionNode(JointPose Pose, ushort Milliseconds);

// A finite, preloaded list. Every chord is checked in the same pulse units as firmware.
public sealed class ExecutionTrajectory
{
    public ExecutionNode[] Nodes { get; }
    public double Duration => Nodes.Sum(n => n.Milliseconds) / 1000.0;
    public ExecutionTrajectory(TimedTrajectory preview, JointLimits limits)
    {
        var r = preview.Rates;
        var timed = new TimedTrajectory(preview.Path, new PlanningRates(Math.Min(r.Feed, 5),
            Math.Min(r.V1, 2), Math.Min(r.V2, 1.5), Math.Min(r.A1, 2), Math.Min(r.A2, 1.5)));
        var nodes = new List<ExecutionNode> { new(PtpProtocol.Quantize(timed.At(0).Pose), 0) };
        int previousTime = 0;
        for (int piece = 1; piece < timed.StopTimes.Length; piece++)
        {
            double start = timed.StopTimes[piece - 1], end = timed.StopTimes[piece];
            int count = Math.Max(1, (int)Math.Ceiling((end - start) / 0.25));
            for (int i = 1; i <= count; i++)
            {
                double time = start + (end - start) * i / count;
                int ms = (int)Math.Round(time * 1000);
                var pose = PtpProtocol.Quantize(timed.At(time).Pose);
                int dt = ms - previousTime;
                if (dt is < 50 or > 500 || nodes.Count >= 2048)
                    throw new ArgumentException("执行轨迹超出 2048 点容量或时间间隔范围；请缩短路径。");
                // Sample the Cartesian chord error after pulse quantization. This is a
                // planning tolerance, not a claim about measured pen tracking accuracy.
                var from = PulsePose(nodes[^1].Pose); var to = PulsePose(pose);
                for (int k = 0; k <= 8; k++)
                {
                    double u = k / 8.0;
                    var joint = new JointPose(from.Q1 + (to.Q1 - from.Q1) * u, from.Q2 + (to.Q2 - from.Q2) * u);
                    var expected = timed.At(previousTime / 1000.0 + (time - previousTime / 1000.0) * u).Target;
                    if ((timed.Path.Model.Forward(joint).Tip - expected).Length > 0.15)
                        throw new ArgumentException("执行轨迹的弦线近似误差超过 0.15 mm；请降低速度或调整图形。");
                }
                nodes.Add(new(pose, (ushort)dt)); previousTime = ms;
            }
            // Explicit dwell at rectangle corners and at the final endpoint.
            if (nodes.Count >= 2048) throw new ArgumentException("执行轨迹超过 2048 点容量。");
            nodes.Add(new(nodes[^1].Pose, 50));
        }
        Nodes = nodes.ToArray(); Validate(Nodes, limits);
    }
    public static int Counts(int axis, double angle) => (int)Math.Round(
        Math.Round(angle * 100, MidpointRounding.AwayFromZero) * (axis == 0 ? 64000 : 81696) / 36000,
        MidpointRounding.AwayFromZero);
    private static JointPose PulsePose(JointPose p) => new(Counts(0, p.Q1) * 360.0 / 64000, Counts(1, p.Q2) * 360.0 / 81696);
    public static void Validate(ExecutionNode[] nodes, JointLimits limits)
    {
        if (nodes.Length is < 2 or > 2048 || nodes[0].Milliseconds != 0)
            throw new ArgumentException("执行轨迹容量或首点无效。");
        int[] previousVelocity = [0, 0]; int totalMs = 0;
        for (int i = 0; i < nodes.Length; i++)
        {
            var n = nodes[i];
            if (!double.IsFinite(n.Pose.Q1) || !double.IsFinite(n.Pose.Q2) ||
                Math.Abs(n.Pose.Q1) > 180 || Math.Abs(n.Pose.Q2) > 180 || !limits.Contains(n.Pose))
                throw new ArgumentException($"第 {i+1} 点超出固件范围：q1={n.Pose.Q1:F2}°，q2={n.Pose.Q2:F2}°；允许 q1 [{limits.Min1:F2},{limits.Max1:F2}]°、q2 [{limits.Min2:F2},{limits.Max2:F2}]°。请缩小或移动图形。");
            if (i == 0) continue;
            int dt = n.Milliseconds, previousDt = i == 1 ? dt : nodes[i - 1].Milliseconds;
            if (dt is < 50 or > 500 || (totalMs += dt) > 600000) throw new ArgumentException("执行时间无效。");
            for (int axis = 0; axis < 2; axis++)
            {
                int d = Counts(axis, axis == 0 ? n.Pose.Q1 : n.Pose.Q2) -
                    Counts(axis, axis == 0 ? nodes[i - 1].Pose.Q1 : nodes[i - 1].Pose.Q2);
                int velocity = d * 1000 / dt;
                if (Math.Abs(d) * 1000 > 600 * dt || Math.Abs(velocity - previousVelocity[axis]) * 2000 > 800 * (dt + previousDt) ||
                    (i == nodes.Length - 1 && Math.Abs(velocity) * 2000 > 800 * dt))
                    throw new ArgumentException("执行轨迹超过脉冲速度或相邻段速度变化上限；请降低速度。");
                previousVelocity[axis] = velocity;
            }
        }
    }
    public static void SelfTest()
    {
        var limits = new JointLimits(-90, 90, -160, 160);
        foreach (var shape in new[] { DrawingShape.Line, DrawingShape.Rectangle, DrawingShape.Circle })
        {
            var path = DrawingTrajectory.Create(new PlanarArm(180, 176.38), shape, 240, 10,
                shape == DrawingShape.Line ? 250 : 10, shape == DrawingShape.Circle ? 0 : 20, 0, limits);
            var plan = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(20, 5, 3, 10, 6)), limits);
            if (plan.Nodes.Length < 2 || plan.Duration <= 0) throw new InvalidOperationException("Execution plan missing");
        }
        static void Reject(ExecutionNode[] nodes, JointLimits range)
        {
            try { Validate(nodes, range); } catch (ArgumentException) { return; }
            throw new InvalidOperationException("Unsafe execution plan accepted");
        }
        Reject([new(new(0, 0), 0), new(new(1, 0), 50)], limits);
        Reject([new(new(0, 0), 0), new(new(0, 0), 501)], limits);
        Reject([new(new(0, 0), 0), new(new(0, 170), 250)], limits);
        Reject([new(new(double.NaN, 0), 0), new(new(0, 0), 250)], limits);
    }
}
