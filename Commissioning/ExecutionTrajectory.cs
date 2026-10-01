namespace TwoLinkCommissioning;

public readonly record struct ExecutionNode(JointPose Pose, ushort Milliseconds);

// A finite, preloaded list. Every chord is checked in the same pulse units as firmware.
public sealed class ExecutionTrajectory
{
    public ExecutionNode[] Nodes { get; }
    public TimedTrajectory Timing { get; }
    public double Duration => Nodes.Sum(n => n.Milliseconds) / 1000.0;
    public ExecutionTrajectory(TimedTrajectory preview, JointLimits limits)
    {
        var profile = MotionProfile.From(preview.Rates);
        Timing = new TimedTrajectory(preview.Path, profile.DrawingRates(preview.Rates.Feed));
        // Quantized pulse chords must pass the real device bounds as well as IK.
        // Slow the same visible time plan when quantization needs extra margin.
        for (int attempt=0; ; attempt++) {
            try { Nodes=Build(Timing,limits,profile); return; }
            catch (ArgumentException ex) when(attempt<12 && (ex.Message.Contains("脉冲速度") || ex.Message.Contains("弦线近似"))) {
                Timing.Stretch(1.15);
            }
        }
    }
    private static ExecutionNode[] Build(TimedTrajectory timed, JointLimits limits, MotionProfile profile)
    {
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
                if (dt is < DeviceContract.TrajNodeMinMs or > DeviceContract.TrajNodeMaxMs || nodes.Count >= DeviceContract.TrajCapacity)
                    throw new ArgumentException($"执行轨迹超出 {DeviceContract.TrajCapacity} 点容量或时间间隔范围；请缩短路径。");
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
            if (nodes.Count >= DeviceContract.TrajCapacity) throw new ArgumentException($"执行轨迹超过 {DeviceContract.TrajCapacity} 点容量。");
            nodes.Add(new(nodes[^1].Pose, DeviceContract.TrajNodeMinMs));
        }
        var result=nodes.ToArray(); Validate(result, limits, profile); return result;
    }
    public static int Counts(int axis, JointPose pose)
    {
        var wire = PtpProtocol.Quantize(pose);
        return Counts(axis, axis == 0 ? wire.Q1 : wire.Q2 - wire.Q1);
    }
    public static int Counts(int axis, double angle) => (int)Math.Round(
        Math.Round(angle * 100, MidpointRounding.AwayFromZero) * (axis == 0 ? DeviceContract.Arm1StepsPerRev : DeviceContract.Arm2StepsPerRev) / 36000,
        MidpointRounding.AwayFromZero);
    private static JointPose PulsePose(JointPose p) {
        double q1=Counts(0,p)*360.0/DeviceContract.Arm1StepsPerRev;
        return new(q1,q1+Counts(1,p)*360.0/DeviceContract.Arm2StepsPerRev);
    }
    public static void Validate(ExecutionNode[] nodes, JointLimits limits, MotionProfile? profile = null)
    {
        if (nodes.Length is < 2 or > DeviceContract.TrajCapacity || nodes[0].Milliseconds != 0)
            throw new ArgumentException("执行轨迹容量或首点无效。");
        int[] previousVelocity = [0, 0]; int totalMs = 0;
        for (int i = 0; i < nodes.Length; i++)
        {
            var n = nodes[i];
            if (!double.IsFinite(n.Pose.Q1) || !double.IsFinite(n.Pose.Q2) ||
                Math.Abs(n.Pose.Q1) > 180 || Math.Abs(n.Pose.Q2) > 180 || !limits.Contains(n.Pose))
                throw new ArgumentException($"第 {i+1} 点超出当前关节范围：q1={n.Pose.Q1:F2}°，q2={n.Pose.Q2:F2}°；允许 q1 [{limits.Min1:F2},{limits.Max1:F2}]°、q2 [{limits.Min2:F2},{limits.Max2:F2}]°。请调整图形或关节范围。");
            if (i == 0) continue;
            int dt = n.Milliseconds, previousDt = i == 1 ? dt : nodes[i - 1].Milliseconds;
            if (dt is < DeviceContract.TrajNodeMinMs or > DeviceContract.TrajNodeMaxMs || (totalMs += dt) > DeviceContract.TrajTotalMaxMs) throw new ArgumentException("执行时间无效。");
            for (int axis = 0; axis < 2; axis++)
            {
                int maxRate = profile is { } p ? Math.Min(DeviceContract.PlanRateMax,axis==0?p.R1:p.R2) : DeviceContract.PlanRateMax;
                int maxAccel = profile is { } a ? Math.Min(DeviceContract.PlanAccelMax,axis==0?a.A1:a.A2) : DeviceContract.PlanAccelMax;
                int d = Counts(axis, n.Pose) - Counts(axis, nodes[i - 1].Pose);
                int velocity = d * 1000 / dt;
                if (Math.Abs(d) * 1000 > maxRate * dt || Math.Abs(velocity - previousVelocity[axis]) * 2000 > maxAccel * (dt + previousDt) ||
                    (i == nodes.Length - 1 && Math.Abs(velocity) * 2000 > maxAccel * dt))
                    throw new ArgumentException("执行轨迹超过脉冲速度或相邻段速度变化上限；请降低速度。");
                previousVelocity[axis] = velocity;
            }
        }
    }
    public static void SelfTest()
    {
        if (Counts(0,new JointPose(30,60))!=Counts(0,30) || Counts(1,new JointPose(30,60))!=Counts(1,30) ||
            Counts(1,new JointPose(0.006,0.014))!=0)
            throw new InvalidOperationException("Absolute q2 was not converted from wire-quantized link angles to elbow travel");
        var limits = new JointLimits(-90, 90, -160, 160);
        foreach (var shape in new[] { DrawingShape.Line, DrawingShape.Rectangle, DrawingShape.Circle })
        {
            var path = DrawingTrajectory.Create(new PlanarArm(180, 176.38), shape, 240, 10,
                shape == DrawingShape.Line ? 250 : 10, shape == DrawingShape.Circle ? 0 : 20, 0, limits);
            var plan = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(20, 5, 3, 10, 6)), limits);
            if (plan.Nodes.Length < 2 || plan.Duration <= 0) throw new InvalidOperationException("Execution plan missing");
            var slow=new ExecutionTrajectory(new TimedTrajectory(path,new PlanningRates(1,1,1,1,1)),limits);
            Validate(slow.Nodes,limits,MotionProfile.From(new PlanningRates(1,1,1,1,1)));
            if(slow.Duration<=plan.Duration || slow.Timing.Rates.Feed!=1 || slow.Timing.Path!=path)
                throw new InvalidOperationException("Global rates did not affect every shape");
        }
        var circle=DrawingTrajectory.Create(new PlanarArm(180,176.38),DrawingShape.Circle,260,0,40,0,0,limits);
        double Time(double v2)=>new ExecutionTrajectory(new TimedTrajectory(circle,new PlanningRates(40,8,v2,15,13)),limits).Duration;
        if(Time(2.3)>=Time(1.8)) throw new InvalidOperationException("Hidden 1.5 degree/s clamp still overrides global q2 speed");
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
