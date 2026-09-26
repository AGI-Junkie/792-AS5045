using System.Windows;

namespace TwoLinkCommissioning;

public readonly record struct JointPose(double Q1, double Q2);
public readonly record struct JointLimits(double Min1, double Max1, double Min2, double Max2)
{
    public bool Contains(JointPose p) => p.Q1 >= Min1 - 1e-8 && p.Q1 <= Max1 + 1e-8 &&
                                         p.Q2 >= Min2 - 1e-8 && p.Q2 <= Max2 + 1e-8;
}

// Angles in degrees: CCW positive, both joints zero at the straight reference pose.
// This geometric model has no serial or motor-control dependency.
public sealed class PlanarArm
{
    public double L1 { get; }
    public double L2 { get; }
    public double InnerRadius => Math.Abs(L1 - L2);
    public double OuterRadius => L1 + L2;
    private const double Rad = Math.PI / 180;

    public PlanarArm(double l1, double l2)
    {
        if (!double.IsFinite(l1) || !double.IsFinite(l2) || l1 <= 0 || l2 <= 0 || l1 > 10000 || l2 > 10000)
            throw new ArgumentException("连杆长度须为 0～10000 mm 之间的正数。");
        L1 = l1; L2 = l2;
    }

    public (Point Elbow, Point Tip) Forward(JointPose p)
    {
        if (!double.IsFinite(p.Q1) || !double.IsFinite(p.Q2)) throw new ArgumentException("角度须为有限数值。");
        var elbow = new Point(L1 * Math.Cos(p.Q1 * Rad), L1 * Math.Sin(p.Q1 * Rad));
        return (elbow, new Point(elbow.X + L2 * Math.Cos((p.Q1 + p.Q2) * Rad),
                                elbow.Y + L2 * Math.Sin((p.Q1 + p.Q2) * Rad)));
    }

    public JointPose[] Inverse(double x, double y)
    {
        if (!double.IsFinite(x) || !double.IsFinite(y) || Math.Abs(x) > 20000 || Math.Abs(y) > 20000)
            throw new ArgumentException("目标坐标须为 ±20000 mm 内的有限数值。");
        double radius = Math.Sqrt(x * x + y * y);
        if (radius > OuterRadius + 1e-8 || radius < InnerRadius - 1e-8)
            throw new ArgumentException($"目标几何不可达：距原点 {radius:F2} mm，可达半径 {InnerRadius:F2}～{OuterRadius:F2} mm。");
        if (radius < 1e-8)
            throw new ArgumentException("等长连杆在原点完全折叠时解不唯一，请选择其他目标。");
        double c2 = Math.Clamp((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1);
        double angle = Math.Acos(c2);
        JointPose Solve(double q2) => new(Normalize((Math.Atan2(y, x) -
            Math.Atan2(L2 * Math.Sin(q2), L1 + L2 * Math.Cos(q2))) / Rad), q2 / Rad);
        // Keep +180/-180 distinct for joint-limit checks, even though their poses coincide.
        return angle < 1e-10 ? [Solve(angle)] : [Solve(angle), Solve(-angle)];
    }

    private static double Normalize(double degrees) => (degrees + 540) % 360 - 180;

    public static void SelfTest()
    {
        static void Check(bool condition) { if (!condition) throw new InvalidOperationException("Kinematics check failed"); }
        static void Reject(Action action)
        {
            try { action(); } catch (ArgumentException) { return; }
            throw new InvalidOperationException("Invalid kinematics input accepted");
        }
        var model = new PlanarArm(180, 176.38);
        Check(Math.Abs(model.OuterRadius - 356.38) < 1e-9);
        Check(Math.Abs(model.InnerRadius - 3.62) < 1e-9);
        foreach (var point in new[] { new Point(300, 0), new Point(100, 220), new Point(-100, -220),
                                     new Point(-300, 0), new Point(0, 100), new Point(356.38, 0), new Point(3.62, 0) })
        {
            foreach (var pose in model.Inverse(point.X, point.Y))
                Check((model.Forward(pose).Tip - point).Length < 1e-6);
        }
        var branches = model.Inverse(300, 0);
        Check(branches.Length == 2 && branches[0].Q2 > 0 && branches[1].Q2 < 0);
        var limits = new JointLimits(-90, 90, 0, 160);
        Check(limits.Contains(branches[0]) && !limits.Contains(branches[1]));
        Check(new JointLimits(-90, 90, -5, 5).Contains(new JointPose(-90, 5)));
        Reject(() => model.Inverse(357, 0)); Reject(() => model.Inverse(3, 0));
        Reject(() => model.Inverse(double.NaN, 0)); Reject(() => model.Inverse(double.PositiveInfinity, 0));
        Reject(() => new PlanarArm(0, 1)); Reject(() => new PlanarArm(double.NaN, 1));
        Reject(() => new PlanarArm(100, 100).Inverse(0, 0));
        var unequal = new PlanarArm(100, 200);
        Check((unequal.Forward(unequal.Inverse(100, 0)[0]).Tip - new Point(100, 0)).Length < 1e-6);
        Check(unequal.Inverse(100, 0).Length == 2);
    }
}
