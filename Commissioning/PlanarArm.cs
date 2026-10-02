using System.Windows;

namespace TwoLinkCommissioning;

public readonly record struct JointPose(double Q1, double Q2);
// q1 and q2 are absolute link angles, so motor 2 (the elbow) travels q2 - q1.  The two boxes alone let the elbow reach
// [Min2 - Max1, Max2 - Min1] (up to +/-360 deg), so the elbow has a range of its own: host planning, the reachable
// region, point moves and the firmware (type 9, axis 2) all test a pose against all three.
public readonly record struct JointLimits(double Min1, double Max1, double Min2, double Max2,
    double ElbowMin = -JointLimits.DefaultElbow, double ElbowMax = JointLimits.DefaultElbow)
{
    // Provisional (README): the value the firmware boots with, not a measured mechanical stroke.
    public const double DefaultElbow = DeviceContract.ElbowDefaultCd / 100.0;
    public static double Elbow(JointPose p) => p.Q2 - p.Q1;
    // (min, max) of one firmware range entry: 0 = q1, 1 = q2, 2 = elbow.  Q1 holds the minimum, Q2 the maximum.
    public JointPose Bounds(int axis) => axis switch { 0 => new(Min1, Max1), 1 => new(Min2, Max2), _ => new(ElbowMin, ElbowMax) };
    public bool Contains(JointPose p) => p.Q1 >= Min1 - 1e-8 && p.Q1 <= Max1 + 1e-8 &&
                                         p.Q2 >= Min2 - 1e-8 && p.Q2 <= Max2 + 1e-8 &&
                                         Elbow(p) >= ElbowMin - 1e-8 && Elbow(p) <= ElbowMax + 1e-8;
    // Room left to each elbow bound (degrees, negative once past it).
    public (double ToMin, double ToMax) ElbowMargins(JointPose p) => (Elbow(p) - ElbowMin, ElbowMax - Elbow(p));
}

// Angles in degrees from the fixed +X zero direction: q1 is link 1, q2 is link 2.
// Motor 2's elbow travel is q2 - q1; both angles are zero at the straight reference.
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
            throw Lang.Error("err.linkLength");
        L1 = l1; L2 = l2;
    }

    public (Point Elbow, Point Tip) Forward(JointPose p)
    {
        if (!double.IsFinite(p.Q1) || !double.IsFinite(p.Q2)) throw Lang.Error("err.angleFinite");
        var elbow = new Point(L1 * Math.Cos(p.Q1 * Rad), L1 * Math.Sin(p.Q1 * Rad));
        return (elbow, new Point(elbow.X + L2 * Math.Cos(p.Q2 * Rad),
                                elbow.Y + L2 * Math.Sin(p.Q2 * Rad)));
    }

    // Largest tip speed (mm/s) with |q1 rate| <= v1 and |q2 - q1 rate| <= v2 (deg/s), i.e. the two motor rates.
    // The tip velocity is convex in the rates, so a corner of the rate box is worst; the best corner has both motors
    // turning the same way with the arm straight: L1*v1 + L2*(v1 + v2).
    public double MaxTipSpeed(double v1, double v2) => (L1 * v1 + L2 * (v1 + v2)) * Rad;

    public JointPose[] Inverse(double x, double y)
    {
        if (!double.IsFinite(x) || !double.IsFinite(y) || Math.Abs(x) > 20000 || Math.Abs(y) > 20000)
            throw Lang.Error("err.targetFinite");
        double radius = Math.Sqrt(x * x + y * y);
        if (radius > OuterRadius + 1e-8 || radius < InnerRadius - 1e-8)
            throw Lang.Error("err.targetUnreachable", radius, InnerRadius, OuterRadius);
        if (radius < 1e-8)
            throw Lang.Error("err.equalFold");
        double c2 = Math.Clamp((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1);
        double angle = Math.Acos(c2);
        JointPose Solve(double elbow) {
            double q1 = Math.Atan2(y, x) - Math.Atan2(L2 * Math.Sin(elbow), L1 + L2 * Math.Cos(elbow));
            return new(Normalize(q1 / Rad), Normalize((q1 + elbow) / Rad));
        }
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
        // q2 is absolute, so its sign does not identify the elbow-up/down branch.
        Check(branches.Length == 2 && branches[0].Q2 > branches[0].Q1 && branches[1].Q2 < branches[1].Q1);
        var limits = new JointLimits(-90, 90, 0, 160);
        Check(limits.Contains(branches[0]) && !limits.Contains(branches[1]));
        Check(new JointLimits(-90, 90, -5, 5).Contains(new JointPose(-90, 5)));
        // The elbow is bounded on its own, whatever the two boxes allow: q1 = -90, q2 = +5 is an elbow of 95.
        var tight = new JointLimits(-90, 90, -160, 160, -10, 60);
        Check(tight.Contains(new JointPose(0, 60)) && tight.Contains(new JointPose(0, -10)) && tight.Contains(new JointPose(40, 100)));
        Check(!tight.Contains(new JointPose(0, 60.01)) && !tight.Contains(new JointPose(0, -10.01)) && !tight.Contains(new JointPose(-90, 5)));
        Check(Math.Abs(JointLimits.Elbow(branches[0])) > 60 && !tight.Contains(branches[0]) && !tight.Contains(branches[1])); // r = 300 needs an elbow of +/-65.4
        Check(new JointLimits(-30, 30, -60, 60).ElbowMax == JointLimits.DefaultElbow && JointLimits.DefaultElbow == 155);
        var (toMin, toMax) = tight.ElbowMargins(new JointPose(10, 40));
        Check(Math.Abs(toMin - 40) < 1e-9 && Math.Abs(toMax - 30) < 1e-9);
        var bent = model.Forward(new JointPose(30, 60));
        Check((bent.Elbow - new Point(180 * Math.Cos(30 * Rad), 90)).Length < 1e-8);
        Check((bent.Tip - new Point(bent.Elbow.X + 176.38 / 2, bent.Elbow.Y + 176.38 * Math.Sin(60 * Rad))).Length < 1e-8);
        Check(model.Inverse(bent.Tip.X, bent.Tip.Y).Any(p => Math.Abs(p.Q1 - 30) < 1e-8 && Math.Abs(p.Q2 - 60) < 1e-8));
        Reject(() => model.Inverse(357, 0)); Reject(() => model.Inverse(3, 0));
        Reject(() => model.Inverse(double.NaN, 0)); Reject(() => model.Inverse(double.PositiveInfinity, 0));
        Reject(() => new PlanarArm(0, 1)); Reject(() => new PlanarArm(double.NaN, 1));
        Reject(() => new PlanarArm(100, 100).Inverse(0, 0));
        var unequal = new PlanarArm(100, 200);
        Check((unequal.Forward(unequal.Inverse(100, 0)[0]).Tip - new Point(100, 0)).Length < 1e-6);
        Check(unequal.Inverse(100, 0).Length == 2);
        // Tip speed bound: no elbow angle and no corner of the rate box beats it, and the straight arm reaches it.
        double bound = model.MaxTipSpeed(10, 6), fastest = 0;
        for (int elbow = -180; elbow <= 180; elbow++)
            foreach (var (w1, w2) in new[] { (10.0, 6.0), (10.0, -6.0), (-10.0, 6.0), (-10.0, -6.0) })
            {
                const double h = 1e-5;
                var from = model.Forward(new JointPose(20, 20 + elbow)).Tip;
                var to = model.Forward(new JointPose(20 + w1 * h, 20 + elbow + (w1 + w2) * h)).Tip;
                fastest = Math.Max(fastest, (to - from).Length / h);
            }
        Check(Math.Abs(bound - fastest) < 1e-2 && Math.Abs(bound - (180 * 10 + 176.38 * 16) * Rad) < 1e-9);
    }
}
