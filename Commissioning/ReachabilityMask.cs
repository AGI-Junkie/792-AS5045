using System.Windows;
using System.Windows.Media;

namespace TwoLinkCommissioning;

// The straight link boundary is a visual reference; exact tip IK remains a separate check.
internal sealed class ReachabilityMask
{
    private readonly PlanarArm model;
    private readonly JointLimits? limits;
    private Geometry? linkBoundary;

    private ReachabilityMask(PlanarArm model, JointLimits? limits)
    {
        this.model = model;
        this.limits = limits;
    }

    public static ReachabilityMask Build(PlanarArm model, JointLimits? limits) => new(model, limits);
    public bool Matches(PlanarArm arm, JointLimits? range) =>
        model.L1 == arm.L1 && model.L2 == arm.L2 && limits == range;
    public Geometry LinkBoundary => linkBoundary ??= CreateLinkBoundary();

    public bool IsWithinLinkBoundary(Point point) => double.IsFinite(point.X) && double.IsFinite(point.Y) &&
        LinkBoundary.FillContains(point, 1e-8, ToleranceType.Absolute);

    private Geometry CreateLinkBoundary()
    {
        if (limits is not { } range)
        {
            var circle = new EllipseGeometry(new Point(), model.OuterRadius, model.OuterRadius);
            circle.Freeze();
            return circle;
        }
        // The editor accepts ordered ranges containing zero, within +/-180 degrees.
        for (int axis = 0; axis < 3; axis++)
        {
            var bounds = range.Bounds(axis);
            if (!double.IsFinite(bounds.Q1) || !double.IsFinite(bounds.Q2) || bounds.Q1 < -180 ||
                bounds.Q2 > 180 || bounds.Q1 > 0 || bounds.Q2 < 0 || bounds.Q1 >= bounds.Q2)
                return Geometry.Empty;
        }
        double low1 = Math.Max(range.Min1, range.Min2 - range.ElbowMax);
        double high1 = Math.Min(range.Max1, range.Max2 - range.ElbowMin);
        var low = model.Forward(new JointPose(low1, Math.Max(range.Min2, low1 + range.ElbowMin)));
        var high = model.Forward(new JointPose(high1, Math.Min(range.Max2, high1 + range.ElbowMax)));
        Point Outer(Point tip, Point elbow)
        {
            Vector direction = tip - elbow;
            direction.Normalize();
            double along = Vector.Multiply((Vector)elbow, direction);
            double distance = -along + Math.Sqrt(along * along + model.OuterRadius * model.OuterRadius - model.L1 * model.L1);
            return elbow + direction * distance;
        }
        var outline = new StreamGeometry();
        using (var context = outline.Open())
        {
            context.BeginFigure(new Point(), true, true);
            context.LineTo(low.Elbow, true, false);
            // Continue the elbow-to-tip line to the outer circle without adding another corner.
            context.LineTo(Outer(low.Tip, low.Elbow), true, false);
            var radius = new Size(model.OuterRadius, model.OuterRadius);
            // Split at +X so the full +/-180 range also produces a complete circle.
            context.ArcTo(new Point(model.OuterRadius, 0), radius, 0, false, SweepDirection.Clockwise, true, false);
            context.ArcTo(Outer(high.Tip, high.Elbow), radius, 0, false, SweepDirection.Clockwise, true, false);
            context.LineTo(high.Elbow, true, false);
        }
        outline.Freeze();
        return outline;
    }

    public bool IsReachable(Point tip)
    {
        if (!double.IsFinite(tip.X) || !double.IsFinite(tip.Y)) return false;
        double radius = Math.Sqrt(tip.X * tip.X + tip.Y * tip.Y);
        if (radius > model.OuterRadius + 1e-8 || radius < model.InnerRadius - 1e-8) return false;
        if (limits is not { } range) return true;
        if (!double.IsFinite(range.Min1) || !double.IsFinite(range.Max1) ||
            !double.IsFinite(range.Min2) || !double.IsFinite(range.Max2) ||
            range.Min1 > range.Max1 || range.Min2 > range.Max2) return false;

        // Equal links at the origin have infinitely many q1 solutions; Inverse deliberately rejects this singular point.
        if (radius < 1e-8 && model.L1 == model.L2)
            return range.Contains(new JointPose(range.Min1, Equivalent(range.Min1 + 180, range.Min2)));
        foreach (var pose in model.Inverse(tip.X, tip.Y))
            if (range.Contains(pose) || range.Contains(new JointPose(
                Equivalent(pose.Q1, range.Min1), Equivalent(pose.Q2, range.Min2)))) return true;
        return false;
    }

    // Preserve equivalent boundary poses, including q1 = +180 versus Inverse's normalized -180.
    private static double Equivalent(double angle, double minimum) =>
        angle + 360 * Math.Ceiling((minimum - angle - 1e-8) / 360);

    public static void SelfTest()
    {
        static void Check(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException("可达区域检查失败：" + message);
        }
        var arm = new PlanarArm(180, 176.38);
        var defaults = new JointLimits(-30, 30, -60, 60);
        var mask = Build(arm, defaults);
        Check(mask.IsReachable(new Point(309, 0)), "r=309 的两支逆解均在默认范围内");
        Check(!mask.IsReachable(new Point(300, 0)), "r=300 超出默认 q2 范围");
        Check(mask.IsReachable(new Point(arm.OuterRadius, 0)), "外圆伸直边界可达");
        Check(!mask.IsReachable(new Point(arm.OuterRadius + 0.1, 0)), "外圆之外不可达");
        Check(!mask.IsReachable(new Point(0, 350)), "q1 限位必须生效");
        Check(!mask.IsReachable(new Point(double.NaN, 0)), "拒绝非有限坐标");

        foreach (double q2 in new[] { -60.0, 60.0 })
        {
            var range = new JointLimits(19, 21, q2 - 1, q2 + 1);
            var asymmetric = Build(arm, range);
            Check(asymmetric.IsReachable(arm.Forward(new JointPose(20, q2)).Tip), "非对称范围、正负逆解");
            Check(!asymmetric.IsReachable(arm.Forward(new JointPose(-20, q2)).Tip), "不可镜像 q1 范围");
        }
        Check(Build(arm, new JointLimits(20, 20.01, 59.99, 60)).IsReachable(
            arm.Forward(new JointPose(20, 60)).Tip), "窄范围和限位角点");
        Check(Build(arm, new JointLimits(179, 181, 179, 181)).IsReachable(new Point(-arm.OuterRadius, 0)), "q1 跨 180°");
        Check(Build(arm, new JointLimits(359, 361, 299, 301)).IsReachable(
            arm.Forward(new JointPose(0, -60)).Tip), "跨 360° 等价角");
        Check(Build(arm, new JointLimits(-1, 1, 179, 180, -180, 180)).IsReachable(
            arm.Forward(new JointPose(0, 180)).Tip), "正 180° 折叠边界");
        Check(Build(arm, new JointLimits(-1, 1, -180, -179, -180, 180)).IsReachable(
            arm.Forward(new JointPose(0, -180)).Tip), "负 180° 折叠边界");
        Check(!Build(arm, new JointLimits(-1, 1, 179, 180)).IsReachable(
            arm.Forward(new JointPose(0, 180)).Tip), "默认肘范围 ±155° 不含 180° 折叠");
        var equal = new PlanarArm(100, 100);
        Check(Build(equal, new JointLimits(40, 50, -140, -130, -180, 180)).IsReachable(new Point()), "等长连杆原点有合法折叠解");
        Check(!Build(equal, new JointLimits(40, 50, -140, -130)).IsReachable(new Point()), "原点折叠要求肘 ±180°，默认肘范围不含");
        Check(!Build(equal, defaults).IsReachable(new Point()), "等长原点仍受 q2 限制");
        var free = Build(arm, null);
        Check(free.IsReachable(new Point(arm.InnerRadius, 0)), "几何内圆边界");
        Check(!free.IsReachable(new Point(arm.InnerRadius - 0.1, 0)), "内圆盲区");
        Check(!mask.Matches(new PlanarArm(181, 176.38), defaults), "杆长改变使缓存失效");
        Check(!mask.Matches(arm, new JointLimits(-31, 30, -60, 60)), "范围改变使缓存失效");
        Check(!mask.Matches(arm, defaults with { ElbowMax = 90 }), "肘范围改变使缓存失效");
        // The elbow range alone shapes the region: |elbow| = 90 at r = sqrt(L1^2 + L2^2) = 252, 5.3 deg at r = 356.
        var wide = new JointLimits(-180, 180, -180, 180);
        Check(Build(arm, wide with { ElbowMin = 0, ElbowMax = 90 }).IsReachable(new Point(300, 0)), "肘 [0,90]：r=300 取 +65° 分支");
        Check(!Build(arm, wide with { ElbowMin = 0, ElbowMax = 90 }).IsReachable(new Point(240, 0)), "肘 [0,90]：r=240 需要 >90°");
        Check(Build(arm, wide with { ElbowMin = -90, ElbowMax = 0 }).IsReachable(new Point(300, 0)), "肘 [-90,0]：取另一分支");
        Check(Build(arm, wide with { ElbowMin = 10, ElbowMax = 90 }).IsReachable(new Point(350, 0)), "肘 [10,90]：r=350 约 22°");
        Check(!Build(arm, wide with { ElbowMin = 10, ElbowMax = 90 }).IsReachable(new Point(356, 0)), "肘 [10,90]：接近伸直（<10°）不可达");
        Check(!Build(arm, wide with { ElbowMin = -20, ElbowMax = 20 }).IsReachable(new Point(300, 0)), "肘 ±20°：r=300 需要 65°");

        Check(mask.LinkBoundary.IsFrozen && ReferenceEquals(mask.LinkBoundary, mask.LinkBoundary), "复用冻结的直线边界");
        Check(mask.IsWithinLinkBoundary(new Point(100, 0)) && !mask.IsReachable(new Point(100, 0)), "连杆可经过不等于笔尖可达");
        Check(!mask.IsWithinLinkBoundary(new Point(double.NaN, 0)), "边界拒绝非有限点");
        Check(!mask.IsWithinLinkBoundary(new Point(arm.OuterRadius + 0.1, 0)), "保留最大伸展半径");
        var linkRange = new JointLimits(-100, 100, -150, 150, -180, 180);
        var links = Build(arm, linkRange);
        Check(links.IsWithinLinkBoundary(new Point(0, 100)) && !links.IsReachable(new Point(0, 100)), "旧圆弧红区内的连杆空间应留白");
        Check(!links.IsWithinLinkBoundary(new Point(-100, 0)), "极限连杆后方仍为红区");
        foreach (var pose in new[] { new JointPose(-100, -150), new JointPose(100, 150) })
        {
            var (elbow, tip) = arm.Forward(pose);
            foreach (var (from, to) in new[] { (new Point(), elbow), (elbow, tip) })
                foreach (double fraction in new[] { 0.2, 0.5, 0.8 })
                {
                    Point edge = from + (to - from) * fraction;
                    Check(links.IsWithinLinkBoundary(edge + new Vector(0.1, 0)), "极限端点连线右侧应留白");
                    Check(!links.IsWithinLinkBoundary(edge - new Vector(0.1, 0)), "极限端点连线左侧应标红");
                }
        }
        var asymmetricLinks = Build(arm, linkRange with { Min1 = -30, Min2 = -60 });
        Check(asymmetricLinks.IsWithinLinkBoundary(new Point(0, 100)) &&
            !asymmetricLinks.IsWithinLinkBoundary(new Point(0, -100)), "直线边界不可镜像非对称范围");
        var full = Build(arm, new JointLimits(-180, 180, -180, 180, -180, 180));
        foreach (var point in new[] { new Point(0, 100), new Point(0, -100), new Point(-100, 0), new Point(100, 0), new Point() })
            Check(full.IsWithinLinkBoundary(point), "全角范围覆盖整个圆盘，包括连杆可经过的内圆");
        Check(free.IsWithinLinkBoundary(new Point()), "无范围时连杆空间包含基座");
        Check(!Build(arm, defaults with { Min1 = double.NaN }).IsWithinLinkBoundary(new Point(100, 0)), "无效范围不显示可用区域");
        foreach (var (range, low, high) in new[] {
            (linkRange with { ElbowMin = -20, ElbowMax = 30 }, new JointPose(-100,-120), new JointPose(100,130)),
            (new JointLimits(-100,100,-10,10,-20,30), new JointPose(-40,-10), new JointPose(30,10)) })
            foreach (var pose in new[] { low, high })
            {
                var (elbow, tip) = arm.Forward(pose);
                Point edge = elbow + (tip-elbow)*0.5;
                var bounded = Build(arm, range);
                Check(range.Contains(pose) && bounded.IsWithinLinkBoundary(edge+new Vector(0.1,0)) &&
                    !bounded.IsWithinLinkBoundary(edge-new Vector(0.1,0)), "肘限制必须约束极限连线的两根杆");
            }
    }
}
