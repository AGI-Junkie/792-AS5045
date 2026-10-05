using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace TwoLinkCommissioning;

// This layer describes geometric reachability, not firmware permission or collision clearance.
internal sealed class ReachabilityMask
{
    private const int Pixels = 512;
    private readonly PlanarArm model;
    private readonly JointLimits? limits;
    private BitmapSource? bitmap;

    private ReachabilityMask(PlanarArm model, JointLimits? limits)
    {
        this.model = model;
        this.limits = limits;
    }

    public static ReachabilityMask Build(PlanarArm model, JointLimits? limits) => new(model, limits);
    public double ViewRadius => model.OuterRadius;
    public bool Matches(PlanarArm arm, JointLimits? range) =>
        model.L1 == arm.L1 && model.L2 == arm.L2 && limits == range;
    public BitmapSource Bitmap => bitmap ??= CreateBitmap();

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

    private Point PixelCenter(int x, int y) => new(
        ((x + 0.5) / Pixels * 2 - 1) * ViewRadius,
        (1 - (y + 0.5) / Pixels * 2) * ViewRadius);

    private BitmapSource CreateBitmap()
    {
        // ponytail: fixed world raster approximates boundaries to one pixel; increase resolution if closer zoom needs it.
        // Cache once per lengths/range, so telemetry and camera changes never rerun inverse kinematics for this layer.
        var pixels = new byte[Pixels * Pixels * 4];
        for (int y = 0; y < Pixels; y++)
            for (int x = 0; x < Pixels; x++)
            {
                if (IsReachable(PixelCenter(x, y))) continue;
                int index = (y * Pixels + x) * 4;
                pixels[index] = 0x38; pixels[index + 1] = 0x23;
                pixels[index + 2] = 0xB4; pixels[index + 3] = 0x38;
            }
        var result = BitmapSource.Create(Pixels, Pixels, 96, 96, PixelFormats.Bgra32, null, pixels, Pixels * 4);
        result.Freeze();
        return result;
    }

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

        var image = mask.Bitmap;
        Check(image.IsFrozen && ReferenceEquals(image, mask.Bitmap), "遮罩缓存必须复用冻结图像");
        var bytes = new byte[Pixels * Pixels * 4];
        image.CopyPixels(bytes, Pixels * 4, 0);
        int red = 0, transparent = 0;
        for (int y = 0; y < Pixels; y += 17)
            for (int x = 0; x < Pixels; x += 17)
            {
                int index = (y * Pixels + x) * 4;
                bool reachable = mask.IsReachable(mask.PixelCenter(x, y));
                Check(bytes[index + 3] == (reachable ? 0 : 0x38), "像素颜色与精确点判定一致");
                if (reachable) transparent++;
                else { red++; Check(bytes[index + 2] == 0xB4 && bytes[index + 1] == 0x23, "不可达像素为透明红色"); }
            }
        Check(red > 0 && transparent > 0, "同时覆盖可达与不可达像素");
    }
}
