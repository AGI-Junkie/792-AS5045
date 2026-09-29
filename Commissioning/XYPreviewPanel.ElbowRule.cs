using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;
using Path = System.IO.Path;

namespace TwoLinkCommissioning;

// Live view of the operator rule in README "关节范围填写规则". With q2 = absolute link angle the elbow travels
// q2 - q1, and firmware/host only enforce the q1 and q2 boxes, so the elbow band they imply must be kept inside the
// safe travel E by hand. This shows that band, the box, and the part of the box that breaks the rule, updating as
// the fields change. Display only: it never blocks a sync and never invalidates the plan.
//
// Cost: the plot tree is created once and updated in place, and updates run 60 ms after the last edit at
// Background priority. Rebuilding ~25 elements on every keystroke made the preview-edit replay show 230 ms UI stalls
// (input callback and UI lease gap) that the panel without this view never had: 0 of 24 replays before, 5 of 24 with it.
public sealed partial class XYPreviewPanel
{
    // Provisional safe elbow travel from straight, per side, in degrees. NOT a measured mechanical range.
    // Deliberately not saved: parameters.json has a fixed 15-field schema that older hosts also read.
    private const string DefaultElbowTravel = "155";
    private const double PlotSize = 220;
    private static readonly Brush ElbowOk = WorkbenchTheme.Brush("#1E7B3A");
    private readonly TextBox elbowRight = Field(DefaultElbowTravel), elbowLeft = Field(DefaultElbowTravel);
    private readonly TextBlock elbowState = new() { FontSize = 12, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4) };
    private readonly Canvas elbowPlot = new() { Width = PlotSize, Height = PlotSize, Background = Brushes.White, ClipToBounds = true,
        HorizontalAlignment = HorizontalAlignment.Left, Margin = new Thickness(4) };
    private readonly Polygon elbowLegal = new() { Fill = WorkbenchTheme.Brush("#3327AE60"), Visibility = Visibility.Collapsed };
    private readonly Polygon elbowBox = new() { Fill = WorkbenchTheme.Brush("#14245BE8"), Stroke = WorkbenchTheme.Blue, StrokeThickness = 1.5, Visibility = Visibility.Collapsed };
    private readonly Polygon[] elbowBad = [
        new() { Fill = WorkbenchTheme.Brush("#99B42338"), Tag = "elbow-illegal", Visibility = Visibility.Collapsed },
        new() { Fill = WorkbenchTheme.Brush("#99B42338"), Tag = "elbow-illegal", Visibility = Visibility.Collapsed }];
    private readonly Polyline elbowPath = new() { Stroke = WorkbenchTheme.Ink, StrokeThickness = 1.5, Visibility = Visibility.Collapsed };
    private readonly Ellipse elbowDot = new() { Width = 6, Height = 6, Fill = WorkbenchTheme.Ink, Visibility = Visibility.Collapsed };
    private readonly DispatcherTimer elbowTimer = new(DispatcherPriority.Background) { Interval = TimeSpan.FromMilliseconds(60) };
    private string? elbowSignature;
    private bool elbowPending;
    // The black line follows the plan's own rule: recomputed by Refresh() (preview / restore), not on every keystroke.
    private JointPose[] elbowNeeded = [];
    private int elbowPathRevision;

    private static double PX(double q1) => (q1 + 180) * PlotSize / 360;
    private static double PY(double q2) => (180 - q2) * PlotSize / 360;
    private static PointCollection Pixels(IEnumerable<Point> degrees)
    {
        var points = new PointCollection(degrees.Select(p => new Point(PX(p.X), PY(p.Y))));
        points.Freeze();
        return points;
    }

    private FrameworkElement BuildElbowRulePanel()
    {
        var panel = new StackPanel();
        var travel = new WrapPanel { Margin = new Thickness(0, 6, 0, 0) };
        travel.Children.Add(WorkbenchTheme.Text("肘安全行程 E / °", 12));
        foreach (var (side, field) in new[] { ("右", elbowRight), ("左", elbowLeft) })
        {
            field.Width = 56;
            System.Windows.Automation.AutomationProperties.SetName(field, "肘安全行程 " + side);
            field.ToolTip = $"肘（电机 2）从伸直位向{side}能安全转过的角度。未测量前按 {DefaultElbowTravel}°；只影响此处显示，不保存，重启后恢复默认。";
            var pair = new StackPanel { Orientation = Orientation.Horizontal };
            pair.Children.Add(new TextBlock { Text = side, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(4, 0, 0, 0) });
            pair.Children.Add(field); travel.Children.Add(pair);
            field.TextChanged += (_, _) => RefreshElbowRule();
        }

        // Static part of the plot: created once.
        void GridLine(double q, bool axis)
        {
            var stroke = axis ? WorkbenchTheme.Brush("#B8C4D6") : WorkbenchTheme.Line;
            elbowPlot.Children.Add(new System.Windows.Shapes.Line { X1 = PX(q), Y1 = 0, X2 = PX(q), Y2 = PlotSize, Stroke = stroke, StrokeThickness = 1 });
            elbowPlot.Children.Add(new System.Windows.Shapes.Line { X1 = 0, Y1 = PY(q), X2 = PlotSize, Y2 = PY(q), Stroke = stroke, StrokeThickness = 1 });
        }
        void Label(string text, double left, double top)
        {
            var t = new TextBlock { Text = text, FontSize = 9, Foreground = WorkbenchTheme.Muted };
            Canvas.SetLeft(t, left); Canvas.SetTop(t, top); elbowPlot.Children.Add(t);
        }
        foreach (double q in new[] { -180.0, -90.0, 90.0, 180.0 }) GridLine(q, false);
        GridLine(0, true);
        Label("q1 →", PlotSize - 26, PlotSize / 2 + 1); Label("q2 ↑", PlotSize / 2 + 3, 1); Label("0", PlotSize / 2 + 2, PlotSize / 2 + 1);
        foreach (double q in new[] { -90.0, 90.0 })
        {
            string text = q.ToString("0", CultureInfo.InvariantCulture);
            Label(text, PX(q) - 8, PY(0) + 1); Label(text, PX(0) + 2, PY(q) - 6);
        }
        foreach (var element in ElbowShapes) elbowPlot.Children.Add(element);   // z-order: strip, box, red, path, dot
        elbowTimer.Tick += (_, _) => { elbowTimer.Stop(); UpdateElbowView(); };

        System.Windows.Automation.AutomationProperties.SetName(elbowPlot, "肘行程规则图");
        panel.Children.Add(travel); panel.Children.Add(elbowState); panel.Children.Add(elbowPlot);
        panel.Children.Add(WorkbenchTheme.Text("横轴 q1、纵轴 q2（−180°～180°）。蓝框＝固件允许的范围（你填的）；绿带＝肘在安全行程内；" +
            "红＝固件允许、但肘会超出安全行程；黑线＝上次「检查并预览」时图形需要的关节角。固件不检查肘行程，红色部分要靠你缩小范围。", 11, WorkbenchTheme.Muted));
        return panel;
    }

    private Shape[] ElbowShapes => [elbowLegal, elbowBox, elbowBad[0], elbowBad[1], elbowPath, elbowDot];

    private void CaptureElbowPath() { elbowNeeded = NeededJointPath(); elbowPathRevision++; }

    private static bool TryTravel(TextBox field, out double value) =>
        double.TryParse(field.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) && double.IsFinite(value) && value > 0 && value <= 360;
    private static string Deg(double v) => v.ToString("+0.##;-0.##;0", CultureInfo.InvariantCulture) + "°";
    private static string Mag(double v) => v.ToString("0.##", CultureInfo.InvariantCulture) + "°";

    // Joint angles the current shape needs, computed without the limit check (the plan itself is invalidated on every edit).
    private JointPose[] NeededJointPath()
    {
        try
        {
            var model = new PlanarArm(Number(l1), Number(l2));
            double tx = Number(x), ty = Number(y);
            if (shape.SelectedIndex == 0)
            {
                var poses = model.Inverse(tx, ty);
                return [poses[Math.Min(Math.Max(branch.SelectedIndex, 0), poses.Length - 1)]];
            }
            var path = DrawingTrajectory.Create(model, (DrawingShape)(shape.SelectedIndex - 1), tx, ty, Number(a),
                shape.SelectedIndex == 3 ? 0 : Number(b), Math.Max(0, branch.SelectedIndex));
            int step = Math.Max(1, path.Samples.Length / 400);
            return path.Samples.Where((_, i) => i % step == 0).Select(s => s.Pose).ToArray();
        }
        catch (ArgumentException) { return []; }
    }

    // Keeps the points with a*q1 + b*q2 + c >= 0 (one Sutherland-Hodgman step; points are (q1, q2) in degrees).
    private static List<Point> ClipHalfPlane(List<Point> polygon, double a, double b, double c)
    {
        var result = new List<Point>();
        for (int i = 0; i < polygon.Count; i++)
        {
            Point p = polygon[i], q = polygon[(i + 1) % polygon.Count];
            double fp = a * p.X + b * p.Y + c, fq = a * q.X + b * q.Y + c;
            if (fp >= 0) result.Add(p);
            if ((fp >= 0) != (fq >= 0)) { double t = fp / (fp - fq); result.Add(new Point(p.X + t * (q.X - p.X), p.Y + t * (q.Y - p.Y))); }
        }
        return result;
    }

    // Touching the boundary leaves a zero-area sliver; treat it as empty.
    private static bool HasArea(List<Point> polygon)
    {
        double twice = 0;
        for (int i = 0; i < polygon.Count; i++) { Point p = polygon[i], q = polygon[(i + 1) % polygon.Count]; twice += p.X * q.Y - q.X * p.Y; }
        return Math.Abs(twice) > 1e-6;
    }

    private static void ShowPolygon(Polygon polygon, List<Point>? degrees)
    {
        if (degrees is null || !HasArea(degrees)) { polygon.Visibility = Visibility.Collapsed; return; }
        polygon.Points = Pixels(degrees);
        polygon.Visibility = Visibility.Visible;
    }

    private void SetElbowText(Brush color, string text)
    {
        if (!ReferenceEquals(elbowState.Foreground, color)) elbowState.Foreground = color;
        if (elbowState.Text != text) elbowState.Text = text;
    }

    // Called from RefreshRangeHelp (every parameter keystroke and device status update) and by the E fields: only
    // the range fields, E and the cached path matter, so editing the shape parameters costs a string compare. The first call (construction) draws at once so the first frame is complete.
    private void RefreshElbowRule()
    {
        string signature = string.Join("|", new[] { min1, max1, min2, max2, elbowRight, elbowLeft }.Select(f => f.Text)) + $"|{elbowPathRevision}";
        if (signature == elbowSignature) return;
        bool first = elbowSignature is null;
        elbowSignature = signature;
        elbowPending = true;
        if (first) { UpdateElbowView(); return; }
        elbowTimer.Stop(); elbowTimer.Start();   // debounce: redraw once the edits pause, off the input handler
    }

    // Synchronous update for self-tests and snapshots.
    internal void FlushElbowRule()
    {
        if (!elbowPending) return;
        elbowTimer.Stop();
        UpdateElbowView();
    }

    private void UpdateElbowView()
    {
        elbowPending = false;
        JointLimits limits;
        try { limits = PreviewLimits(); }
        catch (ArgumentException)
        {
            foreach (var element in ElbowShapes) element.Visibility = Visibility.Collapsed;
            SetElbowText(WorkbenchTheme.Muted, "范围无效，无法计算肘范围：先修正上面的 q1/q2 范围。");
            return;
        }

        bool haveTravel = TryTravel(elbowRight, out double right) & TryTravel(elbowLeft, out double left);
        var square = new List<Point> { new(-180, -180), new(180, -180), new(180, 180), new(-180, 180) };
        var box = new List<Point> { new(limits.Min1, limits.Min2), new(limits.Max1, limits.Min2), new(limits.Max1, limits.Max2), new(limits.Min1, limits.Max2) };
        // elbow e = q2 - q1 must satisfy -right <= e <= left
        ShowPolygon(elbowLegal, haveTravel ? ClipHalfPlane(ClipHalfPlane(square, -1, 1, right), 1, -1, left) : null);
        ShowPolygon(elbowBox, box);
        ShowPolygon(elbowBad[0], haveTravel ? ClipHalfPlane(box, -1, 1, -left) : null);    // e > left
        ShowPolygon(elbowBad[1], haveTravel ? ClipHalfPlane(box, 1, -1, -right) : null);   // e < -right

        var needed = elbowNeeded;
        if (needed.Length > 1) { elbowPath.Points = Pixels(needed.Select(p => new Point(p.Q1, p.Q2))); elbowPath.Visibility = Visibility.Visible; }
        else elbowPath.Visibility = Visibility.Collapsed;
        if (needed.Length == 1)
        {
            Canvas.SetLeft(elbowDot, PX(needed[0].Q1) - 3); Canvas.SetTop(elbowDot, PY(needed[0].Q2) - 3);
            elbowDot.Visibility = Visibility.Visible;
        }
        else elbowDot.Visibility = Visibility.Collapsed;

        if (!haveTravel)
        {
            SetElbowText(WorkbenchTheme.Red, $"肘 (q2−q1) 可达 {Deg(limits.ElbowMin)} … {Deg(limits.ElbowMax)}。肘安全行程 E 无效（须为 0～360 的数），无法判断。");
            return;
        }
        double overRight = Math.Max(0, -right - limits.ElbowMin), overLeft = Math.Max(0, limits.ElbowMax - left);
        string travelText = $"[−{Mag(right)}, +{Mag(left)}]";
        if (overRight <= 1e-9 && overLeft <= 1e-9)
            SetElbowText(ElbowOk, $"肘 (q2−q1) 可达 {Deg(limits.ElbowMin)} … {Deg(limits.ElbowMax)}，在安全行程 {travelText} 内 ✓");
        else
        {
            var parts = new List<string>();
            if (overRight > 1e-9) parts.Add($"右侧多 {Mag(overRight)}");
            if (overLeft > 1e-9) parts.Add($"左侧多 {Mag(overLeft)}");
            SetElbowText(WorkbenchTheme.Red, $"肘 (q2−q1) 可达 {Deg(limits.ElbowMin)} … {Deg(limits.ElbowMax)}，超出安全行程 {travelText}：{string.Join("、", parts)}。" +
                "固件不会拦，请缩小 q1/q2 范围（图中红色部分），或按实测行程修改 E。");
        }
    }

    private static void ElbowRuleSelfTest()
    {
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Elbow rule: " + why); }
        static int Illegal(XYPreviewPanel p) => p.elbowBad.Count(g => g.Visibility == Visibility.Visible);
        // Same numbers as the offline run on the real firmware C: q1 +/-135, q2 +/-155 -> elbow -290..+290.
        var wide = new JointLimits(-135, 135, -155, 155);
        Check(wide.ElbowMin == -290 && wide.ElbowMax == 290, "band formula differs from the measured firmware band");
        var symmetric = new JointLimits(-90, 90, -65, 65); var tight = new JointLimits(-60, 5, -5, 60);
        Check(symmetric.ElbowMin == -155 && symmetric.ElbowMax == 155 && tight.ElbowMin == -10 && tight.ElbowMax == 120, "README examples");
        Check(DefaultTrialRange.ElbowMin == -90 && DefaultTrialRange.ElbowMax == 90, "boot defaults");

        var p = new XYPreviewPanel();     // the first draw is synchronous
        Check(p.elbowState.Text.Contains('✓') && Illegal(p) == 0, "defaults must satisfy the rule");
        p.RestoreLimitsForSelfTest("-140", "140", "-155", "155"); p.FlushElbowRule();   // what parameters.json held on 2026-09-29
        Check(p.elbowState.Text.Contains("超出") && p.elbowState.Text.Contains("295") && Illegal(p) == 2, "a 295-degree band must be flagged on both sides");
        p.RestoreLimitsForSelfTest("-90", "90", "-65", "65"); p.FlushElbowRule();
        Check(p.elbowState.Text.Contains('✓') && Illegal(p) == 0, "A+B = 155 must pass at E = 155");
        Check(p.trajectory is not null && p.timing is not null, "test setup: a valid plan");

        // Edits are applied after they pause, not inside the input handler.
        p.min1.Text = "-80";
        Check(p.elbowPending && p.elbowState.Text.Contains("+155°"), "the update must be deferred until Flush or the timer");
        p.FlushElbowRule();
        Check(!p.elbowPending && p.elbowState.Text.Contains("+145°"), "flush must apply the pending edit");
        p.min1.Text = "-90"; p.Refresh();     // editing a range field invalidates the plan (as before); re-preview it
        Check(p.trajectory is not null && p.timing is not null, "test setup: a valid plan");
        p.FlushElbowRule();

        p.elbowRight.Text = "150";      // E is display-only: adjusting it must be live but must not touch the plan
        p.FlushElbowRule();
        Check(p.elbowState.Text.Contains("右侧多 5°") && Illegal(p) == 1, "E right = 150 must flag 5 degrees on the right only");
        Check(p.trajectory is not null && p.timing is not null && p.rangeState.Text.Length > 0, "changing E invalidated the plan");
        p.elbowRight.Text = "155"; p.elbowLeft.Text = "abc"; p.FlushElbowRule();
        Check(p.elbowState.Text.Contains("无效") && Illegal(p) == 0, "an invalid E must say so and not flag anything");
        p.elbowLeft.Text = "155";
        p.min1.Text = "100"; p.FlushElbowRule();           // invalid q1 range
        Check(p.elbowState.Text.Contains("范围无效") && p.elbowBox.Visibility != Visibility.Visible, "invalid limits must be reported and the box hidden");
        p.min1.Text = "-90"; p.FlushElbowRule();
        Check(p.elbowState.Text.Contains('✓') && p.elbowBox.Visibility == Visibility.Visible, "restoring the range must restore the verdict");

        var before = p.elbowLegal.Points;
        p.RefreshRangeHelp(); p.FlushElbowRule();           // device status updates call this constantly: nothing may change
        Check(ReferenceEquals(before, p.elbowLegal.Points) && !p.elbowPending && !p.elbowTimer.IsEnabled, "unchanged inputs must not touch the plot");
        int children = p.elbowPlot.Children.Count;
        p.max1.Text = "80"; p.FlushElbowRule();
        Check(!ReferenceEquals(before, p.elbowLegal.Points) && p.elbowPlot.Children.Count == children, "an edit must update the plot in place, not rebuild it");
        int revision = p.elbowPathRevision;
        p.y.Text = "5";                                     // shape parameters: the elbow view must do no work at all
        Check(!p.elbowPending && p.elbowPathRevision == revision, "editing a shape parameter touched the elbow view");
        p.y.Text = "0"; p.Refresh(); p.FlushElbowRule();    // the preview captures the needed path
        Check(p.elbowPathRevision == revision + 1 && p.elbowPath.Visibility == Visibility.Visible, "the preview must capture and show the needed path");
        p.x.Text = "1000"; p.Refresh(); p.FlushElbowRule(); // unreachable target: no needed path, the rule view must still work
        Check(p.elbowPath.Visibility != Visibility.Visible && p.elbowDot.Visibility != Visibility.Visible && p.elbowState.Text.Contains('✓'), "unreachable target broke the rule view");
    }

    // Writes the rule view for a compliant and a violating setting, next to the other offline screenshots.
    private static void RenderElbowRuleSnapshots(string directory)
    {
        foreach (var (name, limits) in new[] { ("elbow-rule-ok", new[] { "-60", "5", "-5", "60" }), ("elbow-rule-violated", new[] { "-140", "140", "-155", "155" }) })
        {
            var panel = new XYPreviewPanel();
            panel.x.Text = "300"; panel.y.Text = "0"; panel.a.Text = "40";
            panel.RestoreLimitsForSelfTest(limits[0], limits[1], limits[2], limits[3]);
            panel.FlushElbowRule();
            // 280 px = the width of the left column that hosts ParameterContent (ControlWindow.Layout.cs).
            var host = new Border { Background = Brushes.White, Width = 280, Height = 1250, Child = panel.ParameterContent };
            host.Measure(new Size(280, 1250)); host.Arrange(new Rect(0, 0, 280, 1250)); host.UpdateLayout();
            var image = new RenderTargetBitmap(280, 1250, 96, 96, PixelFormats.Pbgra32); image.Render(host);
            var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(image));
            using var stream = System.IO.File.Create(Path.Combine(directory, name + ".png")); png.Save(stream);
        }
    }
}
