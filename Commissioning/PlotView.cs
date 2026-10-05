using System.Globalization;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;

namespace TwoLinkCommissioning;

internal sealed class PlotSeries(string name, double[] x, double[] y, Brush color)
{
    public string Name { get; } = name;
    public double[] X { get; } = x;
    public double[] Y { get; } = y;
    public Brush Color { get; } = color;
    public double Thickness { get; init; } = 1.6;
    public bool Dashed { get; init; }
    public bool Dots { get; init; }
    public string Unit { get; init; } = "";
    public int Decimals { get; init; } = 3;
}
internal sealed record PlotMark(double X, string Label, Brush Color, bool Labelled = true);
internal sealed record PlotLine(double Y, string Label, Brush Color);

// A small time-series chart drawn straight to the screen: axes, grid, lines, event markers, reference lines, a replay
// cursor with the value of every series at the cursor, and a source note. The chart never says more than its data does.
internal sealed class PlotView : FrameworkElement
{
    public string Title { get; set; } = "";
    public string Source { get; set; } = "";
    public string XUnit { get; set; } = "s";
    public List<string> Notes { get; } = [];
    public List<PlotSeries> Series { get; } = [];
    public List<PlotMark> Marks { get; } = [];
    public List<PlotLine> Lines { get; } = [];
    public bool SymmetricY { get; set; }
    public (double Min, double Max) XRange { get; set; } = (0, 1);
    private double? cursor;
    public double? CursorX { get => cursor; set { if (cursor != value) { cursor = value; InvalidateVisual(); } } }
    public event Action<double>? CursorChanged;

    private static readonly Brush Ink = WorkbenchTheme.Ink, Muted = WorkbenchTheme.Muted, Grid = WorkbenchTheme.GridLine, Frame = WorkbenchTheme.Line;
    private const double Left = 54, Right = 12, Top = 24, Bottom = 20;

    public PlotView() { MinHeight = 150; Cursor = Cursors.Cross; ClipToBounds = true; }

    public void Clear() { Series.Clear(); Marks.Clear(); Lines.Clear(); Notes.Clear(); Source = ""; }
    private Rect Plot => new(Left, Top, Math.Max(10, ActualWidth - Left - Right), Math.Max(10, ActualHeight - Top - Bottom - (Notes.Count + (Source.Length > 0 ? 1 : 0)) * 13));

    private FormattedText Text(string text, double size, Brush brush, bool bold = false, bool italic = false) =>
        new(text, CultureInfo.CurrentUICulture, FlowDirection.LeftToRight,
            new Typeface(new FontFamily("Microsoft YaHei UI, Segoe UI"), italic ? FontStyles.Italic : FontStyles.Normal, bold ? FontWeights.SemiBold : FontWeights.Normal, FontStretches.Normal),
            size, brush, VisualTreeHelper.GetDpi(this).PixelsPerDip);

    private static double Step(double raw) { double p = Math.Pow(10, Math.Floor(Math.Log10(raw))), n = raw / p; return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * p; }
    private static string Format(double v, double step) => (Math.Abs(v) < step / 1000 ? 0 : v).ToString(step >= 1 ? "0" : step >= 0.1 ? "0.0" : step >= 0.01 ? "0.00" : step >= 0.001 ? "0.000" : "0.0000", CultureInfo.InvariantCulture);

    private (double Min, double Max) YRange()
    {
        double min = double.MaxValue, max = double.MinValue;
        foreach (var s in Series)
            for (int i = 0; i < s.Y.Length && i < s.X.Length; i++)
                if (double.IsFinite(s.Y[i]) && s.X[i] >= XRange.Min && s.X[i] <= XRange.Max) { min = Math.Min(min, s.Y[i]); max = Math.Max(max, s.Y[i]); }
        foreach (var l in Lines) { min = Math.Min(min, l.Y); max = Math.Max(max, l.Y); }
        if (min > max) return (-1, 1);
        if (SymmetricY) { double m = Math.Max(Math.Abs(min), Math.Abs(max)); min = -m; max = m; }
        if (max - min < 1e-9) { min -= 0.5; max += 0.5; }
        double pad = (max - min) * 0.08;
        return (min - pad, max + pad);
    }

    // Linear interpolation of a series at x (the value shown at the cursor); NaN outside its data.
    public static double ValueAt(PlotSeries s, double x)
    {
        int n = Math.Min(s.X.Length, s.Y.Length);
        if (n == 0 || x < s.X[0] || x > s.X[n - 1]) return double.NaN;
        int i = Array.BinarySearch(s.X, 0, n, x);
        if (i >= 0) return s.Y[i];
        i = ~i; if (i == 0 || i >= n) return double.NaN;
        double u = (x - s.X[i - 1]) / (s.X[i] - s.X[i - 1]);
        return s.Y[i - 1] + (s.Y[i] - s.Y[i - 1]) * u;
    }

    protected override void OnRender(DrawingContext dc)
    {
        double w = ActualWidth, h = ActualHeight;
        dc.DrawRectangle(Brushes.White, null, new Rect(0, 0, w, h));
        var plot = Plot;
        var (y0, y1) = YRange(); var (x0, x1) = XRange; if (x1 - x0 < 1e-9) x1 = x0 + 1;
        double X(double x) => plot.Left + (x - x0) / (x1 - x0) * plot.Width;
        double Y(double y) => plot.Bottom - (y - y0) / (y1 - y0) * plot.Height;
        var pen = new Pen(Grid, 1);
        double xs = Step((x1 - x0) / Math.Max(2, plot.Width / 80)), ys = Step((y1 - y0) / Math.Max(2, plot.Height / 34));
        for (double gx = Math.Ceiling(x0 / xs) * xs; gx <= x1 + 1e-9; gx += xs)
        {
            dc.DrawLine(pen, new Point(X(gx), plot.Top), new Point(X(gx), plot.Bottom));
            var label = Text(Format(gx, xs), 10, Muted); dc.DrawText(label, new Point(X(gx) - label.Width / 2, plot.Bottom + 3));
        }
        for (double gy = Math.Ceiling(y0 / ys) * ys; gy <= y1 + 1e-9; gy += ys)
        {
            dc.DrawLine(pen, new Point(plot.Left, Y(gy)), new Point(plot.Right, Y(gy)));
            var label = Text(Format(gy, ys), 10, Muted); dc.DrawText(label, new Point(plot.Left - label.Width - 4, Y(gy) - label.Height / 2));
        }
        if (XUnit.Length > 0) dc.DrawText(Text(XUnit, 10, Muted), new Point(plot.Right + 2, plot.Bottom + 3));
        if (y0 < 0 && y1 > 0) dc.DrawLine(new Pen(Frame, 1.2), new Point(plot.Left, Y(0)), new Point(plot.Right, Y(0)));
        dc.DrawRectangle(null, new Pen(Frame, 1), plot);
        dc.PushClip(new RectangleGeometry(plot));
        foreach (var l in Lines)
        {
            var linePen = new Pen(l.Color, 1) { DashStyle = DashStyles.Dash };
            dc.DrawLine(linePen, new Point(plot.Left, Y(l.Y)), new Point(plot.Right, Y(l.Y)));
            var label = Text(l.Label, 10, l.Color); dc.DrawText(label, new Point(plot.Right - label.Width - 4, Y(l.Y) - label.Height - 1));
        }
        int stagger = 0;
        foreach (var m in Marks)
        {
            if (m.X < x0 || m.X > x1) continue;
            dc.DrawLine(new Pen(m.Color, 1) { DashStyle = DashStyles.Dot }, new Point(X(m.X), plot.Top), new Point(X(m.X), plot.Bottom));
            if (m.Labelled) { var label = Text(m.Label, 10, m.Color); dc.DrawText(label, new Point(X(m.X) + 3, plot.Top + 1 + (stagger++ % 3) * 11)); }
        }
        foreach (var s in Series)
        {
            var geometry = new StreamGeometry();
            using (var ctx = geometry.Open())
            {
                bool open = false;
                for (int i = 0; i < Math.Min(s.X.Length, s.Y.Length); i++)
                {
                    if (!double.IsFinite(s.Y[i]) || s.X[i] < x0 - (x1 - x0) * 0.02 || s.X[i] > x1 + (x1 - x0) * 0.02) { open = false; continue; }
                    var p = new Point(X(s.X[i]), Y(s.Y[i]));
                    if (!open) { ctx.BeginFigure(p, false, false); open = true; } else ctx.LineTo(p, true, false);
                }
            }
            geometry.Freeze();
            dc.DrawGeometry(null, new Pen(s.Color, s.Thickness) { DashStyle = s.Dashed ? DashStyles.Dash : DashStyles.Solid, LineJoin = PenLineJoin.Round }, geometry);
            if (s.Dots)
                for (int i = 0; i < Math.Min(s.X.Length, s.Y.Length); i++)
                    if (double.IsFinite(s.Y[i]) && s.X[i] >= x0 && s.X[i] <= x1) dc.DrawEllipse(s.Color, null, new Point(X(s.X[i]), Y(s.Y[i])), 2.2, 2.2);
        }
        if (cursor is { } c && c >= x0 && c <= x1) dc.DrawLine(new Pen(Ink, 1.2), new Point(X(c), plot.Top), new Point(X(c), plot.Bottom));
        dc.Pop();
        // Title, then the legend with the value of every series at the cursor.
        var title = Text(Title, 11.5, Ink, bold: true); dc.DrawText(title, new Point(Left, 3));
        double lx = Left + title.Width + 14;
        foreach (var s in Series)
        {
            string value = cursor is { } at && ValueAt(s, at) is var v && double.IsFinite(v) ? "  " + v.ToString("F" + s.Decimals, CultureInfo.InvariantCulture) + s.Unit : "";
            var entry = Text("▬ " + s.Name + value, 10.5, s.Color);
            if (lx + entry.Width > w - 6) break;
            dc.DrawText(entry, new Point(lx, 5)); lx += entry.Width + 12;
        }
        double ny = plot.Bottom + 17;
        foreach (string note in Notes) { dc.DrawText(Text(note, 10, Ink), new Point(Left, ny)); ny += 13; }
        if (Source.Length > 0) dc.DrawText(Text(Source, 10, Muted, italic: true), new Point(Left, ny));
    }

    internal void RaiseCursor(double t) => CursorChanged?.Invoke(t);   // what a click does, for the self-test
    private void Pick(MouseEventArgs e)
    {
        var plot = Plot; var p = e.GetPosition(this);
        double t = XRange.Min + (Math.Clamp(p.X, plot.Left, plot.Right) - plot.Left) / plot.Width * (XRange.Max - XRange.Min);
        CursorChanged?.Invoke(t);
    }
    protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e) { CaptureMouse(); Pick(e); base.OnMouseLeftButtonDown(e); }
    protected override void OnMouseMove(MouseEventArgs e) { if (IsMouseCaptured && e.LeftButton == MouseButtonState.Pressed) Pick(e); base.OnMouseMove(e); }
    protected override void OnMouseLeftButtonUp(MouseButtonEventArgs e) { ReleaseMouseCapture(); base.OnMouseLeftButtonUp(e); }
}

// The XY plane: planned path, commanded path and the hybrid estimate, plus the two arm links at the replay cursor.
// Everything on it is a model estimate, and the view says so.
internal sealed class XYView : FrameworkElement
{
    public sealed record Path(string Name, double[] X, double[] Y, Brush Color, double Thickness = 1.6, bool Dashed = false);
    public sealed record Arm(string Name, Point[] Joints, Brush Color, bool Dashed);
    public List<Path> Paths { get; } = [];
    public List<Arm> Arms { get; } = [];
    public string Source { get; set; } = "";
    public double ReachRadius { get; set; } = 356;
    private bool wholeArm;
    public bool WholeArm { get => wholeArm; set { wholeArm = value; InvalidateVisual(); } }
    public XYView() { MinHeight = 200; ClipToBounds = true; }
    public void Clear() { Paths.Clear(); Arms.Clear(); Source = ""; }

    private Rect Bounds()
    {
        if (wholeArm) return new Rect(-ReachRadius, -ReachRadius, ReachRadius * 2, ReachRadius * 2);
        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
        foreach (var p in Paths)
            for (int i = 0; i < p.X.Length; i++)
                if (double.IsFinite(p.X[i]) && double.IsFinite(p.Y[i])) { minX = Math.Min(minX, p.X[i]); maxX = Math.Max(maxX, p.X[i]); minY = Math.Min(minY, p.Y[i]); maxY = Math.Max(maxY, p.Y[i]); }
        if (minX > maxX) return new Rect(-ReachRadius, -ReachRadius, ReachRadius * 2, ReachRadius * 2);
        double pad = Math.Max(3, Math.Max(maxX - minX, maxY - minY) * 0.12);
        return new Rect(minX - pad, minY - pad, Math.Max(6, maxX - minX + 2 * pad), Math.Max(6, maxY - minY + 2 * pad));
    }

    protected override void OnRender(DrawingContext dc)
    {
        double w = ActualWidth, h = ActualHeight;
        dc.DrawRectangle(Brushes.White, new Pen(WorkbenchTheme.Line, 1), new Rect(0, 0, w, h));
        const double margin = 10, foot = 30;
        var area = new Rect(margin, margin, Math.Max(10, w - 2 * margin), Math.Max(10, h - margin - foot));
        var box = Bounds();
        double scale = Math.Min(area.Width / box.Width, area.Height / box.Height);
        double ox = area.Left + (area.Width - box.Width * scale) / 2, oy = area.Top + (area.Height + box.Height * scale) / 2;
        Point P(double x, double y) => new(ox + (x - box.Left) * scale, oy - (y - box.Top) * scale);
        dc.PushClip(new RectangleGeometry(area));
        var grid = new Pen(WorkbenchTheme.GridLine, 1);
        double step;
        step = new[] { 1.0, 2, 5, 10, 20, 50, 100 }.FirstOrDefault(s => s * scale >= 38, 100);
        for (double gx = Math.Ceiling(box.Left / step) * step; gx <= box.Right; gx += step) dc.DrawLine(grid, P(gx, box.Top), P(gx, box.Bottom));
        for (double gy = Math.Ceiling(box.Top / step) * step; gy <= box.Bottom; gy += step) dc.DrawLine(grid, P(box.Left, gy), P(box.Right, gy));
        dc.DrawEllipse(null, new Pen(WorkbenchTheme.Brush("#D8A0A0"), 1), P(0, 0), ReachRadius * scale, ReachRadius * scale);
        foreach (var path in Paths)
        {
            var geometry = new StreamGeometry();
            using (var ctx = geometry.Open())
            {
                bool open = false;
                for (int i = 0; i < Math.Min(path.X.Length, path.Y.Length); i++)
                {
                    if (!double.IsFinite(path.X[i]) || !double.IsFinite(path.Y[i])) { open = false; continue; }
                    var p = P(path.X[i], path.Y[i]); if (!open) { ctx.BeginFigure(p, false, false); open = true; } else ctx.LineTo(p, true, false);
                }
            }
            geometry.Freeze();
            dc.DrawGeometry(null, new Pen(path.Color, path.Thickness) { DashStyle = path.Dashed ? DashStyles.Dash : DashStyles.Solid }, geometry);
        }
        foreach (var arm in Arms)
        {
            var pen = new Pen(arm.Color, 3.2) { DashStyle = arm.Dashed ? DashStyles.Dash : DashStyles.Solid, StartLineCap = PenLineCap.Round, EndLineCap = PenLineCap.Round };
            dc.DrawLine(pen, P(arm.Joints[0].X, arm.Joints[0].Y), P(arm.Joints[1].X, arm.Joints[1].Y));
            dc.DrawLine(pen, P(arm.Joints[1].X, arm.Joints[1].Y), P(arm.Joints[2].X, arm.Joints[2].Y));
            dc.DrawEllipse(Brushes.White, new Pen(arm.Color, 1.6), P(arm.Joints[2].X, arm.Joints[2].Y), 4, 4);
        }
        dc.Pop();
        var typeface = new Typeface(new FontFamily("Microsoft YaHei UI, Segoe UI"), FontStyles.Italic, FontWeights.Normal, FontStretches.Normal);
        double ppd = VisualTreeHelper.GetDpi(this).PixelsPerDip;
        dc.DrawText(new FormattedText($"{step:0.##} mm / {Math.Round(step * scale)} px", CultureInfo.CurrentUICulture, FlowDirection.LeftToRight, typeface, 10, WorkbenchTheme.Muted, ppd), new Point(margin + 2, area.Bottom - 14));
        dc.DrawText(new FormattedText(Source, CultureInfo.CurrentUICulture, FlowDirection.LeftToRight, typeface, 10, WorkbenchTheme.Muted, ppd) { MaxTextWidth = Math.Max(20, w - 2 * margin), MaxTextHeight = foot }, new Point(margin, h - foot + 2));
    }
}
