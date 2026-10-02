using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace TwoLinkCommissioning;

// What the wizard needs from the main window. Moves go through the same gated point move as the main window's own button.
internal interface ICalibrationHost
{
    PlanarArm? Model { get; }                    // link lengths the planner uses now
    JointPose? DeviceCommand { get; }            // commanded pose from fresh telemetry; null when there is none
    double? DeviceEncoderQ1 { get; }             // arm1 encoder angle at the same moment (motor side)
    string ReferenceName { get; }                // how the reference was set; "" when there is none
    string Firmware { get; }
    JointLimits? Limits { get; }                 // the ranges the planner checks against
    JointPose? PointTarget { get; }              // the single-point target on screen
    bool CanMove(out string reason);
    bool MoveTo(JointPose pose, out string reason);
    void ApplyLinkLengths(double l1, double l2);
    void ApplyElbowRange(double min, double max);
}

// Measures, records, and saves: link lengths, the true pose after the reference, the mechanical stroke, and the repeatability of arriving at a
// point from either direction. It never changes the machine by itself; the two "use these values" buttons only fill the planning boxes.
internal sealed partial class CalibrationWindow : Window
{
    private sealed class TrialRow
    {
        public string Direction { get; init; } = "+"; public int Index { get; init; }
        public string X { get; set; } = ""; public string Y { get; set; } = "";
        public string Q1Command { get; set; } = ""; public string Q1Encoder { get; set; } = ""; public string Q2Command { get; set; } = "";
    }

    private readonly ICalibrationHost host;
    private readonly string[] stepKeys = ["cal.step.links", "cal.step.zero", "cal.step.stroke", "cal.step.repeat", "cal.step.save", "cal.step.history"];
    private readonly ListBox steps = new() { BorderThickness = new Thickness(0), Background = WorkbenchTheme.Bar, MinWidth = 170 };
    private readonly ContentControl page = new();
    private readonly Panel[] pages;
    private readonly TextBox l1 = Box(), l2 = Box(), uncertainty = Box("0.5"), q1Offset = Box("0"), q2Offset = Box("0"), tipX = Box(), tipY = Box(), zeroNotes = Box(wide: true),
        elbowMin = Box(), elbowMax = Box(), q1Min = Box(), q1Max = Box(), margin = Box("3"), targetQ1 = Box(), targetQ2 = Box(), approach = Box("4"), perDirection = Box("5"), note = Box(wide: true);
    private readonly ComboBox method = new() { Width = 130, Margin = new Thickness(4), IsEditable = true };
    private readonly TextBlock status = new() { TextWrapping = TextWrapping.Wrap, Margin = new Thickness(12, 6, 12, 6), FontWeight = FontWeights.SemiBold };
    private readonly TextBlock linkNote = Note(), zeroNote = Note(), strokeNote = Note(), repeatNote = Note();
    private readonly TextBox summary = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, Background = Brushes.White, FontSize = 12, Margin = new Thickness(0, 6, 0, 6), VerticalScrollBarVisibility = ScrollBarVisibility.Auto, MinHeight = 220 };
    private readonly Button saveButton = new() { Margin = new Thickness(4), Padding = new Thickness(18, 4, 18, 4), HorizontalAlignment = HorizontalAlignment.Left };
    private readonly DataGrid trialGrid = new() { AutoGenerateColumns = false, CanUserAddRows = false, HeadersVisibility = DataGridHeadersVisibility.Column, MinHeight = 160, MaxHeight = 260, Background = Brushes.White, RowHeaderWidth = 0, FontSize = 12 };
    private readonly ListView history = new() { MinHeight = 160 };
    private readonly System.Windows.Threading.DispatcherTimer poll = new() { Interval = TimeSpan.FromMilliseconds(300) };
    private List<TrialRow> trials = [];
    private CalibrationRecord? lastSaved;

    private static TextBox Box(string text = "", bool wide = false) => new() { Text = text, Width = wide ? 360 : 90, Margin = new Thickness(4), Padding = new Thickness(4), HorizontalContentAlignment = wide ? HorizontalAlignment.Left : HorizontalAlignment.Right, FontFamily = WorkbenchTheme.Mono };
    private static TextBlock Note() => new() { TextWrapping = TextWrapping.Wrap, FontSize = 12, Foreground = WorkbenchTheme.Muted, Margin = new Thickness(4, 6, 4, 6) };
    private static TextBlock Help(string key) => Lang.Bind(new TextBlock { TextWrapping = TextWrapping.Wrap, FontSize = 12, Margin = new Thickness(4, 2, 4, 8), Foreground = WorkbenchTheme.Sub }, TextBlock.TextProperty, key);
    private static Border Row(string key, FrameworkElement input, string unit = "", string? extra = null)
    {
        var grid = new Grid { Margin = new Thickness(0, 1, 0, 1) };
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(210) }); grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto }); grid.ColumnDefinitions.Add(new ColumnDefinition());
        var label = WorkbenchTheme.LocalLabel(key); label.VerticalAlignment = VerticalAlignment.Center; grid.Children.Add(label);
        Grid.SetColumn(input, 1); input.VerticalAlignment = VerticalAlignment.Center; grid.Children.Add(input);
        var tail = new TextBlock { Text = unit, Foreground = WorkbenchTheme.Muted, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(4, 0, 0, 0) }; if (extra is not null) Lang.Bind(tail, TextBlock.TextProperty, extra);
        Grid.SetColumn(tail, 2); grid.Children.Add(tail);
        return new Border { Child = grid, BorderBrush = WorkbenchTheme.Faint, BorderThickness = new Thickness(0, 0, 0, 1) };
    }
    private static Button Action(string key, Action click) { var b = new Button { Margin = new Thickness(4), Padding = new Thickness(10, 3, 10, 3), HorizontalAlignment = HorizontalAlignment.Left }; Lang.Bind(b, ContentControl.ContentProperty, key); b.Click += (_, _) => click(); return b; }

    public CalibrationWindow(ICalibrationHost host)
    {
        this.host = host;
        Width = 980; Height = 760; MinWidth = 800; MinHeight = 560; Background = WorkbenchTheme.Background; Foreground = WorkbenchTheme.Ink;
        FontFamily = new FontFamily("Microsoft YaHei UI, Segoe UI"); FontSize = 12;
        WorkbenchTheme.Apply(this);
        foreach (string item in new[] { "cal.method.caliper", "cal.method.ruler", "cal.method.cad", "cal.method.other" }) method.Items.Add(Lang.T(item));
        method.SelectedIndex = 0;
        pages = [BuildLinks(), BuildZero(), BuildStroke(), BuildRepeat(), BuildSave(), BuildHistory()];
        var shell = new DockPanel();
        var statusBar = WorkbenchTheme.Edge(status, WorkbenchTheme.Bar, new Thickness(0, 1, 0, 0)); DockPanel.SetDock(statusBar, Dock.Bottom); shell.Children.Add(statusBar);
        var stepBar = WorkbenchTheme.Edge(steps, WorkbenchTheme.Bar, new Thickness(0, 0, 1, 0)); DockPanel.SetDock(stepBar, Dock.Left); shell.Children.Add(stepBar);
        shell.Children.Add(new ScrollViewer { Content = page, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, Padding = new Thickness(14) });
        Content = shell;
        steps.SelectionChanged += (_, _) => { if (steps.SelectedIndex >= 0) { page.Content = pages[steps.SelectedIndex]; if (steps.SelectedIndex == 4) RefreshSummary(); if (steps.SelectedIndex == 5) RefreshHistory(); } };
        foreach (var box in new[] { l1, l2, uncertainty, q1Offset, q2Offset, tipX, tipY, elbowMin, elbowMax, q1Min, q1Max, margin, targetQ1, targetQ2, approach, perDirection }) box.TextChanged += (_, _) => Refresh();
        poll.Tick += (_, _) => Refresh(); poll.Start(); Closed += (_, _) => poll.Stop();
        Lang.Changed += ApplyLanguage; Closed += (_, _) => Lang.Changed -= ApplyLanguage;
        ApplyLanguage();   // the boxes start empty: the planner's own numbers are not a measurement
        steps.SelectedIndex = 0;
    }

    private static string Mm(double v) => v.ToString("0.###", CultureInfo.InvariantCulture);
    private static bool Num(TextBox box, out double value) => double.TryParse(box.Text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out value) && double.IsFinite(value);
    private static double? Opt(TextBox box) => Num(box, out double v) ? v : null;

    private void ApplyLanguage()
    {
        Title = Lang.T("cal.title");
        int selected = steps.SelectedIndex; steps.Items.Clear();
        for (int i = 0; i < stepKeys.Length; i++) steps.Items.Add(new ListBoxItem { Content = StepLabel(i), Padding = new Thickness(10, 7, 10, 7) });
        steps.SelectedIndex = Math.Max(0, selected);
        saveButton.Content = Lang.T("cal.save");
        int m = method.SelectedIndex; string[] keys = ["cal.method.caliper", "cal.method.ruler", "cal.method.cad", "cal.method.other"];
        if (m >= 0) { method.Items.Clear(); foreach (string k in keys) method.Items.Add(Lang.T(k)); method.SelectedIndex = m; }
        BuildTrialColumns(); Refresh();
    }

    private string StepLabel(int i)
    {
        bool done = i switch { 0 => LinkDraft() is not null, 1 => ZeroDraft() is not null, 2 => StrokeDraft() is not null, 3 => RepeatDraft() is { } r && r.Trials.Any(t => t.X is not null && t.Y is not null), _ => false };
        return (i < 4 ? (done ? "✓ " : "○ ") : "   ") + $"{i + 1} {Lang.T(stepKeys[i])}";
    }

    // ---- the sections as records (null = not filled in) --------------------------------------------------------------
    private LinkMeasurement? LinkDraft() => Num(l1, out double a) && Num(l2, out double b) && Num(uncertainty, out double u) && a > 0 && b > 0 && u >= 0 ? new(a, b, method.Text, u) : null;
    private ZeroMeasurement? ZeroDraft() => Num(q1Offset, out double a) && Num(q2Offset, out double b) && (tipX.Text.Trim().Length > 0 || zeroNotes.Text.Trim().Length > 0 || a != 0 || b != 0)
        ? new(host.ReferenceName.Length > 0 ? host.ReferenceName : "—", a, b, Opt(tipX), Opt(tipY), zeroNotes.Text.Trim()) : null;
    private StrokeMeasurement? StrokeDraft() => new[] { Opt(elbowMin), Opt(elbowMax), Opt(q1Min), Opt(q1Max) } is [var e0, var e1, var a0, var a1] && (e0 is not null || e1 is not null || a0 is not null || a1 is not null) ? new(e0, e1, a0, a1) : null;
    private RepeatabilityMeasurement? RepeatDraft()
    {
        if (trials.Count == 0 || !Num(targetQ1, out double q1) || !Num(targetQ2, out double q2) || !Num(approach, out double d)) return null;
        double? P(string text) => double.TryParse(text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) && double.IsFinite(v) ? v : null;
        return new(q1, q2, d, trials.Select(t => new Trial(t.Direction, t.Index, P(t.X), P(t.Y), P(t.Q1Command), P(t.Q1Encoder), P(t.Q2Command))).ToList());
    }
    private CalibrationRecord Draft() => new(1, "", 0, default, host.Firmware, note.Text.Trim(), LinkDraft(), ZeroDraft(), StrokeDraft(), RepeatDraft());

    // Unit vector of the plus-side approach in the plane, from the model: pre-point -> target.
    private Vector? TravelDirection(RepeatabilityMeasurement r)
    {
        if (host.Model is not { } model) return null;
        var target = model.Forward(new JointPose(r.TargetQ1, r.TargetQ2)).Tip; var pre = model.Forward(new JointPose(r.TargetQ1 + r.ApproachDeg, r.TargetQ2)).Tip;
        var v = target - pre; if (v.Length < 1e-9) return null; v.Normalize(); return v;
    }

    internal void Refresh()
    {
        for (int i = 0; i < 4 && i < steps.Items.Count; i++) if (steps.Items[i] is ListBoxItem item && !Equals(item.Content, StepLabel(i))) item.Content = StepLabel(i);
        // 1. links: the difference from what the planner uses
        if (LinkDraft() is { } links && host.Model is { } model)
            linkNote.Text = Lang.T("cal.links.diff", links.L1 - model.L1, links.L2 - model.L2, model.L1, model.L2);
        else linkNote.Text = Lang.T("cal.links.todo", host.Model?.L1 ?? 0, host.Model?.L2 ?? 0);
        // 2. zero: what a tip position implies, if one was measured
        if (Num(tipX, out double x) && Num(tipY, out double y) && LinkDraft() is { } lk)
            zeroNote.Text = Lang.T("cal.zero.derived", Math.Atan2(y, x) * 180 / Math.PI, Math.Sqrt(x * x + y * y) - (lk.L1 + lk.L2), lk.L1 + lk.L2);
        else zeroNote.Text = Lang.T("cal.zero.todo");
        // 3. stroke: the suggestion, with a margin inside the stops
        if (Opt(elbowMin) is { } e0 && Opt(elbowMax) is { } e1 && Num(margin, out double mg) && mg >= 0 && e0 + mg < e1 - mg)
            strokeNote.Text = Lang.T("cal.stroke.suggest", e0 + mg, e1 - mg, mg);
        else strokeNote.Text = Lang.T("cal.stroke.todo");
        // 4. repeatability
        repeatNote.Text = RepeatText();
        saveButton.IsEnabled = Draft() is { } d && Calibrations.Validate(d) is "";
        if (DateTime.UtcNow < messageUntil) return;
        status.Text = host.CanMove(out string why) ? Lang.T("cal.status.canMove") : Lang.T("cal.status.noMove", why);
        status.Foreground = WorkbenchTheme.Sub;
    }
    // A message about what was just done stays up for a while instead of being replaced by the periodic "can I move" line.
    private DateTime messageUntil;
    private void Say(string text, Brush? colour = null, double seconds = 8) { status.Text = text; status.Foreground = colour ?? WorkbenchTheme.Sub; messageUntil = DateTime.UtcNow.AddSeconds(seconds); }

    private string RepeatText()
    {
        if (RepeatDraft() is not { } r) return Lang.T("cal.repeat.todo");
        var stats = RepeatabilityStats.Compute(r.Trials, TravelDirection(r) ?? default);
        string Dir(string name, DirectionStats? s) => s is null ? $"{name}: —" : $"{name}: n={s.Count}, σx={s.StdX:0.000}, σy={s.StdY:0.000}, {Lang.T("cal.repeat.spread")} {s.Spread:0.000} mm";
        return Lang.T("cal.repeat.stats", Dir("+", stats.Plus), Dir("−", stats.Minus), stats.Hysteresis is { } h ? h.ToString("0.000", CultureInfo.InvariantCulture) + " mm" : "—",
            stats.Repeatability is { } rep ? rep.ToString("0.000", CultureInfo.InvariantCulture) + " mm" : "—", stats.Used, stats.Missing);
    }

    // ---- pages ----------------------------------------------------------------------------------------------------
    private StackPanel BuildLinks()
    {
        var p = new StackPanel(); p.Children.Add(Heading("cal.step.links")); p.Children.Add(Help("cal.links.help"));
        p.Children.Add(Row("cal.links.l1", l1, "mm")); p.Children.Add(Row("cal.links.l2", l2, "mm")); p.Children.Add(Row("cal.links.method", method)); p.Children.Add(Row("cal.links.unc", uncertainty, "mm"));
        p.Children.Add(linkNote);
        p.Children.Add(Action("cal.links.apply", ApplyLinks));
        return p;
    }
    private StackPanel BuildZero()
    {
        var p = new StackPanel(); p.Children.Add(Heading("cal.step.zero")); p.Children.Add(Help("cal.zero.help"));
        var reference = new TextBlock { Margin = new Thickness(4, 2, 4, 6), FontWeight = FontWeights.SemiBold }; p.Children.Add(reference);
        poll.Tick += (_, _) => reference.Text = Lang.T("cal.zero.reference", host.ReferenceName.Length > 0 ? host.ReferenceName : Lang.T("cal.zero.noRef"));
        p.Children.Add(Row("cal.zero.q1", q1Offset, "°")); p.Children.Add(Row("cal.zero.q2", q2Offset, "°"));
        p.Children.Add(Row("cal.zero.tipx", tipX, "mm")); p.Children.Add(Row("cal.zero.tipy", tipY, "mm")); p.Children.Add(Row("cal.zero.notes", zeroNotes));
        p.Children.Add(zeroNote);
        return p;
    }
    private StackPanel BuildStroke()
    {
        var p = new StackPanel(); p.Children.Add(Heading("cal.step.stroke")); p.Children.Add(Help("cal.stroke.help"));
        (string key, TextBox box, Func<JointPose, double> read)[] rows = [("cal.stroke.emin", elbowMin, c => c.Q2 - c.Q1), ("cal.stroke.emax", elbowMax, c => c.Q2 - c.Q1), ("cal.stroke.q1min", q1Min, c => c.Q1), ("cal.stroke.q1max", q1Max, c => c.Q1)];
        foreach (var (key, box, read) in rows)
        {
            var line = new StackPanel { Orientation = Orientation.Horizontal }; line.Children.Add(box); line.Children.Add(new TextBlock { Text = "°", VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(0, 0, 8, 0) });
            var capture = Action("cal.stroke.read", () => { if (host.DeviceCommand is { } c) box.Text = read(c).ToString("0.00", CultureInfo.InvariantCulture); }); line.Children.Add(capture);
            poll.Tick += (_, _) => capture.IsEnabled = host.DeviceCommand is not null;
            p.Children.Add(Row(key, line));
        }
        p.Children.Add(Row("cal.stroke.margin", margin, "°")); p.Children.Add(strokeNote);
        p.Children.Add(Action("cal.stroke.apply", ApplyElbow));
        return p;
    }
    private StackPanel BuildRepeat()
    {
        var p = new StackPanel(); p.Children.Add(Heading("cal.step.repeat")); p.Children.Add(Help("cal.repeat.help"));
        p.Children.Add(Row("cal.repeat.q1", targetQ1, "°")); p.Children.Add(Row("cal.repeat.q2", targetQ2, "°")); p.Children.Add(Row("cal.repeat.approach", approach, "°")); p.Children.Add(Row("cal.repeat.n", perDirection));
        var setup = new WrapPanel();
        setup.Children.Add(Action("cal.repeat.useTarget", () => { if (host.PointTarget is { } t) { targetQ1.Text = t.Q1.ToString("0.00", CultureInfo.InvariantCulture); targetQ2.Text = t.Q2.ToString("0.00", CultureInfo.InvariantCulture); } }));
        setup.Children.Add(Action("cal.repeat.generate", GenerateTrials)); p.Children.Add(setup);
        p.Children.Add(trialGrid);
        var run = new WrapPanel { Margin = new Thickness(0, 6, 0, 0) };
        run.Children.Add(Action("cal.repeat.pre", () => MoveSelected(pre: true))); run.Children.Add(Action("cal.repeat.target", () => MoveSelected(pre: false))); run.Children.Add(Action("cal.repeat.record", RecordSelected));
        p.Children.Add(run); p.Children.Add(repeatNote);
        trialGrid.CellEditEnding += (_, _) => Dispatcher.BeginInvoke(new Action(Refresh), System.Windows.Threading.DispatcherPriority.Background);
        return p;
    }
    // The only two things this window can put into the planning parameters, and only when asked.
    internal void ApplyLinks() { if (LinkDraft() is { } l) { host.ApplyLinkLengths(l.L1, l.L2); Say(Lang.T("cal.links.applied", l.L1, l.L2)); Refresh(); } }
    internal void ApplyElbow()
    {
        if (Opt(elbowMin) is { } a && Opt(elbowMax) is { } b && Num(margin, out double m) && m >= 0 && a + m < b - m)
        { host.ApplyElbowRange(Math.Round(a + m, 2), Math.Round(b - m, 2)); Say(Lang.T("cal.stroke.applied", Math.Round(a + m, 2), Math.Round(b - m, 2))); Refresh(); }
    }
    private void BuildTrialColumns()
    {
        trialGrid.Columns.Clear();
        (string key, string prop, bool editable, double width)[] columns = [("cal.col.dir", nameof(TrialRow.Direction), false, 56), ("cal.col.n", nameof(TrialRow.Index), false, 40), ("cal.col.x", nameof(TrialRow.X), true, 150), ("cal.col.y", nameof(TrialRow.Y), true, 150),
            ("cal.col.q1", nameof(TrialRow.Q1Command), false, 90), ("cal.col.enc", nameof(TrialRow.Q1Encoder), false, 100), ("cal.col.q2", nameof(TrialRow.Q2Command), false, 90)];
        foreach (var (key, prop, editable, width) in columns) trialGrid.Columns.Add(new DataGridTextColumn { Header = Lang.T(key), Binding = new System.Windows.Data.Binding(prop), IsReadOnly = !editable, Width = width });
        trialGrid.ItemsSource = null; trialGrid.ItemsSource = trials;
    }
    // Alternating arrivals (+, -, +, -, ...) so slow drift does not masquerade as hysteresis.
    private void GenerateTrials()
    {
        if (!int.TryParse(perDirection.Text.Trim(), out int n) || n is < 2 or > 20) return;
        trials = Enumerable.Range(1, n).SelectMany(i => new[] { new TrialRow { Direction = "+", Index = i }, new TrialRow { Direction = "-", Index = i } }).ToList();
        BuildTrialColumns(); if (trials.Count > 0) trialGrid.SelectedIndex = 0; Refresh();
    }
    private bool TargetPoses(out JointPose target, out JointPose plus, out JointPose minus, out string reason)
    {
        target = plus = minus = default; reason = "";
        if (!Num(targetQ1, out double q1) || !Num(targetQ2, out double q2) || !Num(approach, out double d) || d <= 0 || d > 90) { reason = Lang.T("cal.repeat.badTarget"); return false; }
        target = new(q1, q2); plus = new(q1 + d, q2); minus = new(q1 - d, q2);
        if (host.Limits is { } limits && (!limits.Contains(PtpProtocol.Quantize(plus)) || !limits.Contains(PtpProtocol.Quantize(minus)) || !limits.Contains(PtpProtocol.Quantize(target)))) { reason = Lang.T("cal.repeat.outRange"); return false; }
        return true;
    }
    private void MoveSelected(bool pre)
    {
        string reason = "";
        if (trialGrid.SelectedItem is not TrialRow row || !TargetPoses(out var target, out var plus, out var minus, out reason)) { Say(reason.Length > 0 ? reason : Lang.T("cal.repeat.pick")); return; }
        if (!host.MoveTo(pre ? (row.Direction == "+" ? plus : minus) : target, out reason)) Say(Lang.T("cal.status.noMove", reason));
        else Say(Lang.T(pre ? "cal.repeat.movingPre" : "cal.repeat.movingTarget", row.Direction, row.Index), null, 4);
    }
    private void RecordSelected()
    {
        if (trialGrid.SelectedItem is not TrialRow row) { Say(Lang.T("cal.repeat.pick")); return; }
        if (host.DeviceCommand is not { } c) { Say(Lang.T("cal.status.noTelemetry")); return; }
        row.Q1Command = c.Q1.ToString("0.000", CultureInfo.InvariantCulture); row.Q2Command = c.Q2.ToString("0.000", CultureInfo.InvariantCulture);
        row.Q1Encoder = host.DeviceEncoderQ1?.ToString("0.000", CultureInfo.InvariantCulture) ?? "";
        trialGrid.Items.Refresh(); Refresh();
    }

    private StackPanel BuildSave()
    {
        var p = new StackPanel(); p.Children.Add(Heading("cal.step.save")); p.Children.Add(Help("cal.save.help"));
        p.Children.Add(summary); p.Children.Add(Row("cal.save.note", note));
        saveButton.Click += (_, _) => SaveNow();
        p.Children.Add(saveButton);
        return p;
    }
    internal void RefreshSummary() => summary.Text = SummaryText();
    private string SummaryText()
    {
        var d = Draft(); var text = new System.Text.StringBuilder();
        text.AppendLine(Lang.T("cal.sum.header", host.Firmware, host.ReferenceName.Length > 0 ? host.ReferenceName : "—", Calibrations.NextVersion()));
        string Mark(bool ok) => ok ? "✓" : "○";
        text.AppendLine($"{Mark(d.Links is not null)} {Lang.T("cal.step.links")}" + (d.Links is { } l ? $": L1={Mm(l.L1)} mm, L2={Mm(l.L2)} mm ({l.Method}, ±{Mm(l.UncertaintyMm)} mm)" : ""));
        text.AppendLine($"{Mark(d.Zero is not null)} {Lang.T("cal.step.zero")}" + (d.Zero is { } z ? $": q1 {z.Q1OffsetDeg:+0.00;-0.00;0.00}°, q2 {z.Q2OffsetDeg:+0.00;-0.00;0.00}°" + (z.TipX is { } tx && z.TipY is { } ty ? $", tip ({tx:0.##}, {ty:0.##}) mm" : "") : ""));
        text.AppendLine($"{Mark(d.Stroke is not null)} {Lang.T("cal.step.stroke")}" + (d.Stroke is { } s ? $": {Lang.T("cal.sum.elbow")} [{s.ElbowMin?.ToString("0.##") ?? "—"}, {s.ElbowMax?.ToString("0.##") ?? "—"}]°, q1 [{s.Q1Min?.ToString("0.##") ?? "—"}, {s.Q1Max?.ToString("0.##") ?? "—"}]°" : ""));
        text.AppendLine($"{Mark(d.Repeatability is not null && d.Repeatability.Trials.Any(t => t.X is not null))} {Lang.T("cal.step.repeat")}");
        if (d.Repeatability is not null) text.AppendLine("   " + RepeatText().Replace("\n", "\n   "));
        text.AppendLine(); text.AppendLine(Lang.T("cal.sum.noApply"));
        return text.ToString().TrimEnd();
    }
    internal CalibrationRecord? SaveNow()
    {
        try
        {
            lastSaved = Calibrations.Save(Draft(), host.Firmware);
            Say(Lang.T("cal.saved", lastSaved.Version, lastSaved.Id, Calibrations.Root), WorkbenchTheme.Ok, 60);
            RefreshSummary(); RefreshHistory(); Saved?.Invoke(lastSaved);
            return lastSaved;
        }
        catch (Exception ex) when (ex is ArgumentException or IOException or UnauthorizedAccessException) { Say(Lang.T("cal.saveFail") + ex.Message, WorkbenchTheme.Red, 60); return null; }
    }
    public event Action<CalibrationRecord>? Saved;

    private StackPanel BuildHistory()
    {
        var p = new StackPanel(); p.Children.Add(Heading("cal.step.history")); p.Children.Add(Help("cal.history.help")); p.Children.Add(history);
        var buttons = new WrapPanel();
        buttons.Children.Add(Action("cal.history.activate", () => { if (history.SelectedItem is ListViewItem { Tag: CalibrationRecord r }) { Calibrations.SetActive(r.Id); ActiveChanged?.Invoke(r); RefreshHistory(); } }));
        buttons.Children.Add(Action("cal.history.folder", () => { try { Directory.CreateDirectory(Calibrations.Root); System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo("explorer.exe", Calibrations.Root) { UseShellExecute = true }); } catch (Exception ex) when (ex is IOException or System.ComponentModel.Win32Exception) { status.Text = ex.Message; } }));
        p.Children.Add(buttons);
        return p;
    }
    public event Action<CalibrationRecord>? ActiveChanged;
    internal void RefreshHistory()
    {
        var active = Calibrations.Active();
        history.Items.Clear();
        foreach (var r in Calibrations.List())
        {
            var stats = r.Repeatability is { } rep ? RepeatabilityStats.Compute(rep.Trials, new Vector(1, 0)) : null;
            string text = $"{(r.Id == active?.Id ? "★ " : "  ")}v{r.Version}  {r.CreatedUtc.ToLocalTime():yyyy-MM-dd HH:mm}  " +
                (r.Links is { } l ? $"L1 {l.L1:0.##}/L2 {l.L2:0.##}  " : "") + (r.Zero is { } z ? $"q1 {z.Q1OffsetDeg:+0.##;-0.##;0}°/q2 {z.Q2OffsetDeg:+0.##;-0.##;0}°  " : "") +
                (stats?.Repeatability is { } rr ? $"{Lang.T("cal.repeat.spread")} {rr:0.###} mm  " : "") + (r.Note.Length > 0 ? "— " + r.Note : "");
            history.Items.Add(new ListViewItem { Content = text, Tag = r, FontFamily = WorkbenchTheme.Mono });
        }
        if (history.Items.Count == 0) history.Items.Add(new ListViewItem { Content = Lang.T("cal.history.none"), IsEnabled = false });
    }

    private static TextBlock Heading(string key) => Lang.Bind(new TextBlock { FontSize = 16, FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 0, 0, 6) }, TextBlock.TextProperty, key);

    // ---- for the self-test ----------------------------------------------------------------------------------------
    internal string StatusText => status.Text;
    internal bool CanSave => saveButton.IsEnabled;
    internal string[] StepLabels => steps.Items.Cast<ListBoxItem>().Select(i => i.Content?.ToString() ?? "").ToArray();
    internal string RepeatSummary => repeatNote.Text;
    internal string LinkText => linkNote.Text;
    internal string ZeroText => zeroNote.Text;
    internal string StrokeText => strokeNote.Text;
    internal int TrialCount => trials.Count;
    internal int HistoryCount => history.Items.Cast<ListViewItem>().Count(i => i.Tag is CalibrationRecord);
    internal void Set(string field, string value)
    {
        (field switch { "l1" => l1, "l2" => l2, "unc" => uncertainty, "q1off" => q1Offset, "q2off" => q2Offset, "tipx" => tipX, "tipy" => tipY, "emin" => elbowMin, "emax" => elbowMax, "q1min" => q1Min, "q1max" => q1Max, "margin" => margin,
            "tq1" => targetQ1, "tq2" => targetQ2, "approach" => approach, "n" => perDirection, "note" => note, _ => throw new ArgumentException(field) }).Text = value;
    }
    internal void Generate() => GenerateTrials();
    internal void FillTrial(int index, double x, double y) { trials[index].X = x.ToString(CultureInfo.InvariantCulture); trials[index].Y = y.ToString(CultureInfo.InvariantCulture); trialGrid.Items.Refresh(); Refresh(); }
    internal void SelectTrial(int index) => trialGrid.SelectedIndex = index;
    internal void Move(bool pre) => MoveSelected(pre);
    internal void Record() => RecordSelected();
    internal void ClickSteps(int index) => steps.SelectedIndex = index;
    internal string Selected(int index, string what) => what switch { "q1" => trials[index].Q1Command, "enc" => trials[index].Q1Encoder, "q2" => trials[index].Q2Command, _ => "" };
    internal string SummaryShown => summary.Text;
    internal void ChooseActive(int listIndex) { history.SelectedIndex = listIndex; if (history.SelectedItem is ListViewItem { Tag: CalibrationRecord r }) { Calibrations.SetActive(r.Id); ActiveChanged?.Invoke(r); RefreshHistory(); } }
}
