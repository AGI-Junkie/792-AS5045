using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

// The device's own log of a session, indexed by time of day, so a replay can show what the firmware and host said around the cursor.
internal sealed class LogIndex
{
    public double[] Seconds { get; }
    public string[] Lines { get; }
    public string Path { get; }
    private LogIndex(string path, double[] seconds, string[] lines) { Path = path; Seconds = seconds; Lines = lines; }

    public static LogIndex? Load(string path)
    {
        var seconds = new List<double>(); var lines = new List<string>();
        try
        {
            foreach (string line in File.ReadLines(path))
            {
                // "[19:08:46.137] text": local time of day of the host receiving the line.
                if (line.Length < 15 || line[0] != '[' || line[13] != ']' || !TimeSpan.TryParseExact(line.AsSpan(1, 12), @"hh\:mm\:ss\.fff", CultureInfo.InvariantCulture, out var t)) continue;
                seconds.Add(t.TotalSeconds); lines.Add(line);
            }
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { return null; }
        return new LogIndex(path, seconds.ToArray(), lines.ToArray());
    }

    // The session log that was open when the run started: its name carries the connect time, its last write the end.
    public static string? Find(string logRoot, DateTime startLocal)
    {
        if (!Directory.Exists(logRoot)) return null;
        string? best = null; DateTime bestStart = DateTime.MinValue;
        foreach (string file in Directory.EnumerateFiles(logRoot, "stage1-*.log"))
        {
            string name = System.IO.Path.GetFileNameWithoutExtension(file);   // stage1-20261001-190846-103
            var parts = name.Split('-');
            if (parts.Length < 3 || !DateTime.TryParseExact(parts[1] + parts[2], "yyyyMMddHHmmss", CultureInfo.InvariantCulture, DateTimeStyles.None, out var start)) continue;
            if (start <= startLocal.AddSeconds(2) && File.GetLastWriteTime(file) >= startLocal.AddSeconds(-2) && start > bestStart) { best = file; bestStart = start; }
        }
        return best;
    }

    // Lines within `radius` seconds of `time` (time of day), at most `limit`, nearest the cursor first-class: the window is centred on it.
    public IEnumerable<string> Around(double time, double radius, int limit, Func<string, bool>? keep = null)
    {
        int lo = Array.BinarySearch(Seconds, time - radius); if (lo < 0) lo = ~lo;
        int count = 0;
        for (int i = lo; i < Seconds.Length && Seconds[i] <= time + radius && count < limit; i++)
            if (keep is null || keep(Lines[i])) { count++; yield return Lines[i]; }
    }
}

// Opens finished recordings: a list to pick from (two can be laid over each other), the error and the plan's demands drawn against
// time, the path in the plane, and a replay cursor that moves every view and the log together.
internal sealed partial class ExperimentWindow : Window
{
    internal static string? LogRootOverride;
    private static string LogRoot => LogRootOverride ?? System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "Logs");

    private sealed class Row
    {
        public required ExperimentRun Run { get; init; }
        public string Time { get; set; } = ""; public string Kind { get; set; } = ""; public string Shape { get; set; } = ""; public string Speed { get; set; } = "";
        public string Reference { get; set; } = ""; public string Status { get; set; } = ""; public string Rmse { get; set; } = ""; public string Peak { get; set; } = "";
        public string Hz { get; set; } = ""; public string Fault { get; set; } = ""; public string Calibration { get; set; } = "";
    }

    private readonly string root;
    private readonly DataGrid grid = new() { AutoGenerateColumns = false, IsReadOnly = true, SelectionMode = DataGridSelectionMode.Extended, CanUserAddRows = false, HeadersVisibility = DataGridHeadersVisibility.Column, GridLinesVisibility = DataGridGridLinesVisibility.Horizontal, HorizontalGridLinesBrush = WorkbenchTheme.GridLine, RowHeaderWidth = 0, FontSize = 12, Background = Brushes.White, MinHeight = 120 };
    private readonly TextBlock listHint = new() { TextWrapping = TextWrapping.Wrap, FontSize = 11, Foreground = WorkbenchTheme.Muted, Margin = new Thickness(8, 4, 8, 4) };
    private readonly TextBox summary = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, FontSize = 12, BorderThickness = new Thickness(0), Background = Brushes.Transparent, Margin = new Thickness(6), VerticalScrollBarVisibility = ScrollBarVisibility.Auto, VerticalContentAlignment = VerticalAlignment.Top, VerticalAlignment = VerticalAlignment.Stretch };
    private readonly PlotView qPlot = new(), errPlot = new(), speedPlot = new(), accelPlot = new();
    private readonly XYView xy = new() { MinHeight = 240 };
    private readonly Slider timeSlider = new() { Minimum = 0, Maximum = 1, MinWidth = 260, Margin = new Thickness(6, 2, 6, 2), VerticalAlignment = VerticalAlignment.Center };
    private readonly Button play = new() { MinWidth = 90, Margin = new Thickness(4) }, refresh = new() { Margin = new Thickness(4), Padding = new Thickness(10, 2, 10, 2) }, openFolder = new() { Margin = new Thickness(4), Padding = new Thickness(10, 2, 10, 2) },
        exportList = new() { Margin = new Thickness(4), Padding = new Thickness(10, 2, 10, 2) };
    private readonly ComboBox speed = new() { Width = 70, Margin = new Thickness(4) };
    private readonly TextBlock timeText = new() { Margin = new Thickness(8, 0, 8, 0), VerticalAlignment = VerticalAlignment.Center, FontFamily = WorkbenchTheme.Mono, FontSize = 12 };
    private readonly TextBox logBox = new() { IsReadOnly = true, FontFamily = WorkbenchTheme.Mono, FontSize = 11, HorizontalScrollBarVisibility = ScrollBarVisibility.Auto, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, MinHeight = 90, TextWrapping = TextWrapping.NoWrap };
    private readonly CheckBox quietLog = new() { IsChecked = true, Margin = new Thickness(6, 2, 6, 2), FontSize = 11 };
    private readonly TextBlock logTitle = new() { FontWeight = FontWeights.SemiBold, FontSize = 11, Margin = new Thickness(6, 4, 6, 0) };
    private readonly CheckBox wholeArm = new() { Margin = new Thickness(6, 2, 6, 2), FontSize = 11 };
    private readonly DispatcherTimer timer = new() { Interval = TimeSpan.FromMilliseconds(33) };
    private readonly Dictionary<string, RunAnalysis> cache = [];
    private readonly List<RunAnalysis> shown = [];
    private List<Row> rows = [];
    private int unreadable;
    private double tMin, tMax, cursor;
    private bool settingSlider;
    private LogIndex? log;
    private string? logFor;

    private static readonly Brush[][] Colours = [
        [WorkbenchTheme.Blue, WorkbenchTheme.Teal, WorkbenchTheme.Brush("#7B8794"), WorkbenchTheme.Brush("#7C3AED")],
        [WorkbenchTheme.Brush("#C66B05"), WorkbenchTheme.Brush("#B7791F"), WorkbenchTheme.Brush("#A8A29E"), WorkbenchTheme.Brush("#DB2777")]];

    public ExperimentWindow(string experimentRoot)
    {
        root = experimentRoot;
        Width = 1320; Height = 860; MinWidth = 960; MinHeight = 640; Background = WorkbenchTheme.Background; Foreground = WorkbenchTheme.Ink;
        FontFamily = new FontFamily("Microsoft YaHei UI, Segoe UI"); FontSize = 12;
        WorkbenchTheme.Apply(this);
        var shell = new Grid(); Content = shell;
        shell.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(500), MinWidth = 300 });
        shell.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
        shell.ColumnDefinitions.Add(new ColumnDefinition { MinWidth = 520 });
        // Left: the list, its buttons, and the written summary of what is shown.
        var left = new Grid(); left.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto }); left.RowDefinitions.Add(new RowDefinition { Height = new GridLength(2, GridUnitType.Star), MinHeight = 130 });
        left.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto }); left.RowDefinitions.Add(new RowDefinition { Height = new GridLength(3, GridUnitType.Star), MinHeight = 120 });
        var bar = new WrapPanel { Margin = new Thickness(4) }; bar.Children.Add(refresh); bar.Children.Add(openFolder); bar.Children.Add(exportList);
        left.Children.Add(WorkbenchTheme.Edge(new StackPanel { Children = { bar, listHint } }, WorkbenchTheme.Bar));
        Grid.SetRow(grid, 1); left.Children.Add(grid);
        var summaryTitle = WorkbenchTheme.Caption("exp.summary"); Grid.SetRow(summaryTitle, 2); left.Children.Add(summaryTitle);
        var summaryBox = new Border { Background = Brushes.White, Child = summary }; Grid.SetRow(summaryBox, 3); left.Children.Add(summaryBox);
        shell.Children.Add(left);
        var splitter = new GridSplitter { Width = 5, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Stretch, Background = WorkbenchTheme.Line, ResizeBehavior = GridResizeBehavior.PreviousAndNext }; Grid.SetColumn(splitter, 1); shell.Children.Add(splitter);
        // Right: replay bar on top, charts and the plane in the middle, the log below.
        var right = new Grid(); Grid.SetColumn(right, 2);
        right.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto }); right.RowDefinitions.Add(new RowDefinition { Height = new GridLength(5, GridUnitType.Star) }); right.RowDefinitions.Add(new RowDefinition { Height = new GridLength(2, GridUnitType.Star), MinHeight = 110 });
        var replay = new DockPanel { Margin = new Thickness(4) };
        DockPanel.SetDock(play, Dock.Left); replay.Children.Add(play); DockPanel.SetDock(speed, Dock.Left); replay.Children.Add(speed);
        DockPanel.SetDock(timeText, Dock.Right); replay.Children.Add(timeText); replay.Children.Add(timeSlider);
        right.Children.Add(WorkbenchTheme.Edge(replay, WorkbenchTheme.Bar));
        var middle = new Grid(); Grid.SetRow(middle, 1);
        middle.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(3, GridUnitType.Star) }); middle.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(2, GridUnitType.Star), MinWidth = 260 });
        var charts = new StackPanel(); foreach (var plot in new[] { qPlot, errPlot, speedPlot, accelPlot }) { plot.Height = 190; plot.Margin = new Thickness(2, 2, 2, 4); plot.CursorChanged += t => { PausePlayback(); SetCursor(t); }; charts.Children.Add(plot); }
        middle.Children.Add(new ScrollViewer { Content = charts, VerticalScrollBarVisibility = ScrollBarVisibility.Auto });
        var plane = new DockPanel { Margin = new Thickness(2) }; Grid.SetColumn(plane, 1);
        DockPanel.SetDock(wholeArm, Dock.Top); plane.Children.Add(wholeArm); plane.Children.Add(xy);
        middle.Children.Add(plane);
        right.Children.Add(middle);
        var logPanel = new DockPanel(); Grid.SetRow(logPanel, 2);
        var logHead = new DockPanel(); DockPanel.SetDock(quietLog, Dock.Right); logHead.Children.Add(quietLog); logHead.Children.Add(logTitle); DockPanel.SetDock(logHead, Dock.Top);
        logPanel.Children.Add(logHead); logPanel.Children.Add(logBox);
        right.Children.Add(WorkbenchTheme.Edge(logPanel, Brushes.White, new Thickness(0, 1, 0, 0)));
        shell.Children.Add(right);

        speed.Items.Add(new ComboBoxItem { Content = "0.5×", Tag = 0.5 }); speed.Items.Add(new ComboBoxItem { Content = "1×", Tag = 1.0 }); speed.Items.Add(new ComboBoxItem { Content = "2×", Tag = 2.0 }); speed.Items.Add(new ComboBoxItem { Content = "5×", Tag = 5.0 });
        speed.SelectedIndex = 1;
        grid.SelectionChanged += (_, _) => ShowSelection();
        timeSlider.ValueChanged += (_, _) => { if (!settingSlider) { PausePlayback(); SetCursor(timeSlider.Value); } };
        play.Click += (_, _) => TogglePlayback();
        refresh.Click += (_, _) => Reload();
        exportList.Click += (_, _) => ExportList();
        openFolder.Click += (_, _) => { try { Directory.CreateDirectory(root); System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo("explorer.exe", root) { UseShellExecute = true }); } catch (Exception ex) when (ex is IOException or System.ComponentModel.Win32Exception) { summary.Text = ex.Message; } };
        quietLog.Checked += (_, _) => UpdateLog(); quietLog.Unchecked += (_, _) => UpdateLog();
        wholeArm.Checked += (_, _) => xy.WholeArm = true; wholeArm.Unchecked += (_, _) => xy.WholeArm = false;
        timer.Tick += (_, _) => Advance(0.033 * (speed.SelectedItem is ComboBoxItem { Tag: double s } ? s : 1));
        Lang.Changed += ApplyLanguage; Closed += (_, _) => { Lang.Changed -= ApplyLanguage; timer.Stop(); };
        ApplyLanguage();
        Reload();
    }

    // ---- list ------------------------------------------------------------------------------------------------------
    private string ShapeName(ExperimentRun run) => run.Settings is { } s && s.Shape >= 0 && s.Shape <= 10 ? Lang.T("xy.shape" + s.Shape) : "—";
    private Row MakeRow(ExperimentRun run)
    {
        var s = run.Settings;
        return new Row {
            Run = run, Time = run.StartedUtc.ToLocalTime().ToString("MM-dd HH:mm:ss", CultureInfo.InvariantCulture), Kind = Lang.T(run.Kind == "point" ? "exp.kind.point" : "exp.kind.trajectory"), Shape = ShapeName(run),
            Speed = s is null ? "—" : $"{s.V1:0.#} / {s.V2:0.#}", Reference = run.Reference,
            Status = run.Complete ? Lang.T("exp.status.complete") : run.Outcome == "interrupted" ? Lang.T("exp.status.interrupted") : Lang.T("exp.status.other", run.Outcome),
            Rmse = run.Rmse is { } r ? r.ToString("0.0000", CultureInfo.InvariantCulture) : "—", Peak = run.Peak is { } p ? p.ToString("0.0000", CultureInfo.InvariantCulture) : "—",
            Hz = run.TelemetryHz > 0 ? run.TelemetryHz + " Hz" : "~2 Hz", Fault = run.HasFault ? "⚠" : "", Calibration = run.CalibrationVersion.Length > 0 ? run.CalibrationVersion : "—" };
    }
    private void BuildColumns()
    {
        grid.Columns.Clear();
        (string key, string property, double width)[] columns = [("exp.col.time", nameof(Row.Time), 100), ("exp.col.kind", nameof(Row.Kind), 48), ("exp.col.shape", nameof(Row.Shape), 70), ("exp.col.speed", nameof(Row.Speed), 96),
            ("exp.col.ref", nameof(Row.Reference), 96), ("exp.col.status", nameof(Row.Status), 110), ("exp.col.rmse", nameof(Row.Rmse), 64), ("exp.col.peak", nameof(Row.Peak), 64), ("exp.col.hz", nameof(Row.Hz), 52), ("exp.col.cal", nameof(Row.Calibration), 46), ("exp.col.fault", nameof(Row.Fault), 36)];
        foreach (var (key, property, width) in columns)
            grid.Columns.Add(new DataGridTextColumn { Header = Lang.T(key), Binding = new System.Windows.Data.Binding(property), Width = width, SortMemberPath = property });
    }

    private void Reload()
    {
        var selected = grid.SelectedItems.OfType<Row>().Select(r => r.Run.Directory).ToHashSet();
        var (runs, bad) = ExperimentArchive.List(root); unreadable = bad;
        rows = runs.Select(MakeRow).ToList();
        grid.ItemsSource = rows;
        foreach (var row in rows.Where(r => selected.Contains(r.Run.Directory))) grid.SelectedItems.Add(row);
        listHint.Text = rows.Count == 0 ? Lang.T("exp.sum.none") : Lang.T("exp.hint") + (unreadable > 0 ? " " + Lang.T("exp.sum.unreadable", unreadable) : "");
        ShowSelection();
    }

    private void ApplyLanguage()
    {
        Title = Lang.T("exp.title");
        refresh.Content = Lang.T("exp.refresh"); openFolder.Content = Lang.T("btn.openData"); exportList.Content = Lang.T("exp.export");
        quietLog.Content = Lang.T("exp.log.quiet"); wholeArm.Content = Lang.T("exp.xy.whole");
        logTitle.Text = Lang.T("exp.log.title");
        play.Content = timer.IsEnabled ? Lang.T("exp.pause") : Lang.T("exp.play");
        BuildColumns();
        if (rows.Count > 0) { var selected = grid.SelectedItems.OfType<Row>().Select(r => r.Run.Directory).ToHashSet(); rows = rows.Select(r => MakeRow(r.Run)).ToList(); grid.ItemsSource = rows; foreach (var row in rows.Where(r => selected.Contains(r.Run.Directory))) grid.SelectedItems.Add(row); }
        listHint.Text = rows.Count == 0 ? Lang.T("exp.sum.none") : Lang.T("exp.hint") + (unreadable > 0 ? " " + Lang.T("exp.sum.unreadable", unreadable) : "");
        if (shown.Count > 0) ShowSelection();
    }

    // The table as a CSV: what to hand over with the thesis figures, and the check that each drawing's plan lines up with its samples.
    private void ExportList()
    {
        var dialog = new Microsoft.Win32.SaveFileDialog { Title = Lang.T("exp.export"), Filter = Lang.T("vp.csvFilter"), DefaultExt = ".csv", FileName = $"experiments-{DateTime.Now:yyyyMMdd-HHmmss}.csv" };
        if (dialog.ShowDialog(this) != true) return;
        try { File.WriteAllText(dialog.FileName, ExperimentArchive.ReportCsv(rows.Select(r => r.Run)), new System.Text.UTF8Encoding(true)); summary.Text = Lang.T("exp.exported", dialog.FileName); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { summary.Text = Lang.T("exp.exportFail") + ex.Message; }
    }

    // ---- selection -> charts -----------------------------------------------------------------------------------------
    private RunAnalysis Analysis(ExperimentRun run)
    {
        if (!cache.TryGetValue(run.Directory, out var a)) cache[run.Directory] = a = RunAnalysis.Create(run, ExperimentArchive.Load(run));
        return a;
    }

    private void ShowSelection()
    {
        PausePlayback();
        var selected = grid.SelectedItems.OfType<Row>().Select(r => r.Run).Take(2).ToList();
        shown.Clear();
        foreach (var run in selected)
        {
            try { shown.Add(Analysis(run)); }
            catch (Exception ex) when (ex is IOException or FormatException or UnauthorizedAccessException) { summary.Text = Lang.T("exp.sum.readFail", run.Id) + ex.Message; }
        }
        foreach (var plot in new[] { qPlot, errPlot, speedPlot, accelPlot }) plot.Clear();
        xy.Clear(); logBox.Clear(); log = null; logFor = null;
        if (shown.Count == 0)
        {
            foreach (var plot in new[] { qPlot, errPlot, speedPlot, accelPlot }) { plot.Title = ""; plot.InvalidateVisual(); }
            xy.InvalidateVisual(); timeText.Text = ""; summary.Text = rows.Count == 0 ? Lang.T("exp.sum.none") : Lang.T("exp.hint");
            return;
        }
        bool compare = shown.Count == 2;
        tMin = shown.Min(a => Math.Min(a.T.Length > 0 ? a.T[0] : 0, a.HasPlan ? 0 : double.MaxValue)); tMax = shown.Max(a => Math.Max(a.T.Length > 0 ? a.T[^1] : 0, a.Duration));
        if (tMax - tMin < 0.5) tMax = tMin + 0.5;
        for (int i = 0; i < shown.Count; i++)
        {
            var a = shown[i]; var c = Colours[i]; string tag = compare ? (i == 0 ? "A " : "B ") : "";
            qPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.cmd"), a.T, a.Q1Command, c[0]) { Unit = "°" });
            qPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.enc"), a.T, a.Q1Encoder, c[1]) { Dashed = true, Thickness = 1.3, Unit = "°" });
            if(a.Q2Measured.Length==a.T.Length && a.T.Length>0) {
                qPlot.Series.Add(new PlotSeries(tag + "q2 " + Lang.T("exp.s.cmd"),a.T,a.Q2Command,c[3]) { Unit="°" });
                qPlot.Series.Add(new PlotSeries(tag + "q2 " + Lang.T("exp.s.enc"),a.T,a.Q2Measured,c[1]) { Unit="°",Dashed=true });
                errPlot.Series.Add(new PlotSeries(tag + "q2 " + Lang.T("exp.s.err"),a.T,a.Q2Error,c[3]) { Unit="°",Decimals=4 });
            }
            if (a.HasPlan) qPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.plan"), a.PlanT, a.PlanQ1, c[2]) { Dashed = true, Thickness = 1.1, Unit = "°" });
            errPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.err"), a.T, a.Q1Error, c[0]) { Unit = "°", Decimals = 4, Dots = a.T.Length < 120 });
            if (a.T.Length > 0) { errPlot.Lines.Add(new PlotLine(a.Rmse, tag + "RMSE " + a.Rmse.ToString("0.0000", CultureInfo.InvariantCulture), c[0])); errPlot.Lines.Add(new PlotLine(-a.Rmse, "", c[0])); }
            foreach (var ev in a.Events)
            {
                int dwell = a.Events.Where(e => e.Kind == "dwell" && e.Time <= ev.Time).Count();
                string label = ev.Kind switch { "start" => Lang.T("exp.ev.start"), "stop" => Lang.T("exp.ev.stop"), _ => Lang.T("exp.ev.dwell", dwell) };
                var mark = new PlotMark(ev.Time, tag + label, c[0], Labelled: i == 0);
                errPlot.Marks.Add(mark); qPlot.Marks.Add(mark with { Labelled = false });
            }
            if (ExperimentArchive.ReadFault(a.Run) is { } fault && a.Aligned)
                errPlot.Marks.Add(new PlotMark((fault.DeviceMs - a.AlignmentOffsetMs) / 1000.0, tag + Lang.T("exp.ev.fault") + " " + fault.Reason, WorkbenchTheme.Red));
            if (a.HasPlan)
            {
                speedPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.m1"), a.PlanVT, a.PlanV1, c[0]) { Unit = "°/s", Decimals = 2 });
                speedPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.m2"), a.PlanVT, a.PlanV2, c[3]) { Unit = "°/s", Decimals = 2 });
                accelPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.m1"), a.PlanAT, a.PlanA1, c[0]) { Unit = "°/s²", Decimals = 2 });
                accelPlot.Series.Add(new PlotSeries(tag + Lang.T("exp.s.m2"), a.PlanAT, a.PlanA2, c[3]) { Unit = "°/s²", Decimals = 2 });
                if (a.Axes.Length == 4)
                {
                    speedPlot.Lines.Add(new PlotLine(a.Axes[0].Cap, tag + Lang.T("exp.s.cap1", a.Axes[0].Cap), c[0])); speedPlot.Lines.Add(new PlotLine(-a.Axes[0].Cap, "", c[0]));
                    speedPlot.Lines.Add(new PlotLine(a.Axes[1].Cap, tag + Lang.T("exp.s.cap2", a.Axes[1].Cap), c[3])); speedPlot.Lines.Add(new PlotLine(-a.Axes[1].Cap, "", c[3]));
                    accelPlot.Lines.Add(new PlotLine(a.Axes[2].Cap, tag + Lang.T("exp.s.cap1", a.Axes[2].Cap), c[0])); accelPlot.Lines.Add(new PlotLine(-a.Axes[2].Cap, "", c[0]));
                    accelPlot.Lines.Add(new PlotLine(a.Axes[3].Cap, tag + Lang.T("exp.s.cap2", a.Axes[3].Cap), c[3])); accelPlot.Lines.Add(new PlotLine(-a.Axes[3].Cap, "", c[3]));
                }
                foreach (var ev in a.Events) speedPlot.Marks.Add(new PlotMark(ev.Time, "", c[0], false));
            }
            // The plane: the plan, the commanded path and the hybrid estimate, all of them model estimates.
            if (a.HasPlan) xy.Paths.Add(new XYView.Path(tag + Lang.T("exp.xy.plan"), a.PlanX, a.PlanY, c[2], 1.4, true));
            xy.Paths.Add(new XYView.Path(tag + Lang.T("exp.xy.cmd"), a.XCommand, a.YCommand, c[0], 1.8));
            xy.Paths.Add(new XYView.Path(tag + Lang.T("exp.xy.hybrid"), a.XHybrid, a.YHybrid, c[1], 1.3));
        }
        qPlot.Title = Lang.T("exp.chart.q1"); errPlot.Title = Lang.T("exp.chart.err"); speedPlot.Title = Lang.T("exp.chart.speed"); accelPlot.Title = Lang.T("exp.chart.accel");
        errPlot.SymmetricY = true; speedPlot.SymmetricY = true; accelPlot.SymmetricY = true;
        qPlot.Source = Lang.T("exp.src.q1"); errPlot.Source = Lang.T("exp.src.err"); speedPlot.Source = Lang.T("exp.src.plan"); accelPlot.Source = Lang.T("exp.src.plan");
        for (int i = 0; i < shown.Count; i++) errPlot.Notes.Add((compare ? (i == 0 ? "A " : "B ") : "") + Lang.T("exp.sum.err", shown[i].Rmse, shown[i].Peak, shown[i].T.Length, shown[i].Run.TelemetryHz > 0 ? shown[i].Run.TelemetryHz + " Hz" : "~2 Hz"));
        for (int i = 0; i < shown.Count; i++) speedPlot.Notes.Add((compare ? (i == 0 ? "A " : "B ") : "") + LimiterText(shown[i]));
        xy.Source = Lang.T("exp.src.xy"); xy.ReachRadius = shown[0].Model.OuterRadius;
        foreach (var plot in new[] { qPlot, errPlot, speedPlot, accelPlot }) plot.XRange = (tMin, tMax);
        settingSlider = true; timeSlider.Minimum = tMin; timeSlider.Maximum = tMax; settingSlider = false;
        summary.Text = SummaryText();
        SetCursor(tMin);
    }

    private static string LimiterText(RunAnalysis a)
    {
        if (a.Axes.Length != 4) return Lang.T("exp.limit.unknown");
        string Name(AxisUse x) => Lang.T(x.Name is "v1" or "a1" ? "exp.s.m1" : "exp.s.m2") + (x.Name[0] == 'v' ? Lang.T("exp.limit.speed") : Lang.T("exp.limit.accel"));
        var near = a.Axes.Where(x => x.Utilization >= 0.9).Select(x => $"{Name(x)} {x.Utilization:P0}").ToArray();
        return near.Length > 0 ? Lang.T("exp.limit.some", string.Join("，", near)) : Lang.T("exp.limit.none", a.Axes.Max(x => x.Utilization));
    }

    private string SummaryText()
    {
        var text = new System.Text.StringBuilder();
        for (int i = 0; i < shown.Count; i++)
        {
            var a = shown[i]; var run = a.Run; var s = run.Settings;
            string tag = shown.Count == 2 ? (i == 0 ? "A · " : "B · ") : "";
            text.AppendLine(tag + Lang.T("exp.sum.run", run.StartedUtc.ToLocalTime().ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture), Lang.T(run.Kind == "point" ? "exp.kind.point" : "exp.kind.trajectory"), ShapeName(run),
                s?.V1 ?? 0, s?.V2 ?? 0, s?.A1 ?? 0, s?.A2 ?? 0, run.Reference, run.Complete ? Lang.T("exp.status.complete") : run.Outcome == "interrupted" ? Lang.T("exp.status.interrupted") : Lang.T("exp.status.other", run.Outcome)));
            text.AppendLine(Lang.T("exp.sum.cal", run.CalibrationId.Length > 0 ? run.CalibrationId : Lang.T("exp.sum.noCal")));
            if (a.T.Length == 0) { text.AppendLine(Lang.T("exp.sum.nodata")); text.AppendLine(); continue; }
            text.AppendLine(Lang.T("exp.sum.err", a.Rmse, a.Peak, a.T.Length, run.TelemetryHz > 0 ? run.TelemetryHz + " Hz" : "~2 Hz"));
            if (a.Segments.Length > 0) text.AppendLine(Lang.T("exp.sum.seg", string.Join("  ", a.Segments.Select(g => $"{g.Index}: {g.Rmse:0.0000}"))));
            if (a.HasPlan) text.AppendLine(Lang.T("exp.sum.limit", LimiterText(a)));
            if (a.Aligned) text.AppendLine(a.AlignmentResidualDeg <= 1 ? Lang.T("exp.sum.align.good", a.AlignmentOffsetMs, a.AlignmentResidualDeg) : Lang.T("exp.sum.align.poor", a.AlignmentResidualDeg));
            if (a.Data.SkippedRows > 0) text.AppendLine(Lang.T("exp.sum.skipped", a.Data.SkippedRows));
            if (ExperimentArchive.ReadFault(run) is { } fault)
                text.AppendLine(Lang.T("exp.sum.fault", Lang.T("snap.phase." + (fault.Phase is "home" or "ptp" or "traj_run" or "traj_upload" or "jog" or "idle" or "disabled" ? fault.Phase : "other")), fault.Reason, fault.Driver ? " — " + Lang.T("snap.drvUndetermined") : ""));
            text.AppendLine();
        }
        if (shown.Count == 2 && shown[0].T.Length > 0 && shown[1].T.Length > 0)
        {
            string Change(double a, double b) => a <= 0 ? "—" : (b / a - 1).ToString("+0%;-0%;0%", CultureInfo.InvariantCulture);
            text.AppendLine(Lang.T("exp.sum.compare", Change(shown[0].Rmse, shown[1].Rmse), Change(shown[0].Peak, shown[1].Peak), Change(shown[0].Duration, shown[1].Duration)));
        }
        if (grid.SelectedItems.Count > 2) text.AppendLine(Lang.T("exp.sum.maxTwo"));
        return text.ToString().TrimEnd();
    }

    // ---- replay -----------------------------------------------------------------------------------------------------
    internal void SetCursor(double t)
    {
        cursor = Math.Clamp(t, tMin, tMax);
        settingSlider = true; timeSlider.Value = cursor; settingSlider = false;
        foreach (var plot in new[] { qPlot, errPlot, speedPlot, accelPlot }) plot.CursorX = cursor;
        xy.Arms.Clear();
        for (int i = 0; i < shown.Count; i++)
        {
            var a = shown[i]; if (a.T.Length == 0) continue;
            double q1 = a.At(a.Q1Command, cursor), q2 = a.At(a.Q2Command, cursor), enc = a.At(a.Q1Encoder, cursor);
            var (elbow, tip) = a.Model.Forward(new JointPose(q1, q2));
            xy.Arms.Add(new XYView.Arm((i == 0 ? "A" : "B") + " cmd", [new Point(0, 0), elbow, tip], Colours[i][0], false));
            var (hElbow, hTip) = a.Model.Forward(new JointPose(enc, a.Q2Measured.Length==a.T.Length ? a.At(a.Q2Measured,cursor) : enc + (q2 - q1)));
            xy.Arms.Add(new XYView.Arm((i == 0 ? "A" : "B") + " hybrid", [new Point(0, 0), hElbow, hTip], Colours[i][1], true));
        }
        xy.InvalidateVisual();
        timeText.Text = Lang.T("exp.time", cursor, tMax);
        UpdateLog();
    }

    private void TogglePlayback()
    {
        if (timer.IsEnabled) { PausePlayback(); return; }
        if (shown.Count == 0) return;
        if (cursor >= tMax - 1e-6) SetCursor(tMin);
        timer.Start(); play.Content = Lang.T("exp.pause");
    }
    internal void PausePlayback() { timer.Stop(); play.Content = Lang.T("exp.play"); }
    internal void Advance(double seconds) { if (cursor + seconds >= tMax) { SetCursor(tMax); PausePlayback(); } else SetCursor(cursor + seconds); }

    private void UpdateLog()
    {
        if (shown.Count == 0) { logBox.Clear(); return; }
        var a = shown[0];
        if (logFor != a.Run.Directory)
        {
            logFor = a.Run.Directory;
            string? file = LogIndex.Find(LogRoot, a.Run.StartedUtc.ToLocalTime());
            log = file is null ? null : LogIndex.Load(file);
        }
        if (log is null) { logBox.Text = Lang.T("exp.log.none"); return; }
        if (a.T.Length == 0) { logBox.Clear(); return; }
        int i = 0; double best = double.MaxValue;
        for (int k = 0; k < a.T.Length; k++) { double d = Math.Abs(a.T[k] - cursor); if (d < best) { best = d; i = k; } }
        var host = a.HostUtc[i].AddSeconds(cursor - a.T[i]).ToLocalTime();
        double tod = host.TimeOfDay.TotalSeconds;
        static bool Quiet(string line)
        {
            string body = line.Length > 15 ? line[15..] : line;
            return !(body.StartsWith("TELEM ") || body.StartsWith("TELEM_DIAG") || body.StartsWith("TIMING ") || body.StartsWith("ENC_DIAG") || body.StartsWith("PTP_RX") || body.StartsWith("PTP_STATE") ||
                     body.StartsWith("MOTION_RATES") || body.StartsWith("PTP_CHECK") || body.StartsWith("CONTROL ") || body.StartsWith("STATE ") || body.StartsWith("LINK ") || body.StartsWith("TRAJ_STATE"));
        }
        var lines = log.Around(tod, 1.0, 60, quietLog.IsChecked == true ? Quiet : null).ToList();
        logBox.Text = lines.Count == 0 ? Lang.T("exp.log.empty") : string.Join("\n", lines);
    }

    // ---- for the self-test ------------------------------------------------------------------------------------------
    internal PlotView[] Plots => [qPlot, errPlot, speedPlot, accelPlot];
    internal XYView Plane => xy;
    internal int ListedRuns => rows.Count;
    internal int ShownRuns => shown.Count;
    internal double CursorTime => cursor;
    internal (double Min, double Max) Range => (tMin, tMax);
    internal string SummaryText_ => summary.Text;
    internal string LogText => logBox.Text;
    internal bool Playing => timer.IsEnabled;
    internal void SelectForSelfTest(params int[] indexes) { grid.SelectedItems.Clear(); foreach (int i in indexes) grid.SelectedItems.Add(rows[i]); }
    internal string[] ColumnHeaders => grid.Columns.Select(c => c.Header?.ToString() ?? "").ToArray();
    internal string RowText(int index) => string.Join("|", new[] { rows[index].Time, rows[index].Kind, rows[index].Shape, rows[index].Speed, rows[index].Reference, rows[index].Status, rows[index].Rmse, rows[index].Peak, rows[index].Hz, rows[index].Calibration, rows[index].Fault });
    internal bool QuietLog { get => quietLog.IsChecked == true; set => quietLog.IsChecked = value; }
    internal void TogglePlayForSelfTest() => TogglePlayback();
}
