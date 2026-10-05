using System.Diagnostics;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private readonly TextBlock connectionBadge = WorkbenchTheme.Text(Lang.T("badge.offline"), 12, Brushes.White);
    private readonly TextBlock referenceStatus = WorkbenchTheme.Text(Lang.T("status.ref") + Lang.T("status.ref.off"), 12, Brushes.White), rangeStatus = WorkbenchTheme.Text(Lang.T("status.range") + Lang.T("status.range.off"), 12, Brushes.White);
    private readonly TextBox liveValues = new() { IsReadOnly = true, BorderThickness = new Thickness(0), Background = Brushes.Transparent, TextWrapping = TextWrapping.Wrap, Text = Lang.T("live.waiting"), FontSize = 12 };
    private readonly TextBox experimentResult = new() { IsReadOnly = true, BorderThickness = new Thickness(0), Background = Brushes.Transparent, TextWrapping = TextWrapping.Wrap, Text = Lang.T("live.experimentHint"), FontSize = 12 };
    private readonly Canvas errorPlot = new() { Background = WorkbenchTheme.Bar, ClipToBounds = true };
    private readonly Queue<double> errorHistory = new();
    private readonly Queue<double> errorHistory2 = new();
    private readonly Button readInputs = MakeButton(Lang.T("btn.readInputs"));
    private readonly Expander diagnostics = new() { Header=Lang.T("diag.header"), IsExpanded=true, Background=WorkbenchTheme.Head, BorderBrush=WorkbenchTheme.Line, BorderThickness=new Thickness(0,1,0,0), Padding=new Thickness(0) };
    private static string[] JointNames => [Lang.T("joint.q1cmd"), Lang.T("joint.q1enc"), Lang.T("joint.q1err"), Lang.T("joint.q2cmd"), Lang.T("joint.q2enc"), Lang.T("joint.q2err"), Lang.T("joint.x"), Lang.T("joint.y"), Lang.T("joint.drops")];
    private static readonly string[] JointUnits = ["°","°","°","°","°","°","mm","mm",""];
    private static string[] LedNames => [Lang.T("led.enable"), Lang.T("led.ref"), Lang.T("led.limits"), Lang.T("led.arm1"), Lang.T("led.arm2"), Lang.T("led.fault")];
    private readonly TextBlock[] jointValues = Enumerable.Range(0, JointNames.Length).Select(_ => new TextBlock { Text = "—", FontFamily = WorkbenchTheme.Mono, TextAlignment = TextAlignment.Right, VerticalAlignment = VerticalAlignment.Center }).ToArray();
    private readonly Ellipse[] leds = Enumerable.Range(0, 6).Select(_ => new Ellipse { Width = 10, Height = 10, Fill = WorkbenchTheme.Idle, VerticalAlignment = VerticalAlignment.Center }).ToArray();
    private TextBlock[] jointNameBlocks = [];
    private TextBlock[] ledNameBlocks = [];
    private TextBlock? chartTitleBlock, chartNoteBlock, telemRateLabel;
    private readonly ComboBox telemRateBox = new() { Width = 118, Margin = new Thickness(4, 2, 4, 2) };
    private static readonly int[] TelemChoices = [0, DeviceContract.TelemRateMin, DefaultTelemHz, DeviceContract.TelemRateMax];
    private TabItem? feedbackTab, logTab;
    private Button? stopButton, refreshButton, folderButton, analysisButton, calibrationButton;
    private ExperimentWindow? analysisWindow;
    // One analysis window at a time: asking again brings the open one forward and refreshes its list (a run may have just finished).
    private void OpenAnalysis()
    {
        if (analysisWindow is { IsLoaded: true } open) { open.Activate(); return; }
        analysisWindow = new ExperimentWindow(ExperimentRoot) { Owner = this };
        analysisWindow.Closed += (_,_) => analysisWindow = null;
        analysisWindow.Show();
    }
    private TextBlock? noteJogManual, noteJogStop;
    private string? lastExperimentDirectory;
    private void RefreshLanguageLayout()
    {
        Title = Lang.T("app.title");
        connectionBadge.Text = compatible ? (framedControl ? Lang.T("badge.v7") : hardwareUart ? Lang.T("badge.v6") : supportsTelemetry ? Lang.T("badge.v5") : Lang.T("badge.old")) : Lang.T("badge.offline");
        referenceStatus.Text = Lang.T("status.ref") + (ptp is { Referenced: true } ? Lang.T("status.ref.on") + (telemetry is { Referenced: true } ? " · " + telemetry.ReferenceName : "") : Lang.T("status.ref.off"));
        rangeStatus.Text = Lang.T("status.range") + (LimitsReady ? Lang.T("status.range.on") : Lang.T("status.range.off"));
        diagnostics.Header = Lang.T("diag.header");
        if (stopButton is { } stop) stop.Content = Lang.T("btn.stop");
        if (refreshButton is { } refresh) refresh.Content = Lang.T("btn.refreshPorts");
        if (folderButton is { } folder) folder.Content = Lang.T("btn.openData");
        if (analysisButton is { } analysis) analysis.Content = Lang.T("btn.analysis");
        if (calibrationButton is { } calibration) calibration.Content = Lang.T("btn.calibration");
        readInputs.Content = Lang.T("btn.readInputs");
        if (noteJogManual is { } n1) n1.Text = Lang.T("note.jogManual");
        if (noteJogStop is { } n2) n2.Text = Lang.T("note.jogStop");
        if (chartTitleBlock is { } ct) ct.Text = Lang.T("chart.title");
        if (chartNoteBlock is { } cn) cn.Text = Lang.T("note.chart");
        if (telemRateLabel is { } tl) tl.Text = Lang.T("lbl.telemRate");
        Lang.RefreshText(faultDetail);
        if (feedbackTab is { } ft) ft.Header = Lang.T("tab.feedback");
        if (logTab is { } lt) { lt.Header = Lang.T("tab.log"); lt.ToolTip = Lang.T("tab.log.tip"); }
        if (jointNameBlocks.Length == JointNames.Length) {
            string[] names = JointNames;
            for (int i = 0; i < JointNames.Length; i++) jointNameBlocks[i].Text = names[i];
        }
        if (ledNameBlocks.Length == 6) {
            string[] names = LedNames;
            for (int i = 0; i < 6; i++) ledNameBlocks[i].Text = names[i];
        }
    }
    private void BuildLayout()
    {
        Title = Lang.T("app.title");
        Language = System.Windows.Markup.XmlLanguage.GetLanguage(Lang.IsEn ? "en-NZ" : "zh-CN");
        Lang.Text(state, () => Lang.T("state.disconnected"));
        Lang.Text(inputs, () => Lang.T("state.inputsEmpty"));
        Width = 1380; Height = 930; MinWidth = 1100; MinHeight = 760;
        FontFamily = new FontFamily("Microsoft YaHei UI, Segoe UI"); FontSize = 12;
        Background = WorkbenchTheme.Background; Foreground = WorkbenchTheme.Ink;
        WorkbenchTheme.Apply(this);
        var shell = new DockPanel { Background = WorkbenchTheme.Background }; Content = shell;

        // Status bar (bottom-most), then the log dock, then the toolbar; the body fills the rest.
        var status = new DockPanel { Height = 24, Background = WorkbenchTheme.Blue, LastChildFill = false };
        var link = WorkbenchTheme.Text("115200 · KitProg", 12, Brushes.White); link.FontFamily = WorkbenchTheme.Mono; link.Margin = new Thickness(12,0,12,0); link.VerticalAlignment = VerticalAlignment.Center;
        DockPanel.SetDock(link, Dock.Right); status.Children.Add(link);
        foreach(var item in new[] { connectionBadge, referenceStatus, rangeStatus }) { item.Margin = new Thickness(12,0,10,0); item.VerticalAlignment = VerticalAlignment.Center; DockPanel.SetDock(item, Dock.Left); status.Children.Add(item); }
        DockPanel.SetDock(status, Dock.Bottom); shell.Children.Add(status);

        var tabs = new TabControl { Height = 190 };
        diagnostics.Content = tabs; DockPanel.SetDock(diagnostics, Dock.Bottom); shell.Children.Add(diagnostics);

        var stop = MakeButton(Lang.T("btn.stop")); stop.MinWidth = 210; stop.MinHeight = 36; stop.FontSize = 14;
        stop.Background = WorkbenchTheme.Red; stop.Foreground = Brushes.White; stop.BorderBrush = WorkbenchTheme.Brush("#8E1B1B");
        stop.FontWeight = FontWeights.Bold; stop.Click += (_,_) => Stop();
        stopButton = stop;
        var toolbar = new DockPanel { Margin = new Thickness(10,0,10,0) };
        DockPanel.SetDock(stop, Dock.Right); toolbar.Children.Add(stop);
        // Language toggle sits immediately left of the stop button (dock right: later siblings stay further left).
        langToggle.ToolTip = Lang.T("lang.toggle.tip");
        System.Windows.Automation.AutomationProperties.SetName(langToggle, Lang.T("lang.toggle.tip"));
        langToggle.MinWidth = 52; langToggle.MinHeight = 36; langToggle.FontSize = 13;
        langToggle.Click += (_,_) => Lang.Toggle();
        DockPanel.SetDock(langToggle, Dock.Right); toolbar.Children.Add(langToggle);
        var devices = new WrapPanel { VerticalAlignment = VerticalAlignment.Center };
        ports.Width = 100; ports.Margin = new Thickness(2); ports.FontFamily = WorkbenchTheme.Mono; devices.Children.Add(ports);
        var refresh = MakeButton(Lang.T("btn.refreshPorts")); refresh.Click += (_,_) => RefreshPorts(); devices.Children.Add(refresh);
        refreshButton = refresh;
        connect.MinWidth = 64;
        connect.Click += (_,_) => { if(serial is null) Connect(); else Disconnect(() => Lang.T("fault.unplug")); }; devices.Children.Add(connect);
        devices.Children.Add(new Border { Width = 1, Height = 26, Background = WorkbenchTheme.Line, Margin = new Thickness(8,0,8,0) });
        foreach(var button in new[] { connect,enable,home,manualReference }) WorkbenchTheme.Primary(button);
        manualReference.Content = Lang.T("btn.manualRef");
        manualReference.ToolTip = Lang.T("btn.manualRef.tip");
        ToolTipService.SetShowOnDisabled(manualReference,true);
        enable.Click += (_,_) => RequestEnable(); home.Click += (_,_) => RequestHome(); manualReference.Click += (_,_) => RequestManualReference();
        devices.Children.Add(enable); devices.Children.Add(manualReference); devices.Children.Add(home);
        toolbar.Children.Add(devices);
        var toolbarBar = WorkbenchTheme.Edge(toolbar); toolbarBar.MinHeight = 48; DockPanel.SetDock(toolbarBar, Dock.Top); shell.Children.Add(toolbarBar);
        var workflowBar = BuildWorkflowStrip(); DockPanel.SetDock(workflowBar, Dock.Top); shell.Children.Add(workflowBar);

        // Body: property grid | preview | live status. The two splitters resize the side columns.
        var body = new Grid(); shell.Children.Add(body);
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(288), MinWidth = 250, MaxWidth = 560 });
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
        body.ColumnDefinitions.Add(new ColumnDefinition { MinWidth = 360 });
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(248), MinWidth = 220, MaxWidth = 520 });
        GridSplitter Splitter(int column) {
            var splitter = new GridSplitter { Width = 5, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Stretch,
                ResizeBehavior = GridResizeBehavior.PreviousAndNext, ResizeDirection = GridResizeDirection.Columns, Background = WorkbenchTheme.Line,
                Cursor = Cursors.SizeWE, Focusable = false };
            Lang.Bind(splitter, ToolTipProperty, "note.splitter"); Grid.SetColumn(splitter, column); body.Children.Add(splitter); return splitter;
        }

        // Left: property grid owned by the preview panel.
        var properties = new DockPanel();
        var propertiesCaption = Caption("section.properties"); DockPanel.SetDock(propertiesCaption, Dock.Top); properties.Children.Add(propertiesCaption); properties.Children.Add(xyPreview.ParameterContent);
        xyPreview.ParametersSaved += () => Append("HOST parameters saved to file and used for planning; speed/accel sync to the device on their own, the joint range only while disabled: " + xyPreview.SettingsDescription);
        body.Children.Add(new Border { Background = Brushes.White, Child = properties });
        Splitter(1);

        Grid.SetColumn(xyPreview, 2); body.Children.Add(xyPreview);
        Splitter(3);

        // Right: joint feedback, input LEDs, device state, reference and jog; every group folds.
        var side = new StackPanel();
        StackPanel Group(string title) { var group = new StackPanel(); side.Children.Add(WorkbenchTheme.Section(title, group)); return group; }
        var jointGroup = Group("section.joints");
        jointNameBlocks = new TextBlock[JointNames.Length];
        string[] jnames = JointNames;
        for(int i = 0; i < jnames.Length; i++) {
            var row = new Grid { Margin = new Thickness(10,3,10,3) };
            row.ColumnDefinitions.Add(new ColumnDefinition()); row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(64) }); row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(30) });
            var name = WorkbenchTheme.Label(jnames[i]); name.VerticalAlignment = VerticalAlignment.Center; jointNameBlocks[i] = name; Grid.SetColumn(jointValues[i], 1);
            var unit = new TextBlock { Text = JointUnits[i], Foreground = WorkbenchTheme.Muted, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(6,0,0,0) }; Grid.SetColumn(unit, 2);
            row.Children.Add(name); row.Children.Add(jointValues[i]); row.Children.Add(unit); jointGroup.Children.Add(Rule(row));
        }
        liveValues.Margin = new Thickness(7,2,7,2); liveValues.Foreground = WorkbenchTheme.Muted; jointGroup.Children.Add(liveValues);
        var inputGroup = Group("section.inputs");
        ledNameBlocks = new TextBlock[6];
        string[] lnames = LedNames;
        for(int i = 0; i < lnames.Length; i++) {
            var row = new StackPanel { Orientation = Orientation.Horizontal, Margin = new Thickness(10,3,10,3) };
            leds[i].Margin = new Thickness(0,0,8,0); row.Children.Add(leds[i]);
            var ledLabel = new TextBlock { Text = lnames[i] }; ledNameBlocks[i] = ledLabel; row.Children.Add(ledLabel); inputGroup.Children.Add(Rule(row));
        }
        state.FontSize = 12; state.Margin = new Thickness(10,6,10,2); inputGroup.Children.Add(state);
        inputs.FontSize = 11; inputs.Foreground = WorkbenchTheme.Muted; inputs.Margin = new Thickness(10,2,10,2); inputGroup.Children.Add(inputs);
        lastStop.FontSize = 12; lastStop.Margin = new Thickness(10,2,10,2); inputGroup.Children.Add(lastStop);
        inputGroup.Children.Add(faultDetail);
        readInputs.Click += (_,_) => SafeAction(() => { Send((byte)'i'); Send((byte)'e'); Send((byte)'s'); }); readInputs.Margin = new Thickness(10,6,10,6); inputGroup.Children.Add(readInputs);
        var jogGroup = Group("section.jog");
        noteJogManual = Note(Lang.T("note.jogManual")); jogGroup.Children.Add(noteJogManual);
        var jogging = new WrapPanel { Margin = new Thickness(8,0,8,0) }; jogGroup.Children.Add(jogging);
        AddJog(jogging,"arm1 ← A",1); AddJog(jogging,"arm1 D →",2); AddJog(jogging,"arm2 ←",4); AddJog(jogging,"arm2 →",8);
        noteJogStop = Note(Lang.T("note.jogStop")); jogGroup.Children.Add(noteJogStop);
        var right = new Border { Background = Brushes.White,
            Child = new ScrollViewer { Content = side, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled } };
        Grid.SetColumn(right, 4); body.Children.Add(right);

        var feedback = new Grid(); feedback.ColumnDefinitions.Add(new ColumnDefinition()); feedback.ColumnDefinitions.Add(new ColumnDefinition());
        feedback.Children.Add(new ScrollViewer { Content=experimentResult, VerticalScrollBarVisibility=ScrollBarVisibility.Auto, Margin = new Thickness(12,6,0,6) });
        var chart = new Grid { Margin = new Thickness(16,6,12,6) }; Grid.SetColumn(chart,1); feedback.Children.Add(chart);
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        chart.RowDefinitions.Add(new RowDefinition());
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        chartTitleBlock=WorkbenchTheme.Text(Lang.T("chart.title"),12,WorkbenchTheme.Sub); chart.Children.Add(chartTitleBlock);
        Grid.SetRow(errorPlot,1); chart.Children.Add(errorPlot);
        chartNoteBlock=WorkbenchTheme.Text(Lang.T("note.chart"),11,WorkbenchTheme.Muted); Grid.SetRow(chartNoteBlock,2); chart.Children.Add(chartNoteBlock);
        var folder = MakeButton(Lang.T("btn.openData")); folderButton = folder; folder.Click += (_,_) => OpenDataFolder();
        foreach (int hz in TelemChoices) telemRateBox.Items.Add(Lang.Bind(new ComboBoxItem { Tag = hz }, ContentControl.ContentProperty, hz == 0 ? "telem.status" : hz == DefaultTelemHz ? "telem.hz10" : hz == DeviceContract.TelemRateMin ? "telem.hz5" : "telem.hz20"));
        telemRateBox.SelectedIndex = Array.IndexOf(TelemChoices, DefaultTelemHz);
        telemRateBox.SelectionChanged += (_,_) => { if (telemRateBox.SelectedIndex >= 0) { desiredTelemHz = TelemChoices[telemRateBox.SelectedIndex]; telemAttempts = 0; UpdateControls(); } };
        telemRateLabel = WorkbenchTheme.Text(Lang.T("lbl.telemRate"), 12, WorkbenchTheme.Sub); telemRateLabel.VerticalAlignment = VerticalAlignment.Center;
        var analysis = MakeButton(Lang.T("btn.analysis")); analysisButton = analysis; analysis.Click += (_,_) => OpenAnalysis();
        var calibrate = MakeButton(Lang.T("btn.calibration")); calibrationButton = calibrate; calibrate.Click += (_,_) => OpenCalibration();
        var chartButtons = new WrapPanel(); chartButtons.Children.Add(folder); chartButtons.Children.Add(analysis); chartButtons.Children.Add(calibrate); chartButtons.Children.Add(telemRateLabel); chartButtons.Children.Add(telemRateBox);
        Grid.SetRow(chartButtons,3); chart.Children.Add(chartButtons);
        errorPlot.SizeChanged += (_,_) => DrawErrorPlot();
        feedbackTab = new TabItem { Header = Lang.T("tab.feedback"), Content = feedback };
        tabs.Items.Add(feedbackTab);
        var logPanel = new DockPanel(); logPath.FontSize = 11; DockPanel.SetDock(logPath,Dock.Bottom); logPanel.Children.Add(logPath); logPanel.Children.Add(log);
        logTab = new TabItem { Header = Lang.T("tab.log"), ToolTip = Lang.T("tab.log.tip"), Content = logPanel };
        tabs.Items.Add(logTab);
        SizeChanged += (_,_) => {
            bool compact=ActualHeight<860;
            tabs.Height=compact?140:190;
            chartTitleBlock.Visibility=errorPlot.Visibility=chartNoteBlock.Visibility=compact?Visibility.Collapsed:Visibility.Visible;
        };
        Lang.Text(experimentResult, () => Lang.T("live.experimentHint"));
        ShowFeedback(null, () => Lang.T("live.waiting"));
    }
    private IEnumerable<Expander> Sections() => Descendants(this).OfType<Expander>();
    private static IEnumerable<object> Descendants(DependencyObject root) =>
        LogicalTreeHelper.GetChildren(root).OfType<DependencyObject>().SelectMany(child => new object[] { child }.Concat(Descendants(child)));
    private static Border Caption(string title) { var caption = WorkbenchTheme.Caption(title); caption.Height = 26; return caption; }
    private static Border Rule(UIElement row) => new() { BorderBrush = WorkbenchTheme.Faint, BorderThickness = new Thickness(0,0,0,1), Child = row };
    private static TextBlock Note(string text) => new() { Text = text, FontSize = 11, Foreground = WorkbenchTheme.Muted, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(10,4,10,4) };
    // One place that turns a telemetry sample (or its absence) into the joint rows and input LEDs.
    private void ShowFeedback(TelemetrySample? sample, Func<string> note)
    {
        Lang.Text(liveValues, note);
        var v = Enumerable.Repeat("—", JointNames.Length).ToArray();
        if (sample is { Referenced: true }) {
            v[0] = $"{sample.Command.Q1:F3}"; v[1] = $"{sample.Measured1:F3}"; v[2] = $"{sample.Error1:+0.000;-0.000;0.000}"; v[3] = $"{sample.Command.Q2:F3}";
            if(sample.Measured2 is double q2 && sample.Error2 is double error2) { v[4] = $"{q2:F3}"; v[5] = $"{error2:+0.000;-0.000;0.000}"; }
            try { var tip = xyPreview.Model.Forward(new(sample.Measured1, (sample.Measured2 ?? sample.Measured1 + sample.Command.Q2 - sample.Command.Q1))).Tip; v[6] = $"{tip.X:F2}"; v[7] = $"{tip.Y:F2}"; } catch (ArgumentException) { /* invalid link lengths: leave the estimate blank */ }
        }
        if (sample is not null) v[8] = sample.Drops.ToString();
        for (int i = 0; i < v.Length; i++) jointValues[i].Text = v[i];
        for (int bit = 0; bit < 4; bit++) leds[2 + bit].Fill = sample is null ? WorkbenchTheme.Idle : (sample.Inputs >> bit & 1) == 0 ? WorkbenchTheme.Ok : WorkbenchTheme.Red;
    }
    private void OpenDataFolder()
    {
        try { string path = lastExperimentDirectory ?? ExperimentRoot; Directory.CreateDirectory(path); Process.Start(new ProcessStartInfo("explorer.exe",path) { UseShellExecute = true }); }
        catch(Exception ex) { Lang.Text(experimentResult, () => Lang.T("fault.openFolder") + ex.Message); }
    }
    private void DrawErrorPlot()
    {
        errorPlot.Children.Clear(); double w = Math.Max(50,errorPlot.ActualWidth), h = Math.Max(24,errorPlot.ActualHeight);
        double scale = Math.Max(0.02,errorHistory.Concat(errorHistory2).Select(Math.Abs).DefaultIfEmpty().Max());
        errorPlot.Children.Add(new Line { X1 = 0, X2 = w, Y1 = h/2, Y2 = h/2, Stroke = WorkbenchTheme.Line });
        var line = new Polyline { Stroke = WorkbenchTheme.Teal, StrokeThickness = 2 };
        int i = 0; foreach(double e in errorHistory) line.Points.Add(new Point(8 + i++ * (w-16)/Math.Max(1,errorHistory.Count-1),h/2-e/scale*(h/2-10)));
        errorPlot.Children.Add(line);
        var second = new Polyline { Stroke = WorkbenchTheme.Blue, StrokeThickness = 2 };
        i = 0; foreach(double e in errorHistory2) second.Points.Add(new Point(8 + i++ * (w-16)/Math.Max(1,errorHistory2.Count-1),h/2-e/scale*(h/2-10)));
        errorPlot.Children.Add(second);
        var label = WorkbenchTheme.Text($"±{scale:F3}°",10,WorkbenchTheme.Muted); errorPlot.Children.Add(label);
    }
    private void SaveWorkbenchSnapshot(string path, double width, double height)
    {
        Width=width; Height=height; Show(); UpdateLayout();
        xyPreview.FlushViewport(); UpdateLayout();   // a pose change only schedules a redraw; the snapshot must show what the timer would paint
        xyPreview.CheckViewportLayout();
        bool wasEnabled=connect.IsEnabled;
        connect.IsEnabled=false; connect.ApplyTemplate();
        var buttonSurface=(Border)connect.Template.FindName("surface",connect);
        if(buttonSurface.Opacity!=0.5) throw new InvalidOperationException("Disabled primary button must be visibly dimmed");
        connect.IsEnabled=wasEnabled;
        if(wasEnabled && buttonSurface.Opacity<=0.5) throw new InvalidOperationException("Enabled button retained disabled appearance");
        var content=(FrameworkElement)Content;
        var image = new RenderTargetBitmap((int)content.ActualWidth,(int)content.ActualHeight,96,96,PixelFormats.Pbgra32); image.Render(content);
        var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(image));
        using var stream = File.Create(path); png.Save(stream); Hide();
    }
}
