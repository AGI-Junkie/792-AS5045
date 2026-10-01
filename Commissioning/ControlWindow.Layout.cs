using System.Diagnostics;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private readonly TextBlock connectionBadge = WorkbenchTheme.Text("未连接 · 离线预览", 12, Brushes.White);
    private readonly TextBlock referenceStatus = WorkbenchTheme.Text("坐标参考：未建立", 12, Brushes.White), rangeStatus = WorkbenchTheme.Text("关节范围：未同步", 12, Brushes.White);
    private readonly TextBox liveValues = new() { IsReadOnly = true, BorderThickness = new Thickness(0), Background = Brushes.Transparent, TextWrapping = TextWrapping.Wrap, Text = "等待 stage2-v5 实时反馈。", FontSize = 12 };
    private readonly TextBox experimentResult = new() { IsReadOnly = true, BorderThickness = new Thickness(0), Background = Brushes.Transparent, TextWrapping = TextWrapping.Wrap, Text = "每次单点 / 图形实验自动保存计划、反馈 CSV 和结果摘要。", FontSize = 12 };
    private readonly Canvas errorPlot = new() { Background = WorkbenchTheme.Bar, ClipToBounds = true };
    private readonly Queue<double> errorHistory = new();
    private readonly Button readInputs = MakeButton("读取限位 / 编码器");
    private readonly Expander diagnostics = new() { Header="实时反馈与实验 / 原始日志", IsExpanded=true, Background=WorkbenchTheme.Head, BorderBrush=WorkbenchTheme.Line, BorderThickness=new Thickness(0,1,0,0), Padding=new Thickness(0) };
    private static readonly string[] JointNames = ["q1 命令","q1 编码器","q1 误差","q2 命令","末端 X（估算）","末端 Y（估算）","累计丢包"], JointUnits = ["°","°","°","°","mm","mm",""];
    private static readonly string[] LedNames = ["使能","坐标参考","全局限位","arm1 传感器","arm2 NPN","驱动故障"];
    private readonly TextBlock[] jointValues = JointNames.Select(_ => new TextBlock { Text = "—", FontFamily = WorkbenchTheme.Mono, TextAlignment = TextAlignment.Right, VerticalAlignment = VerticalAlignment.Center }).ToArray();
    private readonly Ellipse[] leds = LedNames.Select(_ => new Ellipse { Width = 10, Height = 10, Fill = WorkbenchTheme.Idle, VerticalAlignment = VerticalAlignment.Center }).ToArray();
    private string? lastExperimentDirectory;
    private void BuildLayout()
    {
        Title = "Two-Link Studio · 轨迹工作台 · v7 全局运动参数";
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

        var stop = MakeButton("■  总停止   Space / Esc"); stop.MinWidth = 210; stop.MinHeight = 36; stop.FontSize = 14;
        stop.Background = WorkbenchTheme.Red; stop.Foreground = Brushes.White; stop.BorderBrush = WorkbenchTheme.Brush("#8E1B1B");
        stop.FontWeight = FontWeights.Bold; stop.Click += (_,_) => Stop();
        var toolbar = new DockPanel { Margin = new Thickness(10,0,10,0) };
        DockPanel.SetDock(stop, Dock.Right); toolbar.Children.Add(stop);
        var devices = new StackPanel { Orientation = Orientation.Horizontal, VerticalAlignment = VerticalAlignment.Center };
        ports.Width = 100; ports.Margin = new Thickness(2); ports.FontFamily = WorkbenchTheme.Mono; devices.Children.Add(ports);
        var refresh = MakeButton("刷新"); refresh.Click += (_,_) => RefreshPorts(); devices.Children.Add(refresh);
        connect.MinWidth = 64;
        connect.Click += (_,_) => { if(serial is null) Connect(); else Disconnect("已断开；请重新连接设备"); }; devices.Children.Add(connect);
        devices.Children.Add(new Border { Width = 1, Height = 26, Background = WorkbenchTheme.Line, Margin = new Thickness(8,0,8,0) });
        foreach(var button in new[] { connect,enable,home,manualReference }) WorkbenchTheme.Primary(button);
        manualReference.Content = "人工确认当前位置为伸直零位";
        manualReference.ToolTip = "先人工把两杆沿 +X 摆直，再确认 q1=q2=0；只建立坐标，不驱动电机。";
        ToolTipService.SetShowOnDisabled(manualReference,true);
        enable.Click += (_,_) => RequestEnable(); home.Click += (_,_) => RequestHome(); manualReference.Click += (_,_) => RequestManualReference();
        devices.Children.Add(enable); devices.Children.Add(manualReference); devices.Children.Add(home);
        toolbar.Children.Add(devices);
        var toolbarBar = WorkbenchTheme.Edge(toolbar); toolbarBar.Height = 48; DockPanel.SetDock(toolbarBar, Dock.Top); shell.Children.Add(toolbarBar);

        var body = new Grid(); shell.Children.Add(body);
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(288) });
        body.ColumnDefinitions.Add(new ColumnDefinition());
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(248) });

        // Left: property grid owned by the preview panel.
        var properties = new DockPanel();
        var propertiesCaption = Caption("属性"); DockPanel.SetDock(propertiesCaption, Dock.Top); properties.Children.Add(propertiesCaption); properties.Children.Add(xyPreview.ParameterContent);
        xyPreview.ParametersSaved += () => Append("HOST parameters saved and applied: " + xyPreview.SettingsDescription);
        body.Children.Add(new Border { Background = Brushes.White, BorderBrush = WorkbenchTheme.Line, BorderThickness = new Thickness(0,0,1,0), Child = properties });

        Grid.SetColumn(xyPreview, 1); body.Children.Add(xyPreview);

        // Right: joint feedback, input LEDs, device state, reference and jog.
        var side = new StackPanel();
        side.Children.Add(Caption("关节状态"));
        for(int i = 0; i < JointNames.Length; i++) {
            var row = new Grid { Margin = new Thickness(10,3,10,3) };
            row.ColumnDefinitions.Add(new ColumnDefinition()); row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(64) }); row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(30) });
            var name = WorkbenchTheme.Label(JointNames[i]); name.VerticalAlignment = VerticalAlignment.Center; Grid.SetColumn(jointValues[i], 1);
            var unit = new TextBlock { Text = JointUnits[i], Foreground = WorkbenchTheme.Muted, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(6,0,0,0) }; Grid.SetColumn(unit, 2);
            row.Children.Add(name); row.Children.Add(jointValues[i]); row.Children.Add(unit); side.Children.Add(Rule(row));
        }
        liveValues.Margin = new Thickness(7,2,7,2); liveValues.Foreground = WorkbenchTheme.Muted; side.Children.Add(liveValues);
        side.Children.Add(WorkbenchTheme.GroupHeader("输入 / 状态"));
        for(int i = 0; i < LedNames.Length; i++) {
            var row = new StackPanel { Orientation = Orientation.Horizontal, Margin = new Thickness(10,3,10,3) };
            leds[i].Margin = new Thickness(0,0,8,0); row.Children.Add(leds[i]); row.Children.Add(new TextBlock { Text = LedNames[i] }); side.Children.Add(Rule(row));
        }
        state.FontSize = 12; state.Margin = new Thickness(10,6,10,2); side.Children.Add(state);
        inputs.FontSize = 11; inputs.Foreground = WorkbenchTheme.Muted; inputs.Margin = new Thickness(10,2,10,2); side.Children.Add(inputs);
        lastStop.FontSize = 12; lastStop.Margin = new Thickness(10,2,10,2); side.Children.Add(lastStop);
        readInputs.Click += (_,_) => SafeAction(() => { Send((byte)'i'); Send((byte)'e'); Send((byte)'s'); }); readInputs.Margin = new Thickness(10,6,10,6); side.Children.Add(readInputs);
        side.Children.Add(WorkbenchTheme.GroupHeader("坐标参考与点动"));
        side.Children.Add(Note("可人工摆直确认，也可开关回零。人工确认时两杆必须已伸直。"));
        var jogging = new WrapPanel { Margin = new Thickness(8,0,8,0) }; side.Children.Add(jogging);
        AddJog(jogging,"arm1 ← A",1); AddJog(jogging,"arm1 D →",2); AddJog(jogging,"arm2 ←",4); AddJog(jogging,"arm2 →",8);
        side.Children.Add(Note("点动松开或切换窗口即停；回零和轨迹可在后台继续。"));
        var right = new Border { Background = Brushes.White, BorderBrush = WorkbenchTheme.Line, BorderThickness = new Thickness(1,0,0,0),
            Child = new ScrollViewer { Content = side, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled } };
        Grid.SetColumn(right, 2); body.Children.Add(right);

        experimentResult.TextChanged+=(_,_)=> {
            if(experimentResult.Text.Contains("失败") || experimentResult.Text.StartsWith("实验完成") || experimentResult.Text.StartsWith("实验中止")) diagnostics.IsExpanded=true;
        };
        var feedback = new Grid(); feedback.ColumnDefinitions.Add(new ColumnDefinition()); feedback.ColumnDefinitions.Add(new ColumnDefinition());
        feedback.Children.Add(new ScrollViewer { Content=experimentResult, VerticalScrollBarVisibility=ScrollBarVisibility.Auto, Margin = new Thickness(12,6,0,6) });
        var chart = new Grid { Margin = new Thickness(16,6,12,6) }; Grid.SetColumn(chart,1); feedback.Children.Add(chart);
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        chart.RowDefinitions.Add(new RowDefinition());
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        var chartTitle=WorkbenchTheme.Text("arm1 编码器 − 命令角度 / °",12,WorkbenchTheme.Sub); chart.Children.Add(chartTitle);
        Grid.SetRow(errorPlot,1); chart.Children.Add(errorPlot);
        var chartNote=WorkbenchTheme.Text("约 2 Hz；arm2 无编码器，XY 为估算。",11,WorkbenchTheme.Muted); Grid.SetRow(chartNote,2); chart.Children.Add(chartNote);
        var folder = MakeButton("打开实验数据目录"); folder.Click += (_,_) => OpenDataFolder(); Grid.SetRow(folder,3); chart.Children.Add(folder);
        errorPlot.SizeChanged += (_,_) => DrawErrorPlot();
        tabs.Items.Add(new TabItem { Header = "实时反馈与实验", Content = feedback });
        var logPanel = new DockPanel(); logPath.FontSize = 11; DockPanel.SetDock(logPath,Dock.Bottom); logPanel.Children.Add(logPath); logPanel.Children.Add(log);
        tabs.Items.Add(new TabItem { Header = "原始日志", ToolTip = "日志可选中文字，用 Ctrl+C 复制。", Content = logPanel });
        SizeChanged += (_,_) => {
            bool compact=ActualHeight<860;
            tabs.Height=compact?140:190;
            chartTitle.Visibility=errorPlot.Visibility=chartNote.Visibility=compact?Visibility.Collapsed:Visibility.Visible;
        };
        ShowFeedback(null, liveValues.Text);
    }
    private static Border Caption(string title) { var caption = WorkbenchTheme.Caption(title); caption.Height = 26; return caption; }
    private static Border Rule(UIElement row) => new() { BorderBrush = WorkbenchTheme.Faint, BorderThickness = new Thickness(0,0,0,1), Child = row };
    private static TextBlock Note(string text) => new() { Text = text, FontSize = 11, Foreground = WorkbenchTheme.Muted, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(10,4,10,4) };
    // One place that turns a telemetry sample (or its absence) into the joint rows and input LEDs.
    private void ShowFeedback(TelemetrySample? sample, string note)
    {
        liveValues.Text = note;
        var v = Enumerable.Repeat("—", JointNames.Length).ToArray();
        if (sample is { Referenced: true }) {
            v[0] = $"{sample.Command.Q1:F3}"; v[1] = $"{sample.Measured1:F3}"; v[2] = $"{sample.Error1:+0.000;-0.000;0.000}"; v[3] = $"{sample.Command.Q2:F3}";
            try { var tip = xyPreview.Model.Forward(sample.Command).Tip; v[4] = $"{tip.X:F2}"; v[5] = $"{tip.Y:F2}"; } catch (ArgumentException) { /* invalid link lengths: leave the estimate blank */ }
        }
        if (sample is not null) v[6] = sample.Drops.ToString();
        for (int i = 0; i < v.Length; i++) jointValues[i].Text = v[i];
        for (int bit = 0; bit < 4; bit++) leds[2 + bit].Fill = sample is null ? WorkbenchTheme.Idle : (sample.Inputs >> bit & 1) == 0 ? WorkbenchTheme.Ok : WorkbenchTheme.Red;
    }
    private void OpenDataFolder()
    {
        try { string path = lastExperimentDirectory ?? ExperimentRoot; Directory.CreateDirectory(path); Process.Start(new ProcessStartInfo("explorer.exe",path) { UseShellExecute = true }); }
        catch(Exception ex) { experimentResult.Text = "无法打开目录：" + ex.Message; }
    }
    private void DrawErrorPlot()
    {
        errorPlot.Children.Clear(); double w = Math.Max(50,errorPlot.ActualWidth), h = Math.Max(24,errorPlot.ActualHeight);
        double scale = Math.Max(0.02,errorHistory.Select(Math.Abs).DefaultIfEmpty().Max());
        errorPlot.Children.Add(new Line { X1 = 0, X2 = w, Y1 = h/2, Y2 = h/2, Stroke = WorkbenchTheme.Line });
        var line = new Polyline { Stroke = WorkbenchTheme.Teal, StrokeThickness = 2 };
        int i = 0; foreach(double e in errorHistory) line.Points.Add(new Point(8 + i++ * (w-16)/Math.Max(1,errorHistory.Count-1),h/2-e/scale*(h/2-10)));
        errorPlot.Children.Add(line);
        var label = WorkbenchTheme.Text($"±{scale:F3}°",10,WorkbenchTheme.Muted); errorPlot.Children.Add(label);
    }
    private void SaveWorkbenchSnapshot(string path, double width, double height)
    {
        Width=width; Height=height; Show(); UpdateLayout();
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
