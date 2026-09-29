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
    private readonly TextBlock connectionBadge = WorkbenchTheme.Text("未连接 · 离线预览", 13, Brushes.White);
    private readonly TextBox liveValues = new() { IsReadOnly = true, BorderThickness = new Thickness(0), Background = Brushes.Transparent, TextWrapping = TextWrapping.Wrap, Text = "等待 stage2-v5 实时反馈。", FontSize = 14 };
    private readonly TextBox experimentResult = new() { IsReadOnly = true, BorderThickness = new Thickness(0), Background = Brushes.Transparent, TextWrapping = TextWrapping.Wrap, Text = "每次单点 / 图形实验自动保存计划、反馈 CSV 和结果摘要。", FontSize = 12 };
    private readonly Canvas errorPlot = new() { Background = WorkbenchTheme.Background, ClipToBounds = true };
    private readonly Queue<double> errorHistory = new();
    private readonly Button readInputs = MakeButton("读取限位 / 编码器");
    private readonly Expander diagnostics = new() { Header="实时反馈与实验 / 原始日志", Margin=new Thickness(0,8,0,0) };
    private readonly TabControl sideTabs = new();
    private string? lastExperimentDirectory;
    private void BuildLayout()
    {
        Title = "Two-Link Studio · 轨迹工作台 · v7 全局运动参数";
        Width = 1380; Height = 930; MinWidth = 1100; MinHeight = 760;
        FontFamily = new FontFamily("Microsoft YaHei UI, Segoe UI"); FontSize = 13;
        Background = WorkbenchTheme.Background; Foreground = WorkbenchTheme.Ink;
        WorkbenchTheme.Apply(this);
        var shell = new DockPanel { Background = WorkbenchTheme.Background }; Content = shell;
        var header = new DockPanel { Margin = new Thickness(18,8,18,8) };
        var stop = MakeButton("■  总停止   Space / Esc"); stop.MinWidth = 210;
        stop.Background = WorkbenchTheme.Red; stop.Foreground = Brushes.White; stop.BorderBrush = WorkbenchTheme.Red;
        stop.FontWeight = FontWeights.Bold; stop.Click += (_,_) => Stop();
        DockPanel.SetDock(stop,Dock.Right); header.Children.Add(stop);
        DockPanel.SetDock(connectionBadge,Dock.Right); connectionBadge.Margin = new Thickness(20,0,22,0); connectionBadge.VerticalAlignment = VerticalAlignment.Center; header.Children.Add(connectionBadge);
        var brand = new StackPanel(); brand.Children.Add(WorkbenchTheme.Text("TWO-LINK  /  STUDIO",18,Brushes.White));
        brand.VerticalAlignment=VerticalAlignment.Center; header.Children.Add(brand);
        var headerSurface = new Border { Background = WorkbenchTheme.Ink, Child = header }; DockPanel.SetDock(headerSurface,Dock.Top); shell.Children.Add(headerSurface);
        var body = new Grid { Margin = new Thickness(12) };
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(280) });
        body.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(12) });
        body.ColumnDefinitions.Add(new ColumnDefinition()); shell.Children.Add(body);
        var side = new DockPanel(); body.Children.Add(side);
        sideTabs.Items.Add(new TabItem { Header = "图形参数", Content = xyPreview.ParameterContent });
        var deviceControls = new StackPanel { Margin = new Thickness(2) };
        sideTabs.Items.Add(new TabItem { Header = "设备 / 回零", Content = new ScrollViewer { Content = deviceControls, VerticalScrollBarVisibility = ScrollBarVisibility.Auto } });
        xyPreview.ParametersSaved += () => { sideTabs.SelectedIndex=1; Append("HOST parameters saved and applied: " + xyPreview.SettingsDescription); };
        var connection = new StackPanel(); connection.Children.Add(WorkbenchTheme.Text("设备连接",14));
        var row = new WrapPanel(); ports.Width = 117; row.Children.Add(ports);
        var refresh = MakeButton("刷新"); refresh.Click += (_,_) => RefreshPorts(); row.Children.Add(refresh); connection.Children.Add(row);
        connect.Click += (_,_) => { if(serial is null) { Connect(); if(serial is not null) sideTabs.SelectedIndex=1; } else Disconnect("已断开；请重新连接设备"); }; connection.Children.Add(connect);
        connection.Children.Add(WorkbenchTheme.Text("115200 · KitProg · 连接不自动运动",11,WorkbenchTheme.Muted));
        var connectionCard=WorkbenchTheme.Card(connection); DockPanel.SetDock(connectionCard,Dock.Top); side.Children.Add(connectionCard); side.Children.Add(sideTabs);
        foreach(var button in new[] { connect,enable,home,manualReference }) WorkbenchTheme.Primary(button);
        var preparation = new StackPanel(); preparation.Children.Add(WorkbenchTheme.Text("坐标参考与点动",15));
        enable.Click += (_,_) => RequestEnable(); home.Click += (_,_) => RequestHome();
        manualReference.Content = "人工确认当前位置为伸直零位";
        manualReference.FontSize = 12; manualReference.Padding = new Thickness(5,10,5,10);
        manualReference.Click += (_,_) => RequestManualReference();
        manualReference.ToolTip = "先人工把两杆沿 +X 摆直，再确认 q1=q2=0；只建立坐标，不驱动电机。";
        ToolTipService.SetShowOnDisabled(manualReference,true);
        preparation.Children.Add(enable); preparation.Children.Add(manualReference);
        preparation.Children.Add(home);
        preparation.Children.Add(WorkbenchTheme.Text("可人工摆直确认，也可开关回零。\n人工确认时两杆必须已伸直。",12,WorkbenchTheme.Muted));
        var jogging = new WrapPanel(); preparation.Children.Add(jogging);
        AddJog(jogging,"arm1 ← A",1); AddJog(jogging,"arm1 D →",2); AddJog(jogging,"arm2 ←",4); AddJog(jogging,"arm2 →",8);
        preparation.Children.Add(WorkbenchTheme.Text("点动松开或切换窗口即停；回零和轨迹可在后台继续。",12,WorkbenchTheme.Muted));
        deviceControls.Children.Add(preparation);
        var device = new StackPanel(); device.Children.Add(WorkbenchTheme.Text("设备状态",16));
        state.FontSize = 13; state.Margin = new Thickness(3,5,3,5); device.Children.Add(state);
        inputs.FontSize = 12; device.Children.Add(inputs);
        readInputs.Click += (_,_) => SafeAction(() => { Send((byte)'i'); Send((byte)'e'); Send((byte)'s'); }); device.Children.Add(readInputs);
        lastStop.FontSize = 12; device.Children.Add(lastStop); deviceControls.Children.Add(device);
        var right = new Grid(); Grid.SetColumn(right,2); body.Children.Add(right);
        right.RowDefinitions.Add(new RowDefinition());
        right.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        right.Children.Add(xyPreview);
        var tabs = new TabControl { Height=184, Margin = new Thickness(0,6,0,0) };
        diagnostics.Content=tabs;
        diagnostics.Expanded+=(_,_)=>xyPreview.SetDiagnosticsExpanded(true);
        diagnostics.Collapsed+=(_,_)=>xyPreview.SetDiagnosticsExpanded(false);
        experimentResult.TextChanged+=(_,_)=> {
            if(experimentResult.Text.Contains("失败") || experimentResult.Text.StartsWith("实验完成") || experimentResult.Text.StartsWith("实验中止")) diagnostics.IsExpanded=true;
        };
        Grid.SetRow(diagnostics,1); right.Children.Add(diagnostics);
        var feedback = new Grid(); feedback.ColumnDefinitions.Add(new ColumnDefinition()); feedback.ColumnDefinitions.Add(new ColumnDefinition());
        var values = new StackPanel(); values.Children.Add(liveValues); values.Children.Add(experimentResult);
        feedback.Children.Add(new ScrollViewer { Content=values, VerticalScrollBarVisibility=ScrollBarVisibility.Auto });
        var chart = new Grid { Margin = new Thickness(16,0,0,0) }; Grid.SetColumn(chart,1); feedback.Children.Add(chart);
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        chart.RowDefinitions.Add(new RowDefinition());
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        chart.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        var chartTitle=WorkbenchTheme.Text("arm1 编码器 − 命令角度 / °",12); chart.Children.Add(chartTitle);
        Grid.SetRow(errorPlot,1); chart.Children.Add(errorPlot);
        var chartNote=WorkbenchTheme.Text("约 2 Hz；arm2 无编码器，XY 为估算。",11,WorkbenchTheme.Muted); Grid.SetRow(chartNote,2); chart.Children.Add(chartNote);
        var folder = MakeButton("打开实验数据目录"); folder.Click += (_,_) => OpenDataFolder(); Grid.SetRow(folder,3); chart.Children.Add(folder);
        errorPlot.SizeChanged += (_,_) => DrawErrorPlot();
        tabs.Items.Add(new TabItem { Header = "实时反馈与实验", Content = feedback });
        var logPanel = new DockPanel(); logPath.FontSize = 11; DockPanel.SetDock(logPath,Dock.Bottom); logPanel.Children.Add(logPath); logPanel.Children.Add(log);
        tabs.Items.Add(new TabItem { Header = "原始日志", ToolTip = "日志可选中文字，用 Ctrl+C 复制。", Content = logPanel });
        SizeChanged += (_,_) => {
            bool compact=ActualHeight<860;
            tabs.Height=compact?140:184;
            chartTitle.Visibility=errorPlot.Visibility=chartNote.Visibility=compact?Visibility.Collapsed:Visibility.Visible;
        };
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
