using System.Globalization;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;
using Path = System.IO.Path;

namespace TwoLinkCommissioning;

public sealed partial class XYPreviewPanel : UserControl
{
    // 固件临时软件范围（DRV8825.cydsn/main.c: PTP_MIN1_CD..PTP_MAX2_CD，单位 0.01°）。
    // 留空用默认值；预览、规划和设备执行使用同一范围，设备确认后才允许运动。
    public static readonly JointLimits DefaultTrialRange = new(-30, 30, -60, 60);
    public JointLimits? RequestedLimits { get { try { return PreviewLimits(); } catch (ArgumentException) { return null; } } }
    public PlanningRates RequestedRates => new(Number(feed),Number(v1),Number(v2),Number(acc1),Number(acc2));
    public MotionProfile? RequestedProfile { get { try { _ = Model; return MotionProfile.From(RequestedRates); } catch(ArgumentException) { return null; } } }
    public event Action? LimitsChanged;
    private JointLimits? lastRequestedLimits = DefaultTrialRange;
    private MotionProfile? lastRequestedProfile;
    private void NotifyLimitsChanged()
    {
        var requested = RequestedLimits;
        var profile = RequestedProfile;
        if (requested == lastRequestedLimits && profile == lastRequestedProfile) return;
        lastRequestedLimits = requested;
        lastRequestedProfile = profile;
        LimitsChanged?.Invoke();
    }
    private readonly List<Control> editControls = new();
    private TelemetrySample? startFeedback;
    private JointLimits? planningLimits;
    private bool updating;
    private bool parametersDirty;
    private TextBox[] ParameterFields => [l1,l2,x,y,a,b,min1,max1,min2,max2,feed,v1,v2,acc1,acc2];
    private sealed record SavedParameters(int Schema, int Shape, int Branch, string[] Fields);
    private string SaveParameters() => JsonSerializer.Serialize(new SavedParameters(1,shape.SelectedIndex,branch.SelectedIndex,ParameterFields.Select(f=>f.Text).ToArray()),new JsonSerializerOptions { WriteIndented=true });
    private void RestoreParameters(string json)
    {
        var p=JsonSerializer.Deserialize<SavedParameters>(json);
        if(p is null || p.Schema!=1 || p.Shape is <0 or >3 || p.Branch is <0 or >1 || p.Fields is null ||
            p.Fields.Length!=15 || p.Fields.Any(v=>v is null || v.Length>32)) throw new ArgumentException("参数文件格式不兼容或字段无效。");
        updating=true;
        try { shape.SelectedIndex=p.Shape; branch.SelectedIndex=p.Branch; for(int i=0;i<15;i++) ParameterFields[i].Text=p.Fields[i]; }
        finally { updating=false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }
    private static string ParametersPath => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "parameters.json");
    public event Action? ParametersSaved;
    private void SaveCurrentParameters(string path)
    {
        Refresh();
        try {
            _=Model; _=PreviewLimits(); _=MotionProfile.From(RequestedRates);
            _=Number(x); _=Number(y);
            if(shape.SelectedIndex>0) _=Number(a);
            if(shape.SelectedIndex is 1 or 2) _=Number(b);
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            File.WriteAllText(path + ".tmp", SaveParameters(), new UTF8Encoding(false));
            File.Move(path + ".tmp", path, true);
            ParametersSaved?.Invoke();
        } catch(Exception ex) when(ex is IOException or UnauthorizedAccessException or ArgumentException) { operationError="保存参数失败："+ex.Message; UpdatePointButton(); }
    }
    public void RestoreSavedParameters() => ReadSavedParameters(ParametersPath);
    private void ReadSavedParameters(string path)
    {
        if (!File.Exists(path)) return;
        try {
            if (new FileInfo(path).Length > 32768) throw new ArgumentException("参数文件过大。");
            RestoreParameters(File.ReadAllText(path));
        } catch(Exception ex) when(ex is IOException or UnauthorizedAccessException or JsonException or ArgumentException) { operationError="恢复参数失败："+ex.Message; UpdatePointButton(); }
    }
    private bool detailView = false;
    private JointLimits? overlayLimits = DefaultTrialRange;
    public PlanarArm Model => new(Number(l1), Number(l2));
    public string SettingsDescription => $"shape={shape.SelectedIndex}; branch={branch.SelectedIndex}; L1={l1.Text}; L2={l2.Text}; x={x.Text}; y={y.Text}; a={a.Text}; b={b.Text}; feed={feed.Text}; v1={v1.Text}; v2={v2.Text}; a1={acc1.Text}; a2={acc2.Text}; min1={min1.Text}; max1={max1.Text}; min2={min2.Text}; max2={max2.Text}";
    public void SetStartFeedback(TelemetrySample? sample) {
        bool changed=startFeedback!=sample; startFeedback = sample; UpdatePointButton();
        if(changed) RequestViewportRedraw();
    }
    public FrameworkElement ParameterContent { get; private set; } = null!;
    private FrameworkElement playbackControls = null!;
    public void SetDiagnosticsExpanded(bool expanded)
    {
        if(expanded) PausePlayback();
        playbackControls.Visibility=sampleInfo.Visibility=expanded?Visibility.Collapsed:Visibility.Visible;
    }
    public void SetEditingAllowed(bool allowed)
    {
        foreach(var c in editControls) c.IsEnabled = allowed;
        if (allowed) { export.IsEnabled = play.IsEnabled = rewind.IsEnabled = cursor.IsEnabled = timing is not null; }
        else PausePlayback();
    }
    private void LoadExample()
    {
        updating = true;
        try {
            shape.SelectedIndex=3; branch.SelectedIndex=0; x.Text="340"; y.Text="0"; a.Text="5"; b.Text="10";
        } finally { updating=false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }
    private readonly TextBox l1 = Field("180"), l2 = Field("176.38"), x = Field("340"), y = Field("0");
    private readonly TextBox min1 = Field(""), max1 = Field(""), min2 = Field(""), max2 = Field("");
    private readonly ComboBox branch = new() { Width = 132, Margin = new Thickness(4), ItemsSource = new[] { "肘角 ≥ 0 分支", "肘角 ≤ 0 分支" }, SelectedIndex = 0 };
    private readonly ComboBox shape = new() { Width = 94, Margin = new Thickness(4), ItemsSource = new[] { "单点", "直线", "矩形", "圆" }, SelectedIndex = 3 };
    private readonly TextBox a = Field("5"), b = Field("10");
    private readonly TextBlock xLabel = new(), yLabel = new(), aLabel = new(), bLabel = new();
    private readonly Slider cursor = new() { Minimum = 0, Maximum = 1, SmallChange = 0.1, LargeChange = 1, MinWidth = 180, Margin = new Thickness(4), IsEnabled = false };
    private readonly TextBox feed = Field("20"), v1 = Field("5"), v2 = Field("3"), acc1 = Field("10"), acc2 = Field("6");
    private readonly TextBlock speedState = new() { TextWrapping=TextWrapping.Wrap, FontSize=11, Margin=new Thickness(4), Foreground=WorkbenchTheme.Muted };
    private readonly Button play = new() { Content = "▶ 预览", Margin = new Thickness(4), Padding = new Thickness(12, 5, 12, 5), IsEnabled = false };
    private readonly Button rewind = new() { Content = "↺ 重播", Margin = new Thickness(4), Padding = new Thickness(12, 5, 12, 5), IsEnabled = false };
    private readonly DispatcherTimer playbackTimer = new() { Interval = TimeSpan.FromMilliseconds(33) };
    private readonly Stopwatch playbackClock = new();
    private double playbackFrom;
    private bool advancing;
    private readonly Button export = new() { Content = "导出规划 CSV", Margin = new Thickness(4), Padding = new Thickness(12, 5, 12, 5), IsEnabled = false };
    private readonly TextBox sampleInfo = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, BorderThickness = new Thickness(0), Background = Brushes.Transparent, Margin = new Thickness(4) };
    private DrawingTrajectory? trajectory;
    private TimedTrajectory? timing;
    private readonly Button movePoint = new() { Content = "低速移到此 XY 点", Margin = new Thickness(4), Padding = new Thickness(12, 5, 12, 5), IsEnabled = false };
    private readonly TextBlock moveState = new() { Text = "低速实机试运行：连接后人工确认伸直零位，或执行开关回零。", Margin = new Thickness(4), TextWrapping = TextWrapping.Wrap };
    private readonly TextBlock actionSummary = new() { FontWeight=FontWeights.SemiBold, MaxWidth=390, TextTrimming=TextTrimming.CharacterEllipsis, VerticalAlignment=VerticalAlignment.Center };
    private readonly Border actionStatus = new() { CornerRadius=new CornerRadius(6), Padding=new Thickness(10,8,10,8), Margin=new Thickness(8,4,4,4), VerticalAlignment=VerticalAlignment.Center };
    private readonly ProgressBar actionProgress = new() { Height=4, Minimum=0, Maximum=100, Foreground=WorkbenchTheme.Teal, Background=WorkbenchTheme.Line, Margin=new Thickness(4,6,4,0), Visibility=Visibility.Collapsed };
    private string? deviceStatusMessage, operationError;
    private bool deviceStatusFault, deviceStatusBusy;
    private JointPose? pointTarget;
    private JointLimits? firmwareLimits;
    private ReachabilityMask? reachability;
    private bool overlayUnreachable = true;
    private readonly List<Button> rangeHelp = [];
    private readonly TextBlock rangeState = new() { FontSize=11, TextWrapping=TextWrapping.Wrap, Margin=new Thickness(4), Foreground=WorkbenchTheme.Muted };
    private bool pointMoveAllowed;
    private bool deviceLimitsConfirmed;
    private string deviceMoveReason = "等待连接并建立人工或开关坐标参考。";
    public event Action<JointPose>? PointMoveRequested;
    public event Action<ExecutionTrajectory>? TrajectoryRequested;
    private readonly Button execute = new() { Content = "上传并执行图形", Margin = new Thickness(4), Padding = new Thickness(12, 5, 12, 5), IsEnabled = false };
    private readonly TextBox executionInfo = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, Background = Brushes.Transparent, BorderThickness = new Thickness(0), Margin = new Thickness(4) };
    private ExecutionTrajectory? execution;
    private bool trajectorySupported;
    private readonly Canvas canvas = new() { Background = Brushes.White, ClipToBounds = true, MinHeight=260 };
    private readonly TextBox result = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, BorderThickness = new Thickness(0),
        Background = Brushes.Transparent, Margin = new Thickness(4), MinHeight = 75 };
    private readonly TextBlock notice = new() { TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4), Foreground = Brushes.DarkSlateGray };

    public XYPreviewPanel()
    {
        Background = Brushes.White;
        var shell = new Grid { Margin = new Thickness(12) };
        shell.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        shell.RowDefinitions.Add(new RowDefinition());
        shell.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        Content = new Border { Background = Brushes.White, CornerRadius = new CornerRadius(10), BorderBrush = WorkbenchTheme.Line, BorderThickness = new Thickness(1), Child = shell };
        var title = new DockPanel();
        var sample = new Button { Content = "载入小圆示例", Margin = new Thickness(3) };
        sample.Click += (_,_) => LoadExample(); DockPanel.SetDock(sample,Dock.Right); title.Children.Add(sample);
        editControls.Add(sample); title.Children.Add(BuildViewportToolbar()); shell.Children.Add(title);
        var main = new Grid { Margin = new Thickness(0,8,0,8) }; Grid.SetRow(main,1); shell.Children.Add(main);
        var controls = new StackPanel { Margin = new Thickness(8) };
        ParameterContent = new ScrollViewer { Content = controls, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled };
        controls.Children.Add(WorkbenchTheme.Text("图形与几何",16));
        var target = new WrapPanel(); target.Children.Add(shape); target.Children.Add(branch); controls.Children.Add(target);
        var coordinates = new WrapPanel(); AddLabeledField(coordinates,xLabel,x); AddLabeledField(coordinates,yLabel,y); controls.Children.Add(coordinates);
        var dimensions = new WrapPanel(); AddLabeledField(dimensions,aLabel,a); AddLabeledField(dimensions,bLabel,b); controls.Children.Add(dimensions);
        var preview = new Button { Content = "检查并预览" }; preview.Click += (_,_) => Refresh();
        var previewActions = new WrapPanel(); previewActions.Children.Add(preview); previewActions.Children.Add(export); controls.Children.Add(previewActions);
        export.Click += (_,_) => Export(); editControls.Add(preview); editControls.Add(export);
        var advanced = new StackPanel();
        var lengths = new WrapPanel(); AddField(lengths,"L1 / mm",l1); AddField(lengths,"L2 / mm",l2); advanced.Children.Add(lengths);
        var rangeTitle = new WrapPanel { Margin = new Thickness(0,8,0,0) };
        rangeTitle.Children.Add(WorkbenchTheme.Text("关节范围 / °",14));
        var rangeHelpIcon = HelpIcon(RangeHelpText());
        rangeHelp.Add(rangeHelpIcon);
        rangeTitle.Children.Add(rangeHelpIcon); controls.Children.Add(rangeTitle);
        controls.Children.Add(new TextBlock { Text="默认 q1 ∈ [−30°, +30°]\n默认 q2 ∈ [−60°, +60°]", Foreground=WorkbenchTheme.Muted, FontSize=12, Margin=new Thickness(4) });
        var ranges = new Grid();
        for(int column=0;column<3;column++) ranges.ColumnDefinitions.Add(new ColumnDefinition { Width=GridLength.Auto });
        for(int row=0;row<3;row++) ranges.RowDefinitions.Add(new RowDefinition { Height=GridLength.Auto });
        void RangeCell(UIElement element, int row, int column) { Grid.SetRow(element,row); Grid.SetColumn(element,column); ranges.Children.Add(element); }
        RangeCell(WorkbenchTheme.Text("最小",12),0,1); RangeCell(WorkbenchTheme.Text("最大",12),0,2);
        RangeCell(WorkbenchTheme.Text("q1",12),1,0); RangeCell(min1,1,1); RangeCell(max1,1,2);
        RangeCell(WorkbenchTheme.Text("q2",12),2,0); RangeCell(min2,2,1); RangeCell(max2,2,2);
        foreach(var (field,name,value) in new[] { (min1,"q1 最小",-30), (max1,"q1 最大",30), (min2,"q2 最小",-60), (max2,"q2 最大",60) }) {
            System.Windows.Automation.AutomationProperties.SetName(field,name);
            field.ToolTip=$"{name} / °；留空使用 {value}°，填写后使用你的值。";
        }
        controls.Children.Add(ranges);
        controls.Children.Add(WorkbenchTheme.Text("留空用默认值 · 修改后红色区域实时更新",11,WorkbenchTheme.Muted));
        controls.Children.Add(rangeState);
        controls.Children.Add(BuildElbowRulePanel());
        var speeds = new WrapPanel(); AddField(speeds,"末端 mm/s",feed); AddField(speeds,"电机1 °/s",v1); AddField(speeds,"电机2 (q2−q1) °/s",v2); AddField(speeds,"电机1 °/s²",acc1); AddField(speeds,"电机2 (q2−q1) °/s²",acc2); advanced.Children.Add(speeds);
        advanced.Children.Add(WorkbenchTheme.Text("全局：杆长用于全部图形、命令 XY 与记录；关节速度/加速度用于绘图、定位、点动和回零。末端 mm/s 用于 XY 绘制与定位，不用于未建立坐标的回零。",12,WorkbenchTheme.Muted));
        advanced.Children.Add(speedState);
        controls.Children.Add(new Expander { Header = "机构尺寸 / 速度（全局）", Content = advanced, Margin = new Thickness(3,8,3,3) });
        var details = new StackPanel(); details.Children.Add(executionInfo); details.Children.Add(moveState); details.Children.Add(result); details.Children.Add(notice);
        controls.Children.Add(new Expander { Header = "查看规划检查详情", Content = details, Margin = new Thickness(3,8,3,3) });
        var save=new Button { Content="保存" };
        save.Click+=(_,_)=>SaveCurrentParameters(ParametersPath); controls.Children.Add(save); editControls.Add(save);
        var visual = new Grid(); main.Children.Add(visual);
        visual.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        visual.RowDefinitions.Add(new RowDefinition());
        visual.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        visual.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        Grid.SetRow(canvas,1); visual.Children.Add(canvas);
        var playback = new DockPanel { Margin = new Thickness(0,6,0,0) };
        playbackControls=playback;
        playback.Children.Add(play); playback.Children.Add(rewind);
        playback.Children.Add(cursor); Grid.SetRow(playback,2); visual.Children.Add(playback);
        sampleInfo.FontSize = 11; Grid.SetRow(sampleInfo,3); visual.Children.Add(sampleInfo);
        InitializeViewport();
        var actions = new StackPanel(); Grid.SetRow(actions,2); shell.Children.Add(actions);
        var motion = new WrapPanel(); motion.Children.Add(movePoint); motion.Children.Add(execute);
        actionStatus.Child=actionSummary; motion.Children.Add(actionStatus); actions.Children.Add(motion); actions.Children.Add(actionProgress);
        executionInfo.FontSize=12; executionInfo.Padding=new Thickness(0);
        WorkbenchTheme.Primary(movePoint); WorkbenchTheme.Primary(execute); movePoint.Click += (_,_) => { if(movePoint.IsEnabled && pointTarget is {} p) PointMoveRequested?.Invoke(PtpProtocol.Quantize(p)); };
        execute.Click += (_,_) => { if(execute.IsEnabled && execution is {} plan) TrajectoryRequested?.Invoke(plan); };
        moveState.FontSize = 12;
        foreach(var control in new Control[] { l1,l2,x,y,a,b,min1,max1,min2,max2,feed,v1,v2,acc1,acc2,shape,branch,play,rewind,cursor }) editControls.Add(control);
        foreach (var field in new[] { l1, l2, x, y, a, b, min1, max1, min2, max2, feed, v1, v2, acc1, acc2 }) field.TextChanged += (_, _) => ParametersChanged();
        foreach (var help in rangeHelp) editControls.Add(help);
        branch.SelectionChanged += (_, _) => ParametersChanged();
        shape.SelectionChanged += (_, _) => { ShapeFields(); ParametersChanged(); };
        cursor.ValueChanged += (_, _) => { if (!advancing) PausePlayback(); DrawSample(); };
        canvas.SizeChanged += (_, _) => RedrawViewport();
        play.Click += (_, _) => TogglePlayback(); rewind.Click += (_, _) => { PausePlayback(); cursor.Value = 0; };
        playbackTimer.Tick += (_, _) => AdvancePlayback(playbackFrom + playbackClock.Elapsed.TotalSeconds);
        Unloaded += (_, _) => PausePlayback();
        ShapeFields();
        Refresh();
    }

    private static TextBox Field(string text) => new() { Text = text, Width = 76, Margin = new Thickness(4), Padding = new Thickness(4) };
    private static void AddField(Panel panel, string label, TextBox field)
    {
        System.Windows.Automation.AutomationProperties.SetName(field,label);
        var pair=new StackPanel { Orientation=Orientation.Horizontal };
        pair.Children.Add(new TextBlock { Text = label, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(4) });
        pair.Children.Add(field); panel.Children.Add(pair);
    }
    private static Button HelpIcon(string tip)
    {
        var help=new Button { Content="?", Width=22, MinHeight=22, Padding=new Thickness(0), Margin=new Thickness(2,4,6,4),
            FontWeight=FontWeights.Bold, ToolTip=tip };
        System.Windows.Automation.AutomationProperties.SetName(help, "关节范围说明");
        System.Windows.Automation.AutomationProperties.SetHelpText(help, tip);
        ToolTipService.SetShowOnDisabled(help, true);
        ToolTipService.SetInitialShowDelay(help, 200);
        return help;
    }
    private string RangeHelpText()
    {
        return "关节范围 / °：q1、q2 两个关节角的规划范围（两轴逆时针为正，0° 为两杆伸直的零位）。\n" +
            "· q1 ＝ arm1：主动臂（经 40:1 减速箱、带 AS5045 编码器反馈）。\n" +
            "· q2 ＝ arm2 相对固定 +X 零方向的夹角；第二轴电机实际转角为 q2−q1（无编码器反馈）。\n" +
            "· 因此 q2 范围不直接限制肘：肘可达 [q2 最小−q1 最大, q2 最大−q1 最小]，固件不检查，见下方“肘行程”图（红色＝违反你设的安全行程 E）。\n" +
            "· 不确定哪个是哪个：按住点动 \"arm1 ← / D →\" 看状态栏，或轻推对应杆件，读数变化的那一路就是它。\n" +
            "· 每个输入框留空时使用对应默认值：q1 ∈ [−30°, +30°]，q2 ∈ [−60°, +60°]。\n" +
            "· 填写后以你填的值为准：" + EnteredEnvelope() + "。\n" +
            "· 透明红色表示在此范围和连杆长度下，两个逆解分支都无法到达的区域；修改后实时更新。\n" +
            "· 红区、轨迹规划和设备执行共用这组范围。未使能时自动同步到支持范围设置的固件，确认一致后才允许运动。\n" +
            "· 两轴范围须在 −180°～180° 内并包含 0°，最多两位小数。修改已使能设备的范围后，先总停止再同步。\n" +
            "· 这是软件关节范围，不是实测碰撞边界。";
    }
    private string EnteredEnvelope()
    {
        try { return Envelope(PreviewLimits()); } catch(ArgumentException) { return "当前输入无效，请检查上下限"; }
    }
    private static string Envelope(JointLimits? limits) => limits is { } range
        ? FormattableString.Invariant($"q1 [{range.Min1:F2}, {range.Max1:F2}]°、q2 [{range.Min2:F2}, {range.Max2:F2}]°")
        : "未填写";
    private void RefreshRangeHelp()
    {
        try { var r=RequestedRates; var p=MotionProfile.From(r); var d=p.DrawingRates(r.Feed);
            speedState.Text=$"实际绘图上限：末端 {d.Feed:F2} mm/s；电机1/电机2(q2−q1) {d.V1:F2}/{d.V2:F2} °/s；加速度 {d.A1:F2}/{d.A2:F2} °/s²。\n"+p.Description;
        } catch(ArgumentException ex) { speedState.Text=ex.Message; }
        var tip=RangeHelpText();
        foreach(var help in rangeHelp) { help.ToolTip=tip; System.Windows.Automation.AutomationProperties.SetHelpText(help,tip); }
        rangeState.Text = RequestedLimits is not { } requested ? "范围无效：须包含 0°、上下限有序，最多两位小数。"
            : firmwareLimits is null ? "红区与规划使用上方范围；设备确认后才能运动。"
            : requested == firmwareLimits && deviceLimitsConfirmed ? "设备已确认：红区、规划、执行范围一致。"
            : "设备范围尚未同步；运动已锁定。\n" + deviceMoveReason + "\n设备当前：" + Envelope(firmwareLimits);
        RefreshElbowRule();
    }
    private void RefreshOverlayLimits()
    {
        try { overlayLimits = PreviewLimits(); } catch(ArgumentException) { overlayLimits = null; }
    }
    private static void AddLabeledField(Panel panel, TextBlock label, TextBox field)
    {
        label.VerticalAlignment = VerticalAlignment.Center; label.Margin = new Thickness(4);
        var pair=new StackPanel { Orientation=Orientation.Horizontal }; pair.Children.Add(label); pair.Children.Add(field); panel.Children.Add(pair);
    }
    private void ShapeFields()
    {
        int s = shape.SelectedIndex;
        xLabel.Text = s == 3 ? "圆心 X / mm" : s == 2 ? "左下角 X / mm" : s == 1 ? "起点 X / mm" : "X / mm";
        yLabel.Text = s == 3 ? "圆心 Y / mm" : s == 2 ? "左下角 Y / mm" : s == 1 ? "起点 Y / mm" : "Y / mm";
        aLabel.Text = s == 3 ? "半径 / mm" : s == 2 ? "宽 / mm" : "终点 X / mm";
        bLabel.Text = s == 2 ? "高 / mm" : "终点 Y / mm";
        a.Visibility = aLabel.Visibility = s == 0 ? Visibility.Collapsed : Visibility.Visible;
        b.Visibility = bLabel.Visibility = s is 0 or 3 ? Visibility.Collapsed : Visibility.Visible;
        System.Windows.Automation.AutomationProperties.SetName(x,xLabel.Text); System.Windows.Automation.AutomationProperties.SetName(y,yLabel.Text);
        System.Windows.Automation.AutomationProperties.SetName(a,aLabel.Text); System.Windows.Automation.AutomationProperties.SetName(b,bLabel.Text);
    }
    private static double Number(TextBox field)
    {
        if (!double.TryParse(field.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double value) || !double.IsFinite(value))
            throw new ArgumentException(System.Windows.Automation.AutomationProperties.GetName(field) + "：请输入有效数值，小数点使用 .，不能留空或输入 NaN/Infinity。");
        return value;
    }
    private JointLimits? Limits()
    {
        if (new[] { min1, max1, min2, max2 }.All(f => string.IsNullOrWhiteSpace(f.Text))) return null;
        static double Bound(TextBox field, double fallback)
        {
            double value = string.IsNullOrWhiteSpace(field.Text) ? fallback : Number(field);
            double rounded = Math.Round(value * 100, MidpointRounding.AwayFromZero) / 100;
            if (Math.Abs(value - rounded) > 1e-8) throw new ArgumentException("关节范围最多保留两位小数（0.01°）。");
            return rounded;
        }
        var limits = new JointLimits(Bound(min1,-30), Bound(max1,30), Bound(min2,-60), Bound(max2,60));
        if (limits.Min1 < -180 || limits.Max1 > 180 || limits.Min2 < -180 || limits.Max2 > 180 ||
            limits.Min1 >= limits.Max1 || limits.Min2 >= limits.Max2 ||
            limits.Min1 > 0 || limits.Max1 < 0 || limits.Min2 > 0 || limits.Max2 < 0)
            throw new ArgumentException("关节范围须位于 -180°～180° 内、包含 0°，且最小值小于最大值；留空项使用默认值。");
        return limits;
    }
    private JointLimits PreviewLimits() => Limits() ?? DefaultTrialRange;
    private void Refresh()
    {
        if (updating) return;
        parametersDirty = false;
        cameraReady = false; autoFit = true; operationError=null;
        RefreshOverlayLimits(); CaptureElbowPath(); RefreshRangeHelp();
        pointTarget = null; movePoint.IsEnabled = false; execution = null; execute.IsEnabled = false; executionInfo.Clear();
        PausePlayback(); trajectory = null; timing = null;
        export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = false;
        canvas.Children.Clear(); result.Clear(); sampleInfo.Clear(); notice.Text = "";
        try
        {
            var model = new PlanarArm(Number(l1), Number(l2));
            _ = MotionProfile.From(RequestedRates);
            double tx = Number(x), ty = Number(y);
            JointLimits? limits = PreviewLimits(); planningLimits = limits;
            if (shape.SelectedIndex > 0)
            {
                JointLimits? bounds = limits;
                trajectory = DrawingTrajectory.Create(model, (DrawingShape)(shape.SelectedIndex - 1), tx, ty,
                    Number(a), shape.SelectedIndex == 3 ? 0 : Number(b), Math.Max(0, branch.SelectedIndex), bounds);
                timing = new TimedTrajectory(trajectory, RequestedRates);
                pointTarget = timing.At(0).Pose; RefreshExecution(); UpdatePointButton();
                notice.Text = bounds is null ? "关节范围未测定；机械范围未检查。CSV 为规划数据，不能直接驱动机械臂。" :
                    "已检查全部采样点的关节范围；未验证机械碰撞、线缆干涉或从当前位置到起点的路径。";
                result.Foreground = Brushes.DarkSlateGray;
                string conclusion = "整条轨迹检查通过：使用当前关节范围";
                result.Text = $"{conclusion}；{trajectory.Samples.Length} 点；路径长度 {timing.Length:F2} mm；预计 {timing.Duration:F2} 秒。\n" +
                    $"q1：{trajectory.Samples.Min(s => s.Pose.Q1):F2}～{trajectory.Samples.Max(s => s.Pose.Q1):F2}°；" +
                    $"q2：{trajectory.Samples.Min(s => s.Pose.Q2):F2}～{trajectory.Samples.Max(s => s.Pose.Q2):F2}°。\n" +
                    "预览与执行共用设备上限内的时间规划；执行另含每段末尾 50 ms 停留。";
                if (trajectory.NearSingularity) result.Text += "\n提示：部分姿态接近奇异位形，微小 XY 变化可能引起较大关节变化。";
                cursor.Maximum = timing.Duration; cursor.Value = 0;
                export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = true; DrawSample();
                return;
            }
            var poses = model.Inverse(tx, ty);
            var selected = poses[Math.Min(branch.SelectedIndex < 0 ? 0 : branch.SelectedIndex, poses.Length - 1)];
            pointTarget = selected; UpdatePointButton();
            notice.Text = limits is null ? "两轴左转＝逆时针已确认；此处机械范围未填写。实际单点移动另受固件标定锁和固件范围限制。" :
                "目标按当前关节范围检查，设备确认同一范围后才允许运动；尚未验证机械碰撞或线缆干涉。";
            result.Foreground = Brushes.DarkSlateGray;
            result.Text = $"目标 ({tx:F2}, {ty:F2}) mm；几何半径 {model.InnerRadius:F2}～{model.OuterRadius:F2} mm\n" +
                string.Join("\n", poses.Select((p, i) => $"解 {i + 1}：q1={p.Q1:F2}°，q2={p.Q2:F2}°；" +
                    (limits is null ? "机械范围未检查" : limits.Value.Contains(p) ? "在填写的范围内" : "超出填写的范围")));
            if (Math.Abs(Math.Sin((selected.Q2 - selected.Q1) * Math.PI / 180)) < 1e-6)
                result.Text += "\n当前是伸直或完全折叠的奇异位形；不能据此认为连续轨迹可执行。";
            Draw(model, selected, tx, ty, limits is null || limits.Value.Contains(selected), Reachability(model).IsReachable(model.Forward(selected).Tip));
        }
        catch (ArgumentException ex) { InvalidPlan(ex.Message); }
    }
    private void ParametersChanged()
    {
        if (updating) return;
        parametersDirty = true;
        cameraReady = false; autoFit = true;
        RefreshOverlayLimits(); RefreshRangeHelp();
        pointerInfo.Text = PointerHint;
        InvalidPlan("参数已修改，请点击“检查并预览”或“保存”后应用。");
        NotifyLimitsChanged();
    }
    private void InvalidPlan(string reason)
    {
        PausePlayback(); trajectory = null; timing = null;
        export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = false;
        canvas.Children.Clear(); sampleInfo.Clear(); result.Foreground = Brushes.Firebrick; result.Text = reason;
        pointTarget = null; movePoint.IsEnabled = false;
        execution = null; execute.IsEnabled = false; executionInfo.Clear();
        UpdatePointButton();
        RequestViewportRedraw();
    }
    public void SetPointMoveState(bool allowed, JointLimits? limits, string reason, bool supportsTrajectory = false, bool limitsConfirmed = true)
    {
        pointMoveAllowed = allowed; firmwareLimits = limits; trajectorySupported = supportsTrajectory; deviceMoveReason = reason;
        deviceLimitsConfirmed = limitsConfirmed;
        RefreshOverlayLimits(); RefreshRangeHelp();
        UpdatePointButton();
    }
    private void RefreshExecution()
    {
        execution = null; executionInfo.Clear();
        if (timing is null || planningLimits is not { } range) return;
        try {
            execution = new ExecutionTrajectory(timing, range);
            timing = execution.Timing;
            executionInfo.Text = $"低速执行计划 · {execution.Nodes.Length} 点 · {execution.Duration:F2} 秒 · 每轴 ≤{DeviceContract.PlanRateMax} 微步/秒";
        } catch (ArgumentException ex) { executionInfo.Text = "执行检查未通过：" + ex.Message; }
    }
    private void UpdatePointButton()
    {
        movePoint.Content = shape.SelectedIndex == 0 ? "低速移到此 XY 点" : "低速移到轨迹起点";
        bool limitsMatch = deviceLimitsConfirmed && RequestedLimits is { } requested && requested == firmwareLimits && requested == planningLimits;
        movePoint.IsEnabled = pointMoveAllowed && limitsMatch && pointTarget is { } p && firmwareLimits is { } range && range.Contains(PtpProtocol.Quantize(p)) && (planningLimits is not { } local || local.Contains(p)) &&
            (shape.SelectedIndex == 0 || (execution is not null && trajectorySupported));
        bool atStart = execution is not null && startFeedback is { Referenced: true, Inputs: 0 } &&
            startFeedback.Steps1 == ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose) && startFeedback.Steps2 == ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose);
        execute.IsEnabled = pointMoveAllowed && limitsMatch && trajectorySupported && execution is not null && atStart;
        moveState.Text = deviceMoveReason;
        if (RequestedLimits is { } currentRange) moveState.Text += "\n当前软件范围：" + Envelope(currentRange);
        if (firmwareLimits is not null && !limitsMatch) moveState.Text += "\n设备尚未确认当前范围/速度，运动已锁定；预览仍按输入设置显示。";
        if (pointTarget is { } target && firmwareLimits is { } bounds) {
            target = PtpProtocol.Quantize(target);
            string label = bounds.Contains(target) ? "目标在固件范围内" : shape.SelectedIndex == 0 ? "目标超出固件范围，不能移动" : "轨迹起点超出固件范围，不能移动";
            moveState.Text += FormattableString.Invariant($"\n{label}：q1={target.Q1:F2}°，q2={target.Q2:F2}°；固件允许 q1 [{bounds.Min1:F2}, {bounds.Max1:F2}]°、q2 [{bounds.Min2:F2}, {bounds.Max2:F2}]°。");
        } else if (pointTarget is null) moveState.Text += "\n参数检查未通过：" + result.Text;
        if (execution is not null) moveState.Text += atStart ? "\n已到当前轨迹起点；手动落笔后可执行。" : "\n图形执行：先移到起点并等待到位反馈（需 v5 固件）。";
        if (execution is null && shape.SelectedIndex > 0) moveState.Text += "\n" + (executionInfo.Text.Length > 0 ? executionInfo.Text : "先完成图形参数检查。");
        UpdateActionStatus();
        execute.ToolTip = moveState.Text + "\n流程：准备 → 抬笔移到起点 → 落笔绘制 → 保存结果。"; ToolTipService.SetShowOnDisabled(execute,true);
        movePoint.ToolTip = moveState.Text;
        ToolTipService.SetShowOnDisabled(movePoint, true);
    }
    public void SetDeviceStatus(string? message, bool fault, bool busy, double percent, bool indeterminate)
    {
        deviceStatusMessage=message; deviceStatusFault=fault; deviceStatusBusy=busy;
        // Unknown progress uses the status text; the theme's looping animation can starve input/heartbeat.
        actionProgress.Visibility=busy && !indeterminate?Visibility.Visible:Visibility.Collapsed;
        actionProgress.IsIndeterminate=false; actionProgress.Value=percent;
        UpdateActionStatus();
    }
    private void UpdateActionStatus()
    {
        string text; int severity; // 0 ready, 1 waiting, 2 rejected; text also conveys the state without color.
        if(deviceStatusFault || deviceStatusBusy) { text=(deviceStatusFault?"被拒绝 · ":"等待中 · ")+(deviceStatusMessage??deviceMoveReason); severity=deviceStatusFault?2:1; }
        else if(operationError is not null) { text="操作失败 · "+operationError; severity=2; }
        else if(parametersDirty) { text="等待中 · 参数已修改，请检查或保存后应用"; severity=1; }
        else if(pointTarget is null) { text="被拒绝 · "+result.Text; severity=2; }
        else if(planningLimits is {} local && !local.Contains(pointTarget.Value)) { text="被拒绝 · 目标超出范围，调整图形参数"; severity=2; }
        else if(firmwareLimits is not null && (!deviceLimitsConfirmed || firmwareLimits != RequestedLimits)) { text="等待中 · 设备尚未确认当前范围/速度"; severity=1; }
        else if(shape.SelectedIndex>0 && firmwareLimits is not null && execution is null) { text="被拒绝 · 整条轨迹未通过检查"; severity=2; }
        else if(!pointMoveAllowed) { text="等待中 · "+(deviceStatusMessage??deviceMoveReason); severity=1; }
        else if(execute.IsEnabled) { text="可执行 · 已到起点，请落笔后执行"; severity=0; }
        else if(shape.SelectedIndex==0 && movePoint.IsEnabled) { text="可移动 · 目标检查通过"; severity=0; }
        else if(!trajectorySupported) { text="等待中 · 当前固件不支持图形执行"; severity=1; }
        else { text="等待中 · 请先抬笔，低速移到起点"; severity=1; }
        actionSummary.Text=text;
        actionSummary.Foreground=WorkbenchTheme.Brush(severity==0?"#166534":severity==1?"#854D0E":"#991B1B");
        actionStatus.Background=WorkbenchTheme.Brush(severity==0?"#DCFCE7":severity==1?"#FEF3C7":"#FEE2E2");
        actionStatus.ToolTip=text+"\n"+moveState.Text+"\n完整参数见左侧“查看规划检查详情”。";
        System.Windows.Automation.AutomationProperties.SetName(actionStatus,text);
    }
    public void PausePlayback()
    { playbackTimer.Stop(); playbackClock.Stop(); play.Content = "▶ 预览"; }
    private void TogglePlayback()
    {
        if (timing is null) return;
        if (playbackTimer.IsEnabled) { PausePlayback(); return; }
        if (cursor.Value >= timing.Duration) cursor.Value = 0;
        playbackFrom = cursor.Value; playbackClock.Restart(); playbackTimer.Start(); play.Content = "Ⅱ 暂停预览";
    }
    private void AdvancePlayback(double time)
    {
        if (!playbackTimer.IsEnabled || timing is null) return;
        advancing = true;
        try { cursor.Value = Math.Min(time, timing.Duration); }
        finally { advancing = false; }
        if (timing is null || time >= timing.Duration) PausePlayback();
    }
    private void DrawSample()
    {
        if (timing is not { } plan) return;
        try
        {
            var sample = plan.At(cursor.Value); canvas.Children.Clear();
            Draw(plan.Path.Model, sample.Pose, sample.Target.X, sample.Target.Y, true, SampleReachable(plan.Path.Model, sample.Target));
            sampleInfo.Text = $"离线预览 · 时间 {sample.Time:F2}/{plan.Duration:F2} s · 路程 {sample.Distance:F2} mm · " +
                $"X={sample.Target.X:F2}，Y={sample.Target.Y:F2} mm";
            sampleInfo.ToolTip=$"仅屏幕预览，不代表执行进度。q1={sample.Pose.Q1:F2}°，q2={sample.Pose.Q2:F2}°；末端速度 {sample.Speed:F2} mm/s。";
        }
        catch (ArgumentException ex) { InvalidPlan(ex.Message); }
    }
    private void Export()
    {
        if (timing is not { } plan) return;
        PausePlayback();
        var dialog = new Microsoft.Win32.SaveFileDialog { Title = "保存规划轨迹（非实测数据）", Filter = "CSV 文件 (*.csv)|*.csv",
            DefaultExt = ".csv", FileName = $"timed-preview-{plan.Path.Shape}-{DateTime.Now:yyyyMMdd-HHmmss}.csv" };
        if (dialog.ShowDialog() != true) return;
        try { File.WriteAllText(dialog.FileName, plan.ToCsv(), new UTF8Encoding(true)); sampleInfo.Text = "已保存规划数据：" + dialog.FileName; }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or ArgumentException)
        { sampleInfo.Text = "保存失败：" + ex.Message; }
    }
    private static void UnifiedLimitsSelfTest()
    {
        var panel = new XYPreviewPanel();
        int changes = 0;
        panel.LimitsChanged += () => changes++;
        panel.x.Text="260"; panel.y.Text="0"; panel.a.Text="50";
        panel.min1.Text="-135"; panel.max1.Text="135"; panel.min2.Text="-155"; panel.max2.Text="155";
        panel.Refresh();
        var requested = new JointLimits(-135,135,-155,155);
        var plan = panel.timing;
        var mask = panel.Reachability(panel.Model);
        if (changes == 0 || panel.RequestedLimits != requested || panel.planningLimits != requested ||
            panel.overlayLimits != requested || plan is null || panel.execution is null || !panel.play.IsEnabled)
            throw new InvalidOperationException("截图圆必须按填写范围生成红区、预览和执行计划");
        if(!ReferenceEquals(panel.timing,panel.execution.Timing) || Math.Abs(panel.execution.Duration-panel.timing.Duration-0.05)>0.002)
            throw new InvalidOperationException("预览和实际执行没有共用同一时间规划");
        panel.SetPointMoveState(true, DefaultTrialRange, "旧设备范围尚未同步", true);
        if (!ReferenceEquals(plan,panel.timing) || !ReferenceEquals(mask,panel.Reachability(panel.Model)) ||
            !panel.play.IsEnabled || panel.movePoint.IsEnabled || panel.execute.IsEnabled)
            throw new InvalidOperationException("连接不同范围的设备改变了预览，或提前开放运动");
        panel.SetStartFeedback(new TelemetrySample(0,1,1,true,2,ExecutionTrajectory.Counts(0,panel.execution!.Nodes[0].Pose),
            ExecutionTrajectory.Counts(1,panel.execution.Nodes[0].Pose),0,0,0,0));
        if (panel.execute.IsEnabled) throw new InvalidOperationException("到位反馈绕过范围同步");
        panel.SetPointMoveState(true,requested,"已确认统一范围",true);
        if (!ReferenceEquals(plan,panel.timing) || !panel.movePoint.IsEnabled || !panel.execute.IsEnabled ||
            !panel.rangeState.Text.Contains("范围一致"))
            throw new InvalidOperationException("设备确认统一范围后未开放同一条轨迹");
        panel.SetPointMoveState(true,requested,"仍有范围请求等待完整确认",true,false);
        if (panel.movePoint.IsEnabled || panel.execute.IsEnabled || panel.rangeState.Text.Contains("范围一致"))
            throw new InvalidOperationException("报告范围相同不能绕过在途配置确认");
        panel.SetPointMoveState(false,null,"离线",false);
        if (!ReferenceEquals(plan,panel.timing) || panel.movePoint.IsEnabled || panel.execute.IsEnabled || !panel.play.IsEnabled)
            throw new InvalidOperationException("断开设备改变了预览或仍允许运动");
        panel.max1.Text="135.001";
        if (panel.RequestedLimits is not null || panel.overlayLimits is not null)
            throw new InvalidOperationException("预览接受了固件无法表达的范围精度");
        panel.max1.Text="135.01";
        if (panel.RequestedLimits?.Max1 != 135.01) throw new InvalidOperationException("0.01度精度不一致");
        panel.min1.Text="1";
        if (panel.RequestedLimits is not null) throw new InvalidOperationException("范围必须包含伸直参考零位");
        panel.min1.Text="-135"; panel.v1.Text="2"; panel.l1.Text="181";
        var beforeExample=panel.RequestedLimits; var beforeRates=panel.RequestedRates;
        panel.LoadExample();
        if (panel.RequestedLimits != beforeExample || panel.RequestedRates != beforeRates || panel.Model.L1!=181)
            throw new InvalidOperationException("载入示例覆盖了全局杆长、速度或范围");
    }
    public static void SelfTest(string? snapshotPath = null)
    {
        ReachabilityMask.SelfTest();
        UnifiedLimitsSelfTest();
        ElbowRuleSelfTest();
        var panel = new XYPreviewPanel();
        panel.ViewportSelfTest();
        if(panel.detailView) throw new InvalidOperationException("中间视图默认必须是全臂视图");
        if(!panel.overlayUnreachable || panel.overlayLimits is not { } trial || trial != DefaultTrialRange)
            throw new InvalidOperationException("默认应按固件范围显示不可达区域");
        if(!panel.Reachability(new PlanarArm(180, 176.38)).IsReachable(new Point(330, 0)) ||
            panel.Reachability(new PlanarArm(180, 176.38)).IsReachable(new Point(300, 0)))
            throw new InvalidOperationException("默认范围下的可达判定不符");
        if(!ReferenceEquals(panel.Reachability(panel.Model), panel.Reachability(panel.Model)))
            throw new InvalidOperationException("可达掩码未复用");
        var previousMask = panel.Reachability(panel.Model);
        panel.min1.Text = "-90"; panel.max1.Text = "90";
        if(panel.overlayLimits != new JointLimits(-90, 90, -60, 60))
            throw new InvalidOperationException("填写范围应立即生效，未填写的项继承默认值");
        panel.RedrawViewport();
        if(ReferenceEquals(previousMask,panel.Reachability(panel.Model)) || !panel.Reachability(panel.Model).IsReachable(
            panel.Model.Forward(new JointPose(85, 50)).Tip) || panel.canvas.Children.Count == 0 ||
            panel.timing is not null || panel.movePoint.IsEnabled || panel.execute.IsEnabled)
            throw new InvalidOperationException("修改范围应实时重画工作区，但不得恢复旧运动计划");
        panel.SetPointMoveState(false, DefaultTrialRange, "模拟设备范围", true);
        if(panel.overlayLimits != new JointLimits(-90, 90, -60, 60))
            throw new InvalidOperationException("设备范围不得覆盖用户的可达区域设置");
        var enlarged = panel.Reachability(panel.Model);
        panel.l1.Text = "190"; panel.RedrawViewport();
        if(ReferenceEquals(enlarged,panel.Reachability(panel.Model)))
            throw new InvalidOperationException("修改连杆长度后未更新红色区域");
        panel.l1.Text = "180";
        panel.min1.Text="100";
        if(panel.overlayLimits is not null) throw new InvalidOperationException("无效关节范围仍保留旧红区");
        panel.RedrawViewport();
        if(panel.canvas.Children.Count!=0) throw new InvalidOperationException("无效关节范围必须清除旧遮罩");
        panel.min1.Clear(); panel.max1.Clear(); panel.Refresh();
        if(panel.overlayLimits!=DefaultTrialRange || !panel.RangeHelpText().Contains("[−30°, +30°]"))
            throw new InvalidOperationException("清空输入未恢复默认范围或问号提示缺少默认值");
        // 只填 q2 最大时，其余留空项仍使用对应默认值。
        panel.max2.Text="45"; panel.Refresh();
        if(panel.overlayLimits!=new JointLimits(-30,30,-60,45) || panel.planningLimits!=new JointLimits(-30,30,-60,45))
            throw new InvalidOperationException("检查并预览未按当前输入（含留空项默认值）建立范围");
        panel.x.Text="300"; panel.Refresh();
        if(panel.timing is not null) throw new InvalidOperationException("收紧 q2 后 300 半径的整圆仍被接受");
        panel.x.Text="340"; panel.max2.Text="10"; panel.Refresh();
        if(panel.timing is not null) throw new InvalidOperationException("q2 收紧到 [-60°,10°] 后 340/0/r5 仍被接受");
        panel.max2.Clear(); panel.Refresh();
        if(panel.timing is null) throw new InvalidOperationException("清空 q2 最大后未恢复按默认范围检查");
        panel.overlayUnreachable = false; panel.Refresh();
        if(panel.PointerHint.Contains("不可达")) throw new InvalidOperationException("关闭图层后提示未复位");
        panel.overlayUnreachable = true; panel.Refresh();
        string saved=panel.SaveParameters(); panel.x.Text="400"; panel.RestoreParameters(saved);
        if(panel.x.Text!="340" || panel.pointTarget is null) throw new InvalidOperationException("Parameter roundtrip failed");
        try { panel.RestoreParameters("{\"Schema\":99}"); throw new InvalidOperationException("Invalid parameters accepted"); } catch(ArgumentException) { }
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟已回零状态；临时软件范围未标定", true);
        if (panel.execute.IsEnabled || !panel.movePoint.IsEnabled || panel.execution is null)
            throw new InvalidOperationException("Default trial circle is not executable in delivery bounds");
        if(!panel.actionSummary.Text.StartsWith("等待中")) throw new InvalidOperationException("Before-start status must be waiting");
        panel.SetStartFeedback(new TelemetrySample(0,1,1,true,2,ExecutionTrajectory.Counts(0,panel.execution.Nodes[0].Pose),ExecutionTrajectory.Counts(1,panel.execution.Nodes[0].Pose),0,0,0,0));
        if (!panel.execute.IsEnabled) throw new InvalidOperationException("At-start feedback did not enable execution");
        if(!panel.actionSummary.Text.StartsWith("可执行")) throw new InvalidOperationException("At-start conclusion not prominent");
        panel.SetDeviceStatus("模拟故障",true,false,0,false);
        if(!panel.actionSummary.Text.StartsWith("被拒绝")) throw new InvalidOperationException("Fault must override ready status");
        panel.SetDeviceStatus("上传 1 / 10 点",false,true,10,false);
        if(!panel.actionSummary.Text.Contains("上传") || panel.actionProgress.Visibility!=Visibility.Visible) throw new InvalidOperationException("Upload status missing");
        panel.SetDeviceStatus(null,false,false,0,false);
        if(!panel.actionSummary.Text.StartsWith("可执行") || panel.actionProgress.Visibility!=Visibility.Collapsed) throw new InvalidOperationException("Stale busy status retained");
        var fresh=panel.startFeedback; panel.SetStartFeedback(null);
        if(panel.execute.IsEnabled || panel.actionSummary.Text.StartsWith("可执行")) throw new InvalidOperationException("Missing feedback retained ready conclusion");
        panel.SetStartFeedback(fresh);
        panel.x.Text = "335";
        if (panel.movePoint.IsEnabled || panel.execute.IsEnabled || panel.timing is not null || panel.play.IsEnabled)
            throw new InvalidOperationException("Typing must invalidate the applied plan without rebuilding it before Check/Save");
        panel.Refresh(); panel.x.Text = "340"; panel.Refresh();
        panel.x.Text = "300"; panel.a.Text = "30";
        panel.Refresh();
        if (panel.movePoint.IsEnabled || panel.execute.IsEnabled || panel.result.Text.Contains("检查通过") ||
            !panel.result.Text.Contains("允许 q1"))
            throw new InvalidOperationException("300/0/r30 must fail the full preview check before any start movement");
        panel.SetEditingAllowed(true); panel.SetStartFeedback(fresh);
        if (panel.movePoint.IsEnabled || panel.execute.IsEnabled || panel.play.IsEnabled)
            throw new InvalidOperationException("Feedback/editing update bypassed the rejected full plan");
        panel.x.Text = "340"; panel.a.Text = "5";
        panel.Refresh();
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 32), "模拟范围收紧", true);
        if (panel.movePoint.IsEnabled || panel.execute.IsEnabled || panel.timing is null || !panel.play.IsEnabled ||
            !panel.actionSummary.Text.Contains("设备尚未确认"))
            throw new InvalidOperationException("设备范围不一致必须锁定运动、保留原预览");
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟范围恢复", true);
        if (!panel.movePoint.IsEnabled || !panel.execute.IsEnabled || !panel.result.Text.Contains("整条轨迹检查通过"))
            throw new InvalidOperationException("Restored firmware limits did not rebuild the full plan");
        panel.x.Text = "335"; panel.a.Text = "15";
        panel.Refresh();
        if (!panel.movePoint.IsEnabled || panel.execution is null || !panel.result.Text.Contains("整条轨迹检查通过"))
            throw new InvalidOperationException("335/0/r15 must produce a full executable circle inside current firmware limits");
        panel.x.Text = "340"; panel.a.Text = "5";
        string parameterFile = Path.Combine(Path.GetTempPath(), "two-link-parameters-" + Guid.NewGuid().ToString("N") + ".json");
        int saves = 0; panel.ParametersSaved += () => saves++;
        try {
            panel.SaveCurrentParameters(parameterFile);
            string applied = File.ReadAllText(parameterFile);
            if (saves != 1 || applied != panel.SaveParameters() || panel.editControls.OfType<Button>().Any(c => Equals(c.Content,"载入参数")))
                throw new InvalidOperationException("Direct parameter save did not apply/persist or retained the load button");
            panel.x.Text = "NaN"; panel.SaveCurrentParameters(parameterFile);
            if (saves != 1 || File.ReadAllText(parameterFile) != applied)
                throw new InvalidOperationException("Invalid numeric input overwrote saved parameters");
            panel.ReadSavedParameters(parameterFile);
            if (panel.SaveParameters() != applied || !panel.movePoint.IsEnabled || !panel.execute.IsEnabled)
                throw new InvalidOperationException("Saved parameters did not restore the checked plan");
            panel.x.Text="300"; panel.a.Text="30"; panel.SaveCurrentParameters(parameterFile);
            if(saves!=2 || panel.pointTarget is not null || panel.movePoint.IsEnabled)
                throw new InvalidOperationException("Global settings must save even when the current shape is out of bounds, without enabling it");
        } finally { File.Delete(parameterFile); File.Delete(parameterFile + ".tmp"); }
        panel.x.Text = "250"; panel.a.Text = "50";
        panel.Refresh();
        if (panel.movePoint.IsEnabled || panel.execute.IsEnabled ||
            !panel.moveState.Text.Contains("软件范围") || !panel.moveState.Text.Contains("参数检查未通过"))
            throw new InvalidOperationException("Grey start button must report the live 250/0/r50 target and firmware limits: " + panel.moveState.Text + " / " + panel.result.Text);
        if(!panel.actionSummary.Text.StartsWith("被拒绝")) throw new InvalidOperationException("Range failure must reject the full plan");
        panel.SetPointMoveState(false, new JointLimits(-30, 30, -60, 60), "未使能：请重新使能并完整回零。", true);
        if (!panel.moveState.Text.Contains("未使能") || !panel.moveState.Text.Contains("软件范围"))
            throw new InvalidOperationException("Both device and geometric blockers must remain visible");
        panel.x.Text = "340"; panel.a.Text = "5";
        panel.Refresh();
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟完整回零", true);
        if (!panel.movePoint.IsEnabled || !panel.execute.IsEnabled || panel.moveState.Text.Contains("起点超出"))
            throw new InvalidOperationException("Corrected circle must clear its old range blocker");
        panel.shape.SelectedIndex = 0;
        panel.l1.Text = "";
        if (panel.canvas.Children.Count != 0) throw new InvalidOperationException("Missing dimension accepted");
        panel.l1.Text = "180";
        panel.Refresh();
        if (!panel.result.Text.Contains("解 2")) throw new InvalidOperationException("Preview branches missing");
        panel.SetPointMoveState(true, DefaultTrialRange, "simulation only");
        if (!panel.movePoint.IsEnabled) throw new InvalidOperationException("Point gate did not open with valid mocked state");
        panel.SetPointMoveState(false, null, "单点执行已锁定：电流、回零、零偏和关节范围需实机确认。");
        if (panel.movePoint.IsEnabled) throw new InvalidOperationException("Point gate did not close");
        panel.x.Text = "400";
        panel.Refresh();
        if (!panel.result.Text.Contains("不可达") || panel.canvas.Children.Count != 0) throw new InvalidOperationException("Stale preview retained");
        panel.x.Text = "300"; panel.min1.Text = "-90"; panel.max1.Text = "90"; panel.min2.Text = "0"; panel.max2.Text = "160";
        panel.Refresh();
        if (!panel.result.Text.Contains("超出填写的范围") || !panel.result.Text.Contains("在填写的范围内")) throw new InvalidOperationException("Range checks missing");
        panel.x.Text = "NaN";
        if (panel.canvas.Children.Count != 0) throw new InvalidOperationException("Invalid target drawn");
        foreach (var field in new[] { panel.min1, panel.max1, panel.min2, panel.max2 }) field.Clear();
        panel.min1.Text="-90"; panel.max1.Text="90"; panel.min2.Text="-160"; panel.max2.Text="160";
        panel.x.Text = "220"; panel.shape.SelectedIndex = 3;
        panel.Refresh();
        if (!panel.export.IsEnabled || panel.trajectory is null) throw new InvalidOperationException("Circle preview missing");
        panel.cursor.Value = panel.cursor.Maximum;
        if (!panel.sampleInfo.Text.Contains($"时间 {panel.timing!.Duration:F2}/{panel.timing.Duration:F2}"))
            throw new InvalidOperationException("Trajectory cursor not updated");
        panel.TogglePlayback(); panel.AdvancePlayback(panel.timing.Duration / 2);
        if (!panel.playbackTimer.IsEnabled || panel.cursor.Value != panel.timing.Duration / 2)
            throw new InvalidOperationException("Playback did not advance");
        panel.PausePlayback(); double paused = panel.cursor.Value; panel.AdvancePlayback(panel.timing.Duration);
        if (panel.cursor.Value != paused) throw new InvalidOperationException("Paused preview advanced");
        panel.TogglePlayback(); panel.AdvancePlayback(panel.timing.Duration + 10);
        if (panel.playbackTimer.IsEnabled || panel.cursor.Value != panel.timing.Duration)
            throw new InvalidOperationException("Playback did not finish");
        panel.TogglePlayback(); panel.v1.Text = "0";
        if (panel.playbackTimer.IsEnabled || panel.timing is not null || panel.export.IsEnabled || panel.play.IsEnabled)
            throw new InvalidOperationException("Invalid timing retained playback/export");
        panel.v1.Text = "5";
        panel.x.Text = "400";
        if (panel.export.IsEnabled || panel.cursor.IsEnabled || panel.trajectory is not null || panel.canvas.Children.Count != 0)
            throw new InvalidOperationException("Stale trajectory/export retained");
        panel.x.Text = "220"; panel.min1.Text = "-1"; panel.max1.Text = "1"; panel.min2.Text = "-1"; panel.max2.Text = "1";
        panel.Refresh();
        if (panel.export.IsEnabled) throw new InvalidOperationException("Out-of-range trajectory exported");
        if (snapshotPath is not null)
        {
            panel.x.Text = "340"; panel.a.Text = "5"; panel.y.Text = "0";
            foreach (var field in new[] { panel.min1, panel.max1, panel.min2, panel.max2 }) field.Clear();
            panel.SetPointMoveState(false, new JointLimits(-30, 30, -60, 60), "此图未连接设备；实机使能后可人工确认伸直零位，无需强制回零。范围为临时软件限制。", true);
            panel.Refresh();
            panel.Width = 1050; panel.Height = 860;
            panel.Measure(new Size(panel.Width, panel.Height));
            panel.Arrange(new Rect(0, 0, panel.Width, panel.Height)); panel.UpdateLayout();
            var image = new RenderTargetBitmap(1050, 860, 96, 96, PixelFormats.Pbgra32);
            image.Render(panel);
            var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(image));
            using var stream = File.Create(snapshotPath); png.Save(stream);
            File.WriteAllText(System.IO.Path.ChangeExtension(snapshotPath, ".csv"), panel.timing!.ToCsv(), new UTF8Encoding(true));
            RenderElbowRuleSnapshots(Path.GetDirectoryName(snapshotPath)!);
            // Render every supported shape through the same controls and planner.
            panel.SetStartFeedback(null);
            panel.SetPointMoveState(false,null,"离线预览；未连接设备。",true);
            for(int selected=0;selected<=3;selected++) {
                panel.shape.SelectedIndex=selected;
                panel.x.Text="260"; panel.y.Text="0";
                panel.a.Text=selected==1?"290":"25";panel.b.Text="30";
                panel.min1.Text="-135";panel.max1.Text="135";panel.min2.Text="-155";panel.max2.Text="155";
                panel.Refresh();
                if(selected>0 && (panel.execution is null || !ReferenceEquals(panel.timing,panel.execution.Timing)))
                    throw new InvalidOperationException("Shape preview does not share execution timing");
                panel.Measure(new Size(panel.Width,panel.Height));panel.Arrange(new Rect(0,0,panel.Width,panel.Height));panel.UpdateLayout();
                var shapeImage=new RenderTargetBitmap(1050,860,96,96,PixelFormats.Pbgra32);shapeImage.Render(panel);
                var shapePng=new PngBitmapEncoder();shapePng.Frames.Add(BitmapFrame.Create(shapeImage));
                using var shapeStream=File.Create(Path.Combine(Path.GetDirectoryName(snapshotPath)!,new[]{"point","line","rectangle","circle"}[selected]+".preview.png"));shapePng.Save(shapeStream);
                if(selected==2) {
                    panel.FitView(true);panel.UpdateLayout();
                    var detailImage=new RenderTargetBitmap(1050,860,96,96,PixelFormats.Pbgra32);detailImage.Render(panel);
                    var detailPng=new PngBitmapEncoder();detailPng.Frames.Add(BitmapFrame.Create(detailImage));
                    using var detailStream=File.Create(Path.Combine(Path.GetDirectoryName(snapshotPath)!,"rectangle.detail.png"));detailPng.Save(detailStream);
                    panel.FitView(false);
                }
            }
        }
    }
}
