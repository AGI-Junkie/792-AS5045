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
    // The tip speed is an output of the joint rates, not an input: the planner's tip-speed term gets its ceiling
    // (1000 mm/s, the validation limit) and never binds. `feed` only displays the theoretical maximum.
    private const double NoTipLimit = 1000;
    public PlanningRates RequestedRates => new(NoTipLimit,Number(v1),Number(v2),Number(acc1),Number(acc2));
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
    private bool commandView;
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
            WriteParameters(path);
            ParametersSaved?.Invoke();
        } catch(Exception ex) when(ex is IOException or UnauthorizedAccessException or ArgumentException) { operationError="保存参数失败："+ex.Message; UpdatePointButton(); }
    }
    private void WriteParameters(string path)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path + ".tmp", SaveParameters(), new UTF8Encoding(false));
        File.Move(path + ".tmp", path, true);
    }
    // The click only fires while a validated plan matches the fields, so what is written is exactly what is uploaded.
    // Best effort and silent: a full disk must not block a motion the operator already approved.
    private void RememberUploaded()
    {
        try { WriteParameters(ParametersPath); } catch(Exception ex) when(ex is IOException or UnauthorizedAccessException) { }
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
    private bool detailView = true;
    private JointLimits? overlayLimits = DefaultTrialRange;
    public PlanarArm Model => new(Number(l1), Number(l2));
    public string SettingsDescription => $"shape={shape.SelectedIndex}; branch={branch.SelectedIndex}; L1={l1.Text}; L2={l2.Text}; x={x.Text}; y={y.Text}; a={a.Text}; b={b.Text}; feed={feed.Text}; v1={v1.Text}; v2={v2.Text}; a1={acc1.Text}; a2={acc2.Text}; min1={min1.Text}; max1={max1.Text}; min2={min2.Text}; max2={max2.Text}";
    public void SetStartFeedback(TelemetrySample? sample) {
        bool changed=startFeedback!=sample; startFeedback = sample; UpdatePointButton();
        commandView=sample is {Referenced:true} && (commandView || deviceStatusBusy);
        if(changed) RequestViewportRedraw();
    }
    public FrameworkElement ParameterContent { get; private set; } = null!;
    public void SetEditingAllowed(bool allowed)
    {
        foreach(var c in editControls) c.IsEnabled = allowed;
        if (allowed) { export.IsEnabled = play.IsEnabled = rewind.IsEnabled = cursor.IsEnabled = timing is not null; }
        else PausePlayback();
    }
    // a and b mean radius, end point or size depending on the shape, so a value typed for one shape is meaningless for
    // another (a circle's radius 5 became a line ending at X=5). Each shape keeps its own a/b; a shape never visited
    // starts from a small example next to the start point that fits the default joint range.
    private readonly string[][] shapeDims = [["",""],["",""],["",""],["",""]];
    private int shownShape = 3;
    private static string Mm(double value) => Math.Round(value, 2).ToString("0.##", CultureInfo.InvariantCulture);
    private string[] ExampleDims(int index)
    {
        double px = double.TryParse(x.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double vx) && double.IsFinite(vx) ? vx : 340;
        double py = double.TryParse(y.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double vy) && double.IsFinite(vy) ? vy : 0;
        return index switch { 1 => [Mm(px + 10), Mm(py + 10)], 2 => ["10", "10"], 3 => ["5", "10"], _ => ["5", "10"] };
    }
    private void SwitchShape()
    {
        if (!updating && shownShape >= 0) shapeDims[shownShape] = [a.Text, b.Text];
        shownShape = shape.SelectedIndex;
        if (updating || shownShape < 0) return;
        var dims = shapeDims[shownShape][0] == "" ? ExampleDims(shownShape) : shapeDims[shownShape];
        updating = true;   // the caller runs one ParametersChanged for the whole switch
        try { a.Text = dims[0]; b.Text = dims[1]; } finally { updating = false; }
    }
    private void LoadExample()
    {
        updating = true;
        try {
            branch.SelectedIndex=0; x.Text="340"; y.Text="0";
            var dims = ExampleDims(shape.SelectedIndex); a.Text=dims[0]; b.Text=dims[1];
        } finally { updating=false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }
    private readonly TextBox l1 = Field("180"), l2 = Field("176.38"), x = Field("340"), y = Field("0");
    private readonly TextBox min1 = Field(""), max1 = Field(""), min2 = Field(""), max2 = Field("");
    private readonly ComboBox branch = new() { Width = 132, Margin = new Thickness(4), ItemsSource = new[] { "肘角 ≥ 0 分支", "肘角 ≤ 0 分支" }, SelectedIndex = 0 };
    private readonly ComboBox shape = new() { Width = 94, Margin = new Thickness(4), ItemsSource = new[] { "单点", "直线", "矩形", "圆" }, SelectedIndex = 3 };
    private readonly TextBox a = Field("5"), b = Field("10");
    private readonly TextBlock xLabel = new(), yLabel = new(), aLabel = new(), bLabel = new();
    private readonly Slider cursor = new() { Minimum = 0, Maximum = 1, SmallChange = 0.1, LargeChange = 1, MinWidth = 180, Margin = new Thickness(4), IsEnabled = false };
    private readonly TextBox feed = new() { IsReadOnly = true, Width = 76, Margin = new Thickness(4), Padding = new Thickness(4), Background = WorkbenchTheme.Background, Foreground = WorkbenchTheme.Muted },
        v1 = Field("5"), v2 = Field("3"), acc1 = Field("10"), acc2 = Field("6");
    private readonly TextBlock speedState = new() { MinHeight=64, TextWrapping=TextWrapping.Wrap, FontSize=11, Margin=new Thickness(4), Foreground=WorkbenchTheme.Muted };
    private readonly Button play = new() { Content = "▶ 预览", Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly Button rewind = new() { Content = "↺ 重播", Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly DispatcherTimer playbackTimer = new() { Interval = TimeSpan.FromMilliseconds(33) };
    private readonly Stopwatch playbackClock = new();
    private double playbackFrom;
    private bool advancing;
    private readonly Button export = new() { Content = "导出规划 CSV", Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly TextBox sampleInfo = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, BorderThickness = new Thickness(0), Background = Brushes.Transparent, Margin = new Thickness(4) };
    private DrawingTrajectory? trajectory;
    private TimedTrajectory? timing;
    private readonly Button movePoint = new() { Content = "低速移到此 XY 点", Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
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
    private readonly TextBlock rangeState = new() { MinHeight=32, FontSize=11, TextWrapping=TextWrapping.Wrap, Margin=new Thickness(4), Foreground=WorkbenchTheme.Muted };
    private bool pointMoveAllowed;
    private bool deviceLimitsConfirmed;
    private string deviceMoveReason = "等待连接并建立人工或开关坐标参考。";
    public event Action<JointPose>? PointMoveRequested;
    public event Action<ExecutionTrajectory>? TrajectoryRequested;   // upload only: the device does not move
    public event Action? ExecuteRequested;                            // start the plan the device already holds
    private readonly Button upload = new() { Content = "上传图形", Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly Button run = new() { Content = "执行图形", Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private ExecutionTrajectory? uploadedPlan;   // complete plan waiting on the device for 执行图形
    private readonly TextBox executionInfo = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, Background = Brushes.Transparent, BorderThickness = new Thickness(0), Margin = new Thickness(4) };
    private ExecutionTrajectory? execution;
    private bool trajectorySupported;
    private readonly Canvas canvas = new() { Background = Brushes.White, ClipToBounds = true, MinHeight=260 };
    private readonly TextBox result = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, BorderThickness = new Thickness(0),
        Background = Brushes.Transparent, Margin = new Thickness(4), MinHeight = 75 };
    private readonly TextBlock notice = new() { TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4), Foreground = Brushes.DarkSlateGray };

    private Border aRow = null!, bRow = null!;
    public XYPreviewPanel()
    {
        Background = Brushes.White;
        var shell = new Grid();
        shell.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        shell.RowDefinitions.Add(new RowDefinition());
        shell.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        Content = shell;
        var title = new DockPanel { Margin = new Thickness(6,2,6,2) };
        var sample = new Button { Content = "载入示例", ToolTip = "载入当前图形的小示例（起点约 340, 0 mm，在默认关节范围内）；只改图形参数。" };
        sample.Click += (_,_) => LoadExample(); DockPanel.SetDock(sample,Dock.Right); title.Children.Add(sample);
        editControls.Add(sample); title.Children.Add(BuildViewportToolbar()); shell.Children.Add(WorkbenchTheme.Edge(title));
        var main = new Grid(); Grid.SetRow(main,1); shell.Children.Add(main);
        // Property grid (left column of the window): "name | value | unit" rows in titled groups.
        var controls = new StackPanel();
        var scroll = new ScrollViewer { Content = controls, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled };
        Border Prop(string label, TextBox field, string unit, string? name = null, string? tip = null) {
            System.Windows.Automation.AutomationProperties.SetName(field, name ?? label + " / " + unit);
            var row = WorkbenchTheme.Row(WorkbenchTheme.Label(label), field, unit); if (tip is not null) row.ToolTip = tip; return row;
        }
        controls.Children.Add(WorkbenchTheme.GroupHeader("图形与几何"));
        controls.Children.Add(WorkbenchTheme.Row(WorkbenchTheme.Label("形状"), shape));
        controls.Children.Add(WorkbenchTheme.Row(WorkbenchTheme.Label("逆解分支"), branch));
        controls.Children.Add(WorkbenchTheme.Row(xLabel, x)); controls.Children.Add(WorkbenchTheme.Row(yLabel, y));
        controls.Children.Add(aRow = WorkbenchTheme.Row(aLabel, a)); controls.Children.Add(bRow = WorkbenchTheme.Row(bLabel, b));
        controls.Children.Add(WorkbenchTheme.GroupHeader("机构尺寸"));
        controls.Children.Add(Prop("L1", l1, "mm")); controls.Children.Add(Prop("L2", l2, "mm"));
        var rangeHelpIcon = HelpIcon(RangeHelpText());
        rangeHelp.Add(rangeHelpIcon);
        controls.Children.Add(WorkbenchTheme.GroupHeader("关节范围", rangeHelpIcon));
        foreach(var (field,name,value) in new[] { (min1,"q1 最小",-30), (max1,"q1 最大",30), (min2,"q2 最小",-60), (max2,"q2 最大",60) }) {
            controls.Children.Add(Prop(name, field, "°", name));
            field.ToolTip=$"{name} / °；留空使用 {value}°，填写后使用你的值。";
        }
        controls.Children.Add(new TextBlock { Text="默认 q1 ∈ [−30°, +30°] · q2 ∈ [−60°, +60°]\n留空用默认值 · 修改后红色区域实时更新", Foreground=WorkbenchTheme.Muted, FontSize=11, Margin=new Thickness(10,4,10,0) });
        rangeState.Margin = new Thickness(10,4,10,4); controls.Children.Add(rangeState);
        controls.Children.Add(WorkbenchTheme.GroupHeader("运动参数（全局）"));
        string Name(int axis, int config, int plan, string text) => $"{text}（≤{Cap(axis,config,plan):0.##}）";
        controls.Children.Add(Prop("电机1 速度", v1, "°/s", Name(0,DeviceContract.Arm1RateMax,DeviceContract.PlanRateMax,"电机1 °/s")));
        controls.Children.Add(Prop("电机2 速度", v2, "°/s", Name(1,DeviceContract.Arm2RateMax,DeviceContract.PlanRateMax,"电机2 (q2−q1) °/s"), "电机2 转的是 q2−q1"));
        controls.Children.Add(Prop("电机1 加速度", acc1, "°/s²", Name(0,DeviceContract.Arm1AccelMax,DeviceContract.PlanAccelMax,"电机1 °/s²")));
        controls.Children.Add(Prop("电机2 加速度", acc2, "°/s²", Name(1,DeviceContract.Arm2AccelMax,DeviceContract.PlanAccelMax,"电机2 (q2−q1) °/s²"), "电机2 转的是 q2−q1"));
        feed.ToolTip = "由杆长和两个电机速度算出的末端速度上限：机械臂伸直、两轴都以最高速度同向转动时。只读；实际绘图更慢。";
        controls.Children.Add(Prop("末端最大", feed, "mm/s", "末端最大 mm/s（只读）"));
        controls.Children.Add(new TextBlock { Text="全局：杆长用于全部图形、命令 XY 与记录；关节速度/加速度用于绘图、定位、点动和回零。\n" +
            $"上限：电机1 ≤{Cap(0,DeviceContract.Arm1RateMax,DeviceContract.PlanRateMax):0.##} °/s、≤{Cap(0,DeviceContract.Arm1AccelMax,DeviceContract.PlanAccelMax):0.##} °/s²；" +
            $"电机2 (q2−q1) ≤{Cap(1,DeviceContract.Arm2RateMax,DeviceContract.PlanRateMax):0.##} °/s、≤{Cap(1,DeviceContract.Arm2AccelMax,DeviceContract.PlanAccelMax):0.##} °/s²。", Foreground=WorkbenchTheme.Muted, FontSize=11, TextWrapping=TextWrapping.Wrap, Margin=new Thickness(10,4,10,0) });
        speedState.Margin = new Thickness(10,2,10,4); controls.Children.Add(speedState);
        var details = new StackPanel(); details.Children.Add(executionInfo); details.Children.Add(moveState); details.Children.Add(result); details.Children.Add(notice);
        controls.Children.Add(new Expander { Header = "查看规划检查详情", Content = details, Margin = new Thickness(10,6,10,3) });
        var preview = new Button { Content = "检查并预览", MinWidth = 120, Background = Brushes.White, Foreground = WorkbenchTheme.Blue, BorderBrush = WorkbenchTheme.Blue }; preview.Click += (_,_) => Refresh();
        var save = new Button { Content = "保存" }; save.Click += (_,_) => SaveCurrentParameters(ParametersPath);
        export.Click += (_,_) => Export(); editControls.Add(preview); editControls.Add(export); editControls.Add(save);
        var buttons = new WrapPanel { Margin = new Thickness(8,4,8,4) }; buttons.Children.Add(preview); buttons.Children.Add(save); buttons.Children.Add(export);
        // The action buttons stay visible under the scrolling groups.
        var pinned = WorkbenchTheme.Edge(buttons, edge: new Thickness(0,1,0,0)); DockPanel.SetDock(pinned, Dock.Bottom);
        ParameterContent = new DockPanel { Children = { pinned, scroll } };
        var visual = new Grid(); main.Children.Add(visual);
        visual.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        visual.RowDefinitions.Add(new RowDefinition());
        visual.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        visual.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        Grid.SetRow(canvas,1); visual.Children.Add(canvas);
        var playback = new DockPanel { Margin = new Thickness(6,2,6,2) };
        playback.Children.Add(play); playback.Children.Add(rewind);
        playback.Children.Add(cursor); var transport = WorkbenchTheme.Edge(playback, edge: new Thickness(0,1,0,0)); Grid.SetRow(transport,2); visual.Children.Add(transport);
        sampleInfo.FontSize = 11; Grid.SetRow(sampleInfo,3); visual.Children.Add(sampleInfo);
        InitializeViewport();
        var actions = new StackPanel { Margin = new Thickness(6,2,6,4) }; var actionBar = WorkbenchTheme.Edge(actions, edge: new Thickness(0,1,0,0)); Grid.SetRow(actionBar,2); shell.Children.Add(actionBar);
        var motion = new WrapPanel(); motion.Children.Add(movePoint); motion.Children.Add(upload); motion.Children.Add(run);
        actionStatus.Child=actionSummary; motion.Children.Add(actionStatus); actions.Children.Add(motion); actions.Children.Add(actionProgress);
        executionInfo.FontSize=12; executionInfo.Padding=new Thickness(0);
        foreach(var b in new[] { movePoint, upload, run }) { WorkbenchTheme.Primary(b); ToolTipService.SetShowOnDisabled(b, true); }
        movePoint.Click += (_,_) => { if(movePoint.IsEnabled && pointTarget is {} p) PointMoveRequested?.Invoke(PtpProtocol.Quantize(p)); };
        upload.Click += (_,_) => { if(upload.IsEnabled && execution is {} plan) { RememberUploaded(); TrajectoryRequested?.Invoke(plan); } };
        run.Click += (_,_) => { if(run.IsEnabled) ExecuteRequested?.Invoke(); };
        moveState.FontSize = 12;
        foreach(var control in new Control[] { l1,l2,x,y,a,b,min1,max1,min2,max2,v1,v2,acc1,acc2,shape,branch,play,rewind,cursor }) editControls.Add(control);
        foreach (var field in new[] { l1, l2, x, y, a, b, min1, max1, min2, max2, v1, v2, acc1, acc2 }) field.TextChanged += (_, _) => ParametersChanged();
        foreach (var help in rangeHelp) editControls.Add(help);
        branch.SelectionChanged += (_, _) => ParametersChanged();
        shape.SelectionChanged += (_, _) => { SwitchShape(); ShapeFields(); ParametersChanged(); };
        cursor.ValueChanged += (_, _) => { if (!advancing) { PausePlayback(); commandView=false; } DrawSample(); };
        canvas.SizeChanged += (_, _) => RedrawViewport();
        play.Click += (_, _) => TogglePlayback(); rewind.Click += (_, _) => { PausePlayback(); commandView=false; cursor.Value = 0; RedrawViewport(); };
        playbackTimer.Tick += (_, _) => AdvancePlayback(playbackFrom + playbackClock.Elapsed.TotalSeconds);
        Unloaded += (_, _) => PausePlayback();
        ShapeFields();
        Refresh();
    }

    // Drawing/positioning cap of one axis in degrees: the smaller of the configurable and the plan limit (microsteps).
    private static double Cap(int axis, int configMax, int planMax) => Math.Min(configMax, planMax) / MotionProfile.StepsPerDegree(axis);
    private static TextBox Field(string text) => new() { Text = text, Width = 76, Margin = new Thickness(4), Padding = new Thickness(4) };
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
            "· 因此 q2 范围不直接限制肘：肘可达 [q2 最小−q1 最大, q2 最大−q1 最小]，固件不检查。\n" +
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
        string tipMax = "—";   // also shown while the link lengths or speeds are invalid
        try { var r=RequestedRates; var p=MotionProfile.From(r); var d=p.DrawingRates(r.Feed);
            SetText(speedState,$"实际生效：电机1/电机2 {d.V1:F2}/{d.V2:F2} °/s，加速度 {d.A1:F2}/{d.A2:F2} °/s²。填得比上限大会被压到上限。");
            try { tipMax = Model.MaxTipSpeed(d.V1, d.V2).ToString("0.#", CultureInfo.InvariantCulture); } catch(ArgumentException) { }
        } catch(ArgumentException ex) { SetText(speedState,ex.Message); }
        if (feed.Text != tipMax) feed.Text = tipMax;
        var tip=RangeHelpText();
        foreach(var help in rangeHelp) if(!Equals(help.ToolTip,tip)) { help.ToolTip=tip; System.Windows.Automation.AutomationProperties.SetHelpText(help,tip); }
        SetText(rangeState, RequestedLimits is not { } requested ? "范围无效：须包含 0°、上下限有序，最多两位小数。"
            : firmwareLimits is null ? "红区与规划使用上方范围；设备确认后才能运动。"
            : requested == firmwareLimits && deviceLimitsConfirmed ? "设备已确认：红区、规划、执行范围一致。"
            : "设备范围尚未同步；运动已锁定。\n" + deviceMoveReason + "\n设备当前：" + Envelope(firmwareLimits));
    }
    private void RefreshOverlayLimits()
    {
        try { overlayLimits = PreviewLimits(); } catch(ArgumentException) { overlayLimits = null; }
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
        aRow.Visibility = a.Visibility; bRow.Visibility = b.Visibility;
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
        commandView = false;
        cameraReady = false; autoFit = true; operationError=null;
        RefreshOverlayLimits(); RefreshRangeHelp();
        pointTarget = null; movePoint.IsEnabled = false; execution = null; upload.IsEnabled = false; executionInfo.Clear();
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
        execution = null; upload.IsEnabled = false; executionInfo.Clear();
        UpdatePointButton();
        RequestViewportRedraw();
    }
    public void SetPointMoveState(bool allowed, JointLimits? limits, string reason, bool supportsTrajectory = false, bool limitsConfirmed = true, ExecutionTrajectory? uploaded = null)
    {
        pointMoveAllowed = allowed; firmwareLimits = limits; trajectorySupported = supportsTrajectory; deviceMoveReason = reason;
        deviceLimitsConfirmed = limitsConfirmed; uploadedPlan = uploaded;
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
    // WPF compares reference values by reference, so re-assigning an equal string still raises a change: the status
    // box repaints and an open ToolTip closes and reopens on every device poll. Write only what actually differs.
    private static void SetText(TextBlock block, string text) { if (block.Text != text) block.Text = text; }
    private static void SetTip(FrameworkElement element, string tip) { if (!Equals(element.ToolTip, tip)) element.ToolTip = tip; }
    private void UpdatePointButton()
    {
        movePoint.Content = shape.SelectedIndex == 0 ? "低速移到此 XY 点" : "低速移到轨迹起点";
        bool limitsMatch = deviceLimitsConfirmed && RequestedLimits is { } requested && requested == firmwareLimits && requested == planningLimits;
        // While the device holds an uploaded plan it refuses every other move; only 执行图形 or 总停止 leaves that state.
        movePoint.IsEnabled = pointMoveAllowed && limitsMatch && uploadedPlan is null && pointTarget is { } p && firmwareLimits is { } range && range.Contains(PtpProtocol.Quantize(p)) && (planningLimits is not { } local || local.Contains(p)) &&
            (shape.SelectedIndex == 0 || (execution is not null && trajectorySupported));
        bool atStart = execution is not null && startFeedback is { Referenced: true, Inputs: 0 } &&
            startFeedback.Steps1 == ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose) && startFeedback.Steps2 == ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose);
        upload.IsEnabled = pointMoveAllowed && limitsMatch && trajectorySupported && execution is not null && atStart;
        // 执行图形 starts what the device holds, so it needs that exact plan to still be the one on screen.
        bool held = uploadedPlan is { } sent && execution is { } shown && sent.Nodes.SequenceEqual(shown.Nodes);
        run.IsEnabled = upload.IsEnabled && held;
        string move = deviceMoveReason;
        if (RequestedLimits is { } currentRange) move += "\n当前软件范围：" + Envelope(currentRange);
        if (firmwareLimits is not null && !limitsMatch) move += "\n设备尚未确认当前范围/速度，运动已锁定；预览仍按输入设置显示。";
        if (pointTarget is { } target && firmwareLimits is { } bounds) {
            target = PtpProtocol.Quantize(target);
            string label = bounds.Contains(target) ? "目标在固件范围内" : shape.SelectedIndex == 0 ? "目标超出固件范围，不能移动" : "轨迹起点超出固件范围，不能移动";
            move += FormattableString.Invariant($"\n{label}：q1={target.Q1:F2}°，q2={target.Q2:F2}°；固件允许 q1 [{bounds.Min1:F2}, {bounds.Max1:F2}]°、q2 [{bounds.Min2:F2}, {bounds.Max2:F2}]°。");
        } else if (pointTarget is null) move += "\n参数检查未通过：" + result.Text;
        if (execution is not null) move += held ? "\n图形已上传；手动落笔后点“执行图形”。" : atStart ? "\n已到当前轨迹起点；点“上传图形”（上传时不运动），落笔后再点“执行图形”。" : "\n图形执行：先移到起点并等待到位反馈（需 v5 固件）。";
        if (uploadedPlan is not null && !held) move += "\n设备里已有另一份图形：重新上传，或先总停止。";
        if (execution is null && shape.SelectedIndex > 0) move += "\n" + (executionInfo.Text.Length > 0 ? executionInfo.Text : "先完成图形参数检查。");
        SetText(moveState, move);
        UpdateActionStatus();
        SetTip(upload, move + "\n流程：准备 → 抬笔移到起点 → 上传图形 → 落笔 → 执行图形 → 保存结果。");
        SetTip(run, move);
        SetTip(movePoint, move);
    }
    public void SetDeviceStatus(string? message, bool fault, bool busy, double percent, bool indeterminate)
    {
        bool changed=deviceStatusBusy!=busy;
        if(busy) commandView=true;
        deviceStatusMessage=message; deviceStatusFault=fault; deviceStatusBusy=busy;
        // Unknown progress uses the status text; the theme's looping animation can starve input/heartbeat.
        actionProgress.Visibility=busy && !indeterminate?Visibility.Visible:Visibility.Collapsed;
        actionProgress.IsIndeterminate=false; actionProgress.Value=percent;
        UpdateActionStatus();
        if(changed) RequestViewportRedraw();
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
        else if(run.IsEnabled) { text="可执行 · 图形已上传，请落笔后点“执行图形”"; severity=0; }
        else if(uploadedPlan is not null) { text="等待中 · 设备里的图形与当前参数不一致，请重新上传"; severity=1; }
        else if(upload.IsEnabled) { text="可上传 · 已到起点；上传时不运动"; severity=0; }
        else if(shape.SelectedIndex==0 && movePoint.IsEnabled) { text="可移动 · 目标检查通过"; severity=0; }
        else if(!trajectorySupported) { text="等待中 · 当前固件不支持图形执行"; severity=1; }
        else { text="等待中 · 请先抬笔，低速移到起点"; severity=1; }
        SetText(actionSummary,text);
        actionSummary.Foreground=StatusInk[severity];
        actionStatus.Background=StatusFill[severity];
        SetTip(actionStatus,text+"\n"+moveState.Text+"\n完整参数见左侧“查看规划检查详情”。");
        if(System.Windows.Automation.AutomationProperties.GetName(actionStatus)!=text) System.Windows.Automation.AutomationProperties.SetName(actionStatus,text);
    }
    // One shared brush per severity: a fresh brush on every call is a new value every time.
    private static readonly SolidColorBrush[] StatusInk = new[] { "#166534", "#854D0E", "#991B1B" }.Select(WorkbenchTheme.Brush).ToArray(),
        StatusFill = new[] { "#DCFCE7", "#FEF3C7", "#FEE2E2" }.Select(WorkbenchTheme.Brush).ToArray();
    public void PausePlayback()
    { playbackTimer.Stop(); playbackClock.Stop(); play.Content = "▶ 预览"; }
    private void TogglePlayback()
    {
        if (timing is null) return;
        commandView=false;
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
            var pose=FollowCommand?startFeedback!.Command:sample.Pose;
            var tip=plan.Path.Model.Forward(pose).Tip;
            sampleInfo.Text = (FollowCommand?"设备命令姿态（非实测） · ":$"离线预览 · 时间 {sample.Time:F2}/{plan.Duration:F2} s · ") +
                $"q1={pose.Q1:F2}° · q2={pose.Q2:F2}° · X={tip.X:F2}，Y={tip.Y:F2} mm";
            sampleInfo.ToolTip=FollowCommand?"按设备命令步数和连杆长度估算；arm2 / 末端没有实测反馈。":
                $"仅屏幕预览，不代表执行进度。路程 {sample.Distance:F2} mm；末端速度 {sample.Speed:F2} mm/s。";
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
            !panel.play.IsEnabled || panel.movePoint.IsEnabled || panel.upload.IsEnabled)
            throw new InvalidOperationException("连接不同范围的设备改变了预览，或提前开放运动");
        panel.SetStartFeedback(new TelemetrySample(0,1,1,true,2,ExecutionTrajectory.Counts(0,panel.execution!.Nodes[0].Pose),
            ExecutionTrajectory.Counts(1,panel.execution.Nodes[0].Pose),0,0,0,0));
        if (panel.upload.IsEnabled) throw new InvalidOperationException("到位反馈绕过范围同步");
        panel.SetPointMoveState(true,requested,"已确认统一范围",true);
        if (!ReferenceEquals(plan,panel.timing) || !panel.movePoint.IsEnabled || !panel.upload.IsEnabled ||
            !panel.rangeState.Text.Contains("范围一致"))
            throw new InvalidOperationException("设备确认统一范围后未开放同一条轨迹");
        panel.SetPointMoveState(true,requested,"仍有范围请求等待完整确认",true,false);
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled || panel.rangeState.Text.Contains("范围一致"))
            throw new InvalidOperationException("报告范围相同不能绕过在途配置确认");
        panel.SetPointMoveState(false,null,"离线",false);
        if (!ReferenceEquals(plan,panel.timing) || panel.movePoint.IsEnabled || panel.upload.IsEnabled || !panel.play.IsEnabled)
            throw new InvalidOperationException("断开设备改变了预览或仍允许运动");
        panel.max1.Text="135.001";
        if (panel.RequestedLimits is not null || panel.overlayLimits is not null)
            throw new InvalidOperationException("预览接受了固件无法表达的范围精度");
        panel.max1.Text="135.01";
        if (panel.RequestedLimits?.Max1 != 135.01) throw new InvalidOperationException("0.01度精度不一致");
        panel.min1.Text="1";
        if (panel.RequestedLimits is not null) throw new InvalidOperationException("范围必须包含伸直参考零位");
        panel.min1.Text="-135"; panel.v1.Text="2"; panel.l1.Text="181";
        // The tip speed is a read-only result: (181*1.9969 + 176.38*(1.9969+2.9965)) deg/s * pi/180 = 21.7 mm/s
        if(!panel.feed.IsReadOnly || panel.feed.Text!="21.7") throw new InvalidOperationException("末端速度必须是由杆长和电机速度算出的只读理论最大值：" + panel.feed.Text);
        panel.l1.Text="";
        if(panel.feed.Text!="—") throw new InvalidOperationException("杆长无效时末端最大速度必须显示 —");
        panel.l1.Text="181";
        var beforeExample=panel.RequestedLimits; var beforeRates=panel.RequestedRates;
        panel.LoadExample();
        if (panel.RequestedLimits != beforeExample || panel.RequestedRates != beforeRates || panel.Model.L1!=181)
            throw new InvalidOperationException("载入示例覆盖了全局杆长、速度或范围");
    }
    private static void ShapeDefaultsSelfTest()
    {
        var panel = new XYPreviewPanel();
        void Show(int index) { panel.shape.SelectedIndex = index; panel.Refresh(); }
        // A radius typed for the circle used to become a line ending at X=5: rejected before the first look.
        panel.a.Text = "7"; Show(1);
        if (panel.a.Text == "7" || panel.execution is null || panel.aLabel.Text != "终点 X / mm")
            throw new InvalidOperationException("切换到直线沿用了圆的半径，或直线默认值不可执行：" + panel.result.Text);
        panel.a.Text = "348"; panel.b.Text = "6"; Show(2);
        if (panel.a.Text != "10" || panel.b.Text != "10" || panel.execution is null)
            throw new InvalidOperationException("矩形默认宽高不可执行：" + panel.result.Text);
        Show(1);
        if (panel.a.Text != "348" || panel.b.Text != "6") throw new InvalidOperationException("切回直线丢了它自己的终点");
        Show(3);
        if (panel.a.Text != "7") throw new InvalidOperationException("切回圆丢了它自己的半径");
        panel.RestoreParameters(panel.SaveParameters());
        Show(0);
        if (panel.pointTarget is null || panel.execution is not null) throw new InvalidOperationException("单点没有目标，或误生成了执行计划");
        // The example button loads the current shape's own example, and each one passes the full check at the default range.
        for (int index = 0; index < 4; index++)
        {
            panel.shape.SelectedIndex = index; panel.x.Text = "300"; panel.LoadExample();
            if (panel.x.Text != "340" || panel.pointTarget is null || (index > 0 && (panel.execution is null || panel.RequestedLimits != DefaultTrialRange)))
                throw new InvalidOperationException($"图形 {index} 的示例不可执行：" + panel.result.Text);
        }
    }
    public static void SelfTest(string? snapshotPath = null)
    {
        ReachabilityMask.SelfTest();
        UnifiedLimitsSelfTest();
        ShapeDefaultsSelfTest();
        var panel = new XYPreviewPanel();
        panel.ViewportSelfTest();
        ArmMotionPreviewSelfTest();
        if(!panel.detailView) throw new InvalidOperationException("中间视图默认必须适合完整机械臂运动");
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
            panel.timing is not null || panel.movePoint.IsEnabled || panel.upload.IsEnabled)
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
        if (panel.upload.IsEnabled || !panel.movePoint.IsEnabled || panel.execution is null)
            throw new InvalidOperationException("Default trial circle is not executable in delivery bounds");
        if(!panel.actionSummary.Text.StartsWith("等待中")) throw new InvalidOperationException("Before-start status must be waiting");
        panel.SetStartFeedback(new TelemetrySample(0,1,1,true,2,ExecutionTrajectory.Counts(0,panel.execution.Nodes[0].Pose),ExecutionTrajectory.Counts(1,panel.execution.Nodes[0].Pose),0,0,0,0));
        if (!panel.upload.IsEnabled) throw new InvalidOperationException("At-start feedback did not enable execution");
        if(!panel.actionSummary.Text.StartsWith("可上传") || panel.run.IsEnabled) throw new InvalidOperationException("At-start conclusion not prominent, or 执行图形 opened before anything was uploaded");
        panel.SetDeviceStatus("模拟故障",true,false,0,false);
        if(!panel.actionSummary.Text.StartsWith("被拒绝")) throw new InvalidOperationException("Fault must override ready status");
        panel.SetDeviceStatus("上传 1 / 10 点",false,true,10,false);
        if(!panel.actionSummary.Text.Contains("上传") || panel.actionProgress.Visibility!=Visibility.Visible) throw new InvalidOperationException("Upload status missing");
        panel.SetDeviceStatus(null,false,false,0,false);
        if(!panel.actionSummary.Text.StartsWith("可上传") || panel.actionProgress.Visibility!=Visibility.Collapsed) throw new InvalidOperationException("Stale busy status retained");
        var fresh=panel.startFeedback; panel.SetStartFeedback(null);
        if(panel.upload.IsEnabled || panel.actionSummary.Text.StartsWith("可上传")) throw new InvalidOperationException("Missing feedback retained ready conclusion");
        panel.SetStartFeedback(fresh);
        // 上传图形 and 执行图形 are separate: only a plan the device already holds (and that still matches the screen) can be run.
        var held = panel.execution!;
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟设备已就绪", true, true, held);
        if (!panel.run.IsEnabled || !panel.upload.IsEnabled || panel.movePoint.IsEnabled || !panel.actionSummary.Text.StartsWith("可执行"))
            throw new InvalidOperationException("A held plan must open 执行图形, allow re-upload and close single moves");
        int runs = 0; panel.ExecuteRequested += () => runs++; panel.run.RaiseEvent(new RoutedEventArgs(Button.ClickEvent));
        if (runs != 1) throw new InvalidOperationException("执行图形 did not request the run");
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟设备已就绪", true, true, null);
        if (panel.run.IsEnabled || !panel.upload.IsEnabled || !panel.movePoint.IsEnabled)
            throw new InvalidOperationException("Without a held plan 执行图形 must stay closed and moves open");
        var other = new ExecutionTrajectory(new TimedTrajectory(panel.timing!.Path, panel.RequestedRates with { V1 = 2, V2 = 1.5 }), panel.planningLimits!.Value);
        if (other.Nodes.SequenceEqual(held.Nodes)) throw new InvalidOperationException("Self-test needs a genuinely different plan");
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟设备已就绪", true, true, other);
        if (panel.run.IsEnabled || !panel.actionSummary.Text.StartsWith("等待中"))
            throw new InvalidOperationException("A different plan on the device must not be started by 执行图形");
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟设备已就绪", true, true, null);
        panel.x.Text = "335";
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled || panel.timing is not null || panel.play.IsEnabled)
            throw new InvalidOperationException("Typing must invalidate the applied plan without rebuilding it before Check/Save");
        panel.Refresh(); panel.x.Text = "340"; panel.Refresh();
        panel.x.Text = "300"; panel.a.Text = "30";
        panel.Refresh();
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled || panel.result.Text.Contains("检查通过") ||
            !panel.result.Text.Contains("允许 q1"))
            throw new InvalidOperationException("300/0/r30 must fail the full preview check before any start movement");
        panel.SetEditingAllowed(true); panel.SetStartFeedback(fresh);
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled || panel.play.IsEnabled)
            throw new InvalidOperationException("Feedback/editing update bypassed the rejected full plan");
        panel.x.Text = "340"; panel.a.Text = "5";
        panel.Refresh();
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 32), "模拟范围收紧", true);
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled || panel.timing is null || !panel.play.IsEnabled ||
            !panel.actionSummary.Text.Contains("设备尚未确认"))
            throw new InvalidOperationException("设备范围不一致必须锁定运动、保留原预览");
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟范围恢复", true);
        if (!panel.movePoint.IsEnabled || !panel.upload.IsEnabled || !panel.result.Text.Contains("整条轨迹检查通过"))
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
            if (panel.SaveParameters() != applied || !panel.movePoint.IsEnabled || !panel.upload.IsEnabled)
                throw new InvalidOperationException("Saved parameters did not restore the checked plan");
            panel.x.Text="300"; panel.a.Text="30"; panel.SaveCurrentParameters(parameterFile);
            if(saves!=2 || panel.pointTarget is not null || panel.movePoint.IsEnabled)
                throw new InvalidOperationException("Global settings must save even when the current shape is out of bounds, without enabling it");
        } finally { File.Delete(parameterFile); File.Delete(parameterFile + ".tmp"); }
        panel.x.Text = "250"; panel.a.Text = "50";
        panel.Refresh();
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled ||
            !panel.moveState.Text.Contains("软件范围") || !panel.moveState.Text.Contains("参数检查未通过"))
            throw new InvalidOperationException("Grey start button must report the live 250/0/r50 target and firmware limits: " + panel.moveState.Text + " / " + panel.result.Text);
        if(!panel.actionSummary.Text.StartsWith("被拒绝")) throw new InvalidOperationException("Range failure must reject the full plan");
        panel.SetPointMoveState(false, new JointLimits(-30, 30, -60, 60), "未使能：请重新使能并完整回零。", true);
        if (!panel.moveState.Text.Contains("未使能") || !panel.moveState.Text.Contains("软件范围"))
            throw new InvalidOperationException("Both device and geometric blockers must remain visible");
        panel.x.Text = "340"; panel.a.Text = "5";
        panel.Refresh();
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟完整回零", true);
        if (!panel.movePoint.IsEnabled || !panel.upload.IsEnabled || panel.moveState.Text.Contains("起点超出"))
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
                if(panel.timing is {} motion) panel.cursor.Value=motion.Duration*0.35;
                panel.Measure(new Size(panel.Width,panel.Height));panel.Arrange(new Rect(0,0,panel.Width,panel.Height));panel.UpdateLayout();
                var shapeImage=new RenderTargetBitmap(1050,860,96,96,PixelFormats.Pbgra32);shapeImage.Render(panel);
                var shapePng=new PngBitmapEncoder();shapePng.Frames.Add(BitmapFrame.Create(shapeImage));
                using var shapeStream=File.Create(Path.Combine(Path.GetDirectoryName(snapshotPath)!,new[]{"point","line","rectangle","circle"}[selected]+".preview.png"));shapePng.Save(shapeStream);
                if(selected>0) {
                    panel.FitView(true);panel.UpdateLayout();
                    var detailImage=new RenderTargetBitmap(1050,860,96,96,PixelFormats.Pbgra32);detailImage.Render(panel);
                    var detailPng=new PngBitmapEncoder();detailPng.Frames.Add(BitmapFrame.Create(detailImage));
                    using var detailStream=File.Create(Path.Combine(Path.GetDirectoryName(snapshotPath)!,new[]{"point","line","rectangle","circle"}[selected]+".detail.png"));detailPng.Save(detailStream);
                    panel.FitView(false);
                }
            }
        }
    }
}
