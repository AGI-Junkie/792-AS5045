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

namespace TwoLinkCommissioning;

public sealed partial class XYPreviewPanel : UserControl
{
    private readonly List<Control> editControls = new();
    private TelemetrySample? startFeedback;
    private JointLimits? planningLimits;
    private bool updating;
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
        finally { updating=false; ShapeFields(); Refresh(); }
    }
    private void ParameterFile(bool save)
    {
        operationError=null; UpdatePointButton();
        try {
            if(save) {
                if(pointTarget is null) throw new ArgumentException("请先修正参数错误，再保存参数。");
                var dialog=new Microsoft.Win32.SaveFileDialog { Title="保存轨迹参数", Filter="参数文件 (*.json)|*.json", FileName="two-link-parameters.json" };
                if(dialog.ShowDialog()==true) File.WriteAllText(dialog.FileName,SaveParameters(),new UTF8Encoding(false));
            } else {
                var dialog=new Microsoft.Win32.OpenFileDialog { Title="载入轨迹参数（不会执行运动）", Filter="参数文件 (*.json)|*.json" };
                if(dialog.ShowDialog()==true) {
                    if(new FileInfo(dialog.FileName).Length>32768) throw new ArgumentException("参数文件过大。");
                    RestoreParameters(File.ReadAllText(dialog.FileName));
                }
            }
        } catch(Exception ex) when(ex is IOException or UnauthorizedAccessException or JsonException or ArgumentException) { operationError="参数文件操作失败："+ex.Message; UpdatePointButton(); }
    }
    private bool detailView = true;
    public PlanarArm Model => new(Number(l1), Number(l2));
    public string SettingsDescription => $"shape={shape.SelectedIndex}; branch={branch.SelectedIndex}; L1={l1.Text}; L2={l2.Text}; x={x.Text}; y={y.Text}; a={a.Text}; b={b.Text}; feed={feed.Text}; v1={v1.Text}; v2={v2.Text}; a1={acc1.Text}; a2={acc2.Text}; min1={min1.Text}; max1={max1.Text}; min2={min2.Text}; max2={max2.Text}";
    public void SetStartFeedback(TelemetrySample? sample) {
        bool changed=startFeedback!=sample; startFeedback = sample; UpdatePointButton();
        if(changed) RedrawViewport();
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
            shape.SelectedIndex=3; branch.SelectedIndex=0; l1.Text="180"; l2.Text="176.38"; x.Text="340"; y.Text="0"; a.Text="5"; b.Text="10";
            feed.Text="20"; v1.Text="5"; v2.Text="3"; acc1.Text="10"; acc2.Text="6";
            foreach(var c in new[]{min1,max1,min2,max2}) c.Clear();
        } finally { updating=false; ShapeFields(); Refresh(); }
    }
    private readonly TextBox l1 = Field("180"), l2 = Field("176.38"), x = Field("340"), y = Field("0");
    private readonly TextBox min1 = Field(""), max1 = Field(""), min2 = Field(""), max2 = Field("");
    private readonly ComboBox branch = new() { Width = 132, Margin = new Thickness(4), ItemsSource = new[] { "q2 ≥ 0 分支", "q2 ≤ 0 分支" }, SelectedIndex = 0 };
    private readonly ComboBox shape = new() { Width = 94, Margin = new Thickness(4), ItemsSource = new[] { "单点", "直线", "矩形", "圆" }, SelectedIndex = 3 };
    private readonly TextBox a = Field("5"), b = Field("10");
    private readonly TextBlock xLabel = new(), yLabel = new(), aLabel = new(), bLabel = new();
    private readonly Slider cursor = new() { Minimum = 0, Maximum = 1, SmallChange = 0.1, LargeChange = 1, MinWidth = 180, Margin = new Thickness(4), IsEnabled = false };
    private readonly TextBox feed = Field("20"), v1 = Field("5"), v2 = Field("3"), acc1 = Field("10"), acc2 = Field("6");
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
    private bool pointMoveAllowed;
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
        advanced.Children.Add(WorkbenchTheme.Text("可收紧规划范围；不能放宽固件的软件范围。",12,WorkbenchTheme.Muted));
        var ranges = new WrapPanel(); AddField(ranges,"q1 最小",min1); AddField(ranges,"最大 / °",max1); AddField(ranges,"q2 最小",min2); AddField(ranges,"最大 / °",max2); advanced.Children.Add(ranges);
        var speeds = new WrapPanel(); AddField(speeds,"末端 mm/s",feed); AddField(speeds,"q1 °/s",v1); AddField(speeds,"q2 °/s",v2); AddField(speeds,"q1 °/s²",acc1); AddField(speeds,"q2 °/s²",acc2); advanced.Children.Add(speeds);
        advanced.Children.Add(WorkbenchTheme.Text("执行时另受低速上限约束。+X 为伸直方向，+Y 向上；两轴逆时针为正。",12,WorkbenchTheme.Muted));
        controls.Children.Add(new Expander { Header = "机构尺寸 / 范围 / 速度", Content = advanced, Margin = new Thickness(3,8,3,3) });
        var details = new StackPanel(); details.Children.Add(executionInfo); details.Children.Add(moveState); details.Children.Add(result); details.Children.Add(notice);
        controls.Children.Add(new Expander { Header = "查看规划检查详情", Content = details, Margin = new Thickness(3,8,3,3) });
        var files=new WrapPanel(); var save=new Button { Content="保存参数" }; var load=new Button { Content="载入参数" };
        save.Click+=(_,_)=>ParameterFile(true); load.Click+=(_,_)=>ParameterFile(false); files.Children.Add(save); files.Children.Add(load); controls.Children.Add(files);
        editControls.Add(save); editControls.Add(load);
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
        foreach (var field in new[] { l1, l2, x, y, a, b, min1, max1, min2, max2, feed, v1, v2, acc1, acc2 }) field.TextChanged += (_, _) => Refresh();
        branch.SelectionChanged += (_, _) => Refresh();
        shape.SelectionChanged += (_, _) => { ShapeFields(); Refresh(); };
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
        var limits = new JointLimits(Number(min1), Number(max1), Number(min2), Number(max2));
        if (limits.Min1 < -180 || limits.Max1 > 180 || limits.Min2 < -180 || limits.Max2 > 180 ||
            limits.Min1 >= limits.Max1 || limits.Min2 >= limits.Max2)
            throw new ArgumentException("两轴范围须完整填写，位于 -180°～180° 内，且最小值小于最大值。");
        return limits;
    }
    private void Refresh()
    {
        if (updating) return;
        cameraReady = false; autoFit = true; operationError=null;
        pointTarget = null; movePoint.IsEnabled = false; execution = null; execute.IsEnabled = false; executionInfo.Clear();
        PausePlayback(); trajectory = null; timing = null;
        export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = false;
        canvas.Children.Clear(); result.Clear(); sampleInfo.Clear(); notice.Text = "";
        try
        {
            var model = new PlanarArm(Number(l1), Number(l2));
            double tx = Number(x), ty = Number(y);
            JointLimits? limits = Limits(); planningLimits = limits;
            if (shape.SelectedIndex > 0)
            {
                trajectory = DrawingTrajectory.Create(model, (DrawingShape)(shape.SelectedIndex - 1), tx, ty,
                    Number(a), shape.SelectedIndex == 3 ? 0 : Number(b), Math.Max(0, branch.SelectedIndex), limits);
                timing = new TimedTrajectory(trajectory, new PlanningRates(Number(feed), Number(v1), Number(v2), Number(acc1), Number(acc2)));
                pointTarget = timing.At(0).Pose; RefreshExecution(); UpdatePointButton();
                notice.Text = limits is null ? "关节范围未测定；机械范围未检查。CSV 为规划数据，不能直接驱动机械臂。" :
                    "已检查全部采样点的关节范围；未验证机械碰撞、线缆干涉或从当前位置到起点的路径。";
                result.Foreground = Brushes.DarkSlateGray;
                result.Text = $"几何检查通过：{trajectory.Samples.Length} 点；路径长度 {timing.Length:F2} mm；预计 {timing.Duration:F2} 秒。\n" +
                    $"q1：{trajectory.Samples.Min(s => s.Pose.Q1):F2}～{trajectory.Samples.Max(s => s.Pose.Q1):F2}°；" +
                    $"q2：{trajectory.Samples.Min(s => s.Pose.Q2):F2}～{trajectory.Samples.Max(s => s.Pose.Q2):F2}°。\n" +
                    "此处显示离线规划；实际执行另生成低速分段轨迹，执行时间见上方。";
                if (trajectory.NearSingularity) result.Text += "\n提示：部分姿态接近奇异位形，微小 XY 变化可能引起较大关节变化。";
                cursor.Maximum = timing.Duration; cursor.Value = 0;
                export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = true; DrawSample();
                return;
            }
            var poses = model.Inverse(tx, ty);
            var selected = poses[Math.Min(branch.SelectedIndex < 0 ? 0 : branch.SelectedIndex, poses.Length - 1)];
            pointTarget = selected; UpdatePointButton();
            notice.Text = limits is null ? "两轴左转＝逆时针已确认；此处机械范围未填写。实际单点移动另受固件标定锁和固件范围限制。" :
                "这里的范围检查只针对目标姿态；尚未验证机械碰撞或线缆干涉。单点执行以固件标定范围为准。";
            result.Foreground = Brushes.DarkSlateGray;
            result.Text = $"目标 ({tx:F2}, {ty:F2}) mm；几何半径 {model.InnerRadius:F2}～{model.OuterRadius:F2} mm\n" +
                string.Join("\n", poses.Select((p, i) => $"解 {i + 1}：q1={p.Q1:F2}°，q2={p.Q2:F2}°；" +
                    (limits is null ? "机械范围未检查" : limits.Value.Contains(p) ? "在填写的范围内" : "超出填写的范围")));
            if (Math.Abs(Math.Sin(selected.Q2 * Math.PI / 180)) < 1e-6)
                result.Text += "\n当前是伸直或完全折叠的奇异位形；不能据此认为连续轨迹可执行。";
            Draw(model, selected, tx, ty, limits is null || limits.Value.Contains(selected));
        }
        catch (ArgumentException ex) { InvalidPlan(ex.Message); }
    }
    private void InvalidPlan(string reason)
    {
        PausePlayback(); trajectory = null; timing = null;
        export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = false;
        canvas.Children.Clear(); sampleInfo.Clear(); result.Foreground = Brushes.Firebrick; result.Text = reason;
        pointTarget = null; movePoint.IsEnabled = false;
        execution = null; execute.IsEnabled = false; executionInfo.Clear();
        UpdatePointButton();
    }
    public void SetPointMoveState(bool allowed, JointLimits? limits, string reason, bool supportsTrajectory = false)
    {
        bool changed = firmwareLimits != limits;
        pointMoveAllowed = allowed; firmwareLimits = limits; trajectorySupported = supportsTrajectory; deviceMoveReason = reason;
        if (changed) RefreshExecution();
        UpdatePointButton();
    }
    private void RefreshExecution()
    {
        execution = null; executionInfo.Clear();
        if (timing is null || firmwareLimits is not { } range) return;
        try {
            execution = new ExecutionTrajectory(timing, range);
            executionInfo.Text = $"低速执行计划 · {execution.Nodes.Length} 点 · {execution.Duration:F2} 秒 · 每轴 ≤600 微步/秒";
        }
        catch (ArgumentException ex) { executionInfo.Text = "图形执行未就绪：" + ex.Message; }
    }
    private void UpdatePointButton()
    {
        movePoint.Content = shape.SelectedIndex == 0 ? "低速移到此 XY 点" : "低速移到轨迹起点";
        movePoint.IsEnabled = pointMoveAllowed && pointTarget is { } p && firmwareLimits is { } range && range.Contains(PtpProtocol.Quantize(p)) && (planningLimits is not { } local || local.Contains(p));
        bool atStart = execution is not null && startFeedback is { Referenced: true, Inputs: 0 } &&
            startFeedback.Steps1 == ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose.Q1) && startFeedback.Steps2 == ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose.Q2);
        execute.IsEnabled = pointMoveAllowed && trajectorySupported && execution is not null && atStart;
        moveState.Text = deviceMoveReason;
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
        actionProgress.Visibility=busy?Visibility.Visible:Visibility.Collapsed;
        actionProgress.IsIndeterminate=indeterminate; actionProgress.Value=percent;
        UpdateActionStatus();
    }
    private void UpdateActionStatus()
    {
        string text; int severity; // 0 ready, 1 waiting, 2 rejected; text also conveys the state without color.
        if(deviceStatusFault || deviceStatusBusy) { text=(deviceStatusFault?"被拒绝 · ":"等待中 · ")+(deviceStatusMessage??deviceMoveReason); severity=deviceStatusFault?2:1; }
        else if(operationError is not null) { text="操作失败 · "+operationError; severity=2; }
        else if(pointTarget is null) { text="被拒绝 · "+result.Text; severity=2; }
        else if((firmwareLimits is {} bounds && !bounds.Contains(PtpProtocol.Quantize(pointTarget.Value))) ||
            (planningLimits is {} local && !local.Contains(pointTarget.Value))) { text="被拒绝 · 目标超出范围，调整图形参数"; severity=2; }
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
            Draw(plan.Path.Model, sample.Pose, sample.Target.X, sample.Target.Y, true);
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
    public static void SelfTest(string? snapshotPath = null)
    {
        var panel = new XYPreviewPanel();
        panel.ViewportSelfTest();
        string saved=panel.SaveParameters(); panel.x.Text="400"; panel.RestoreParameters(saved);
        if(panel.x.Text!="340" || panel.pointTarget is null) throw new InvalidOperationException("Parameter roundtrip failed");
        try { panel.RestoreParameters("{\"Schema\":99}"); throw new InvalidOperationException("Invalid parameters accepted"); } catch(ArgumentException) { }
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟已回零状态；临时软件范围未标定", true);
        if (panel.execute.IsEnabled || !panel.movePoint.IsEnabled || panel.execution is null)
            throw new InvalidOperationException("Default trial circle is not executable in delivery bounds");
        if(!panel.actionSummary.Text.StartsWith("等待中")) throw new InvalidOperationException("Before-start status must be waiting");
        panel.SetStartFeedback(new TelemetrySample(0,1,1,true,2,ExecutionTrajectory.Counts(0,panel.execution.Nodes[0].Pose.Q1),ExecutionTrajectory.Counts(1,panel.execution.Nodes[0].Pose.Q2),0,0,0,0));
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
        panel.x.Text = "250"; panel.a.Text = "50";
        if (panel.movePoint.IsEnabled || panel.execute.IsEnabled ||
            !panel.moveState.Text.Contains("起点超出") || !panel.moveState.Text.Contains("65.34"))
            throw new InvalidOperationException("Grey start button must report the live 250/0/r50 target and firmware limits");
        if(!panel.actionSummary.Text.StartsWith("被拒绝") || panel.actionSummary.Text.Contains("65.34")) throw new InvalidOperationException("Range detail must not replace concise rejection");
        panel.SetPointMoveState(false, new JointLimits(-30, 30, -60, 60), "未使能：请重新使能并完整回零。", true);
        if (!panel.moveState.Text.Contains("未使能") || !panel.moveState.Text.Contains("起点超出"))
            throw new InvalidOperationException("Both device and geometric blockers must remain visible");
        panel.x.Text = "340"; panel.a.Text = "5";
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟完整回零", true);
        if (!panel.movePoint.IsEnabled || !panel.execute.IsEnabled || panel.moveState.Text.Contains("起点超出"))
            throw new InvalidOperationException("Corrected circle must clear its old range blocker");
        panel.shape.SelectedIndex = 0;
        panel.l1.Text = "";
        if (panel.canvas.Children.Count != 0) throw new InvalidOperationException("Missing dimension accepted");
        panel.l1.Text = "180";
        if (!panel.result.Text.Contains("解 2")) throw new InvalidOperationException("Preview branches missing");
        panel.SetPointMoveState(true, new JointLimits(-90, 90, -160, 160), "simulation only");
        if (!panel.movePoint.IsEnabled) throw new InvalidOperationException("Point gate did not open with valid mocked state");
        panel.SetPointMoveState(false, null, "单点执行已锁定：电流、回零、零偏和关节范围需实机确认。");
        if (panel.movePoint.IsEnabled) throw new InvalidOperationException("Point gate did not close");
        panel.x.Text = "400";
        if (!panel.result.Text.Contains("不可达") || panel.canvas.Children.Count != 0) throw new InvalidOperationException("Stale preview retained");
        panel.x.Text = "300"; panel.min1.Text = "-90"; panel.max1.Text = "90"; panel.min2.Text = "0"; panel.max2.Text = "160";
        if (!panel.result.Text.Contains("超出填写的范围") || !panel.result.Text.Contains("在填写的范围内")) throw new InvalidOperationException("Range checks missing");
        panel.x.Text = "NaN";
        if (panel.canvas.Children.Count != 0) throw new InvalidOperationException("Invalid target drawn");
        foreach (var field in new[] { panel.min1, panel.max1, panel.min2, panel.max2 }) field.Clear();
        panel.x.Text = "220"; panel.shape.SelectedIndex = 3;
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
        if (panel.export.IsEnabled) throw new InvalidOperationException("Out-of-range trajectory exported");
        if (snapshotPath is not null)
        {
            panel.x.Text = "340"; panel.a.Text = "5"; panel.y.Text = "0";
            foreach (var field in new[] { panel.min1, panel.max1, panel.min2, panel.max2 }) field.Clear();
            panel.SetPointMoveState(false, new JointLimits(-30, 30, -60, 60), "此图未连接设备；实机使能后可人工确认伸直零位，无需强制回零。范围为临时软件限制。", true);
            panel.Width = 1050; panel.Height = 860;
            panel.Measure(new Size(panel.Width, panel.Height));
            panel.Arrange(new Rect(0, 0, panel.Width, panel.Height)); panel.UpdateLayout();
            var image = new RenderTargetBitmap(1050, 860, 96, 96, PixelFormats.Pbgra32);
            image.Render(panel);
            var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(image));
            using var stream = File.Create(snapshotPath); png.Save(stream);
            File.WriteAllText(System.IO.Path.ChangeExtension(snapshotPath, ".csv"), panel.timing!.ToCsv(), new UTF8Encoding(true));
        }
    }
}
