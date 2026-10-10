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
    // The solid arm follows the device whenever it has a valid pose. It follows the plan instead only while the operator
    // scrubs or plays the offline preview (offlineSim), or when the device has no usable pose (deviceNote says why).
    private bool offlineSim;
    private string? deviceNote;
    private JointLimits? planningLimits;
    private bool updating;
    private bool parametersDirty;
    // New fields are appended, so a file saved by an older version is a prefix: 15 fields before the elbow range, 17 before
    // the extra shape fields (c, rotation, scale, custom contour).
    private const int LegacyFieldCount = 15, ElbowFieldCount = 17, CustomFieldIndex = 20;
    private TextBox[] ParameterFields => [l1,l2,x,y,a,b,min1,max1,min2,max2,feed,v1,v2,acc1,acc2,elbowMin,elbowMax,c,rotation,scale,custom];
    private sealed record SavedParameters(int Schema, int Shape, int Branch, string[] Fields, int Direction = 0, FreehandPoint[]? Freehand = null);
    private string SaveParameters()
    {
        if (ProcessingFreehand || freehandNeedsProcessing) throw Lang.Error("fh.processing");
        return JsonSerializer.Serialize(new SavedParameters(1,shape.SelectedIndex,branch.SelectedIndex,ParameterFields.Select(f=>f.Text).ToArray(),direction.SelectedIndex,freehandPoints.ToArray()),new JsonSerializerOptions { WriteIndented=true });
    }
    private void RestoreParameters(string json)
    {
        var p=JsonSerializer.Deserialize<SavedParameters>(json);
        if(p is null || p.Schema!=1 || p.Shape < 0 || p.Shape >= shape.Items.Count || p.Branch is <0 or >1 || p.Direction is <0 or >1 || p.Fields is null ||
            p.Fields.Length!=LegacyFieldCount && p.Fields.Length!=ElbowFieldCount && p.Fields.Length!=ParameterFields.Length ||
            p.Fields.Select((v,i)=>v is null || v.Length>(i==CustomFieldIndex?4096:32)).Any(bad=>bad)) throw Lang.Error("fault.paramFormat");
        if (p.Freehand is { } points) FreehandTrajectory.Validate(points);
        updating=true;
        try { shape.SelectedIndex=p.Shape; branch.SelectedIndex=p.Branch; direction.SelectedIndex=p.Direction;
            // A field the file does not have keeps its default: the elbow range, c and the custom contour empty, rotation 0, scale 1.
            for(int i=0;i<ParameterFields.Length;i++) ParameterFields[i].Text=i<p.Fields.Length?p.Fields[i]:ParameterFields[i]==rotation?"0":ParameterFields[i]==scale?"1":"";
            RestoreFreehand(p.Freehand); freehandMode = p.Shape == FreehandIndex; ClampRateFields(); }
        finally { updating=false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }
    private static string ParametersPath => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "parameters.json");
    public event Action? ParametersSaved;
    private async void SaveCurrentParameters(string path)
    {
        if (RecordingFreehand || ProcessingFreehand) return;
        Refresh();
        int version = freehandWorkVersion;
        if (ProcessingFreehand && freehandRequest is { } request) {
            if (!await request.ConfigureAwait(false)) return;
            try { await Dispatcher.InvokeAsync(() => SaveCheckedParameters(path, version)); }
            catch (OperationCanceledException) { }
            catch (InvalidOperationException) when (Dispatcher.HasShutdownStarted) { }
            return;
        }
        SaveCheckedParameters(path, version);
    }
    private void SaveCheckedParameters(string path, int version)
    {
        if (version != freehandWorkVersion || ProcessingFreehand || freehandNeedsProcessing) return;
        try {
            _=Model; _=PreviewLimits(); _=MotionProfile.From(RequestedRates);
            if (shape.SelectedIndex != FreehandIndex) { _=Number(x); _=Number(y); }
            if(shape.SelectedIndex>0) _=ShapeOptionsFromFields(); 
            WriteParameters(path);
            ParametersSaved?.Invoke();
        } catch(Exception ex) when(ex is IOException or UnauthorizedAccessException or ArgumentException) { operationError=() => Lang.T("fault.paramSave")+ex.Message; UpdatePointButton(); }
    }
    private void WriteParameters(string path)
    {
        FreehandTrajectory.Validate(freehandPoints);
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path + ".tmp", SaveParameters(), new UTF8Encoding(false));
        File.Move(path + ".tmp", path, true);
        RememberSaved();
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
            if (new FileInfo(path).Length > 512 * 1024) throw Lang.Error("fault.paramTooBig");
            RestoreParameters(File.ReadAllText(path));
            RememberSaved();
        } catch(Exception ex) when(ex is IOException or UnauthorizedAccessException or JsonException or ArgumentException) { operationError=() => Lang.T("fault.paramRestore")+ex.Message; UpdatePointButton(); }
    }
    private bool detailView;   // false = whole arm and reach circle; the default and only view the operator gets
    private JointLimits? overlayLimits = DefaultTrialRange;
    public PlanarArm Model => new(Number(l1), Number(l2));
    public string SettingsDescription => $"shape={shape.SelectedIndex}; branch={branch.SelectedIndex}; L1={l1.Text}; L2={l2.Text}; x={x.Text}; y={y.Text}; a={a.Text}; b={b.Text}; feed={feed.Text}; v1={v1.Text}; v2={v2.Text}; a1={acc1.Text}; a2={acc2.Text}; min1={min1.Text}; max1={max1.Text}; min2={min2.Text}; max2={max2.Text}; emin={elbowMin.Text}; emax={elbowMax.Text}; c={c.Text}; rot={rotation.Text}; scale={scale.Text}; dir={(direction.SelectedIndex==1?"cw":"ccw")}; custom_chars={custom.Text.Length}";
    // `whyNot` explains a missing device pose on the canvas (homing, no reference, waiting for feedback); null when offline.
    public void SetStartFeedback(TelemetrySample? sample, string? whyNot = null) {
        bool changed=startFeedback!=sample || deviceNote!=whyNot;
        // A newly established or replaced reference (manual zero, homing done) returns the view to the device.
        if(sample is {Referenced:true} && (startFeedback is not {Referenced:true} || startFeedback.Epoch!=sample.Epoch)) offlineSim=false;
        startFeedback = sample; deviceNote = whyNot; UpdatePointButton();
        if(changed) RequestViewportRedraw();
    }
    public FrameworkElement ParameterContent { get; private set; } = null!;
    public void SetEditingAllowed(bool allowed)
    {
        bool changed = editingAllowed != allowed;
        editingAllowed = allowed;
        if (!allowed && changed) { CancelFreehandStroke(); CancelFreehandWork(); }
        foreach(var c in editControls) c.IsEnabled = allowed;
        if (allowed) { export.IsEnabled = play.IsEnabled = rewind.IsEnabled = cursor.IsEnabled = timing is not null; }
        else PausePlayback();
        UpdateFreehandTools();
        if (allowed && changed && freehandNeedsProcessing) QueueFreehandWork();
    }
    // a and b mean radius, end point or size depending on the shape, so a value typed for one shape is meaningless for
    // another (a circle's radius 5 became a line ending at X=5). Each shape keeps its own a/b; a shape never visited
    // starts from a small example next to the start point that fits the default joint range.
    private readonly string[][] shapeDims = Enumerable.Range(0, Enum.GetValues<DrawingShape>().Length + 1).Select(_ => new[] { "", "", "" }).ToArray();
    private int shownShape = 3;
    private static string Mm(double value) => Math.Round(value, 2).ToString("0.##", CultureInfo.InvariantCulture);
    private string[] ExampleDims(int index)
    {
        double px = double.TryParse(x.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double vx) && double.IsFinite(vx) ? vx : 340;
        double py = double.TryParse(y.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double vy) && double.IsFinite(vy) ? vy : 0;
        // Every example sits next to (340, 0) and passes the full check at the default joint range.
        return index switch {
            1 => [Mm(px + 10), Mm(py + 10), ""], 2 => ["10", "10", ""], 3 => ["5", "10", ""],
            4 => ["12", "6", ""],                 // ellipse: semi-axes
            5 => ["10", "0", "180"],              // arc: radius, start angle, sweep
            6 => ["12", "8", "3"],                // rounded rectangle: width, height, corner radius
            7 => ["10", "5", "2"],                // figure 8: half width, half height, y frequency
            8 => ["8", "3", ""],                  // polygon: circumradius, sides (a triangle)
            9 => ["2", "10", "2"],                // spiral: start radius, end radius, turns
            _ => ["5", "10", ""] };
    }
    private const string ExampleContour = "M 0 0 C 0 -8 8 -8 8 0 L 14 0 L 14 6";   // SVG: y points down, so the arch bulges up
    // Which of a, b, c a shape reads; the others keep whatever another shape left in them.
    private static (bool A, bool B, bool C) ShapeUses(int index) => index switch {
        1 or 2 or 4 or 7 or 8 or 9 => (true, true, index is 7 or 9),   // line, rectangle, ellipse, polygon: a,b | figure 8, spiral: a,b,c
        3 => (true, false, false), 5 or 6 => (true, true, true), _ => (false, false, false) };
    private ShapeOptions ShapeOptionsFromFields()
    {
        int index = shape.SelectedIndex; var uses = ShapeUses(index);
        if (index == FreehandIndex) return new ShapeOptions { FreehandPoints = freehandPoints.ToArray(), FreehandFitLevel = FreehandFitLevel };
        _ = Number(x); _ = Number(y);
        if (uses.A) _ = Number(a);
        if (uses.B) _ = Number(b);
        return new ShapeOptions { C = uses.C ? Number(c) : 0, Rotation = Number(rotation), Scale = Number(scale),
            Clockwise = direction.SelectedIndex == 1, Text = index == 10 ? custom.Text : "" };
    }
    private void SwitchShape()
    {
        if (!updating && shownShape >= 0) shapeDims[shownShape] = [a.Text, b.Text, c.Text];
        shownShape = shape.SelectedIndex;
        if (updating || shownShape < 0) return;
        var dims = shapeDims[shownShape][0] == "" ? ExampleDims(shownShape) : shapeDims[shownShape];
        updating = true;   // the caller runs one ParametersChanged for the whole switch
        try { a.Text = dims[0]; b.Text = dims[1]; c.Text = dims[2]; if (shownShape == 10 && custom.Text.Trim() == "") custom.Text = ExampleContour; } finally { updating = false; }
    }
    private void LoadExample()
    {
        CancelFreehandStroke();
        updating = true;
        try {
            branch.SelectedIndex=0; x.Text="340"; y.Text="0";
            var dims = ExampleDims(shape.SelectedIndex); a.Text=dims[0]; b.Text=dims[1]; c.Text=dims[2];
            rotation.Text="0"; scale.Text="1"; direction.SelectedIndex=0;
            if (shape.SelectedIndex == 10) custom.Text = ExampleContour;
            if (shape.SelectedIndex == FreehandIndex) LoadFreehandExample();
        } finally { updating=false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }
    private readonly TextBox l1 = Field("180"), l2 = Field("176.38"), x = Field("340"), y = Field("0");
    private readonly TextBox min1 = Field(""), max1 = Field(""), min2 = Field(""), max2 = Field("");
    private readonly TextBox elbowMin = Field(""), elbowMax = Field("");   // q2 - q1: motor 2's own travel
    private readonly ComboBox branch = new() { Width = 132, Margin = new Thickness(4), SelectedIndex = 0 };
    private readonly ComboBox shape = new() { Width = 94, Margin = new Thickness(4), SelectedIndex = 3 };
    private readonly TextBox a = Field("5"), b = Field("10");
    private readonly TextBox c = Field(""), rotation = Field("0"), scale = Field("1");   // third size; rotation about (x, y) in degrees; uniform scale
    private readonly ComboBox direction = new() { Width = 132, Margin = new Thickness(4), SelectedIndex = 0 };
    private readonly TextBox custom = new() { AcceptsReturn = true, TextWrapping = TextWrapping.NoWrap, MinHeight = 64, MaxHeight = 150, FontFamily = WorkbenchTheme.Mono, FontSize = 11,
        VerticalScrollBarVisibility = ScrollBarVisibility.Auto, HorizontalScrollBarVisibility = ScrollBarVisibility.Auto, Margin = new Thickness(10,2,10,6), Padding = new Thickness(4) };
    private readonly TextBlock xLabel = new(), yLabel = new(), aLabel = new(), bLabel = new(), cLabel = new(), customLabel = new();
    private readonly Slider cursor = new() { Minimum = 0, Maximum = 1, SmallChange = 0.1, LargeChange = 1, MinWidth = 180, Margin = new Thickness(4), IsEnabled = false };
    private readonly TextBox feed = new() { IsReadOnly = true, Width = 76, Margin = new Thickness(4), Padding = new Thickness(4), Background = WorkbenchTheme.Background, Foreground = WorkbenchTheme.Muted },
        v1 = Field("5"), v2 = Field("3"), acc1 = Field("10"), acc2 = Field("6");
    private readonly Button play = new() { Content = Lang.T("btn.play"), Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly Button rewind = new() { Content = Lang.T("btn.rewind"), Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly DispatcherTimer playbackTimer = new() { Interval = TimeSpan.FromMilliseconds(33) };
    private readonly Stopwatch playbackClock = new();
    private double playbackFrom;
    private bool advancing;
    private readonly Button export = new() { Content = Lang.T("btn.export"), Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly TextBox sampleInfo = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, BorderThickness = new Thickness(0), Background = Brushes.Transparent, Margin = new Thickness(4) };
    private DrawingTrajectory? trajectory;
    private TimedTrajectory? timing;
    private readonly Button movePoint = new() { Content = Lang.T("btn.movePoint"), Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly TextBlock moveState = new() { Text = Lang.T("move.default"), Margin = new Thickness(4), TextWrapping = TextWrapping.Wrap };
    private readonly TextBlock actionSummary = new() { FontWeight=FontWeights.SemiBold, MaxWidth=390, TextTrimming=TextTrimming.CharacterEllipsis, VerticalAlignment=VerticalAlignment.Center };
    private readonly TextBlock actionDetails = new() { MaxWidth = 600, TextWrapping = TextWrapping.Wrap };
    private JointRangeFailure? rangeFailure;
    private Expander checksSection = null!;
    private readonly Border actionStatus = new() { CornerRadius=new CornerRadius(6), Padding=new Thickness(10,8,10,8), Margin=new Thickness(8,4,4,4), VerticalAlignment=VerticalAlignment.Center };
    private readonly ProgressBar actionProgress = new() { Height=4, Minimum=0, Maximum=100, Foreground=WorkbenchTheme.Teal, Background=WorkbenchTheme.Line, Margin=new Thickness(4,6,4,0), Visibility=Visibility.Collapsed };
    private string? deviceStatusMessage;
    private Func<string>? operationError;
    private bool deviceStatusFault, deviceStatusBusy;
    private JointPose? pointTarget;
    private JointLimits? firmwareLimits;
    private ReachabilityMask? reachability;
    private bool overlayUnreachable = true;
    private readonly List<Button> rangeHelp = [];
    private bool pointMoveAllowed;
    private bool deviceLimitsConfirmed;
    private string deviceMoveReason = Lang.T("move.waitConnect");
    public event Action<JointPose>? PointMoveRequested;
    public event Action<ExecutionTrajectory>? TrajectoryRequested;   // upload only: the device does not move
    public event Action? ExecuteRequested;                            // start the plan the device already holds
    private readonly Button upload = new() { Content = Lang.T("btn.upload"), Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private readonly Button run = new() { Content = Lang.T("btn.run"), Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), IsEnabled = false };
    private ExecutionTrajectory? uploadedPlan;   // complete plan waiting on the device for 执行图形
    private bool uploadedRan;                    // ...and it has already been drawn: single moves are open again, 执行图形 repeats it
    private readonly TextBox executionInfo = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, Background = Brushes.Transparent, BorderThickness = new Thickness(0), Margin = new Thickness(4) };
    private ExecutionTrajectory? execution;
    private bool trajectorySupported;
    private readonly Canvas canvas = new() { Background = Brushes.White, ClipToBounds = true, MinHeight=260 };
    private readonly TextBox result = new() { IsReadOnly = true, TextWrapping = TextWrapping.Wrap, BorderThickness = new Thickness(0),
        Background = Brushes.Transparent, Margin = new Thickness(4), MinHeight = 75 };
    private readonly TextBlock notice = new() { TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4), Foreground = Brushes.DarkSlateGray };

    private Border aRow = null!, bRow = null!, cRow = null!, customRow = null!;
    private Button? sampleButton, previewButton, saveButton, speedHelp;
    internal void RefreshLanguage()
    {
        if (sampleButton is { } sample) { sample.Content = Lang.T("btn.sample"); sample.ToolTip = Lang.T("btn.sample.tip"); }
        if (previewButton is { } preview) preview.Content = Lang.T("btn.preview");
        if (saveButton is { } save) save.Content = Lang.T("btn.save");
        export.Content = Lang.T("btn.export");
        RefreshRecentLanguage();
        play.Content = playbackTimer.IsEnabled ? Lang.T("btn.playPause") : Lang.T("btn.play");
        rewind.Content = Lang.T("btn.rewind");
        run.Content = Lang.T("btn.run");
        movePoint.Content = shape.SelectedIndex == 0 ? Lang.T("btn.movePoint") : Lang.T("btn.moveToStart");
        feed.ToolTip = Lang.T("xy.feed.tip");
        foreach (var (field, name, value) in new[] { (min1, Lang.T("xy.q1min"), -30), (max1, Lang.T("xy.q1max"), 30), (min2, Lang.T("xy.q2min"), -60), (max2, Lang.T("xy.q2max"), 60),
                     (elbowMin, Lang.T("xy.elbowMin"), -(int)JointLimits.DefaultElbow), (elbowMax, Lang.T("xy.elbowMax"), (int)JointLimits.DefaultElbow) })
            field.ToolTip = Lang.T("xy.rangeField.tip", name, value.ToString());
        ShapeFields();
        foreach (var element in new FrameworkElement[] { result, notice, executionInfo }) Lang.RefreshText(element);
        foreach (var help in rangeHelp) System.Windows.Automation.AutomationProperties.SetName(help, Lang.T("xy.help.name"));
        if (speedHelp is { } speeds) System.Windows.Automation.AutomationProperties.SetName(speeds, Lang.T("xy.speedHelp.name"));
        RefreshLanguageViewport();
        RefreshRangeHelp();
        UpdatePointButton();
        RequestViewportRedraw();
    }
    public XYPreviewPanel()
    {
        foreach (var key in Enumerable.Range(0, Enum.GetValues<DrawingShape>().Length + 1).Select(i => "xy.shape" + i))
            shape.Items.Add(Lang.Bind(new ComboBoxItem(), ContentControl.ContentProperty, key));
        foreach (var key in new[] { "xy.dirCcw", "xy.dirCw" })
            direction.Items.Add(Lang.Bind(new ComboBoxItem(), ContentControl.ContentProperty, key));
        foreach (var key in new[] { "xy.branchPos", "xy.branchNeg" })
            branch.Items.Add(Lang.Bind(new ComboBoxItem(), ContentControl.ContentProperty, key));
        shape.SelectedIndex = 3; branch.SelectedIndex = 0;
        Background = Brushes.White;
        var shell = new Grid();
        shell.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        shell.RowDefinitions.Add(new RowDefinition());
        shell.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
        Content = shell;
        var title = new DockPanel { Margin = new Thickness(6,2,6,2) };
        var sample = new Button { Content = Lang.T("btn.sample"), ToolTip = Lang.T("btn.sample.tip") };
        sampleButton = sample;
        sample.Click += (_,_) => LoadExample(); DockPanel.SetDock(sample,Dock.Right); title.Children.Add(sample);
        editControls.Add(sample); title.Children.Add(BuildViewportToolbar()); shell.Children.Add(WorkbenchTheme.Edge(title));
        var main = new Grid(); Grid.SetRow(main,1); shell.Children.Add(main);
        // Property grid (left column of the window): "name | value | unit" rows in titled groups.
        var controls = new StackPanel();
        var scroll = new ScrollViewer { Content = controls, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled };
        Border Prop(string key, TextBox field, string unit, string? name = null, string? tip = null, Slider? slider = null) {
            Lang.Bind(field, System.Windows.Automation.AutomationProperties.NameProperty, name ?? key);
            var label = WorkbenchTheme.LocalLabel(key);
            Border row;
            if (slider is null) row = WorkbenchTheme.Row(label, field, unit);
            else { Lang.Bind(slider, System.Windows.Automation.AutomationProperties.NameProperty, name ?? key); editControls.Add(slider); row = WorkbenchTheme.SliderRow(label, slider, field, unit); }
            if (tip is not null) Lang.Bind(row, ToolTipProperty, tip);
            return row;
        }
        StackPanel Group(string title, UIElement? trailing = null, bool expanded = true) {
            var body = new StackPanel(); controls.Children.Add(WorkbenchTheme.Section(title, body, trailing, expanded)); return body;
        }
        var recent = Group("section.recent"); recent.Children.Add(BuildRecentRuns());
        var geometry = Group("section.geometry"); geometry.Children.Add(BuildLamps(ShapeGroup, false));
        geometry.Children.Add(WorkbenchTheme.Row(WorkbenchTheme.LocalLabel("xy.shape"), shape));
        geometry.Children.Add(WorkbenchTheme.Row(WorkbenchTheme.LocalLabel("xy.branch"), branch));
        geometry.Children.Add(xRow = WorkbenchTheme.Row(xLabel, x)); geometry.Children.Add(yRow = WorkbenchTheme.Row(yLabel, y));
        geometry.Children.Add(aRow = WorkbenchTheme.Row(aLabel, a)); geometry.Children.Add(bRow = WorkbenchTheme.Row(bLabel, b));
        geometry.Children.Add(cRow = WorkbenchTheme.Row(cLabel, c));
        customLabel.Margin = new Thickness(10,4,10,0); customLabel.Foreground = WorkbenchTheme.Muted; customLabel.TextWrapping = TextWrapping.Wrap;
        geometry.Children.Add(customRow = WorkbenchTheme.Edge(new StackPanel { Children = { customLabel, custom } }, Brushes.White, new Thickness(0,0,0,1)));
        var transform = Group("section.transform"); transform.Children.Add(BuildLamps(ShapeGroup, false));
        transformSection = (FrameworkElement)controls.Children[controls.Children.Count - 1];
        transform.Children.Add(Prop("xy.rotation", rotation, "°")); transform.Children.Add(Prop("xy.scale", scale, "×"));
        transform.Children.Add(WorkbenchTheme.Row(WorkbenchTheme.LocalLabel("xy.direction"), direction));
        var dimensions = Group("section.dimensions"); dimensions.Children.Add(BuildLamps(SizeGroup, false));
        dimensions.Children.Add(Prop("xy.l1", l1, "mm")); dimensions.Children.Add(Prop("xy.l2", l2, "mm"));
        var rangeHelpIcon = HelpIcon(RangeHelpText());
        rangeHelp.Add(rangeHelpIcon);
        var ranges = Group("section.limits", rangeHelpIcon); ranges.Children.Add(BuildLamps(RangeGroup, true));
        foreach(var (field,name,value) in new[] { (min1,"xy.q1min",-30), (max1,"xy.q1max",30), (min2,"xy.q2min",-60), (max2,"xy.q2max",60),
                     (elbowMin,"xy.elbowMin",-(int)JointLimits.DefaultElbow), (elbowMax,"xy.elbowMax",(int)JointLimits.DefaultElbow) }) {
            ranges.Children.Add(Prop(name, field, "°", name));
            field.ToolTip=Lang.T("xy.rangeField.tip", Lang.T(name), value.ToString());
        }
        speedHelp = HelpIcon(SpeedsHelpText(), "xy.speedHelp.name");
        var speeds = Group("section.speeds", speedHelp); speeds.Children.Add(BuildLamps(SpeedGroup, true));
        speeds.Children.Add(Prop("xy.motor1v", v1, "°/s", "xy.motor1v.name", slider: RateSlider(v1, 0, accel: false)));
        speeds.Children.Add(Prop("xy.motor2v", v2, "°/s", "xy.motor2v.name", "xy.motor2.tip", RateSlider(v2, 1, accel: false)));
        speeds.Children.Add(Prop("xy.motor1a", acc1, "°/s²", "xy.motor1a.name", slider: RateSlider(acc1, 0, accel: true)));
        speeds.Children.Add(Prop("xy.motor2a", acc2, "°/s²", "xy.motor2a.name", "xy.motor2.tip", RateSlider(acc2, 1, accel: true)));
        feed.ToolTip = Lang.T("xy.feed.tip");
        speeds.Children.Add(Prop("xy.feed", feed, "mm/s", "xy.feed.name"));
        var checks = Group("section.checks", expanded: false);
        checksSection = (Expander)controls.Children[controls.Children.Count - 1];
        Lang.Bind(result, System.Windows.Automation.AutomationProperties.NameProperty, "diag.detailsName");
        checks.Children.Add(executionInfo); checks.Children.Add(moveState); checks.Children.Add(result); checks.Children.Add(notice);
        var preview = new Button { Content = Lang.T("btn.preview"), MinWidth = 120, Background = Brushes.White, Foreground = WorkbenchTheme.Blue, BorderBrush = WorkbenchTheme.Blue }; preview.Click += (_,_) => Refresh();
        previewButton = preview;
        var save = new Button { Content = Lang.T("btn.save") }; save.Click += (_,_) => SaveCurrentParameters(ParametersPath);
        saveButton = save;
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
        actionSummary.ToolTip = new ToolTip { Content = actionDetails };
        ToolTipService.SetShowDuration(actionSummary, 60000);
        executionInfo.FontSize=12; executionInfo.Padding=new Thickness(0);
        foreach(var b in new[] { movePoint, upload, run }) { WorkbenchTheme.Primary(b); ToolTipService.SetShowOnDisabled(b, true); }
        movePoint.Click += (_,_) => { if(movePoint.IsEnabled && pointTarget is {} p) PointMoveRequested?.Invoke(PtpProtocol.Quantize(p)); };
        upload.Click += (_,_) => { if(upload.IsEnabled && execution is {} plan) { RememberUploaded(); TrajectoryRequested?.Invoke(plan); } };
        run.Click += (_,_) => { if(run.IsEnabled) ExecuteRequested?.Invoke(); };
        moveState.FontSize = 12;
        foreach(var control in new Control[] { l1,l2,x,y,a,b,c,rotation,scale,custom,direction,min1,max1,min2,max2,elbowMin,elbowMax,v1,v2,acc1,acc2,shape,branch,play,rewind,cursor }) editControls.Add(control);
        foreach (var field in new[] { l1, l2, x, y, a, b, c, rotation, scale, custom, min1, max1, min2, max2, elbowMin, elbowMax, v1, v2, acc1, acc2 }) field.TextChanged += (_, _) => ParametersChanged();
        foreach (var field in new[] { v1, v2, acc1, acc2 }) field.LostFocus += (_, _) => ClampRateFields();
        direction.SelectionChanged += (_, _) => ParametersChanged();
        foreach (var help in rangeHelp) editControls.Add(help);
        branch.SelectionChanged += (_, _) => ParametersChanged();
        shape.SelectionChanged += (_, _) => { CancelFreehandStroke(); freehandMode = shape.SelectedIndex == FreehandIndex; SwitchShape(); ShapeFields(); ParametersChanged(); UpdateFreehandTools(); };
        cursor.ValueChanged += (_, _) => { if (!advancing) { PausePlayback(); offlineSim=true; } DrawSample(); };
        canvas.SizeChanged += (_, _) => RedrawViewport();
        play.Click += (_, _) => TogglePlayback(); rewind.Click += (_, _) => { PausePlayback(); offlineSim=true; cursor.Value = 0; RedrawViewport(); };
        playbackTimer.Tick += (_, _) => AdvancePlayback(playbackFrom + playbackClock.Elapsed.TotalSeconds);
        Unloaded += (_, _) => { CancelFreehandWork(); PausePlayback(); };
        ShapeFields();
        Refresh();
    }

    // Drawing/positioning cap of one axis in degrees: the smaller of the configurable and the plan limit (microsteps).
    private static double Cap(int axis, int configMax, int planMax) => Math.Min(configMax, planMax) / MotionProfile.StepsPerDegree(axis);
    private static double RateCap(int axis) => Cap(axis, axis == 0 ? DeviceContract.Arm1RateMax : DeviceContract.Arm2RateMax, DeviceContract.PlanRateMax);
    private static double AccelCap(int axis) => Cap(axis, axis == 0 ? DeviceContract.Arm1AccelMax : DeviceContract.Arm2AccelMax, DeviceContract.PlanAccelMax);
    private static TextBox Field(string text) => new() { Text = text, Width = 76, Margin = new Thickness(4), Padding = new Thickness(4) };
    // The slider only edits `field`, which stays the saved, tested and parsed value; typing in the box moves the slider.
    // It spans the device minimum to the cap in 0.1 steps, so a slider value is always accepted by MotionProfile.From.
    private static Slider RateSlider(TextBox field, int axis, bool accel)
    {
        double min = (accel ? DeviceContract.AccelMin : DeviceContract.RateMin) / MotionProfile.StepsPerDegree(axis);
        var slider = new Slider { Minimum = Math.Ceiling(min * 10) / 10, Maximum = Math.Floor((accel ? AccelCap(axis) : RateCap(axis)) * 10) / 10,
            TickFrequency = 0.1, IsSnapToTickEnabled = true, SmallChange = 0.1, LargeChange = 1 };
        bool syncing = false;
        void FromText() {
            if (!double.TryParse(field.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) || !double.IsFinite(v)) return;
            syncing = true; slider.Value = v; syncing = false;
        }
        slider.ValueChanged += (_, _) => { if (!syncing) field.Text = Mm(slider.Value); };
        field.TextChanged += (_, _) => FromText();
        FromText();
        return slider;
    }
    private static Button HelpIcon(string tip, string nameKey = "xy.help.name")
    {
        var help=new Button { Content="?", Width=22, MinHeight=22, Padding=new Thickness(0), Margin=new Thickness(2,4,6,4),
            FontWeight=FontWeights.Bold, ToolTip=tip };
        System.Windows.Automation.AutomationProperties.SetName(help, Lang.T(nameKey));
        System.Windows.Automation.AutomationProperties.SetHelpText(help, tip);
        ToolTipService.SetShowOnDisabled(help, true);
        ToolTipService.SetInitialShowDelay(help, 200);
        ToolTipService.SetShowDuration(help, 60000);   // these tips are paragraphs; the 5 s default cuts them off
        return help;
    }
    private static void SetHelp(Button help, string tip)
    {
        if (Equals(help.ToolTip, tip)) return;
        help.ToolTip = tip; System.Windows.Automation.AutomationProperties.SetHelpText(help, tip);
    }
    // Everything that used to sit under the speed sliders: what the group governs, the caps, and the values that take effect.
    private string SpeedsHelpText()
    {
        string effective;
        try { var r=RequestedRates; var d=MotionProfile.From(r).DrawingRates(r.Feed); effective = Lang.T("xy.effective", d.V1, d.V2, d.A1, d.A2); }
        catch(ArgumentException ex) { effective = ex.Message; }
        return Lang.T("xy.speedsGlobal", RateCap(0), AccelCap(0), RateCap(1), AccelCap(1)) + "\n" + effective;
    }
    // The range tip ends with the sync status that used to be printed under the range fields.
    private string RangeHelpText()
    {
        return Lang.T("xy.rangeHelp1") + EnteredEnvelope() + Lang.T("xy.rangeHelp2") + "\n\n" + RangeStatusText();
    }
    private string RangeStatusText() => RequestedLimits is not { } requested ? Lang.T("xy.rangeInvalid")
        : firmwareLimits is null ? Lang.T("xy.rangePreviewOnly")
        : requested == firmwareLimits && deviceLimitsConfirmed ? Lang.T("xy.rangeMatched")
        : Lang.T("xy.rangeUnsynced") + deviceMoveReason + "\n" + Lang.T("xy.deviceRange") + Envelope(firmwareLimits);
    private string EnteredEnvelope()
    {
        try { return Envelope(PreviewLimits()); } catch(ArgumentException) { return Lang.T("xy.envelopeInvalid"); }
    }
    private static string Envelope(JointLimits? limits) => limits is { } range
        ? Lang.T("xy.envelopeFormat", range.Min1, range.Max1, range.Min2, range.Max2, range.ElbowMin, range.ElbowMax)
        : Lang.T("xy.envelopeEmpty");
    private void RefreshRangeHelp()
    {
        string tipMax = "—";   // also shown while the link lengths or speeds are invalid
        try { var r=RequestedRates; var d=MotionProfile.From(r).DrawingRates(r.Feed);
            tipMax = Model.MaxTipSpeed(d.V1, d.V2).ToString("0.#", CultureInfo.InvariantCulture);
        } catch(ArgumentException) { }
        if (feed.Text != tipMax) feed.Text = tipMax;
        if (speedHelp is { } speeds) SetHelp(speeds, SpeedsHelpText());
        var tip=RangeHelpText();
        foreach(var help in rangeHelp) SetHelp(help, tip);
    }
    private void RefreshOverlayLimits()
    {
        try { overlayLimits = PreviewLimits(); } catch(ArgumentException) { overlayLimits = null; }
    }
    private void ShapeFields()
    {
        int s = shape.SelectedIndex;
        // Where (x, y) sits: the start of a line, the corner of a rectangle, the contour's origin, otherwise the centre.
        string anchor = s switch { 0 => "xy", 1 => "xy.start", 2 or 6 => "xy.corner", 10 => "xy.origin", _ => "xy.center" };
        xLabel.Text = Lang.T(anchor == "xy" ? "xy.x" : anchor + "X"); yLabel.Text = Lang.T(anchor == "xy" ? "xy.y" : anchor + "Y");
        var (la, lb, lc) = s switch {
            1 => ("xy.endX", "xy.endY", ""), 2 => ("xy.width", "xy.height", ""), 3 => ("xy.radius", "", ""),
            4 => ("xy.semiX", "xy.semiY", ""), 5 => ("xy.radius", "xy.startAngle", "xy.sweep"),
            6 => ("xy.width", "xy.height", "xy.cornerRadius"), 7 => ("xy.halfWidth", "xy.halfHeight", "xy.freq"),
            8 => ("xy.circumRadius", "xy.sides", ""), 9 => ("xy.startRadius", "xy.endRadius", "xy.turns"), _ => ("", "", "") };
        foreach (var (label, key) in new[] { (aLabel, la), (bLabel, lb), (cLabel, lc) }) label.Text = key == "" ? "" : Lang.T(key);
        aRow.Visibility = a.Visibility = aLabel.Visibility = la == "" ? Visibility.Collapsed : Visibility.Visible;
        bRow.Visibility = b.Visibility = bLabel.Visibility = lb == "" ? Visibility.Collapsed : Visibility.Visible;
        cRow.Visibility = c.Visibility = cLabel.Visibility = lc == "" ? Visibility.Collapsed : Visibility.Visible;
        customRow.Visibility = s == 10 ? Visibility.Visible : Visibility.Collapsed;
        xRow.Visibility = yRow.Visibility = transformSection.Visibility = s == FreehandIndex ? Visibility.Collapsed : Visibility.Visible;
        UpdateFreehandTools();
        customLabel.Text = Lang.T("xy.customHelp");
        System.Windows.Automation.AutomationProperties.SetName(custom, Lang.T("xy.customName"));
        System.Windows.Automation.AutomationProperties.SetName(x,xLabel.Text); System.Windows.Automation.AutomationProperties.SetName(y,yLabel.Text);
        System.Windows.Automation.AutomationProperties.SetName(a,aLabel.Text); System.Windows.Automation.AutomationProperties.SetName(b,bLabel.Text);
        System.Windows.Automation.AutomationProperties.SetName(c,cLabel.Text);
    }
    private static double Number(TextBox field)
    {
        if (!double.TryParse(field.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double value) || !double.IsFinite(value))
            throw Lang.Error("err.number", (Func<string>)(() => System.Windows.Automation.AutomationProperties.GetName(field)));
        return value;
    }
    private JointLimits? Limits()
    {
        if (new[] { min1, max1, min2, max2, elbowMin, elbowMax }.All(f => string.IsNullOrWhiteSpace(f.Text))) return null;
        static double Bound(TextBox field, double fallback)
        {
            double value = string.IsNullOrWhiteSpace(field.Text) ? fallback : Number(field);
            double rounded = Math.Round(value * 100, MidpointRounding.AwayFromZero) / 100;
            if (Math.Abs(value - rounded) > 1e-8) throw Lang.Error("err.limitDecimals");
            return rounded;
        }
        var limits = new JointLimits(Bound(min1,-30), Bound(max1,30), Bound(min2,-60), Bound(max2,60),
            Bound(elbowMin,-JointLimits.DefaultElbow), Bound(elbowMax,JointLimits.DefaultElbow));
        for (int axis = 0; axis < 3; axis++) {   // q1, q2 and the elbow share one rule, the firmware's
            var (min, max) = (limits.Bounds(axis).Q1, limits.Bounds(axis).Q2);
            if (min < -180 || max > 180 || min >= max || min > 0 || max < 0) throw Lang.Error("err.limitBounds");
        }
        return limits;
    }
    private JointLimits PreviewLimits() => Limits() ?? DefaultTrialRange;
    private void Refresh()
    {
        if (updating || RecordingFreehand) return;
        CancelFreehandWork();
        if (shape.SelectedIndex != FreehandIndex && freehandNeedsProcessing) QueueFreehandWork();
        parametersDirty = false;
        offlineSim = false;
        cameraReady = false; autoFit = true; operationError=null; rangeFailure = null;
        RefreshOverlayLimits(); RefreshRangeHelp();
        pointTarget = null; movePoint.IsEnabled = false; execution = null; upload.IsEnabled = false; Lang.Text(executionInfo, () => "");
        PausePlayback(); trajectory = null; timing = null;
        export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = false;
        canvas.Children.Clear(); Lang.Text(result, () => ""); sampleInfo.Clear(); Lang.Text(notice, () => "");
        try
        {
            var model = new PlanarArm(Number(l1), Number(l2));
            _ = MotionProfile.From(RequestedRates);
            double tx = shape.SelectedIndex == FreehandIndex ? 0 : Number(x), ty = shape.SelectedIndex == FreehandIndex ? 0 : Number(y);
            JointLimits? limits = PreviewLimits(); planningLimits = limits;
            if (shape.SelectedIndex == FreehandIndex) {
                QueueFreehandWork(model, limits.Value, Math.Max(0, branch.SelectedIndex), RequestedRates); return;
            }
            if (shape.SelectedIndex > 0)
            {
                JointLimits? bounds = limits;
                var uses = ShapeUses(shape.SelectedIndex); var extras = ShapeOptionsFromFields();
                trajectory = DrawingTrajectory.Create(model, (DrawingShape)(shape.SelectedIndex - 1), tx, ty,
                    uses.A ? Number(a) : 0, uses.B ? Number(b) : 0, Math.Max(0, branch.SelectedIndex), bounds, extras);
                timing = new TimedTrajectory(trajectory, RequestedRates);
                pointTarget = timing.At(0).Pose; RefreshExecution(); UpdatePointButton();
                ShowDrawingPreview(bounds, 0);
                return;
            }
            var poses = model.Inverse(tx, ty);
            var selected = poses[Math.Min(branch.SelectedIndex < 0 ? 0 : branch.SelectedIndex, poses.Length - 1)];
            pointTarget = selected; UpdatePointButton();
            Lang.Text(notice, () => limits is null ? Lang.T("notice.pointNoRange") : Lang.T("notice.pointChecked"));
            result.Foreground = Brushes.DarkSlateGray;
            Lang.Text(result, () => Lang.T("plan.target", tx, ty, model.InnerRadius, model.OuterRadius) +
                string.Join("\n", poses.Select((p, i) => Lang.T("plan.solution", i + 1, p.Q1, p.Q2, JointLimits.Elbow(p)) +
                    (limits is null ? Lang.T("plan.noRangeCheck") : limits.Value.Contains(p) ? Lang.T("plan.inRange") : Lang.T("plan.outRange")))) +
                (Math.Abs(Math.Sin((selected.Q2 - selected.Q1) * Math.PI / 180)) < 1e-6 ? Lang.T("plan.singular") : ""));
            Draw(model, selected, tx, ty, limits is null || limits.Value.Contains(selected), Reachability(model).IsReachable(model.Forward(selected).Tip));
            ShowPoseInfo(model, selected);
        }
        catch (ArgumentException ex) { InvalidPlan(() => ex.Message, (ex as Lang.ValidationException)?.RangeFailure); checksSection.IsExpanded = true; }
    }
    private void ParametersChanged()
    {
        if (updating) return;
        CancelFreehandStroke();
        parametersDirty = true;
        cameraReady = false; autoFit = true;
        RefreshOverlayLimits(); RefreshRangeHelp();
        pointerInfo.Text = PointerHint;
        InvalidPlan(() => Lang.T("err.needPreview"));
        NotifyLimitsChanged();
        if (freehandNeedsProcessing && editingAllowed) QueueFreehandWork();
    }
    private void InvalidPlan(Func<string> reason, JointRangeFailure? failure = null, bool cancelFreehandWork = true)
    {
        if (cancelFreehandWork) CancelFreehandWork();
        rangeFailure = failure;
        PausePlayback(); trajectory = null; timing = null;
        export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = false;
        canvas.Children.Clear(); sampleInfo.Clear(); result.Foreground = Brushes.Firebrick; Lang.Text(result, reason); Lang.Text(notice, () => "");
        pointTarget = null; movePoint.IsEnabled = false;
        execution = null; upload.IsEnabled = false; Lang.Text(executionInfo, () => "");
        UpdatePointButton();
        RequestViewportRedraw();
    }
    public void SetPointMoveState(bool allowed, JointLimits? limits, string reason, bool supportsTrajectory = false, bool limitsConfirmed = true, ExecutionTrajectory? uploaded = null, bool ran = false)
    {
        pointMoveAllowed = allowed; firmwareLimits = limits; trajectorySupported = supportsTrajectory; deviceMoveReason = reason;
        deviceLimitsConfirmed = limitsConfirmed; uploadedPlan = uploaded; uploadedRan = uploaded is not null && ran;
        RefreshOverlayLimits(); RefreshRangeHelp();
        UpdatePointButton();
    }
    private void RefreshExecution()
    {
        execution = null; Lang.Text(executionInfo, () => "");
        if (timing is null || planningLimits is not { } range) return;
        try {
            double asked = timing.Duration;   // the speeds as entered, before the per-axis cap
            execution = new ExecutionTrajectory(timing, range);
            timing = execution.Timing;
            ShowExecutionInfo(execution, asked);
        } catch (ArgumentException ex) { Lang.Text(executionInfo, () => Lang.T("plan.execFail") + ex.Message); }
    }
    private void ShowExecutionInfo(ExecutionTrajectory shown, double asked)
    {
        var (peak1, peak2) = shown.PeakRates();
        double capped = shown.Timing.Duration / shown.RetryStretch, slower = (capped / asked - 1) * 100, extra = (shown.RetryStretch - 1) * 100;
        Lang.Text(executionInfo, () => Lang.T("plan.execInfo", shown.Nodes.Length, shown.Duration, DeviceContract.PlanRateMax, peak1, peak2) +
            (slower >= 0.5 ? Lang.T("plan.execLimited", slower, asked, capped, DeviceContract.PlanRateMax) : "") +
            (extra >= 0.5 ? Lang.T("plan.execStretch", extra) : ""));
    }
    private void ShowDrawingPreview(JointLimits? bounds, double repaired)
    {
        UpdatePointButton();
        Lang.Text(notice, () => bounds is null ? Lang.T("notice.noRange") : Lang.T("notice.checked"));
        result.Foreground = Brushes.DarkSlateGray;
        Lang.Text(result, () => Lang.T("plan.trajOk") + (Lang.IsEn ? "; " : "；") + Lang.T("plan.trajStats",
            trajectory!.Samples.Length, timing!.Length, timing.Duration,
            trajectory.Samples.Min(s => s.Pose.Q1), trajectory.Samples.Max(s => s.Pose.Q1),
            trajectory.Samples.Min(s => s.Pose.Q2), trajectory.Samples.Max(s => s.Pose.Q2),
            trajectory.Samples.Min(s => JointLimits.Elbow(s.Pose)), trajectory.Samples.Max(s => JointLimits.Elbow(s.Pose))) +
            (trajectory.NearSingularity ? Lang.T("plan.nearSing") : "") + (repaired > 0 ? Lang.T("fh.repaired", repaired) : ""));
        cursor.Maximum = timing!.Duration; cursor.Value = 0; offlineSim = false;
        export.IsEnabled = cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = editingAllowed; DrawSample();
    }
    // WPF compares reference values by reference, so re-assigning an equal string still raises a change: the status
    // box repaints and an open ToolTip closes and reopens on every device poll. Write only what actually differs.
    private static void SetText(TextBlock block, string text) { if (block.Text != text) block.Text = text; }
    private static void SetTip(FrameworkElement element, string tip) { if (!Equals(element.ToolTip, tip)) element.ToolTip = tip; }
    private void UpdatePointButton()
    {
        movePoint.Content = shape.SelectedIndex == 0 ? Lang.T("btn.movePoint") : Lang.T("btn.moveToStart");
        bool limitsMatch = deviceLimitsConfirmed && RequestedLimits is { } requested && requested == firmwareLimits && requested == planningLimits;
        // While the device holds a plan it has not drawn yet it refuses every other move; only 执行图形 or 总停止 leaves that state.
        // After a run the plan stays on the device, so the move back to the start is allowed.
        movePoint.IsEnabled = !ProcessingFreehand && pointMoveAllowed && limitsMatch && (uploadedPlan is null || uploadedRan) && pointTarget is { } p && firmwareLimits is { } range && range.Contains(PtpProtocol.Quantize(p)) && (planningLimits is not { } local || local.Contains(p)) &&
            (shape.SelectedIndex == 0 || (execution is not null && trajectorySupported));
        bool atStart = execution is not null && startFeedback is { Referenced: true, Inputs: 0 } &&
            startFeedback.Steps1 == ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose) && startFeedback.Steps2 == ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose);
        upload.IsEnabled = !ProcessingFreehand && pointMoveAllowed && limitsMatch && trajectorySupported && execution is not null && atStart;
        UpdateUploadCaption();
        // 执行图形 starts what the device holds, so it needs that exact plan to still be the one on screen.
        bool held = uploadedPlan is { } sent && execution is { } shown && sent.Nodes.SequenceEqual(shown.Nodes);
        planHeld = held; UpdateLamps();
        run.IsEnabled = upload.IsEnabled && held;
        string move = deviceMoveReason;
        if (RequestedLimits is { } currentRange) move += Lang.T("move.currentRange") + Envelope(currentRange);
        if (firmwareLimits is not null && !limitsMatch) move += Lang.T("move.locked");
        if (pointTarget is { } target && firmwareLimits is { } bounds) {
            target = PtpProtocol.Quantize(target);
            string label = bounds.Contains(target) ? Lang.T("move.inFw") : shape.SelectedIndex == 0 ? Lang.T("move.outFw") : Lang.T("move.outFwStart");
            move += Lang.T("move.fwBounds", label, target.Q1, target.Q2, bounds.Min1, bounds.Max1, bounds.Min2, bounds.Max2, JointLimits.Elbow(target), bounds.ElbowMin, bounds.ElbowMax);
        } else if (pointTarget is null) move += Lang.T("move.planFail") + result.Text;
        if (execution is not null) move += held ? (uploadedRan ? Lang.T("move.ran") : Lang.T("move.uploaded")) : atStart ? Lang.T("move.atStart") : Lang.T("move.needStart");
        if (uploadedPlan is not null && !held) move += uploadedRan ? Lang.T("move.nextPlan") : Lang.T("move.otherPlan");
        if (execution is null && shape.SelectedIndex > 0) move += "\n" + (executionInfo.Text.Length > 0 ? executionInfo.Text : Lang.T("move.needCheck"));
        SetText(moveState, move);
        UpdateActionStatus();
        SetTip(upload, move + Lang.T(planOnBoard is null ? "move.uploadTip" : "board.loadTip"));
        SetTip(run, move);
        SetTip(movePoint, move);
    }
    public void SetDeviceStatus(string? message, bool fault, bool busy, double percent, bool indeterminate)
    {
        bool changed=deviceStatusBusy!=busy;
        if(busy) { offlineSim=false; PausePlayback(); }   // the device is moving: show the device, not a simulation
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
        if(deviceStatusFault || deviceStatusBusy) { text=(deviceStatusFault?Lang.T("act.rejected"):Lang.T("act.waiting"))+(deviceStatusMessage??deviceMoveReason); severity=deviceStatusFault?2:1; }
        else if(operationError is not null) { text=Lang.T("act.opFail")+operationError(); severity=2; }
        else if(parametersDirty) { text=Lang.T("act.paramsDirty"); severity=1; }
        else if(pointTarget is null) { text=Lang.T("act.rejected")+result.Text; severity=2; }
        else if(planningLimits is {} local && !local.Contains(pointTarget.Value)) { text=Lang.T("act.outOfRange"); severity=2; }
        // While the device has not confirmed, its own reason says what to do (enabled: STOP ALL first; disabled: wait for the sync).
        else if(firmwareLimits is not null && (!deviceLimitsConfirmed || firmwareLimits != RequestedLimits)) { text=deviceLimitsConfirmed ? Lang.T("act.unsynced") : Lang.T("act.waiting")+deviceMoveReason; severity=1; }
        else if(shape.SelectedIndex>0 && firmwareLimits is not null && execution is null) { text=Lang.T("act.trajFail"); severity=2; }
        else if(!pointMoveAllowed) { text=Lang.T("act.waiting")+(deviceStatusMessage??deviceMoveReason); severity=1; }
        else if(run.IsEnabled) { text=uploadedRan?Lang.T("act.runAgain"):Lang.T("act.runNow"); severity=0; }
        // An undrawn plan blocks every other move; a drawn one does not, the next shape just goes to its own start.
        else if(uploadedPlan is not null && !uploadedRan) { text=Lang.T("act.planOnDevice"); severity=1; }
        else if(upload.IsEnabled) { text=Lang.T("act.uploadReady"); severity=0; }
        else if(uploadedPlan is not null && shape.SelectedIndex>0) { text=Lang.T("act.nextShape"); severity=1; }
        else if(shape.SelectedIndex==0 && movePoint.IsEnabled) { text=Lang.T("act.pointReady"); severity=0; }
        else if(!trajectorySupported) { text=Lang.T("act.noTraj"); severity=1; }
        else { text=Lang.T("act.needPen"); severity=1; }
        SetText(actionSummary,text);
        SetText(actionDetails,text);
        if (System.Windows.Automation.AutomationProperties.GetHelpText(actionSummary) != text)
            System.Windows.Automation.AutomationProperties.SetHelpText(actionSummary, text);
        actionSummary.Foreground=StatusInk[severity];
        actionStatus.Background=StatusFill[severity];
        SetTip(actionStatus,text+"\n"+moveState.Text+Lang.T("act.details"));
        if(System.Windows.Automation.AutomationProperties.GetName(actionStatus)!=text) System.Windows.Automation.AutomationProperties.SetName(actionStatus,text);
    }
    // One shared brush per severity: a fresh brush on every call is a new value every time.
    private static readonly SolidColorBrush[] StatusInk = new[] { "#166534", "#854D0E", "#991B1B" }.Select(WorkbenchTheme.Brush).ToArray(),
        StatusFill = new[] { "#DCFCE7", "#FEF3C7", "#FEE2E2" }.Select(WorkbenchTheme.Brush).ToArray();
    public void PausePlayback()
    { playbackTimer.Stop(); playbackClock.Stop(); play.Content = Lang.T("btn.play"); }
    private void TogglePlayback()
    {
        if (timing is null) return;
        offlineSim=true;
        if (playbackTimer.IsEnabled) { PausePlayback(); return; }
        if (cursor.Value >= timing.Duration) cursor.Value = 0;
        playbackFrom = cursor.Value; playbackClock.Restart(); playbackTimer.Start(); play.Content = Lang.T("btn.playPause");
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
            sampleInfo.Text = (FollowCommand?Lang.T("vp.sampleDevice"):Lang.T("vp.offlineTime", sample.Time, plan.Duration)) +
                PoseText(pose, tip);
            sampleInfo.ToolTip=FollowCommand?Lang.T("vp.sampleTooltipDevice"):
                Lang.T("vp.sampleTooltipOffline", sample.Distance, sample.Speed);
        }
        catch (ArgumentException ex) { InvalidPlan(() => ex.Message, (ex as Lang.ValidationException)?.RangeFailure); checksSection.IsExpanded = true; }
    }
    private void Export()
    {
        if (timing is not { } plan) return;
        PausePlayback();
        var dialog = new Microsoft.Win32.SaveFileDialog { Title = Lang.T("vp.saveDialog"), Filter = Lang.T("vp.csvFilter"),
            DefaultExt = ".csv", FileName = $"timed-preview-{plan.Path.Shape}-{DateTime.Now:yyyyMMdd-HHmmss}.csv" };
        if (dialog.ShowDialog() != true) return;
        try { File.WriteAllText(dialog.FileName, plan.ToCsv(), new UTF8Encoding(true)); sampleInfo.Text = Lang.T("vp.savedPlan") + dialog.FileName; }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or ArgumentException)
        { sampleInfo.Text = Lang.T("vp.saveFail") + ex.Message; }
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
            !((string)panel.rangeHelp[0].ToolTip).Contains("范围一致"))
            throw new InvalidOperationException("设备确认统一范围后未开放同一条轨迹");
        panel.SetPointMoveState(true,requested,"仍有范围请求等待完整确认",true,false);
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled || ((string)panel.rangeHelp[0].ToolTip).Contains("范围一致"))
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
        // The four speed/accel sliders (the first four in the edit list, in row order) mirror their boxes both ways.
        var sliders = panel.editControls.OfType<Slider>().Take(4).ToArray();
        panel.v1.Text = "7.5";
        if (Math.Abs(sliders[0].Value - 7.5) > 1e-9) throw new InvalidOperationException("滑块没有跟随输入框");
        sliders[0].Value = 9.3;
        if (panel.v1.Text != "9.3" || Math.Abs(panel.RequestedRates.V1 - 9.3) > 1e-9) throw new InvalidOperationException("拖动滑块没有写回输入框");
        panel.v1.Text = "99";
        if (sliders[0].Value != sliders[0].Maximum || panel.v1.Text != "99") throw new InvalidOperationException("超上限的输入应让滑块停在上限、且不被改写");
        // The explanations live in the "?" tips: caps and effective values for the speeds, the invalid-input message when a box is bad.
        if (panel.speedHelp?.ToolTip is not string speedTip || !speedTip.Contains("上限") || !speedTip.Contains("实际生效")) throw new InvalidOperationException("运动参数的问号提示缺上限或实际生效值");
        panel.v2.Text = "abc";
        if (!((string)panel.speedHelp!.ToolTip).Contains("有效数值")) throw new InvalidOperationException("速度输入无效时，问号提示应说明原因");
        panel.v2.Text = "3";
        foreach (var slider in sliders) foreach (var end in new[] { slider.Minimum, slider.Maximum }) {
            slider.Value = end;
            _ = MotionProfile.From(panel.RequestedRates);   // throws if a slider end is outside what the firmware accepts
        }
        // The plan summary shows the per-axis peak step rate, and says so when the cap makes the run slower than the speeds entered.
        panel.v1.Text = "5"; panel.v2.Text = "3"; panel.acc1.Text = "10"; panel.acc2.Text = "6"; panel.x.Text = "260"; panel.y.Text = "0"; panel.a.Text = "50"; panel.Refresh();
        if (panel.execution is null || !panel.executionInfo.Text.Contains("每轴峰值") || panel.executionInfo.Text.Contains("比你填的速度慢"))
            throw new InvalidOperationException("低于上限的速度应显示峰值步频，且不提示被上限压慢：" + panel.executionInfo.Text);
        panel.v1.Text = "99"; panel.v2.Text = "99"; panel.acc1.Text = "99"; panel.acc2.Text = "99"; panel.x.Text = "260"; panel.y.Text = "0"; panel.a.Text = "70"; panel.Refresh();   // a big circle at the accel caps is speed-bound
        var (peak1, peak2) = panel.execution?.PeakRates() ?? (double.NaN, double.NaN);
        if (panel.execution is null || !panel.executionInfo.Text.Contains("比你填的速度慢") || !(peak1 <= DeviceContract.PlanRateMax) || !(peak2 <= DeviceContract.PlanRateMax))
            throw new InvalidOperationException("超过上限的速度应提示被压慢，且每轴峰值不超过上限：" + panel.executionInfo.Text + " / " + panel.result.Text + $" / peaks {peak1:0}/{peak2:0}");
    }
    // The elbow (q2 - q1) range from the box to the plan, the point-move gate and the saved file.
    private static void ElbowLimitSelfTest()
    {
        static void Fail(string message) => throw new InvalidOperationException("肘角范围：" + message);
        var panel = new XYPreviewPanel();
        panel.Refresh();   // the default circle: centre (340, 0), r = 5, so the elbow stays within about 29-41 deg
        if (panel.timing is null || panel.RequestedLimits?.ElbowMin != -155 || panel.RequestedLimits?.ElbowMax != 155 || panel.overlayLimits != DefaultTrialRange)
            Fail("默认肘范围应为 ±155° 且默认圆可规划：" + panel.result.Text);
        double elbowAtStart = JointLimits.Elbow(panel.timing!.At(0).Pose);
        panel.elbowMax.Text = "30"; panel.Refresh();
        if (panel.RequestedLimits != new JointLimits(-30, 30, -60, 60, -155, 30) || panel.overlayLimits?.ElbowMax != 30 || panel.timing is not null ||
            panel.execution is not null || !panel.result.Text.Contains("肘"))
            Fail("肘最大 30° 时 q1/q2 范围内的圆必须被拒绝并说明肘角：" + panel.result.Text);
        panel.elbowMax.Text = "45"; panel.Refresh();
        if (panel.timing is null || panel.execution is null) Fail("肘最大 45° 应可规划：" + panel.result.Text);
        // The pose readout shows the elbow and the room to each bound.
        if (!panel.sampleInfo.Text.Contains($"肘={elbowAtStart:F2}°") || !panel.sampleInfo.Text.Contains($"距下限 {elbowAtStart + 155:F1}°") ||
            !panel.sampleInfo.Text.Contains($"距上限 {45 - elbowAtStart:F1}°"))
            Fail("姿态读数缺少肘角或距两侧边界的余量：" + panel.sampleInfo.Text);
        // Each elbow box follows the firmware's rule: contains 0, ordered, within +/-180, two decimals.
        foreach (var (low, high) in new[] { ("1", "45"), ("-155", "-1"), ("-155", "181"), ("-181", "45"), ("-155.001", "45"), ("-155", "45.005"), ("abc", "45") })
        {
            panel.elbowMin.Text = low; panel.elbowMax.Text = high;
            if (panel.RequestedLimits is not null || panel.overlayLimits is not null) Fail($"无效肘范围 [{low}, {high}] 被接受");
        }
        panel.elbowMin.Text = "-155"; panel.elbowMax.Text = "45"; panel.Refresh();   // editing a box invalidates the plan until it is checked again
        // The firmware must report the same elbow range before anything moves, and the saved file keeps it.
        var wanted = panel.RequestedLimits!.Value;
        panel.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, panel.execution!.Nodes[0].Pose), ExecutionTrajectory.Counts(1, panel.execution.Nodes[0].Pose), 0, 0, 0, 0));
        panel.SetPointMoveState(true, wanted with { ElbowMax = 155 }, "设备肘范围仍是默认值", true);
        if (panel.movePoint.IsEnabled || panel.upload.IsEnabled) Fail("设备的肘范围与输入不同，运动必须保持锁定");
        panel.SetPointMoveState(true, wanted, "设备已确认", true);
        if (!panel.movePoint.IsEnabled || !panel.upload.IsEnabled) Fail("设备确认同一肘范围后应开放运动");
        string saved = panel.SaveParameters();
        panel.elbowMin.Text = ""; panel.elbowMax.Text = ""; panel.RestoreParameters(saved);
        if (panel.elbowMin.Text != "-155" || panel.elbowMax.Text != "45") Fail("保存的参数丢了肘范围");
        // A file saved before the elbow range existed has 15 fields; it restores with the elbow at its default.
        var oldFields = panel.ParameterFields.Take(LegacyFieldCount).Select(f => f.Text).ToArray();
        panel.RestoreParameters(JsonSerializer.Serialize(new SavedParameters(1, panel.shape.SelectedIndex, panel.branch.SelectedIndex, oldFields)));
        if (panel.elbowMin.Text != "" || panel.elbowMax.Text != "" || panel.RequestedLimits != DefaultTrialRange) Fail("旧参数文件应以默认肘范围恢复");
        try { panel.RestoreParameters(JsonSerializer.Serialize(new SavedParameters(1, 3, 0, oldFields.Append("-155").ToArray()))); Fail("字段数不对的参数文件被接受"); }
        catch (ArgumentException) { }
        // A single point: wide q boxes, so only the elbow can refuse it. (150, 0) needs an elbow of 130 degrees.
        panel.shape.SelectedIndex = 0; panel.x.Text = "150"; panel.y.Text = "0";
        panel.min1.Text = "-180"; panel.max1.Text = "180"; panel.min2.Text = "-180"; panel.max2.Text = "180";
        panel.elbowMin.Text = ""; panel.elbowMax.Text = "100"; panel.Refresh();
        panel.SetPointMoveState(true, panel.RequestedLimits, "设备已确认", false);
        if (panel.movePoint.IsEnabled || !panel.actionSummary.Text.Contains("超出范围")) Fail("单点目标的肘角超出范围时不得移动：" + panel.actionSummary.Text);
        panel.elbowMax.Text = ""; panel.Refresh();
        panel.SetPointMoveState(true, panel.RequestedLimits, "设备已确认", false);
        if (!panel.movePoint.IsEnabled) Fail("肘角在默认范围内的单点应可移动：" + panel.actionSummary.Text);
    }
    // The newer shapes: which fields each shows, the transform fields, the saved file, and the messages for bad input.
    private static void ShapeFieldsSelfTest()
    {
        static void Fail(string message) => throw new InvalidOperationException("图形参数：" + message);
        var panel = new XYPreviewPanel();
        void Show(int index) { panel.shape.SelectedIndex = index; panel.LoadExample(); WaitForFreehand(panel); }
        double Width(DrawingTrajectory t) => t.Samples.Max(s => s.Target.X) - t.Samples.Min(s => s.Target.X);
        // Each shape shows the fields it reads and hides the rest.
        foreach (var (index, usesA, usesB, usesC, usesText) in new[] { (0,false,false,false,false), (1,true,true,false,false), (2,true,true,false,false), (3,true,false,false,false),
            (4,true,true,false,false), (5,true,true,true,false), (6,true,true,true,false), (7,true,true,true,false), (8,true,true,false,false), (9,true,true,true,false), (10,false,false,false,true), (11,false,false,false,false) })
        {
            Show(index);
            if ((panel.aRow.Visibility == Visibility.Visible) != usesA || (panel.bRow.Visibility == Visibility.Visible) != usesB ||
                (panel.cRow.Visibility == Visibility.Visible) != usesC || (panel.customRow.Visibility == Visibility.Visible) != usesText)
                Fail($"图形 {index} 显示的输入框与它读取的不一致");
            if (index > 0 && (panel.execution is null || panel.timing is null)) Fail($"图形 {index} 的示例不可执行：" + panel.result.Text);
        }
        Show(5); if (panel.cLabel.Text != "扫过角度 / °" || panel.bLabel.Text != "起始角 / °") Fail("圆弧的标签应是半径/起始角/扫过角度");
        Show(3); if (panel.xLabel.Text != "中心 X / mm") Fail("圆的锚点标签");
        Show(2); if (panel.xLabel.Text != "左下角 X / mm") Fail("矩形的锚点标签");
        // Rotation, scale and direction reach the plan.
        Show(4); double flat = Width(panel.trajectory!);
        panel.rotation.Text = "90"; panel.Refresh(); double upright = Width(panel.trajectory!);
        if (Math.Abs(flat - 24) > 0.05 || Math.Abs(upright - 12) > 0.05) Fail($"椭圆旋转 90° 后宽度应由 24 变为 12：{flat}/{upright}");
        panel.rotation.Text = "0"; panel.scale.Text = "0.5"; panel.Refresh();
        if (panel.trajectory is null || Math.Abs(Width(panel.trajectory) - 12) > 0.05) Fail("缩放 0.5 应使椭圆缩小一半：" + panel.result.Text);
        Show(3); double ccwY = panel.trajectory!.Samples[1].Target.Y;
        panel.direction.SelectedIndex = 1; panel.Refresh();
        if (!(ccwY > 0 && panel.trajectory!.Samples[1].Target.Y < 0)) Fail("顺时针圆应从同一点向另一侧出发");
        // Bad input says which field is wrong.
        Show(4); panel.rotation.Text = "abc"; panel.Refresh();
        if (panel.trajectory is not null || !panel.result.Text.Contains("旋转角")) Fail("旋转角无效时应指出该输入框：" + panel.result.Text);
        panel.rotation.Text = "0"; panel.scale.Text = "0"; panel.Refresh();
        if (panel.trajectory is not null || !panel.result.Text.Contains("缩放")) Fail("缩放为 0 应被拒绝并说明：" + panel.result.Text);
        Show(5); panel.c.Text = "0"; panel.Refresh();
        if (panel.trajectory is not null || !panel.result.Text.Contains("扫过角度")) Fail("圆弧扫过 0° 应被拒绝并说明：" + panel.result.Text);
        Show(8); panel.b.Text = "2"; panel.Refresh();
        if (panel.trajectory is not null || !panel.result.Text.Contains("边数")) Fail("多边形 2 条边应被拒绝并说明：" + panel.result.Text);
        Show(10); panel.custom.Text = "M 0 0 L 5 0 M 10 0 L 15 0"; panel.Refresh();
        if (panel.trajectory is not null || !panel.result.Text.Contains("一条连续路径")) Fail("两条轮廓应被拒绝并说明没有抬笔：" + panel.result.Text);
        panel.custom.Text = "0 0\n6 0\n6 4\n0 4\n0 0"; panel.Refresh();
        if (panel.execution is null || panel.trajectory!.Stops.Length != 5) Fail("点列轮廓应得到四条直线：" + panel.result.Text);
        // Each shape remembers its own sizes while another is shown.
        panel.shape.SelectedIndex = 5; panel.c.Text = "123"; panel.shape.SelectedIndex = 3; panel.shape.SelectedIndex = 5;
        if (panel.c.Text != "123") Fail("切换图形后丢了圆弧自己的扫过角度");
        // The saved file keeps everything, newlines of the custom contour included.
        Show(6); panel.rotation.Text = "10"; panel.scale.Text = "1.2"; panel.c.Text = "2"; panel.direction.SelectedIndex = 1;
        string saved = panel.SaveParameters();
        Show(10); panel.custom.Text = "0 0\n5 5"; panel.rotation.Text = "0"; panel.RestoreParameters(saved);
        if (panel.shape.SelectedIndex != 6 || panel.rotation.Text != "10" || panel.scale.Text != "1.2" || panel.c.Text != "2" || panel.direction.SelectedIndex != 1) Fail("保存的文件丢了旋转/缩放/方向/第三尺寸");
        panel.shape.SelectedIndex = 10; panel.custom.Text = "0 0\n10 0\n10 5"; string withContour = panel.SaveParameters();
        panel.custom.Text = ""; panel.RestoreParameters(withContour);
        if (panel.custom.Text != "0 0\n10 0\n10 5") Fail("自定义轮廓没有随参数保存");
        // Files written by older versions are prefixes: rotation and scale come back as 0 and 1, the direction as counter-clockwise.
        foreach (int count in new[] { LegacyFieldCount, ElbowFieldCount })
        {
            Show(3); panel.rotation.Text = "33"; panel.scale.Text = "2"; panel.direction.SelectedIndex = 1;
            var fields = panel.ParameterFields.Take(count).Select(f => f.Text).ToArray();
            panel.RestoreParameters(JsonSerializer.Serialize(new { Schema = 1, Shape = 3, Branch = 0, Fields = fields }));
            if (panel.rotation.Text != "0" || panel.scale.Text != "1" || panel.direction.SelectedIndex != 0 || panel.custom.Text != "") Fail($"{count} 个字段的旧文件应以默认变换恢复");
        }
        foreach (string bad in new[] { JsonSerializer.Serialize(new { Schema = 1, Shape = panel.shape.Items.Count, Branch = 0, Fields = panel.ParameterFields.Select(f => f.Text).ToArray() }),
            JsonSerializer.Serialize(new { Schema = 1, Shape = 3, Branch = 0, Fields = panel.ParameterFields.Select(f => f.Text).ToArray(), Direction = 2 }),
            JsonSerializer.Serialize(new { Schema = 1, Shape = 3, Branch = 0, Fields = panel.ParameterFields.Select((f, i) => i == 0 ? new string('9', 33) : f.Text).ToArray() }),
            JsonSerializer.Serialize(new { Schema = 1, Shape = 10, Branch = 0, Fields = panel.ParameterFields.Select((f, i) => i == CustomFieldIndex ? new string('1', 4097) : f.Text).ToArray() }) })
        {
            try { panel.RestoreParameters(bad); Fail("越界的参数文件被接受"); } catch (ArgumentException) { }
        }
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
        for (int index = 0; index < 11; index++)
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
        ElbowLimitSelfTest();
        ShapeFieldsSelfTest();
        RecentRunsSelfTest();
        ShapeDefaultsSelfTest();
        FreehandSelfTest(snapshotPath);
        var panel = new XYPreviewPanel();
        panel.ViewportSelfTest();
        ArmMotionPreviewSelfTest();
        DeviceSyncSelfTest();
        if(panel.detailView) throw new InvalidOperationException("中间视图默认必须是完整机械臂与可达圆（全臂）视图");
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
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟设备已就绪", true, true, other, true);
        if (panel.run.IsEnabled || !panel.movePoint.IsEnabled || !panel.upload.IsEnabled || panel.actionSummary.Text.Contains("重新上传") || panel.moveState.Text.Contains("总停止"))
            throw new InvalidOperationException("A drawn plan on the device must not hold up the next shape (no STOP ALL, no re-homing): " + panel.actionSummary.Text);
        panel.SetStartFeedback(null);
        if (!panel.movePoint.IsEnabled || !panel.actionSummary.Text.Contains("不用回零"))
            throw new InvalidOperationException("Away from the next shape's start the status must say to move there, without re-homing: " + panel.actionSummary.Text);
        panel.SetStartFeedback(fresh);
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
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "关节范围或全局速度与设备不一致：先总停止", true, false);
        if (!panel.actionSummary.Text.Contains("先总停止")) throw new InvalidOperationException("设备未确认速度/范围时，状态条要带出设备给的处理办法");
        panel.SetPointMoveState(true, new JointLimits(-30, 30, -60, 60), "模拟范围恢复", true);
        if (!panel.movePoint.IsEnabled || !panel.upload.IsEnabled || !panel.result.Text.Contains("整条轨迹检查通过"))
            throw new InvalidOperationException("Restored firmware limits did not rebuild the full plan");
        panel.x.Text = "335"; panel.a.Text = "15";
        panel.Refresh();
        if (!panel.movePoint.IsEnabled || panel.execution is null || !panel.result.Text.Contains("整条轨迹检查通过"))
            throw new InvalidOperationException("335/0/r15 must produce a full executable circle inside current firmware limits");
        panel.x.Text = "340"; panel.a.Text = "5";
        string parameterFile = Path.Combine(Path.GetTempPath(), "Two-Arm-parameters-" + Guid.NewGuid().ToString("N") + ".json");
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
            string[] shapeNames={"point","line","rectangle","circle","ellipse","arc","rounded-rectangle","figure8","polygon","spiral","custom","freehand"};
            for(int selected=0;selected<shapeNames.Length;selected++) {
                panel.shape.SelectedIndex=selected;
                if(selected<=3) {
                    panel.x.Text="260"; panel.y.Text="0";
                    panel.a.Text=selected==1?"290":"25";panel.b.Text="30";
                    panel.min1.Text="-135";panel.max1.Text="135";panel.min2.Text="-155";panel.max2.Text="155";
                    panel.Refresh();
                } else {   // the newer shapes use their examples (which also set the position and the transform back to defaults)
                    panel.LoadExample();
                    if(selected==4) { panel.rotation.Text="30"; panel.Refresh(); }
                }
                WaitForFreehand(panel);
                if(selected>0 && (panel.execution is null || !ReferenceEquals(panel.timing,panel.execution.Timing)))
                    throw new InvalidOperationException("Shape preview does not share execution timing");
                if(panel.timing is {} motion) panel.cursor.Value=motion.Duration*0.35;
                panel.Measure(new Size(panel.Width,panel.Height));panel.Arrange(new Rect(0,0,panel.Width,panel.Height));panel.UpdateLayout();
                var shapeImage=new RenderTargetBitmap(1050,860,96,96,PixelFormats.Pbgra32);shapeImage.Render(panel);
                var shapePng=new PngBitmapEncoder();shapePng.Frames.Add(BitmapFrame.Create(shapeImage));
                using var shapeStream=File.Create(Path.Combine(Path.GetDirectoryName(snapshotPath)!,shapeNames[selected]+".preview.png"));shapePng.Save(shapeStream);
                if(selected>0) {
                    panel.FitView(true);panel.UpdateLayout();
                    var detailImage=new RenderTargetBitmap(1050,860,96,96,PixelFormats.Pbgra32);detailImage.Render(panel);
                    var detailPng=new PngBitmapEncoder();detailPng.Frames.Add(BitmapFrame.Create(detailImage));
                    using var detailStream=File.Create(Path.Combine(Path.GetDirectoryName(snapshotPath)!,shapeNames[selected]+".detail.png"));detailPng.Save(detailStream);
                    panel.FitView(false);
                }
            }
        }
    }
}
