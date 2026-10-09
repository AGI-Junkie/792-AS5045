using System.Globalization;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Media;

namespace TwoLinkCommissioning;

// The recent runs, read back from the experiment archive, and the small status rows that say where each group of
// parameters stands: saved on this computer, confirmed by the device, used by the plan that is on the device.
public sealed partial class XYPreviewPanel
{
    // ---- recent runs ------------------------------------------------------------------------------------------------
    // Every recording keeps the settings it ran with and how it ended. Clicking one loads its shape and sizes, the transform,
    // the branch and the four motion rates, and checks them; nothing moves. It never touches the link lengths or the
    // joint/elbow ranges: those describe the machine, not the drawing, and changing them needs a STOP.
    private static readonly string[] PresetKeys = ["shape", "branch", "x", "y", "a", "b", "c", "rotation", "scale", "direction", "v1", "v2", "a1", "a2"];
    private static readonly string[] FigureKeys = ["shape", "branch", "x", "y", "a", "b", "c", "rotation", "scale", "direction"];
    private const int RecentCount = 8;
    internal static string? RunsRootOverride;   // tests keep the real archive untouched
    private static string RunsRoot => RunsRootOverride ?? ControlWindow.ExperimentRoot;
    private List<ExperimentRun> recentRuns = [];
    private int recentRefreshVersion;
    private readonly List<(Button Star, Button Load)> recentRows = [];
    private readonly StackPanel recentList = new();
    private readonly ContentControl recentHost = new() { IsTabStop = false, Focusable = false };   // locked with the other edit controls while the arm moves
    private readonly TextBlock recentNote = new() { TextWrapping = TextWrapping.Wrap, FontSize = 11, Foreground = WorkbenchTheme.Muted, Margin = new Thickness(10, 2, 10, 4) };

    private void ApplyPreset(IReadOnlyDictionary<string, string> values)
    {
        updating = true;
        try
        {
            foreach (var (key, value) in values)
            {
                if (!PresetKeys.Contains(key)) continue;
                switch (key)
                {
                    case "shape": shape.SelectedIndex = int.Parse(value); shownShape = shape.SelectedIndex; break;
                    case "branch": branch.SelectedIndex = int.Parse(value); break;
                    case "direction": direction.SelectedIndex = int.Parse(value); break;
                    case "x": x.Text = value; break; case "y": y.Text = value; break; case "a": a.Text = value; break; case "b": b.Text = value; break;
                    case "c": c.Text = value; break; case "rotation": rotation.Text = value; break; case "scale": scale.Text = value; break;
                    case "v1": v1.Text = value; break; case "v2": v2.Text = value; break; case "a1": acc1.Text = value; break; case "a2": acc2.Text = value; break;
                }
            }
            if (shownShape >= 0) shapeDims[shownShape] = [a.Text, b.Text, c.Text];
            ClampRateFields();
        }
        finally { updating = false; ShapeFields(); Refresh(); NotifyLimitsChanged(); }
    }

    private static bool ValidPresetValues(Dictionary<string, string>? values) =>
        values is not null && values.Count > 0 && values.Count <= PresetKeys.Length &&
        values.All(pair => PresetKeys.Contains(pair.Key) && pair.Value is not null && pair.Value.Length <= 32) &&
        (!values.TryGetValue("shape", out var s) || int.TryParse(s, out int shapeIndex) && shapeIndex is >= 0 and <= 10) &&
        (!values.TryGetValue("branch", out var br) || br is "0" or "1") && (!values.TryGetValue("direction", out var d) || d is "0" or "1");

    // The recorded settings text (shape=3; x=340; c=; rot=0; dir=ccw; ...) as field values, exactly as they were typed.
    // Older recordings lack the transform: it takes its defaults. A custom contour is not recorded, so such a run gives null.
    internal static Dictionary<string, string>? RunValues(string settings)
    {
        var raw = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (string part in settings.Split(';', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries))
            if (part.IndexOf('=') is int eq and > 0) raw[part[..eq].Trim()] = part[(eq + 1)..].Trim();
        var values = new Dictionary<string, string> { ["c"] = "", ["rotation"] = "0", ["scale"] = "1", ["direction"] = "0" };
        foreach (var (from, to) in new[] { ("shape", "shape"), ("branch", "branch"), ("x", "x"), ("y", "y"), ("a", "a"), ("b", "b"), ("c", "c"), ("rot", "rotation"), ("scale", "scale"), ("v1", "v1"), ("v2", "v2"), ("a1", "a1"), ("a2", "a2") })
            if (raw.TryGetValue(from, out var value)) values[to] = value;
        if (raw.TryGetValue("dir", out var dir)) values["direction"] = dir == "cw" ? "1" : "0";
        return values.TryGetValue("shape", out var s) && s != "10" && ValidPresetValues(values) ? values : null;
    }

    // ponytail: reads every recording's metadata (archive list is capped at 500); keep an index file if that gets slow.
    internal void RefreshRecentRuns()
    {
        recentRefreshVersion++;
        // A point move to a shape's start is not a drawing of its own; a single-point run is.
        recentRuns = ReadRecentRuns(RunsRoot);
        ShowRecentRuns();
        Lang.Text(recentNote, () => recentRuns.Count == 0 ? Lang.T("recent.empty", RecentCount) : Lang.T("recent.hint"));
    }

    private static List<ExperimentRun> ReadRecentRuns(string root) =>
        ExperimentArchive.List(root).Runs.Where(r => r.Kind != "point" || r.Settings?.Shape == 0).ToList();

    internal async Task RefreshRecentRunsAsync()
    {
        int version = ++recentRefreshVersion;
        string root = RunsRoot;
        try {
            var runs = await Task.Run(() => ReadRecentRuns(root));
            if (version != recentRefreshVersion) return;
            recentRuns = runs;
            ShowRecentRuns();
            Lang.Text(recentNote, () => recentRuns.Count == 0 ? Lang.T("recent.empty", RecentCount) : Lang.T("recent.hint"));
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) {
            System.Diagnostics.Trace.TraceError("Recent experiment refresh: " + ex);
        }
    }

    private string RunSummary(ExperimentRun run)
    {
        var s = run.Settings;
        string shapeName = s is { Shape: >= 0 } && s.Shape < shape.Items.Count ? Lang.T("xy.shape" + s.Shape) : "—";
        string status = run.Complete ? "✓ " + Lang.T("exp.status.complete") : "✗ " + (run.Outcome == "interrupted" ? Lang.T("exp.status.interrupted") : Lang.T("exp.status.other", run.Outcome));
        return $"{run.StartedUtc.ToLocalTime():MM-dd HH:mm}  {shapeName}  v{s?.V1:0.#}/{s?.V2:0.#}  {status}" + (run.Rmse is { } rmse ? $"  RMSE {rmse:0.000}°" : "");
    }

    // Starred runs first (however old), then the newest others.
    private void ShowRecentRuns()
    {
        recentList.Children.Clear(); recentRows.Clear();
        // The same figure run again (same shape, sizes, transform, branch and direction) is one row: the newest run of it.
        // Speeds are left out of the identity, a slower repeat is still the same figure. Runs that cannot be loaded stay apart.
        var seen = new HashSet<string>();
        bool NewFigure(ExperimentRun r) => RunValues(r.SettingsText) is not { } v || seen.Add(string.Join("|", FigureKeys.Select(k => v.GetValueOrDefault(k))));
        // A star means "stored on the board" (XYPreviewPanel.Board.cs): those runs come first, however old.
        var starredRuns = recentRuns.Where(r => StarSlot(RunValues(r.SettingsText)) is not null).Where(NewFigure).ToList();
        foreach (var run in starredRuns.Concat(recentRuns.Where(r => StarSlot(RunValues(r.SettingsText)) is null).Where(NewFigure).Take(RecentCount)))
        {
            var values = RunValues(run.SettingsText);
            var stored = StarSlot(values);
            bool starred = stored is not null;
            string summary = RunSummary(run);
            string starTip = starred ? Lang.T("board.star.stored", stored!.Slot + 1) : !boardSupported ? Lang.T("board.star.unavailable") : !boardKnown ? Lang.T("board.star.reading") : Lang.T("board.star.save");
            var star = new Button { Content = starred ? "★" : "☆", Width = 24, Margin = new Thickness(2), Padding = new Thickness(0), Background = Brushes.Transparent, BorderThickness = new Thickness(0),
                Foreground = starred ? WorkbenchTheme.Warn : WorkbenchTheme.Muted, ToolTip = starTip, IsEnabled = boardSupported && boardKnown && !boardBusy && (starred || values is not null) };
            ToolTipService.SetShowOnDisabled(star, true);
            System.Windows.Automation.AutomationProperties.SetName(star, starTip);
            var load = new Button { Content = new TextBlock { Text = summary, TextTrimming = TextTrimming.CharacterEllipsis }, HorizontalContentAlignment = HorizontalAlignment.Left,
                Margin = new Thickness(0, 1, 2, 1), Padding = new Thickness(6, 2, 6, 2), IsEnabled = values is not null,
                ToolTip = summary + "\n" + (values is null ? Lang.T("recent.cannotLoad") : string.Join("  ", values.Select(v => $"{v.Key}={v.Value}"))) };
            ToolTipService.SetShowOnDisabled(load, true);
            System.Windows.Automation.AutomationProperties.SetName(load, summary);
            star.Click += (_, _) => { if (!star.IsEnabled) return; if (stored is not null) StoredStarClicked(star, stored); else StarClicked(star, run, values!); };
            load.Click += (_, _) => { ApplyPreset(values!); Lang.Text(recentNote, () => Lang.T("recent.loaded", RunSummary(run))); };
            var row = new DockPanel { Margin = new Thickness(6, 0, 8, 0) };
            DockPanel.SetDock(star, Dock.Left); row.Children.Add(star); row.Children.Add(load);
            recentList.Children.Add(row); recentRows.Add((star, load));
        }
    }

    private UIElement BuildRecentRuns()
    {
        recentHost.Content = recentList;
        editControls.Add(recentHost);
        boardClearHost.Content = boardClear; editControls.Add(boardClearHost);
        Lang.Text(recentNote, () => Lang.T("recent.empty", RecentCount));
        UpdateBoardNote();
        return new StackPanel { Children = { recentHost, recentNote, boardNote, boardClearHost } };
    }

    private void RefreshRecentLanguage() { ShowRecentRuns(); Lang.RefreshText(recentNote); Lang.RefreshText(boardNote); UpdateBoardClear(); UpdateUploadCaption(); }

    // The motor speed/accel boxes show what will really be used: a value above the per-axis cap (2500 µsteps/s etc.) is written back as the cap,
    // the same number the slider tops out at. Called when a box loses focus and when values are loaded; typing is never rewritten mid-edit.
    private void ClampRateFields()
    {
        foreach (var (field, cap) in new[] { (v1, RateCap(0)), (v2, RateCap(1)), (acc1, AccelCap(0)), (acc2, AccelCap(1)) })
        {
            double top = Math.Floor(cap * 10) / 10;
            if (double.TryParse(field.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double value) && value > top) field.Text = Mm(top);
        }
    }

    // The two values the calibration wizard may offer back to the planner, only when its apply buttons are clicked.
    internal void SetLinkLengths(double length1, double length2)
    {
        updating = true;
        try { l1.Text = Mm(length1); l2.Text = Mm(length2); }
        finally { updating = false; Refresh(); NotifyLimitsChanged(); }
    }
    internal void SetElbowRange(double min, double max)
    {
        updating = true;
        try { elbowMin.Text = Mm(min); elbowMax.Text = Mm(max); }
        finally { updating = false; Refresh(); NotifyLimitsChanged(); }
    }
    internal JointPose? PointTargetPose => pointTarget;

    // ---- where each group of parameters stands ----------------------------------------------------------------------
    // Positions in the snapshot: the 21 text fields in ParameterFields order, then shape, branch, direction.
    private static readonly int[] ShapeGroup = [2, 3, 4, 5, 17, 18, 19, 20, 21, 22, 23, 24], SizeGroup = [0, 1], RangeGroup = [6, 7, 8, 9, 15, 16], SpeedGroup = [11, 12, 13, 14];
    private string[]? savedFields;                       // what was last written to, or read from, the parameter file
    private bool rangeConfirmed = true, speedConfirmed = true;
    private bool planHeld;                               // the plan on the device is the one on screen
    private readonly List<(int[] Group, bool HasDevice, TextBlock[] Lamps)> lampRows = [];

    private string[] SnapshotFields() => ParameterFields.Select(f => f.Text).Concat([shape.SelectedIndex.ToString(), branch.SelectedIndex.ToString(), direction.SelectedIndex.ToString(), JsonSerializer.Serialize(freehandPoints)]).ToArray();
    private void RememberSaved() { savedFields = SnapshotFields(); UpdateLamps(); }

    // The device's own report of the joint ranges and of the speeds, each against what is typed here.
    internal void SetParameterConfirmation(bool ranges, bool speeds)
    {
        if (rangeConfirmed == ranges && speedConfirmed == speeds) return;
        rangeConfirmed = ranges; speedConfirmed = speeds; UpdateLamps();
    }

    private Border BuildLamps(int[] group, bool hasDevice)
    {
        var lamps = Enumerable.Range(0, hasDevice ? 3 : 2).Select(_ => new TextBlock { FontSize = 11, Margin = new Thickness(0, 0, 10, 0) }).ToArray();
        lampRows.Add((group, hasDevice, lamps));
        var row = new WrapPanel { Margin = new Thickness(10, 3, 10, 3) };
        foreach (var lamp in lamps) row.Children.Add(lamp);
        return new Border { Background = Brushes.White, BorderBrush = WorkbenchTheme.Faint, BorderThickness = new Thickness(0, 0, 0, 1), Child = row };
    }

    private static readonly Brush LampOn = WorkbenchTheme.Ok, LampOff = WorkbenchTheme.Brush("#B45309"), LampNeutral = WorkbenchTheme.Idle;
    private void UpdateLamps()
    {
        if (lampRows.Count == 0) return;
        var now = SnapshotFields();
        foreach (var (group, hasDevice, lamps) in lampRows)
        {
            bool saved = savedFields is { } sf && group.All(i => sf[i] == now[i]);
            bool? used = uploadedPlan is null ? null : planHeld;
            bool deviceOk = ReferenceEquals(group, RangeGroup) ? rangeConfirmed : speedConfirmed;
            void Lamp(int index, string text, string tip, Brush colour)
            {
                var lamp = lamps[index];
                if (lamp.Text != text) lamp.Text = text;
                lamp.Foreground = colour;
                if (!Equals(lamp.ToolTip, tip)) lamp.ToolTip = tip;
            }
            Lamp(0, saved ? Lang.T("lamp.saved") : Lang.T("lamp.unsaved"), Lang.T("lamp.saved.tip"), saved ? LampOn : LampOff);
            int next = 1;
            if (hasDevice) { Lamp(next++, deviceOk ? Lang.T("lamp.device") : Lang.T("lamp.noDevice"), Lang.T("lamp.device.tip"), deviceOk ? LampOn : LampOff); }
            Lamp(next, used is null ? Lang.T("lamp.noPlan") : used.Value ? Lang.T("lamp.used") : Lang.T("lamp.differs"), Lang.T("lamp.used.tip"), used is null ? LampNeutral : used.Value ? LampOn : LampOff);
        }
    }

    // Recent runs, and the parameter lamps, through the real controls.
    private static void RecentRunsSelfTest()
    {
        static void Fail(string message) => throw new InvalidOperationException("最近运行与参数状态：" + message);
        string temp = Path.Combine(Path.GetTempPath(), "arm-recent-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(temp);
        RunsRootOverride = Path.Combine(temp, "runs");
        try
        {
            // The settings text as recorded: an empty c stays empty, an older recording gets the default transform,
            // a custom contour (not recorded) or a damaged text cannot be loaded.
            if (RunValues("shape=3; v1=5; v2=3; a1=10; a2=6; x=340; c=") is not { } old || old["c"] != "" || old["rotation"] != "0" || old["scale"] != "1" || old["direction"] != "0" || old["x"] != "340") Fail("旧记录应补上默认变换并保留空的 c");
            if (RunValues("shape=2; dir=cw; v1=1")?["direction"] != "1") Fail("dir=cw 应载入为顺时针");
            if (RunValues("shape=10; v1=5; v2=3; a1=10; a2=6") is not null || RunValues("nonsense") is not null || RunValues("shape=99; v1=1") is not null || RunValues("shape=1; x=" + new string('9', 40)) is not null) Fail("自定义轮廓或无效记录不应能载入");

            var panel = new XYPreviewPanel();
            int requests = 0;
            panel.PointMoveRequested += _ => requests++; panel.TrajectoryRequested += _ => requests++; panel.ExecuteRequested += () => requests++;
            string[] machine = [panel.l1.Text, panel.l2.Text, panel.min1.Text, panel.max1.Text, panel.min2.Text, panel.max2.Text, panel.elbowMin.Text, panel.elbowMax.Text];
            string[] Machine() => [panel.l1.Text, panel.l2.Text, panel.min1.Text, panel.max1.Text, panel.min2.Text, panel.max2.Text, panel.elbowMin.Text, panel.elbowMax.Text];
            panel.RefreshRecentRuns();
            if (panel.recentRows.Count != 0 || !panel.recentNote.Text.Contains("还没有")) Fail("没有记录时应说明：" + panel.recentNote.Text);
            // Two recordings: a rectangle with machine settings that must not be loaded, then (newer) a custom contour.
            const string settings = "shape=2; branch=0; L1=999; L2=176.38; x=340; y=0; a=10; b=6; feed=1; v1=6.5; v2=3; a1=10; a2=6; min1=-1; max1=1; min2=-1; max2=1; emin=-1; emax=1; c=; rot=20; scale=1.2; dir=cw; custom_chars=0";
            ExperimentArchiveTests.WriteSyntheticRun(RunsRootOverride, DrawingShape.Rectangle, 8200, settings, out _);
            Thread.Sleep(15);
            ExperimentArchiveTests.WriteSyntheticRun(RunsRootOverride, DrawingShape.Circle, 8200, settings.Replace("shape=2", "shape=10"), out _);
            panel.RefreshRecentRuns();
            var rows = panel.recentRows.ToList();
            if (rows.Count != 2 || !panel.recentNote.Text.Contains("不会运动")) Fail("应列出两条记录并说明点击只载入");
            if (rows[0].Load.IsEnabled || !rows[1].Load.IsEnabled) Fail("最新的自定义轮廓不能载入，矩形应能载入");
            if (rows[0].Load.ToolTip is not string tip || !tip.Contains("轮廓")) Fail("不能载入的记录应说明原因");
            rows[1].Load.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
            if (panel.shape.SelectedIndex != 2 || panel.a.Text != "10" || panel.c.Text != "" || panel.rotation.Text != "20" || panel.scale.Text != "1.2" || panel.direction.SelectedIndex != 1 || panel.v1.Text != "6.5") Fail("载入应还原图形、尺寸、变换和速度");
            if (panel.parametersDirty || panel.execution is null) Fail("载入后应已检查通过：" + panel.result.Text);
            if (!Machine().SequenceEqual(machine)) Fail("不得改动杆长或关节/肘范围");
            if (!panel.recentNote.Text.Contains("没有运动")) Fail("应说明只载入和检查");
            if (requests != 0) Fail("载入记录不得请求任何运动");
            // While the arm moves the list is locked with the other fields.
            panel.SetEditingAllowed(false);
            if (panel.recentHost.IsEnabled) Fail("运动中不应能载入记录");
            panel.SetEditingAllowed(true);
            // A star means "stored on the board".  Without an archive-capable device every star is off, and says so.
            if (rows.Any(r => r.Star.IsEnabled || !((string)r.Star.ToolTip).Contains("板上存档"))) Fail("没有存档能力时星标应不可点并说明原因");
            SuppressMenus = true;
            var savedRequests = new List<(ExecutionTrajectory Plan, int Slot)>(); var erased = new List<int>();
            static BoardSlot Held(int slot, uint crc, int nodes = 20, int ms = 8200) => new(slot, true, 1, nodes, ms, 0, 0, crc);
            static BoardSlot Free(int slot) => new(slot, false, 0, 0, 0, 0, 0, 0);
            Thread.Sleep(15);
            ExperimentArchiveTests.WriteSyntheticRun(RunsRootOverride, DrawingShape.Circle, 8200, settings.Replace("shape=2", "shape=3"), out _);
            var star = new XYPreviewPanel();
            star.SaveToBoardRequested += (plan, slot) => savedRequests.Add((plan, slot)); star.EraseBoardRequested += erased.Add;
            star.RefreshRecentRuns();
            if (star.recentRows.Count != 3) Fail("应有圆、自定义轮廓和矩形三条记录：" + star.recentRows.Count);
            // The board holds the rectangle: this computer recorded the settings that made it, and the device reports the same plan.
            var ledger = new List<LedgerEntry> { new(1, 0xABCD1234, settings) };
            star.SetBoard(4, true, false, [Free(0), Held(1, 0xABCD1234), null, null], ledger, null);
            string[] stars = star.recentRows.Select(r => (string)r.Star.Content).ToArray();
            if (!stars.SequenceEqual(["★", "☆", "☆"]) || !star.recentRows[0].Star.IsEnabled || !((string)star.recentRows[0].Star.ToolTip).Contains("槽2")) Fail("存在板上的图形应排在最前并标 ★：" + string.Join("", stars));
            if (!star.recentRows[1].Star.IsEnabled || star.recentRows[2].Star.IsEnabled) Fail("可载入的图形能存，自定义轮廓不能存");
            if (!star.boardNote.Text.Contains("槽2：矩形") || !star.boardNote.Text.Contains("槽1：空")) Fail("应列出每个槽：" + star.boardNote.Text);
            // A stale record (the device now reports another plan in that slot) is not a star.
            star.SetBoard(4, true, false, [Free(0), Held(1, 0x11111111), null, null], ledger, null);
            if (star.recentRows.Any(r => (string)r.Star.Content == "★") || !star.boardNote.Text.Contains("别处存的图形")) Fail("设备上的计划与记录不符时不应标 ★，而应说明是别处存的");
            star.SetBoard(4, true, false, [Free(0), Held(1, 0xABCD1234), null, null], ledger, null);
            // A slot this computer has no record of (saved elsewhere, or the record is lost) still gets its own clear button, with the same confirmation.
            star.SetBoard(4, true, false, [Free(0), Held(1, 0x11111111), null, null], ledger, null);
            if (star.BoardClearButtons.Count != 1 || !((string)star.BoardClearButtons[0].Content).Contains("槽2")) Fail("别处存的槽应有清除按钮：" + star.BoardClearButtons.Count);
            star.BoardClearButtons[0].RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
            if (erased.Count != 0 || LastMenu is not { Items.Count: 1 } unknownClear) Fail("清除按钮也应先确认");
            ((MenuItem)unknownClear.Items[0]!).RaiseEvent(new RoutedEventArgs(MenuItem.ClickEvent));
            if (!erased.SequenceEqual([1])) Fail("确认后应请求清除槽 2"); erased.Clear();
            star.SetBoard(4, true, true, [Free(0), Held(1, 0x11111111), null, null], ledger, null);
            if (star.BoardClearButtons.Any(b => b.IsEnabled)) Fail("存档进行中清除按钮应不可点");
            star.SetBoard(4, false, false, [Free(0), Held(1, 0x11111111), null, null], ledger, null);
            if (star.BoardClearButtons.Count != 0) Fail("没读全板上存档时不应显示清除按钮");
            star.SetBoard(4, true, false, [Free(0), Held(1, 0xABCD1234), null, null], ledger, null);
            // ★ asks first (one menu item), then clears that slot.
            star.recentRows[0].Star.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
            if (erased.Count != 0 || LastMenu is not { Items.Count: 1 } clear || ((MenuItem)clear.Items[0]!).Header is not string clearText || !clearText.Contains("槽2")) Fail("点 ★ 应先出一个确认菜单项");
            ((MenuItem)clear.Items[0]!).RaiseEvent(new RoutedEventArgs(MenuItem.ClickEvent));
            if (!erased.SequenceEqual([1])) Fail("确认后应请求清除槽 2");
            // ☆ loads the run's figure, checks it and asks for it to be saved into the first free slot (slot 1 here).
            star.recentRows[1].Star.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
            if (savedRequests.Count != 1 || savedRequests[0].Slot != 0 || star.shape.SelectedIndex != 3 || savedRequests[0].Plan != star.execution || !star.recentNote.Text.Contains("槽1")) Fail("点 ☆ 应载入图形并请求存入第一个空槽");
            if (requests != 0) Fail("存到板子的请求不得驱动手臂运动");
            // With every slot taken the operator picks which one to replace.
            star.SetBoard(4, true, false, [Held(0, 1), Held(1, 0xABCD1234), Held(2, 2), Held(3, 3)], ledger, null);
            savedRequests.Clear();
            star.recentRows[1].Star.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
            if (savedRequests.Count != 0 || LastMenu is not { Items.Count: 4 } pick || ((MenuItem)pick.Items[2]!).Header is not string pickText || !pickText.Contains("替换槽3")) Fail("槽都满了应让操作者选替换哪一个");
            ((MenuItem)pick.Items[2]!).RaiseEvent(new RoutedEventArgs(MenuItem.ClickEvent));
            if (savedRequests.Count != 1 || savedRequests[0].Slot != 2) Fail("选了槽 3 就应请求存入槽 3");
            // While the device writes or moves, the stars are off.
            star.SetBoard(4, true, true, [Free(0), Held(1, 0xABCD1234), null, null], ledger, null);
            if (star.recentRows.Any(r => r.Star.IsEnabled)) Fail("存档进行中星标应不可点");
            // Until every slot has been reported nothing is assumed empty: the stars stay off, and say why.
            star.SetBoard(4, false, false, [null, null, null, null], ledger, null);
            if (star.recentRows.Any(r => r.Star.IsEnabled) || !((string)star.recentRows[1].Star.ToolTip).Contains("读取") || !star.BoardNoteText.Contains("读取")) Fail("没读全板上存档时不应能存");
            // Only the slots the device advertises exist: with two, both held, the menu offers two and the summary lists two.
            star.SetBoard(2, true, false, [Held(0, 1), Held(1, 0xABCD1234), null, null], ledger, null);
            savedRequests.Clear();
            star.recentRows[1].Star.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
            if (LastMenu is not { Items.Count: 2 } two || savedRequests.Count != 0 || star.BoardNoteText.Contains("槽3")) Fail("只有两个槽时菜单和提示行都应只有两个");
            ((MenuItem)two.Items[0]!).RaiseEvent(new RoutedEventArgs(MenuItem.ClickEvent));
            if (savedRequests.Count != 1 || savedRequests[0].Slot != 0) Fail("选了槽 1 就应请求存入槽 1");
            star.SetBoard(4, true, false, [Free(0), Held(1, 0xABCD1234), null, null], ledger, null);
            // The Upload button says when the figure on screen is already on the board.
            var onScreen = star.execution ?? throw new InvalidOperationException("应有已检查的图形");
            if ((string)star.upload.Content != "上传图形") Fail("板上没有这个图形时按钮应是上传图形：" + star.upload.Content);
            star.SetBoard(4, true, false, [Free(0), Held(1, TrajectoryArchive.PlanCrc(onScreen.Nodes), onScreen.Nodes.Length, (int)onScreen.Nodes.Sum(n => (int)n.Milliseconds)), null, null], ledger, null);
            if ((string)star.upload.Content != "从板上加载") Fail("板上已有这个图形时按钮应是从板上加载：" + star.upload.Content);
            star.SetBoard(4, true, false, [Free(0), Held(1, TrajectoryArchive.PlanCrc(onScreen.Nodes), onScreen.Nodes.Length + 1, 1), null, null], ledger, null);
            if ((string)star.upload.Content != "上传图形") Fail("节点数或时长不同就不是同一个计划");
            SuppressMenus = false;
            var broken = new XYPreviewPanel(); broken.RefreshRecentRuns();
            if (broken.recentRows.Count != 3 || broken.recentRows.Any(r => (string)r.Star.Content == "★")) Fail("没有板上记录时不应有 ★");
            // The same figure run again (only the speed differs) stays one row, showing the newest run; a different figure is a new row.
            Thread.Sleep(15);
            ExperimentArchiveTests.WriteSyntheticRun(RunsRootOverride, DrawingShape.Rectangle, 8200, settings.Replace("v1=6.5", "v1=3"), out _);
            broken.RefreshRecentRuns();
            if (broken.recentRows.Count != 3 || !((TextBlock)broken.recentRows[0].Load.Content).Text.Contains("v3/3")) Fail("同一个图形重复运行应只占一行并显示最新一次");
            Thread.Sleep(15);
            ExperimentArchiveTests.WriteSyntheticRun(RunsRootOverride, DrawingShape.Rectangle, 8200, settings.Replace("a=10", "a=12"), out _);
            broken.RefreshRecentRuns();
            if (broken.recentRows.Count != 4) Fail("尺寸不同就是另一个图形，应新增一行");

            // The calibration wizard may put measured link lengths and an elbow range into the boxes: the preview is re-planned from them, nothing moves.
            var before = panel.execution ?? throw new InvalidOperationException("填杆长前应有已检查的图形");
            panel.SetLinkLengths(181.5, 175.25); panel.SetElbowRange(-147, 155);
            // The preview is recomputed from the new numbers (same as typing them), so the plan on screen has different joint angles and
            // would no longer match one already uploaded.
            if (panel.Model.L1 != 181.5 || panel.Model.L2 != 175.25 || panel.RequestedLimits is not { ElbowMin: -147, ElbowMax: 155 } || panel.execution is not { } after || after == before || after.Nodes[0].Pose == before.Nodes[0].Pose) Fail("向导填入的杆长/肘范围应进入规划参数，并按新数值重新规划");
            if (requests != 0) Fail("向导填入参数不得请求任何运动");

            // The speed boxes show the capped value once edited: 99 °/s becomes the slider's top, a value under the cap is left alone.
            var capped = new XYPreviewPanel();
            capped.v1.Text = "99"; capped.v2.Text = "5"; capped.acc1.Text = "999"; capped.v1.RaiseEvent(new RoutedEventArgs(UIElement.LostFocusEvent));
            if (capped.v1.Text != Mm(Math.Floor(RateCap(0) * 10) / 10) || capped.v2.Text != "5" || capped.acc1.Text != Mm(Math.Floor(AccelCap(0) * 10) / 10)) Fail("超过上限的电机速度/加速度应在失焦后改回上限：" + capped.v1.Text + " / " + capped.v2.Text + " / " + capped.acc1.Text);

            // The lamps. Nothing saved yet: unsaved. After a save the group is saved; editing one group only un-saves that group.
            var p = new XYPreviewPanel();
            string Lamps(int[] group) => string.Join("|", p.lampRows.First(r => ReferenceEquals(r.Group, group)).Lamps.Select(l => l.Text));
            if (!Lamps(SpeedGroup).StartsWith("○ 未保存")) Fail("没有保存过时应显示未保存：" + Lamps(SpeedGroup));
            string file = Path.Combine(temp, "parameters.json");
            p.SaveCurrentParameters(file);
            if (!Lamps(SpeedGroup).StartsWith("● 已保存到电脑") || !Lamps(RangeGroup).StartsWith("● 已保存到电脑") || !Lamps(ShapeGroup).StartsWith("● 已保存到电脑")) Fail("保存后各组应显示已保存");
            p.v1.Text = "4";
            if (!Lamps(SpeedGroup).StartsWith("○ 未保存") || !Lamps(RangeGroup).StartsWith("● 已保存到电脑")) Fail("改速度只应让速度组变成未保存：" + Lamps(SpeedGroup));
            p.v1.Text = "5";
            if (!Lamps(SpeedGroup).StartsWith("● 已保存到电脑")) Fail("改回原值应又是已保存");
            p.rotation.Text = "15";
            if (!Lamps(ShapeGroup).StartsWith("○ 未保存")) Fail("改旋转角应让图形组未保存");
            p.rotation.Text = "0"; p.Refresh();
            // The device: ranges and speeds are confirmed separately, which is what a speed edit needs to show.
            p.SetParameterConfirmation(true, true);
            if (!Lamps(SpeedGroup).Contains("● 设备已确认") || !Lamps(RangeGroup).Contains("● 设备已确认")) Fail("两者都确认时应显示设备已确认");
            p.SetParameterConfirmation(true, false);
            if (!Lamps(SpeedGroup).Contains("○ 设备未确认") || !Lamps(RangeGroup).Contains("● 设备已确认")) Fail("只有速度未确认时只应标速度组：" + Lamps(SpeedGroup));
            if (Lamps(ShapeGroup).Split('|').Length != 2) Fail("图形组与设备无关，不应有设备一栏");
            // The plan on the device: used while it is the plan on screen, "differs" as soon as an edit changes the plan.
            if (!Lamps(SpeedGroup).EndsWith("– 设备上没有图形")) Fail("设备上没有图形时应说明：" + Lamps(SpeedGroup));
            p.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, p.execution!.Nodes[0].Pose), ExecutionTrajectory.Counts(1, p.execution.Nodes[0].Pose), 0, 0, 0, 0));
            p.SetPointMoveState(true, p.RequestedLimits, "ok", true, true, p.execution, false);
            if (!Lamps(SpeedGroup).EndsWith("● 已用于上传的图形") || !Lamps(ShapeGroup).EndsWith("● 已用于上传的图形")) Fail("设备持有的就是屏幕上的图形时应显示已用于：" + Lamps(SpeedGroup));
            p.v1.Text = "1"; p.v2.Text = "1"; p.Refresh();   // slow enough to change the plan itself (a small edit can leave it identical)
            if (!Lamps(SpeedGroup).EndsWith("○ 与设备上的图形不同") || !Lamps(RangeGroup).EndsWith("○ 与设备上的图形不同")) Fail("改速度后重新检查，设备上的图形就和屏幕上的不同了：" + Lamps(SpeedGroup));
            // Every lamp has an explanation for the pointer.
            if (p.lampRows.Any(r => r.Lamps.Any(l => l.ToolTip is not string tip || tip.Length < 10))) Fail("每个状态灯都应有说明");
        }
        finally { RunsRootOverride = null; SuppressMenus = false; if (Directory.Exists(temp)) Directory.Delete(temp, true); }
    }
}
