using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Media;
using System.Windows.Shapes;

namespace TwoLinkCommissioning;

// The board's flash archive in the panel: a star on a recent run means "this figure is stored on the board".  ☆ puts the
// run's figure there (load it, upload it, save it into a free slot, or into the slot the operator picks when all are used);
// ★ takes it off again.  A figure on the board is loaded from its slot instead of uploaded: the Upload button says so.
// Nothing here moves the arm or talks to the device; the window does that and reports the slots back through SetBoard.
public sealed partial class XYPreviewPanel
{
    public event Action<ExecutionTrajectory, int>? SaveToBoardRequested;   // plan, slot: upload it if the device does not hold it, then save
    public event Action<int>? EraseBoardRequested;
    public event Action<int>? BoardPathRequested;                          // read the slot's nodes back so its real path can be shown

    private int boardSlotCount;                    // the slots the device advertises (traj_store=<n>); 0 = no archive
    private bool boardCanRead;                    // the firmware sends a slot's nodes back
    private bool boardKnown, boardBusy;            // the inventory is complete; a save/erase/upload-then-save is in progress
    private bool boardSupported => boardSlotCount > 0;
    private IReadOnlyList<BoardSlot?> boardSlots = new BoardSlot?[DeviceContract.TrajStoreSlots];
    private IReadOnlyList<LedgerEntry> boardLedger = [];
    private Func<string>? boardMessage;
    private BoardSlot? planOnBoard;                    // the slot that holds exactly the plan on screen
    internal static bool SuppressMenus;                // tests drive the menu items without opening a popup
    internal static ContextMenu? LastMenu;
    private static void ShowMenu(ContextMenu menu) { LastMenu = menu; if (!SuppressMenus) menu.IsOpen = true; }
    internal string BoardNoteText => boardNote.Text;
    internal string ResultText => result.Text;
    internal bool BoardCanRead => boardCanRead;
    internal bool SaveBoardEnabled => saveBoard.IsEnabled;
    internal string SaveBoardCaption => (string)saveBoard.Content;
    internal string SaveBoardTip => saveBoard.ToolTip as string ?? "";
    internal void ClickSaveBoardForSelfTest() => saveBoard.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
    internal string RecentNoteText => recentNote.Text;
    internal string ActionSummaryText => actionSummary.Text;
    internal void SetSpeedsForSelfTest(string first, string second) { v1.Text = first; v2.Text = second; }
    internal Button? BoardRowLoadButton(int index) => index >= 0 && index < recentRows.Count ? recentRows[index].Load : null;
    internal void ShowBoardPathForSelfTest(int slot, ExecutionNode[] nodes, string? settings) { boardPathWanted = slot; ShowBoardPath(slot, nodes, settings); }
    internal string RecentListText => string.Join("\n", recentList.Children.Cast<UIElement>().Select(c => c is TextBlock t ? t.Text : c is DockPanel d ? string.Join(" ", d.Children.OfType<Button>().Select(b => b.Content as TextBlock is { } label ? label.Text : b.Content?.ToString())) : ""));
    // One clear button per stored slot, so a figure saved from another computer (or whose record was lost) can be cleared too.
    private readonly WrapPanel boardClear = new() { Margin = new Thickness(10, 0, 10, 6) };
    private readonly ContentControl boardClearHost = new() { IsTabStop = false, Focusable = false };   // locked with the other edit controls while the arm moves
    internal IReadOnlyList<Button> BoardClearButtons => boardClear.Children.OfType<Button>().ToList();
    private readonly TextBlock boardNote = new() { TextWrapping = TextWrapping.Wrap, FontSize = 11, Foreground = WorkbenchTheme.Muted, Margin = new Thickness(10, 2, 10, 6) };

    // known: every slot has been reported on this link.  Until then no slot is assumed empty, and saving stays off.
    internal int BoardRebuilds;                    // how often the board state really changed (the controls are rebuilt only then)
    public void SetBoard(int slotCount, bool known, bool busy, IReadOnlyList<BoardSlot?> slots, IReadOnlyList<LedgerEntry> ledger, Func<string>? message, bool canReadNodes = false)
    {
        // The device repeats its LINK line every 500 ms: unchanged state must not tear down the buttons the operator is using
        // (a confirmation menu opened from one of them would be lost).
        slotCount = Math.Clamp(slotCount, 0, DeviceContract.TrajStoreSlots);
        var newSlots = slots.ToArray(); var newLedger = ledger.ToList();
        if (slotCount == boardSlotCount && canReadNodes == boardCanRead && known == boardKnown && busy == boardBusy && newSlots.SequenceEqual(boardSlots) && newLedger.SequenceEqual(boardLedger) && ReferenceEquals(message, boardMessage)) return;
        BoardRebuilds++;
        boardSlotCount = slotCount; boardCanRead = canReadNodes; boardKnown = known; boardBusy = busy; boardSlots = newSlots; boardLedger = newLedger; boardMessage = message;
        ShowRecentRuns();
        UpdateBoardNote();
        UpdateBoardClear();
        UpdatePointButton();
    }

    private void UpdateBoardClear()
    {
        boardClear.Children.Clear();
        if (!boardSupported || !boardKnown) return;
        foreach (var slot in boardSlots.Take(boardSlotCount).OfType<BoardSlot>().Where(s => s.Valid))
        {
            var button = new Button { Content = Lang.T("board.clearButton", slot.Slot + 1), ToolTip = Lang.T("board.clearButton.tip", slot.Slot + 1),
                Margin = new Thickness(0, 0, 6, 0), Padding = new Thickness(8, 1, 8, 1), IsEnabled = !boardBusy };
            ToolTipService.SetShowOnDisabled(button, true);
            button.Click += (_, _) => { if (button.IsEnabled) StoredStarClicked(button, slot); };
            boardClear.Children.Add(button);
        }
    }

    // The slot holding the plan on screen, cached per plan and slot contents: the check is a CRC over the node bytes.
    private (ExecutionTrajectory? Plan, IReadOnlyList<BoardSlot?> Slots, BoardSlot? Found) boardLookup;
    private BoardSlot? PlanOnBoard()
    {
        if (!boardSupported || !boardKnown || execution is null) return null;   // a half-read inventory is not used to decide "already on the board"
        if (!ReferenceEquals(boardLookup.Plan, execution) || !ReferenceEquals(boardLookup.Slots, boardSlots))
            boardLookup = (execution, boardSlots, TrajectoryArchive.SlotFor(boardSlots, execution.Nodes));
        return boardLookup.Found;
    }

    private static bool SameFigure(IReadOnlyDictionary<string, string> a, IReadOnlyDictionary<string, string> b) =>
        PresetKeys.All(key => string.Equals(a.GetValueOrDefault(key), b.GetValueOrDefault(key), StringComparison.Ordinal));

    // The slot a recent run is stored in: this computer recorded those settings for it and the device still reports that plan.
    private BoardSlot? StarSlot(IReadOnlyDictionary<string, string>? values)
    {
        if (!boardSupported || values is null) return null;
        foreach (var slot in boardSlots)
            if (BoardLedger.For(slot, boardLedger) is { } entry && RunValues(entry.Settings) is { } stored && SameFigure(stored, values)) return slot;
        return null;
    }

    private string SlotLabel(BoardSlot? slot)
    {
        if (slot is not { Valid: true }) return Lang.T("board.empty");
        string size = Lang.T("board.size", slot.Nodes, slot.Seconds);
        if (BoardLedger.For(slot, boardLedger) is not { } entry) return Lang.T("board.unknown") + size;
        var values = RunValues(entry.Settings);
        string name = values is not null && int.TryParse(values["shape"], out int index) && index is >= 0 and <= 10 ? Lang.T("xy.shape" + index) : Lang.T("board.custom");
        return name + size;
    }

    private void UpdateBoardNote()
    {
        Lang.Text(boardNote, () =>
        {
            if (!boardSupported) return Lang.T("board.unsupported");
            if (!boardKnown) return Lang.T("board.reading") + (boardMessage is { } pending ? "\n" + pending() : "");
            string slots = string.Join("\n", Enumerable.Range(0, boardSlotCount).Select(i => Lang.T("board.slot", i + 1, SlotLabel(boardSlots.ElementAtOrDefault(i)))));
            return Lang.T("board.summary", slots) + (boardMessage is { } message ? "\n" + message() : "");
        });
    }

    // ☆ and Save To Board: the figure goes to the first free slot; with none free the operator chooses which one to replace.
    private void StarClicked(Button star, ExperimentRun run, Dictionary<string, string> values) => ChooseBoardSlot(star, slot => SaveRunToBoard(run, values, slot));
    private void ChooseBoardSlot(Button anchor, Action<int> save)
    {
        if (TrajectoryArchive.FreeSlot(boardSlots, boardSlotCount) is { } free) { save(free); return; }
        var menu = new ContextMenu { PlacementTarget = anchor };
        for (int i = 0; i < boardSlotCount; i++)
        {
            int slot = i;
            var item = new MenuItem { Header = Lang.T("board.replace", slot + 1, SlotLabel(boardSlots.ElementAtOrDefault(slot))) };
            item.Click += (_, _) => save(slot);
            menu.Items.Add(item);
        }
        ShowMenu(menu);
    }

    // The figure on screen, stored without having to run it first: the board keeps a plan the device holds (uploaded or run), so the
    // window uploads it first when the device does not hold it yet, then saves.  Only a figure that passed its checks is offered, and
    // one the board already has is not offered again.
    private void SaveCurrentToBoard()
    {
        if (execution is not { } plan) return;
        ChooseBoardSlot(saveBoard, slot =>
        {
            Lang.Text(recentNote, () => Lang.T("board.saveCurrentStarted", slot + 1));
            SaveToBoardRequested?.Invoke(plan, slot);
        });
    }

    private void UpdateSaveButton(bool held)
    {
        string? why = !boardSupported ? Lang.T("board.star.unavailable") : !boardKnown ? Lang.T("board.star.reading") : boardBusy ? Lang.T("board.err.busy") :
            execution is null ? Lang.T("board.save.noPlan") : PlanOnBoard() is { } stored ? Lang.T("board.save.stored", stored.Slot + 1) :
            !held && !upload.IsEnabled ? Lang.T("board.save.needStart") : null;   // a plan the device holds is saved as it is; any other must be uploaded first
        saveBoard.IsEnabled = why is null;
        SetTip(saveBoard, why ?? Lang.T("board.save.tip"));
    }

    // ★: one menu item, so a stray click clears nothing.
    private void StoredStarClicked(Button star, BoardSlot slot)
    {
        var item = new MenuItem { Header = Lang.T("board.clear", slot.Slot + 1) };
        item.Click += (_, _) => EraseBoardRequested?.Invoke(slot.Slot);
        ShowMenu(new ContextMenu { PlacementTarget = star, Items = { item } });
    }

    // Loads the run's figure exactly as clicking the run would, then asks for it to be saved; the window uploads it first if the
    // device does not hold it.  A figure that fails its checks is not sent anywhere.
    private void SaveRunToBoard(ExperimentRun run, Dictionary<string, string> values, int slot)
    {
        ApplyPreset(values);
        if (execution is not { } plan) { Lang.Text(recentNote, () => Lang.T("board.err.plan") + result.Text); return; }
        Lang.Text(recentNote, () => Lang.T("board.saveStarted", RunSummary(run), slot + 1));
        SaveToBoardRequested?.Invoke(plan, slot);
    }

    // ---- what the board really holds -------------------------------------------------------------------------------------
    // The nodes read back from a slot, drawn (and played back) in place of the plan made on this computer.  It lasts until the next
    // preview or edit.  The link lengths are the ones saved with the plan when the board has them, so the picture is the board's, not
    // this computer's re-planning.  When the nodes also pass this machine's joint ranges and speeds they become the plan on screen
    // (execution): moved to the start, loaded from the slot (the CRC is the slot's) and run like any other, by the operator's clicks.
    // When they do not, nothing here can be uploaded or run.
    private (int Slot, ExecutionNode[] Nodes, PlanarArm Model, Point[] Path, double Duration)? boardView;
    private int? boardPathWanted;                  // the slot whose nodes were asked for; an edit in the meantime cancels it
    internal bool ShowsBoardPath => boardView is not null;
    internal double BoardPathDuration => boardView?.Duration ?? 0;
    private double? PlaybackLength => timing?.Duration ?? boardView?.Duration;
    private int? execFromBoard;                // the slot the plan on screen was read back from (null: made here)
    private void ClearBoardView() { boardView = null; boardPathWanted = null; execFromBoard = null; }

    private static double? SavedNumber(string? settings, string key) =>
        settings?.Split(';', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries).Select(part => part.Split('=', 2)).FirstOrDefault(kv => kv.Length == 2 && kv[0].Trim() == key) is { } found &&
        double.TryParse(found[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double value) && double.IsFinite(value) && value > 0 ? value : null;

    public void ShowBoardPath(int slot, ExecutionNode[] nodes, string? settings)
    {
        if (boardPathWanted != slot || nodes.Length < 2) return;
        CancelFreehandWork();   // a smoothing or planning job still running must not publish over the board's plan
        double? saved1 = SavedNumber(settings, "L1"), saved2 = SavedNumber(settings, "L2");
        if (saved1 is null || saved2 is null) saved1 = saved2 = null;   // the board's lengths are a pair, or not used at all
        PlanarArm? here = null, model = null; ArgumentException? localError = null;
        try { here = Model; } catch (ArgumentException ex) { localError = ex; }   // a half-typed link length in the boxes
        try { if (saved1 is { } a && saved2 is { } b) model = new PlanarArm(a, b); } catch (ArgumentException) { saved1 = saved2 = null; }
        if ((model ?? here) is not { } arm)
        {
            // Nothing to draw it with: the board saved no (usable) link lengths and the boxes here are not numbers.  Say so.
            boardPathWanted = null;
            Lang.Text(recentNote, () => Lang.T("recent.boardNoModel", slot + 1) + (localError?.Message ?? ""));
            return;
        }
        PausePlayback(); trajectory = null; timing = null; execution = null; pointTarget = null; execFromBoard = null; movePoint.IsEnabled = false; upload.IsEnabled = false;
        rangeFailure = null; operationError = null; Lang.Text(executionInfo, () => ""); Lang.Text(notice, () => ""); sampleInfo.Clear();
        // Does this machine accept it?  The checks a planned figure passes: the joint ranges and speeds in the boxes.  A slot saved under
        // other settings is shown but not offered; the device checks it again when it takes the plan.
        ArgumentException? whyNot = null; ExecutionTrajectory? plan = null; JointLimits? range = null;
        try
        {
            range = PreviewLimits();
            _ = Model;   // the experiment records the plan with this computer's link lengths: a half-typed one means it cannot be run, whatever the board saved
            plan = RequestedProfile is { } profile ? ExecutionTrajectory.FromNodes(nodes, range.Value, profile) : throw Lang.Error("err.planRates");
        }
        catch (ArgumentException ex) { whyNot = ex; plan = null; }   // kept as the exception: its message follows the language
        double duration = nodes.Sum(n => n.Milliseconds) / 1000.0;
        boardView = (slot, nodes, arm, nodes.Select(n => arm.Forward(n.Pose).Tip).ToArray(), duration); boardPathWanted = null;
        int number = slot + 1;
        var start = nodes[0].Pose;
        double here1 = here?.L1 ?? arm.L1, here2 = here?.L2 ?? arm.L2;
        double? kept1 = saved1, kept2 = saved2;   // the text is built when it is shown, so it follows the language
        string Links() => kept1 is null ? Lang.T("plan.boardLinksUnknown", here1, here2) :
            Math.Abs(kept1.Value - here1) < 0.005 && Math.Abs(kept2!.Value - here2) < 0.005 ? Lang.T("plan.boardLinksSame", kept1.Value, kept2!.Value) :
            Lang.T("plan.boardLinksDiffer", kept1.Value, kept2!.Value, here1, here2);
        result.Foreground = Brushes.DarkSlateGray;
        bool runnable = plan is not null;
        Lang.Text(result, () => Lang.T(runnable ? "plan.boardViewRunnable" : "plan.boardView", number, nodes.Length, duration, start.Q1, start.Q2) + Links());
        if (plan is not null)
        {
            execution = plan; planningLimits = range; pointTarget = PtpProtocol.Quantize(nodes[0].Pose); execFromBoard = slot;
            var (peak1, peak2) = plan.PeakRates();
            Lang.Text(executionInfo, () => Lang.T("plan.execInfo", plan.Nodes.Length, plan.Duration, DeviceContract.PlanRateMax, peak1, peak2));
            Lang.Text(recentNote, () => Lang.T("recent.boardRunnable", number));
        }
        else Lang.Text(recentNote, () => Lang.T("recent.boardShown", number) + Lang.T("recent.boardNotRunnable", whyNot?.Message ?? ""));
        cursor.Maximum = duration; cursor.Value = 0; offlineSim = false; cameraReady = false; autoFit = true;
        export.IsEnabled = false; cursor.IsEnabled = play.IsEnabled = rewind.IsEnabled = editingAllowed;
        UpdatePointButton(); DrawBoardSample();
    }

    // The pose of the plan `seconds` in: the executor moves each joint in a straight line between consecutive nodes.
    internal static JointPose NodePoseAt(ExecutionNode[] nodes, double seconds)
    {
        double elapsed = 0;
        for (int i = 1; i < nodes.Length; i++)
        {
            double dt = nodes[i].Milliseconds / 1000.0;
            if (seconds <= elapsed + dt)
            {
                double u = dt <= 0 ? 1 : Math.Clamp((seconds - elapsed) / dt, 0, 1);
                var from = nodes[i - 1].Pose; var to = nodes[i].Pose;
                return new JointPose(from.Q1 + (to.Q1 - from.Q1) * u, from.Q2 + (to.Q2 - from.Q2) * u);
            }
            elapsed += dt;
        }
        return nodes[^1].Pose;
    }

    private void DrawBoardSample()
    {
        if (boardView is not { } view) return;
        var pose = NodePoseAt(view.Nodes, cursor.Value); var tip = view.Model.Forward(pose).Tip;
        canvas.Children.Clear();
        Draw(view.Model, pose, tip.X, tip.Y, true, true);
        sampleInfo.Text = Lang.T("vp.boardTime", cursor.Value, view.Duration) + PoseText(pose, tip);
    }

    private void DrawBoardPath(Canvas plot, Point[] path)
    {
        var line = new Polyline { Stroke = WorkbenchTheme.Teal, StrokeThickness = 2.4, StrokeDashArray = new DoubleCollection([5, 3]), IsHitTestVisible = false };
        foreach (var p in path) line.Points.Add(ToScreen(p));
        plot.Children.Add(line);
        var start = ToScreen(path[0]); var finish = ToScreen(path[^1]);
        foreach (var (spot, filled) in new[] { (start, false), (finish, true) })
        {
            var dot = new Ellipse { Width = 10, Height = 10, Stroke = WorkbenchTheme.Teal, Fill = filled ? WorkbenchTheme.Teal : Brushes.White, StrokeThickness = 1.5, IsHitTestVisible = false };
            Canvas.SetLeft(dot, spot.X - 5); Canvas.SetTop(dot, spot.Y - 5); plot.Children.Add(dot);
        }
        var label = new TextBlock { Text = Lang.T("vp.boardPath"), Foreground = WorkbenchTheme.Teal, FontSize = 11, IsHitTestVisible = false };
        Canvas.SetLeft(label, finish.X + 9); Canvas.SetTop(label, finish.Y + 6); plot.Children.Add(label);
    }

    // The Save To Board button: when it is offered and why not, what a click asks the window for, and the slot choice when all are used.
    private static void SaveButtonSelfTest()
    {
        static void Fail(string message) => throw new InvalidOperationException("存到板子按钮：" + message);
        static BoardSlot Held(int slot, uint crc, int nodes = 20, int ms = 8200) => new(slot, true, 1, nodes, ms, 0, 0, crc);
        static BoardSlot Free(int slot) => new(slot, false, 0, 0, 0, 0, 0, 0);
        SuppressMenus = true;
        try
        {
            var panel = new XYPreviewPanel();
            panel.SelectShapeForSelfTest(3); panel.LoadExampleForSelfTest();
            var plan = panel.execution ?? throw new InvalidOperationException("应有已检查的图形");
            var saved = new List<(ExecutionTrajectory Plan, int Slot)>(); panel.SaveToBoardRequested += (p, slot) => saved.Add((p, slot));
            // Why it is off, one reason at a time.
            if (panel.SaveBoardEnabled || !panel.SaveBoardTip.Contains("板上存档")) Fail("固件不支持板上存档时应不可点并说明原因：" + panel.SaveBoardTip);
            panel.SetBoard(4, false, false, [null, null, null, null], [], null);
            if (panel.SaveBoardEnabled || !panel.SaveBoardTip.Contains("读取")) Fail("没读完板上存档时应不可点：" + panel.SaveBoardTip);
            panel.SetBoard(4, true, true, [Free(0), Free(1), Free(2), Free(3)], [], null);
            if (panel.SaveBoardEnabled) Fail("存档进行中应不可点");
            panel.SetBoard(4, true, false, [Free(0), Free(1), Free(2), Free(3)], [], null);
            if (panel.SaveBoardEnabled || !panel.SaveBoardTip.Contains("上传") || !panel.SaveBoardTip.Contains("手臂在起点")) Fail("设备上没有这个图形、手臂又不在起点时应不可点并说明要先上传：" + panel.SaveBoardTip);
            // The arm at the start of an enabled, homed stage: the figure can be uploaded, so it can be saved -- without ever running.
            panel.SetPointMoveState(true, panel.RequestedLimits, "ok", true, true, null, false);
            panel.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, plan.Nodes[0].Pose), ExecutionTrajectory.Counts(1, plan.Nodes[0].Pose), 0, 0, 0, 0));
            if (!panel.SaveBoardEnabled || panel.SaveBoardCaption != "存到板子" || !panel.SaveBoardTip.Contains("不需要先执行")) Fail("手臂在起点时应可点，并说明不需要先执行：" + panel.SaveBoardTip);
            panel.ClickSaveBoardForSelfTest();
            if (saved.Count != 1 || saved[0].Slot != 0 || !ReferenceEquals(saved[0].Plan, plan) || !panel.recentNote.Text.Contains("槽1")) Fail("应把屏幕上的图形交给第一个空槽");
            // No free slot: the operator picks which one to replace.
            saved.Clear();
            panel.SetBoard(4, true, false, [Held(0, 1), Held(1, 2), Held(2, 3), Held(3, 4)], [], null);
            panel.ClickSaveBoardForSelfTest();
            if (saved.Count != 0 || LastMenu is not { Items.Count: 4 } menu || ((MenuItem)menu.Items[2]!).Header is not string pick || !pick.Contains("替换槽3")) Fail("槽都满了应让操作者选替换哪一个");
            ((MenuItem)menu.Items[2]!).RaiseEvent(new RoutedEventArgs(MenuItem.ClickEvent));
            if (saved.Count != 1 || saved[0].Slot != 2) Fail("选了槽 3 就应请求存入槽 3");
            // Already on the board: not offered again.
            panel.SetBoard(4, true, false, [Free(0), Held(1, TrajectoryArchive.PlanCrc(plan.Nodes), plan.Nodes.Length, (int)plan.Nodes.Sum(n => (int)n.Milliseconds)), null, null], [], null);
            if (panel.SaveBoardEnabled || !panel.SaveBoardTip.Contains("已经存在板子槽2")) Fail("板上已有这个图形时不该再存：" + panel.SaveBoardTip);
            // A plan the device already holds is saved as it is: no start position needed.
            panel.SetBoard(4, true, false, [Free(0), Free(1), Free(2), Free(3)], [], null);
            panel.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, plan.Nodes[0].Pose) + 500, ExecutionTrajectory.Counts(1, plan.Nodes[0].Pose), 0, 0, 0, 0));
            if (panel.SaveBoardEnabled) Fail("不在起点、设备上也没有这个图形时不该能存");
            panel.SetPointMoveState(true, panel.RequestedLimits, "ok", true, true, plan, false);
            if (!panel.SaveBoardEnabled) Fail("设备已经持有这个图形时，不在起点也应能存");
            // No figure that passed its checks: nothing to save.
            panel.x.Text = "9999";
            if (panel.SaveBoardEnabled || !panel.SaveBoardTip.Contains("检查并预览")) Fail("没有检查通过的图形时应不可点：" + panel.SaveBoardTip);
        }
        finally { SuppressMenus = false; }
    }

    // The board's plan as a read-only picture: interpolation, what it switches off, playback, and what cancels it.
    private static void BoardPathSelfTest()
    {
        static void Fail(string message) => throw new InvalidOperationException("板上轨迹预览：" + message);
        var nodes = new[] { new ExecutionNode(new JointPose(0, 0), 0), new ExecutionNode(new JointPose(10, 20), 500), new ExecutionNode(new JointPose(10, 20), 50) };
        var mid = NodePoseAt(nodes, 0.25);
        if (Math.Abs(mid.Q1 - 5) > 1e-9 || Math.Abs(mid.Q2 - 10) > 1e-9 || NodePoseAt(nodes, 0) != nodes[0].Pose || NodePoseAt(nodes, 0.5) != nodes[1].Pose || NodePoseAt(nodes, 99) != nodes[^1].Pose) Fail("节点之间应按直线插值");
        var panel = new XYPreviewPanel();
        string parameters = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "arm-board-path-" + Guid.NewGuid().ToString("N") + ".json");
        ParametersPathOverride = parameters;
        panel.ShowBoardPath(2, nodes, null);
        if (panel.ShowsBoardPath) Fail("没有请求过的轨迹不应显示");
        panel.ShowBoardPathForSelfTest(2, nodes, null);
        if (!panel.ShowsBoardPath || Math.Abs(panel.BoardPathDuration - 0.55) > 1e-9 || panel.execution is not null || panel.upload.IsEnabled || panel.movePoint.IsEnabled || panel.export.IsEnabled ||
            !panel.play.IsEnabled || !panel.rewind.IsEnabled || !panel.cursor.IsEnabled || Math.Abs(panel.cursor.Maximum - 0.55) > 1e-9) Fail("应显示为只读预览：可回放，不能上传、执行或导出");
        if (!panel.ResultText.Contains("板上槽3") || !panel.ResultText.Contains("没有保存杆长") || !panel.recentNote.Text.Contains("只读预览")) Fail("应说明这是板上槽3的只读预览：" + panel.ResultText);
        if (!panel.recentNote.Text.Contains("不能执行") || panel.pointTarget is not null) Fail("这台电脑不接受的轨迹应说明不能执行，也没有起点可移：" + panel.recentNote.Text);
        // Nodes this machine accepts become the plan on screen: moved to the start, loaded and run by the operator's clicks like any other.
        // The experiment says where the plan came from, in a form that cannot be loaded back as a figure.
        var slow = new[] { new ExecutionNode(new JointPose(0, 0), 0), new ExecutionNode(new JointPose(0.5, 1), 500), new ExecutionNode(new JointPose(1, 2), 500), new ExecutionNode(new JointPose(1, 2), 50) };
        panel.ShowBoardPathForSelfTest(1, slow, null);
        if (panel.execution is not { } shown || !shown.Nodes.SequenceEqual(slow) || shown.Timing is not null || panel.pointTarget != slow[0].Pose || !panel.recentNote.Text.Contains("检查通过") || !panel.executionInfo.Text.Contains('4'))
            Fail("这台电脑接受的轨迹应成为屏幕上的计划：" + panel.recentNote.Text + " / " + panel.result.Text);
        if (!panel.SettingsDescription.Contains("shape=10") || !panel.SettingsDescription.Contains("board_slot=2") || RunValues(panel.SettingsDescription) is not null) Fail("实验记录应说明计划来自板上槽2，且不能当图形载回：" + panel.SettingsDescription);
        panel.SetPointMoveState(true, panel.RequestedLimits, "ok", true, true, null, false);
        panel.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, slow[0].Pose), ExecutionTrajectory.Counts(1, slow[0].Pose), 0, 0, 0, 0));
        ExecutionTrajectory? sent = null; panel.TrajectoryRequested += p => sent = p;
        if (!panel.movePoint.IsEnabled || !panel.upload.IsEnabled || panel.run.IsEnabled) Fail("起点在手臂下时应能上传（加载），还没有设备上的计划不能执行：" + panel.moveState.Text);
        panel.upload.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
        if (sent is null || !sent.Nodes.SequenceEqual(slow)) Fail("点上传（加载）应把板上的这条计划交出去");
        // A plan this machine's speeds do not allow stays a picture: no start to move to, nothing to upload.
        panel.ShowBoardPathForSelfTest(1, [new(new(0, 0), 0), new(new(10, 20), 500), new(new(10, 20), 50)], null);
        if (panel.execution is not null || panel.upload.IsEnabled || panel.movePoint.IsEnabled || !panel.actionSummary.Text.Contains("只读预览")) Fail("不被接受的轨迹只能看：" + panel.actionSummary.Text);
        // Only one of the two link lengths (or one that does not parse): not a pair, so none is used, and nothing throws.
        foreach (string partial in new[] { "shape=3; L1=181.5", "shape=3; L2=176.38", "shape=3; L1=oops; L2=176.38" })
        {
            panel.ShowBoardPathForSelfTest(1, nodes, partial);
            if (!panel.ShowsBoardPath || !panel.ResultText.Contains("没有保存杆长")) Fail("只有一个杆长时应按没有保存杆长处理：" + partial + " / " + panel.ResultText);
        }
        // The link lengths in the boxes are not valid numbers: the board's plan can be looked at (drawn with its own lengths) but not run,
        // because the experiment that would record the run needs this computer's model.
        string goodL1 = panel.l1.Text; panel.l1.Text = "abc";
        panel.ShowBoardPathForSelfTest(1, [new(new(0, 0), 0), new(new(0.5, 1), 500), new(new(1, 2), 500), new(new(1, 2), 50)], "shape=3; L1=180; L2=176.38");
        if (!panel.ShowsBoardPath || panel.execution is not null || panel.upload.IsEnabled || !panel.recentNote.Text.Contains("不能执行") || panel.recentNote.Text.Contains(Lang.T("err.planRates"))) Fail("本机杆长无效时板上的计划只能看，原因应是杆长而不是速度：" + panel.recentNote.Text);
        // And when no model at all can draw it (the board saved no lengths): the request ends with an explanation, not silently.
        panel.l1.Text = "abd";   // (an edit takes any board view away)
        panel.ShowBoardPathForSelfTest(1, nodes, "shape=3; L1=180");
        if (panel.ShowsBoardPath || !panel.recentNote.Text.Contains("没有可用的杆长") || panel.boardPathWanted is not null) Fail("没有任何杆长能画时应说明，并结束这次请求：" + panel.recentNote.Text);
        panel.l1.Text = goodL1;
        // Showing the board's plan takes pending freehand work with it.
        int work = panel.freehandWorkVersion;
        panel.ShowBoardPathForSelfTest(1, nodes, null);
        if (panel.freehandWorkVersion == work) Fail("显示板上轨迹应取消还没做完的手绘计算");
        // The link lengths it was saved with are named, and compared with this computer's.
        panel.ShowBoardPathForSelfTest(0, nodes, $"shape=3; L1={panel.l1.Text}; L2={panel.l2.Text}");
        if (!panel.ResultText.Contains("和这台电脑一致")) Fail("杆长一致时应这样说：" + panel.ResultText);
        panel.ShowBoardPathForSelfTest(0, nodes, "shape=3; L1=181.5; L2=176.38");
        if (!panel.ResultText.Contains("181.5") || !panel.ResultText.Contains("不同") || Math.Abs(panel.boardView!.Value.Model.L1 - 181.5) > 1e-9) Fail("杆长不同时应按板上保存的画并指出不同：" + panel.ResultText);
        // Played like any plan, and stopped at its end.
        panel.play.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
        if (!panel.playbackTimer.IsEnabled) Fail("应能播放板上轨迹");
        panel.AdvancePlayback(0.3);
        if (Math.Abs(panel.cursor.Value - 0.3) > 1e-9 || !panel.playbackTimer.IsEnabled) Fail("播放中应推进游标");
        panel.AdvancePlayback(5);
        if (Math.Abs(panel.cursor.Value - 0.55) > 1e-9 || panel.playbackTimer.IsEnabled) Fail("到末尾应停住");
        // Any new preview or edit takes it away; a request an edit has overtaken shows nothing when its nodes arrive.
        panel.Refresh();
        if (panel.ShowsBoardPath || panel.execution is not null && panel.SettingsDescription.Contains("board_slot")) Fail("重新预览应撤掉板上轨迹");
        panel.boardPathWanted = 1; panel.x.Text = "341";
        panel.ShowBoardPath(1, nodes, null);
        if (panel.ShowsBoardPath) Fail("编辑之后到达的轨迹不应显示");
        // Uploading (loading) a plan read back from the board does not touch the saved parameters: the boxes do not describe it.  A plan
        // made here does (the control: the same click writes the file).
        var slowBoard = new[] { new ExecutionNode(new JointPose(0, 0), 0), new ExecutionNode(new JointPose(0.5, 1), 500), new ExecutionNode(new JointPose(1, 2), 500), new ExecutionNode(new JointPose(1, 2), 50) };
        panel.Refresh(); panel.ShowBoardPathForSelfTest(1, slowBoard, null);
        panel.SetPointMoveState(true, panel.RequestedLimits, "ok", true, true, null, false);
        panel.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, slowBoard[0].Pose), ExecutionTrajectory.Counts(1, slowBoard[0].Pose), 0, 0, 0, 0));
        if (!panel.upload.IsEnabled) Fail("起点在手臂下时板上的计划应能上传：" + panel.moveState.Text);
        panel.upload.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
        if (System.IO.File.Exists(parameters)) Fail("上传板上读回的计划不得把输入框写进参数文件");
        panel.SelectShapeForSelfTest(3); panel.LoadExampleForSelfTest();
        var made = panel.execution ?? throw new InvalidOperationException("应有已检查的图形");
        panel.SetPointMoveState(true, panel.RequestedLimits, "ok", true, true, null, false);
        panel.SetStartFeedback(new TelemetrySample(0, 1, 1, true, 2, ExecutionTrajectory.Counts(0, made.Nodes[0].Pose), ExecutionTrajectory.Counts(1, made.Nodes[0].Pose), 0, 0, 0, 0));
        panel.upload.RaiseEvent(new RoutedEventArgs(ButtonBase.ClickEvent));
        if (!System.IO.File.Exists(parameters)) Fail("本机规划的图形上传仍应记住参数（对照）");
        ParametersPathOverride = null;
        System.IO.File.Delete(parameters);
    }

    private void UpdateUploadCaption()
    {
        planOnBoard = PlanOnBoard();
        string caption = planOnBoard is null ? Lang.T("btn.upload") : Lang.T("btn.loadBoard");
        if (!Equals(upload.Content, caption)) upload.Content = caption;
    }
}
