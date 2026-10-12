using System.IO;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Threading;
using System.Windows.Automation.Peers;
using System.Windows.Automation.Provider;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    // No serial port is opened. Exercise the actual controls and request queues in both languages.
    internal static void LanguageSelfTest(string? snapshotPath = null)
    {
        bool language = Lang.IsEn;
        Lang.Set(false, false);
        DisplayTextQueueSelfTest();
        var w = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false };
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Language UI: " + why); }
        try {
            var p = w.xyPreview;
            int requests = 0;
            p.PointMoveRequested += _ => requests++;
            p.TrajectoryRequested += _ => requests++;
            p.ExecuteRequested += () => requests++;
            void Toggle(bool english) {
                byte[] wire = w.pointWire.ToArray(); uint seq = w.pointSequence;
                uint control = w.controlSequence; int axis = w.limitAxis; byte type = w.pendingControlType;
                var held = w.uploaded; var uploading = w.uploading;
                bool executing = w.executing; int index = w.uploadNext;
                string raw = w.log.Text;
                Lang.Set(english, false);
                w.Dispatcher.Invoke(() => { }, DispatcherPriority.ContextIdle);
                Check(w.pointWire.SequenceEqual(wire) && w.pointSequence == seq && w.controlSequence == control &&
                    w.limitAxis == axis && w.pendingControlType == type && requests == 0, "switch queued a device command");
                Check(w.uploaded == held && w.uploading == uploading && w.executing == executing && w.uploadNext == index,
                    "switch altered upload/run state");
                Check(w.log.Text == raw, "switch rewrote the raw log");
                Check(w.Title == Lang.T("app.title") && Equals(w.langToggle.Content, Lang.T("lang.toggle")), "chrome stale");
                Check(string.Equals(w.Language.IetfLanguageTag, english ? "en-NZ" : "zh-CN", StringComparison.OrdinalIgnoreCase), "WPF accessibility language stale");
            }
            foreach (int shape in new[] { 0, 1, 2, 3, (int)DrawingShape.Freehand + 1 }) {
                p.SelectShapeForSelfTest(shape); p.LoadExampleForSelfTest();
                p.CheckLanguagePreservesPreview(() => Toggle(true));
                Check(p.ResultForSelfTest.Contains(shape == 0 ? "Target" : "Trajectory"), "plan details stale");
                p.CheckLanguagePreservesPreview(() => Toggle(false));
            }
            // Existing error messages and invalid edits must survive the switch without applying the edits.
            p.SelectShapeForSelfTest(3); p.LoadExampleForSelfTest();
            p.CheckLanguageInvalidEdit(() => Toggle(true), () => Toggle(false));
            p.SelectShapeForSelfTest(3); p.LoadExampleForSelfTest();
            const uint epoch = 4;
            w.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            w.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=4 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            w.TestLine($"LINK fw=stage2-v7-absolute-q2 proto={DeviceContract.ProtocolVersion} motion_rates=1 joint_limits=1");
            var plan = p.Plan!; var start = plan.Nodes[0].Pose;
            w.TestLine($"TELEM t=10 epoch=4 seq=1 ref=1 source=2 p1={ExecutionTrajectory.Counts(0,start)} p2={ExecutionTrajectory.Counts(1,start)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            w.TestLine("INPUTS: J4-1/P0[4]=1 J4-2/P0[3]=1 NPN/P1[0]=1 nFAULT=1");
            Toggle(true);
            Check(w.state.Text.Contains("Enabled") && w.liveValues.Text.Contains("Manual straight zero") && w.inputs.Text.StartsWith("Input levels"), "existing feedback stale");
            Toggle(false);
            Check(p.UploadButton.IsEnabled, "ready shape is locked");
            w.RequestTrajectory(plan);
            p.CheckLanguagePreservesPreview(() => Toggle(true));
            Check(w.state.Text.StartsWith("Uploading"), "upload status stale");
            Toggle(false);
            for (int i = 0; i < plan.Nodes.Length; i++)
                w.TestLine($"TRAJ_STATE state={(i+1 == plan.Nodes.Length ? 2 : 1)} count={i+1} total={plan.Nodes.Length} index=0 seq={w.pendingPoint} epoch={epoch}");
            w.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=4 seq={w.pointSequence} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            w.TestLine($"TELEM t=11 epoch=4 seq={w.pointSequence} ref=1 source=2 p1={ExecutionTrajectory.Counts(0,start)} p2={ExecutionTrajectory.Counts(1,start)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            Check(w.uploaded == plan && p.RunButton.IsEnabled && !w.executing, "held plan missing");
            p.CheckLanguagePreservesPreview(() => Toggle(true));
            Check(p.RunButton.IsEnabled && w.state.Text.StartsWith("Shape uploaded"), "switch lost held plan");
            Toggle(false); w.RequestExecute();
            w.TestLine($"TRAJ_STATE state=3 count={plan.Nodes.Length} total={plan.Nodes.Length} index=1 seq={w.pendingPoint} epoch=4");
            p.CheckLanguagePreservesPreview(() => Toggle(true));
            Check(w.executing && w.state.Text.StartsWith("Shape run:"), "running state stale");
            w.Stop();
            w.TestLine("STOP: UART receive error/overflow");
            Toggle(false); Check(w.lastStop.Text.StartsWith(Lang.T("explain.uart")), "fault explanation stale");
            Toggle(true); Check(w.lastStop.Text.StartsWith(Lang.T("explain.uart")), "fault explanation stale");
            w.Disconnect(() => Lang.T("fault.unplug"));
            Toggle(false); Check(w.state.Text == Lang.T("fault.unplug"), "disconnect cause stale");
            Toggle(true); Check(w.state.Text == Lang.T("fault.unplug"), "disconnect cause stale");
            // Numerics/protocol/CSV stay invariant even under a comma-decimal system locale.
            var culture = CultureInfo.CurrentCulture;
            try {
                CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("de-DE");
                Check(Lang.T("vp.mouse", 1.25, 2.5).Contains("1.25"), "UI numbers disagree with dot-decimal input");
                ExecutionTrajectory.SelfTest();
            } finally { CultureInfo.CurrentCulture = culture; }
            // The board archive's note and messages follow the language, and show up in the snapshots below.
            StoreFailureLanguageSelfTest();
            p.SetBoard(4, true, false, [new BoardSlot(0, true, 1, 69, 16584, 0, 0, 1), new BoardSlot(1, false, 0, 0, 0, 0, 0, 0), new BoardSlot(2, false, 0, 0, 0, 0, 0, 0), new BoardSlot(3, false, 0, 0, 0, 0, 0, 0)], [], () => Lang.T("board.saved", 1));
            Toggle(false); Check(p.BoardNoteText.Contains("板上存档") && p.BoardNoteText.Contains("别处存的图形") && p.BoardNoteText.Contains("已存到板上槽1"), "board note not Chinese");
            Toggle(true); Check(p.BoardNoteText.Contains("Stored on the board") && p.BoardNoteText.Contains("saved elsewhere") && p.BoardNoteText.Contains("Saved to board slot 1"), "board note not English");
            Toggle(false); Check(p.BoardNoteText.Contains("板上存档"), "board note stale after switching back");
            // A slot that carries its own description is a row of the recent list (header, name, size) in either language.
            p.SetBoard(4, true, false, [new BoardSlot(0, true, 1, 69, 16584, 0, 0, 1), new BoardSlot(1, false, 0, 0, 0, 0, 0, 0), new BoardSlot(2, false, 0, 0, 0, 0, 0, 0), new BoardSlot(3, false, 0, 0, 0, 0, 0, 0)],
                [new LedgerEntry(0, 1, "shape=2; x=330; y=0; a=12; b=7; v1=4; v2=3; a1=8; a2=6")], null);
            Toggle(false); Check(p.RecentListText.Contains("板上图形 · 1/4") && p.RecentListText.Contains("槽1：矩形 · 69 点"), "board row not Chinese: " + p.RecentListText);
            Toggle(true); Check(p.RecentListText.Contains("On the board · 1/4") && p.RecentListText.Contains("slot 1: Rectangle · 69 points"), "board row not English: " + p.RecentListText);
            Toggle(false); Check(p.RecentListText.Contains("板上图形"), "board row stale after switching back");
            // The plan read back from a slot is a read-only preview, described in either language (and drawn in the snapshots below).
            var boardNodes = new List<ExecutionNode> { new(new JointPose(-12, 55), 0) };
            for (int k = 1; k <= 60; k++) boardNodes.Add(new(new JointPose(Math.Round(-12 + 24 * k / 60.0 + 6 * Math.Sin(k / 6.0), 2), Math.Round(55 - 35 * Math.Sin(k * Math.PI / 60), 2)), 100));
            p.ShowBoardPathForSelfTest(0, boardNodes.ToArray(), "shape=2; L1=181.5; L2=176.38");
            Toggle(false); Check(p.ShowsBoardPath && p.ResultText.Contains("板上槽1保存的轨迹") && p.ResultText.Contains("L1=181.5") && p.ResultText.Contains("不同"), "board plan not Chinese: " + p.ResultText);
            Toggle(true); Check(p.ShowsBoardPath && p.ResultText.Contains("board slot 1") && p.ResultText.Contains("which differ") && p.ResultText.Contains("L1=181.5"), "board plan not English: " + p.ResultText);
            Toggle(false); Check(p.ResultText.Contains("板上槽1"), "board plan stale after switching back");
            if (snapshotPath is not null) {
                string directory = Path.GetDirectoryName(Path.GetFullPath(snapshotPath))!;
                Directory.CreateDirectory(directory);
                foreach (bool en in new[] { false, true }) {
                    Toggle(en);
                    foreach (var size in new[] { (1100,760), (1380,930) }) {
                        w.SaveWorkbenchSnapshot(Path.Combine(directory, $"language-{(en ? "en" : "zh")}-{size.Item1}.png"), size.Item1, size.Item2);
                        Check(w.stopButton!.ActualWidth + w.stopButton.Margin.Left + w.stopButton.Margin.Right >= w.stopButton.DesiredSize.Width - 1, "STOP button clipped");
                        Check(w.langToggle.ActualWidth + w.langToggle.Margin.Left + w.langToggle.Margin.Right >= w.langToggle.DesiredSize.Width - 1, "language toggle clipped");
                    }
                }
                File.WriteAllText(Path.Combine(directory, "language-tests.txt"), "PASS: both languages, all shapes, dirty/invalid edits, offline playback, upload/held/run, existing feedback/faults, unchanged command queues, invariant numerics, preference recovery and 1100/1380 layouts.\n");
            }
        } finally { w.Close(); Lang.Set(language, false); }
    }

    private static void DisplayTextQueueSelfTest()
    {
        bool language = Lang.IsEn;
        var first = new TextBox { IsReadOnly = true, Text = Lang.T("live.waiting") };
        var second = new TextBox { IsReadOnly = true, Text = Lang.T("live.experimentHint") };
        var editable = new TextBox();
        var content = new StackPanel { Children = { first, second, editable } };
        var window = new Window { Content = content, Width = 320, Height = 160, Left = -12000, Top = -12000,
            ShowActivated = false, ShowInTaskbar = false };
        WorkbenchTheme.Apply(window);
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Display text queue: " + why); }
        void Drain() => window.Dispatcher.Invoke(() => { }, DispatcherPriority.ContextIdle);
        try {
            window.Show(); Drain();
            int obsolete = 0, latest = 0, changes = 0;
            bool inputBetween = false;
            string oldSecond = second.Text;
            first.TextChanged += (_, _) => {
                changes++;
                window.Dispatcher.BeginInvoke(DispatcherPriority.Input, new Action(() => inputBetween |= second.Text == oldSecond));
            };
            Lang.QueueText(first, () => { obsolete++; return Lang.T("live.experimentHint"); });
            Lang.QueueText(first, () => { latest++; return Lang.T("live.waiting"); });
            Lang.QueueText(second, () => Lang.T("live.experimentHint"));
            Lang.Set(true, false); Lang.QueueText(first); Lang.QueueText(second);
            Check(obsolete == 0 && latest == 0, "loaded editors updated synchronously");
            Drain();
            Check(obsolete == 0 && latest == 1 && changes == 1, "superseded formatter or duplicate update ran");
            Check(inputBetween && first.Text == Lang.T("live.waiting") && second.Text == Lang.T("live.experimentHint"),
                "input did not run between the latest translated editors");
            first.SelectAll();
            Check(first.SelectedText == first.Text && ApplicationCommands.Copy.CanExecute(null, first), "selection/copy unavailable");
            var peer = FrameworkElementAutomationPeer.CreatePeerForElement(first)!;
            Check(peer.GetPattern(PatternInterface.Value) is IValueProvider { IsReadOnly: true } value && value.Value == first.Text &&
                peer.GetPattern(PatternInterface.Text) is ITextProvider text && text.DocumentRange.GetText(-1).TrimEnd('\r', '\n') == first.Text,
                "read-only accessibility value/text patterns changed");
            Check(InputMethod.GetIsInputMethodEnabled(editable), "editable input method disabled");
            Lang.QueueText(first, () => Lang.T("exp.savedNote"));
            content.Children.Remove(first); Drain();
            Check(first.Text == Lang.T("exp.savedNote"), "unloaded editor lost its latest summary");
            content.Children.Add(first); Drain();
            Lang.Set(false, false); Lang.QueueText(first); Drain();
            Check(first.Text == Lang.T("exp.savedNote"), "reloaded editor retained a previous language");
            Lang.QueueText(first, () => Lang.T("live.waiting")); window.Close(); Drain();
            Check(first.Text == Lang.T("live.waiting"), "closing discarded pending display text");
        } finally { window.Close(); Lang.Set(language, false); }
    }

    private static void StoreFailureLanguageSelfTest()
    {
        bool language = Lang.IsEn;
        var w = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false,
            compatible = true, framedControl = true, protocolMatches = true, reportedEpoch = 1 };
        var nodes = new[] { new ExecutionNode(new JointPose(0, 0), 0), new ExecutionNode(new JointPose(0, 0), 50) };
        try {
            foreach (StoreOp op in new[] { StoreOp.Save, StoreOp.Erase })
            foreach (string failure in new[] { "noAnswer", "stalled", "device", "refused" }) {
                Lang.Set(false, false); w.ResetBoard(); w.storeSlotCount = 4;
                if (op == StoreOp.Save) w.BeginBoardSave(nodes, 0, "shape=3");
                else w.RequestEraseBoardSlot(0);
                var job = w.storeJob ?? throw new InvalidOperationException("Language UI: failed archive fixture did not start");
                job.Sequence = 1;
                if (failure == "noAnswer") w.PollStoreJob(job.RequestedAt.AddSeconds(9));
                else if (failure == "stalled") {
                    w.ObserveStoreLine($"TRAJ_STORE state={(op == StoreOp.Save ? "saving" : "erasing")} slot=0 rows=0/1 seq=1 last=none last_slot=0 row_max_us=0");
                    w.PollStoreJob(job.LastSeenAt.AddSeconds(13));
                }
                else if (failure == "device") w.ObserveStoreLine("TRAJ_STORE state=idle slot=0 rows=0/1 seq=1 last=failed last_slot=0 row_max_us=0");
                else w.ObserveStoreLine("TRAJ_STORE refused: stage not idle");
                foreach (bool english in new[] { false, true, false }) {
                    Lang.Set(english, false);
                    string reason = failure switch {
                        "device" => Lang.T("board.err.device", "failed"),
                        "refused" => Lang.T("board.err.refused", "stage not idle"),
                        _ => Lang.T("board.err." + failure),
                    };
                    string expected = Lang.T(op == StoreOp.Save ? "board.saveFailed" : "board.eraseFailed", 1, reason);
                    if (w.boardMessage?.Invoke() != expected || !w.xyPreview.BoardNoteText.Contains(expected))
                        throw new InvalidOperationException($"Language UI: archive {op} {failure} did not switch its whole failure message");
                }
            }
        }
        finally { w.Close(); Lang.Set(language, false); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal void CheckLanguagePreservesPreview(Action toggle)
    {
        WaitForFreehand(this);
        string parameters = SaveParameters(); var plan = execution; var path = trajectory; var time = timing;
        bool dirty = parametersDirty, following = offlineSim, playing = playbackTimer.IsEnabled;
        double position = cursor.Value, scale = pixelsPerMm; var center = cameraCenter;
        int shapeIndex = shape.SelectedIndex, branchIndex = branch.SelectedIndex;
        toggle();
        if(!Equals(unreachableLayer.Content,Lang.T("vp.unreachable")) ||
            !Equals(unreachableLayer.ToolTip,Lang.T("vp.unreachable.tip")) ||
            PointerReach(new Point(100,0)) != Lang.T("vp.boundaryIn")+Lang.T("vp.unreach"))
            throw new InvalidOperationException("Link boundary and tip reachability text did not switch language");
        if (SaveParameters()!=parameters || execution!=plan || trajectory!=path || timing!=time || parametersDirty!=dirty ||
            offlineSim!=following || playbackTimer.IsEnabled!=playing || cursor.Value!=position || cameraCenter!=center || pixelsPerMm!=scale ||
            shape.SelectedIndex!=shapeIndex || branch.SelectedIndex!=branchIndex)
            throw new InvalidOperationException("Language switch changed preview/parameters/playback");
        if (Lang.IsEn) {
            foreach (var section in ControlWindow.LanguageSections(ParameterContent)) {
                if (section.Header is DockPanel header && header.Children.OfType<TextBlock>().Any(t => t.Text.Any(c => c is >= '\u4e00' and <= '\u9fff')))
                    throw new InvalidOperationException("Untranslated property-group header");
            }
        }
    }
    internal void CheckLanguageInvalidEdit(Action english, Action chinese)
    {
        x.Text = "invalid";
        CheckLanguagePreservesPreview(english);
        if (!parametersDirty || !result.Text.StartsWith("Parameters changed")) throw new InvalidOperationException("Dirty edit was applied or not translated");
        Refresh();
        CheckLanguagePreservesPreview(chinese);
        if (!result.Text.Contains("请输入有效数值") || execution is not null) throw new InvalidOperationException("Validation error stale");
        CheckLanguagePreservesPreview(english);
        if (!result.Text.Contains("enter a valid number") || result.Text.Any(c => c is >= '\u4e00' and <= '\u9fff')) throw new InvalidOperationException("Validation error stale");
        x.Text = "340"; Refresh(); WaitForFreehand(this); TogglePlayback(); cursor.Value = Math.Min(1, cursor.Maximum);
        TogglePlayback(); CheckLanguagePreservesPreview(chinese); PausePlayback();
    }
}

public sealed partial class ControlWindow
{
    internal static IEnumerable<Expander> LanguageSections(DependencyObject root) => Descendants(root).OfType<Expander>();
}
