using System.IO;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    // No serial port is opened. Exercise the actual controls and request queues in both languages.
    internal static void LanguageSelfTest(string? snapshotPath = null)
    {
        bool language = Lang.IsEn;
        Lang.Set(false, false);
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
                Check(w.pointWire.SequenceEqual(wire) && w.pointSequence == seq && w.controlSequence == control &&
                    w.limitAxis == axis && w.pendingControlType == type && requests == 0, "switch queued a device command");
                Check(w.uploaded == held && w.uploading == uploading && w.executing == executing && w.uploadNext == index,
                    "switch altered upload/run state");
                Check(w.log.Text == raw, "switch rewrote the raw log");
                Check(w.Title == Lang.T("app.title") && Equals(w.langToggle.Content, Lang.T("lang.toggle")), "chrome stale");
                Check(string.Equals(w.Language.IetfLanguageTag, english ? "en-NZ" : "zh-CN", StringComparison.OrdinalIgnoreCase), "WPF accessibility language stale");
            }
            foreach (int shape in Enumerable.Range(0, 4)) {
                p.SelectShapeForSelfTest(shape); p.LoadExampleForSelfTest();
                p.CheckLanguagePreservesPreview(() => Toggle(true));
                Check(p.ResultForSelfTest.Contains(shape == 0 ? "Target" : "Trajectory"), "plan details stale");
                p.CheckLanguagePreservesPreview(() => Toggle(false));
            }
            // Existing error messages and invalid edits must survive the switch without applying the edits.
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
            p.SetBoard(true, false, [new BoardSlot(0, true, 1, 69, 16584, 0, 0, 1), null, null, null], [], () => Lang.T("board.saved", 1));
            Toggle(false); Check(p.BoardNoteText.Contains("板上存档") && p.BoardNoteText.Contains("别处存的图形") && p.BoardNoteText.Contains("已存到板上槽1"), "board note not Chinese");
            Toggle(true); Check(p.BoardNoteText.Contains("Stored on the board") && p.BoardNoteText.Contains("saved elsewhere") && p.BoardNoteText.Contains("Saved to board slot 1"), "board note not English");
            Toggle(false); Check(p.BoardNoteText.Contains("板上存档"), "board note stale after switching back");
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
}

public sealed partial class XYPreviewPanel
{
    internal void CheckLanguagePreservesPreview(Action toggle)
    {
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
        x.Text = "340"; Refresh(); TogglePlayback(); cursor.Value = Math.Min(1, cursor.Maximum);
        TogglePlayback(); CheckLanguagePreservesPreview(chinese); PausePlayback();
    }
}

public sealed partial class ControlWindow
{
    internal static IEnumerable<Expander> LanguageSections(DependencyObject root) => Descendants(root).OfType<Expander>();
}
