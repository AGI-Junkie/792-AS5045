using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace TwoLinkCommissioning;

// The operating sequence as one strip: connect -> enable -> reference -> check -> move to start -> upload -> run -> result.
// Each step says whether it is done, and the first one that is not says what it needs, or why it cannot be done yet.
public sealed partial class ControlWindow
{
    internal enum StepState { Done, Current, Pending, Skipped }
    internal sealed record WorkflowStep(string Key, StepState State, string Detail, bool Blocked);

    private static readonly string[] WorkflowKeys = ["connect", "enable", "reference", "check", "start", "upload", "run", "result"];
    private readonly Border[] flowChips = WorkflowKeys.Select(_ => new Border { CornerRadius = new CornerRadius(3), Padding = new Thickness(8, 3, 8, 3), Margin = new Thickness(0, 0, 3, 0), BorderThickness = new Thickness(1) }).ToArray();
    private readonly TextBlock[] flowLabels = WorkflowKeys.Select(_ => new TextBlock { FontSize = 12, VerticalAlignment = VerticalAlignment.Center }).ToArray();
    private readonly TextBlock flowHint = new() { FontSize = 12, FontWeight = FontWeights.SemiBold, Margin = new Thickness(10, 2, 10, 4), TextWrapping = TextWrapping.Wrap };
    private WorkflowStep[] workflow = [];
    // What the last recording said, for the last step. Cleared when a new recording starts.
    private string? lastExperimentOutcome;
    private double lastExperimentRmse, lastExperimentPeak;
    private int lastExperimentMotionSamples;
    private bool trajectoryRanOnce;                  // the plan on the device (or just consumed) has been drawn at least once
    private string pointBlockReason = "";            // why a move cannot start now; computed by UpdateControls

    private Border BuildWorkflowStrip()
    {
        var strip = new WrapPanel { Margin = new Thickness(10, 4, 10, 0) };
        for (int i = 0; i < flowChips.Length; i++)
        {
            flowChips[i].Child = flowLabels[i];
            strip.Children.Add(flowChips[i]);
            if (i < flowChips.Length - 1) strip.Children.Add(new TextBlock { Text = "›", Foreground = WorkbenchTheme.Muted, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(0, 0, 3, 0) });
        }
        var panel = new StackPanel { Children = { strip, flowHint } };
        return WorkbenchTheme.Edge(panel, WorkbenchTheme.Bar);
    }

    private static string FirstLine(string text) { int end = text.IndexOf('\n'); return (end < 0 ? text : text[..end]).Trim(); }

    internal WorkflowStep[] ComputeWorkflow()
    {
        var panel = xyPreview;
        bool point = panel.ShapeIndex == 0;
        var (checkOk, checkText) = panel.PlanCheck();
        var startPose = panel.WorkflowStartPose;
        bool atStart = startPose is { } pose && AtStart(pose);
        bool connected = serial is not null && compatible && protocolMatches;
        bool canAct = CanPoint;
        // (done, skipped, detail, blocked): the detail is what the person should know about THIS step right now.
        var steps = new (bool Done, bool Skipped, string Detail, bool Blocked)[8];
        steps[0] = connected ? (true, false, Lang.T("flow.connect.done"), false) :
            serial is null ? (false, false, Lang.T("flow.connect.todo"), false) :
            compatible && !protocolMatches ? (false, false, Lang.T("flow.connect.proto"), true) :
            (false, false, Lang.T("flow.connect.wait"), true);
        steps[1] = enabled ? (true, false, Lang.T("flow.enable.done"), false) :
            !compatible ? (false, false, Lang.T("flow.needConnect"), true) :
            stopPending ? (false, false, Lang.T("flow.stopWait"), true) :
            !LimitsReady ? (false, false, LimitsStatus, true) :
            (false, false, Lang.T("flow.enable.todo"), false);
        steps[2] = enabled && homed ? (true, false, Lang.T("flow.ref.done"), false) :
            !enabled ? (false, false, Lang.T("flow.needEnable"), true) :
            homing ? (false, false, Lang.T("flow.homing"), true) :
            (false, false, Lang.T(supportsManualReference ? "flow.ref.todo" : "flow.ref.todoHome"), false);
        steps[3] = checkOk ? (true, false, checkText, false) : (false, false, checkText, true);
        steps[4] = atStart ? (true, false, Lang.T(point ? "flow.start.donePoint" : "flow.start.done"), false) :
            !checkOk ? (false, false, Lang.T("flow.needCheck"), true) :
            PointBusy ? (false, false, Lang.T("flow.moving"), true) :
            !canAct ? (false, false, pointBlockReason, true) :
            (false, false, Lang.T(point ? "flow.start.todoPoint" : "flow.start.todo"), false);
        int nodes = uploaded?.Nodes.Length ?? uploading?.Nodes.Length ?? panel.Plan?.Nodes.Length ?? 0;
        steps[5] = point ? (false, true, Lang.T("flow.pointSkips"), false) :
            uploaded is not null || executing ? (true, false, Lang.T("flow.upload.done", nodes), false) :
            uploading is not null ? (false, false, Lang.T("flow.uploading", Math.Min(uploadNext, nodes), nodes), true) :
            !checkOk ? (false, false, Lang.T("flow.needCheck"), true) :
            !supportsTrajectory ? (false, false, Lang.T("flow.noTraj"), true) :
            !atStart ? (false, false, Lang.T("flow.needStart"), true) :
            !canAct ? (false, false, pointBlockReason, true) :
            (false, false, Lang.T("flow.upload.todo"), false);
        steps[6] = point ? (false, true, Lang.T("flow.pointSkips"), false) :
            executing ? (false, false, Lang.T("flow.running", trajectoryIndex, trajectoryTotal), true) :
            trajectoryRanOnce ? (true, false, Lang.T(uploaded is not null ? "flow.run.doneAgain" : "flow.run.doneUpload"), false) :
            uploaded is null ? (false, false, Lang.T("flow.needUpload"), true) :
            !atStart ? (false, false, Lang.T("flow.needStart"), true) :
            !canAct ? (false, false, pointBlockReason, true) :
            (false, false, Lang.T("flow.run.todo"), false);
        steps[7] = lastExperimentOutcome == "complete" ?
                (true, false, lastExperimentMotionSamples > 0 ? Lang.T("flow.result.done", lastExperimentRmse, lastExperimentPeak) : Lang.T("flow.result.doneNoMotion"), false) :
            lastExperimentOutcome is not null ? (false, false, Lang.T("flow.result.aborted", lastExperimentOutcome), false) :
            (false, false, Lang.T(point ? "flow.result.todoPoint" : "flow.result.todo"), false);
        // Done steps stay done only while what they stand on does: nothing after a missing step counts as finished.
        int firstOpen = Array.FindIndex(steps, s => !s.Done && !s.Skipped);
        var result = new WorkflowStep[8];
        for (int i = 0; i < 8; i++)
        {
            var s = steps[i];
            var state = s.Skipped ? StepState.Skipped : s.Done ? StepState.Done : i == firstOpen ? StepState.Current : StepState.Pending;
            result[i] = new WorkflowStep(WorkflowKeys[i], state, s.Detail, s.Blocked);
        }
        return result;
    }

    private static readonly SolidColorBrush DoneFill = WorkbenchTheme.Brush("#DCFCE7"), DoneInk = WorkbenchTheme.Brush("#166534"),
        WaitFill = WorkbenchTheme.Brush("#FEF3C7"), WaitInk = WorkbenchTheme.Brush("#854D0E"),
        PendingFill = WorkbenchTheme.Brush("#EEF1F5"), SkipInk = WorkbenchTheme.Brush("#9AA5B1");

    private void UpdateWorkflow()
    {
        workflow = ComputeWorkflow();
        for (int i = 0; i < workflow.Length; i++)
        {
            var step = workflow[i];
            string glyph = step.State switch { StepState.Done => "✓", StepState.Current => step.Blocked ? "!" : "●", StepState.Skipped => "–", _ => "○" };
            string text = $"{glyph} {i + 1} {Lang.T("flow." + step.Key)}";
            (Brush fill, Brush ink, Brush edge) style = step.State switch {
                StepState.Done => (DoneFill, DoneInk, DoneInk),
                StepState.Current when step.Blocked => (WaitFill, WaitInk, WaitInk),
                StepState.Current => (WorkbenchTheme.Blue, Brushes.White, WorkbenchTheme.Blue),
                StepState.Skipped => (Brushes.White, SkipInk, WorkbenchTheme.Line),
                _ => (PendingFill, WorkbenchTheme.Muted, WorkbenchTheme.Line) };
            var (fill, ink, edge) = style;
            if (flowLabels[i].Text != text) flowLabels[i].Text = text;
            flowLabels[i].Foreground = ink; flowChips[i].Background = fill; flowChips[i].BorderBrush = edge;
            flowLabels[i].FontWeight = step.State == StepState.Current ? FontWeights.Bold : FontWeights.Normal;
            string tip = Lang.T("flow." + step.Key) + "：" + Lang.T("flow.state." + step.State) + (step.Detail.Length > 0 ? "\n" + step.Detail : "");
            if (!Equals(flowChips[i].ToolTip, tip)) flowChips[i].ToolTip = tip;
            if (System.Windows.Automation.AutomationProperties.GetName(flowChips[i]) != tip.Replace('\n', ' '))
                System.Windows.Automation.AutomationProperties.SetName(flowChips[i], tip.Replace('\n', ' '));
        }
        var current = workflow.Select((s, i) => (s, i)).FirstOrDefault(x => x.s.State == StepState.Current);
        string hint, hintKey = "";
        if (current.s is null) { hint = Lang.T("flow.allDone"); }
        else
        {
            hintKey = current.s.Blocked ? "flow.hint.blocked" : "flow.hint";
            hint = Lang.T(hintKey, current.i + 1, Lang.T("flow." + current.s.Key), current.s.Detail);
        }
        if (flowHint.Text != hint) flowHint.Text = hint;
        flowHint.Foreground = current.s is null ? DoneInk : current.s.Blocked ? WaitInk : WorkbenchTheme.Blue;
    }

    private void RecordExperimentOutcome(string outcome, ExperimentRecorder recorder)
    {
        lastExperimentOutcome = outcome; lastExperimentRmse = recorder.Rmse; lastExperimentPeak = recorder.Peak; lastExperimentMotionSamples = recorder.MotionSamples;
    }

    // Every state the strip can be in, driven through the same handlers the real device would.
    private static void WorkflowSelfTest()
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Workflow strip: " + reason); }
        var w = new ControlWindow { selfTesting = true };
        try
        {
            string States() => string.Concat(w.workflow.Select(s => s.State switch { StepState.Done => 'D', StepState.Current => s.Blocked ? 'B' : 'C', StepState.Skipped => 'S', _ => 'p' }));
            string Hint() => w.flowHint.Text;
            w.UpdateControls();
            Check(States()[0] == 'C' && States()[1] == 'p' && States()[2] == 'p' && States()[3] == 'D', "offline: connect is next; the plan can already be checked: " + States());
            Check(Hint().Contains("连接") && w.workflow[0].Detail.Contains("串口"), "offline hint must say to pick a port and connect: " + Hint());
            // A firmware answers; its protocol is wrong: blocked, with the reason.
            w.TestLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            w.serial = new System.IO.Ports.SerialPort();
            w.HandleLine($"LINK fw=stage2-v7-test short_hb=1 joint_limits=1 motion_rates=1 q2_absolute=1 proto={DeviceContract.ProtocolVersion + 1}");
            w.UpdateControls();
            Check(States()[0] == 'B' && w.workflow[0].Detail.Contains("协议"), "a wrong protocol must block the first step and say why: " + w.workflow[0].Detail);
            w.TestLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            w.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=4 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            w.TestLine($"LINK fw=stage2-v7-absolute-q2 q2_absolute=1 proto={DeviceContract.ProtocolVersion} motion_rates=1");
            w.UpdateControls();
            Check(States().StartsWith("D") && w.workflow[1].State == StepState.Current && !w.workflow[1].Blocked && w.workflow[1].Detail.Contains("使能"), "connected: enable is next and actionable: " + States() + " " + w.workflow[1].Detail);
            // Enabled, no reference: the reference step is next, and it says how.
            w.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1");
            w.UpdateControls();
            Check(w.workflow[1].State == StepState.Done && w.workflow[2].State == StepState.Current && !w.workflow[2].Blocked && w.workflow[2].Detail.Contains("伸直"), "enabled: reference is next: " + States());
            // The plan on screen is checked; the device is somewhere else: moving to the start is next, not blocked.
            w.xyPreview.SelectShapeForSelfTest(3); w.xyPreview.LoadExampleForSelfTest();
            var start = w.xyPreview.Plan!.Nodes[0].Pose;
            var elsewhere = new JointPose(start.Q1 - 3, start.Q2 - 3);
            void Device(JointPose at, bool referenced = true, uint seq = 1) {
                w.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
                w.TestLine($"PTP_STATE lock=0 ref={(referenced ? 1 : 0)} active=0 epoch=4 seq={seq} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
                w.TestLine($"TELEM t=10 epoch=4 seq={seq} ref={(referenced ? 1 : 0)} source=2 p1={ExecutionTrajectory.Counts(0, at)} p2={ExecutionTrajectory.Counts(1, at)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
                w.UpdateControls();
            }
            Device(elsewhere);
            Check(w.workflow[2].State == StepState.Done && w.workflow[3].State == StepState.Done && w.workflow[4].State == StepState.Current && !w.workflow[4].Blocked, "referenced and checked: moving to the start is next: " + States());
            Check(w.flowHint.Text.Contains("移到起点") || w.flowHint.Text.Contains("起点"), "the hint must name the next step: " + Hint());
            // An invalid edit un-checks the plan: the check step is the current one, blocked, with the reason.
            w.xyPreview.SetParameterForSelfTest("a", "9999");
            w.UpdateControls();
            Check(w.workflow[3].State == StepState.Current && w.workflow[3].Blocked && w.workflow[3].Detail.Length > 0 && w.workflow[4].State == StepState.Pending, "a plan that fails its check must hold the strip at the check step: " + States() + " " + w.workflow[3].Detail);
            w.xyPreview.LoadExampleForSelfTest(); w.UpdateControls();
            // At the start the move is done; upload is next.
            Device(start, seq: 2);
            Check(w.workflow[4].State == StepState.Done && w.workflow[5].State == StepState.Current && !w.workflow[5].Blocked, "at the start: upload is next: " + States());
            // The plan is on the device; running is next.
            w.RequestTrajectory(w.xyPreview.Plan!);
            int n = w.xyPreview.Plan!.Nodes.Length; uint last = 0;
            Check(w.workflow[5].State == StepState.Current && w.workflow[5].Detail.Contains("上传中"), "while uploading the step says how far: " + w.workflow[5].Detail);
            for (int i = 0; i < n; i++) { uint s = last = w.pendingPoint; w.TestLine($"TRAJ_STATE state={(i + 1 == n ? 2 : 1)} count={i + 1} total={n} index=0 seq={s} epoch=4"); }
            Device(start, seq: last);
            Check(w.workflow[5].State == StepState.Done && w.workflow[6].State == StepState.Current && !w.workflow[6].Blocked && w.workflow[7].State == StepState.Pending, "uploaded: run is next: " + States());
            // Running, then finished: the result step reports the recording's error.
            w.RequestExecute();
            w.TestLine($"TRAJ_STATE state=3 count={n} total={n} index=5 seq={w.pendingPoint} epoch=4");
            w.UpdateControls();
            Check(w.workflow[6].State == StepState.Current && w.workflow[6].Detail.Contains("执行中"), "while running the step says how far: " + w.workflow[6].Detail);
            uint run = w.trajectoryRunSequence;
            w.TestLine($"TRAJ_STATE state=4 count={n} total={n} index={n} seq={run} epoch=4");
            Device(start, seq: run);
            Check(w.workflow[6].State == StepState.Done && w.trajectoryRanOnce, "after the run the step is done: " + States());
            w.lastExperimentOutcome = "complete"; w.lastExperimentRmse = 0.0123; w.lastExperimentPeak = 0.031; w.lastExperimentMotionSamples = 40; w.UpdateControls();
            Check(States() == "DDDDDDDD" && w.workflow[7].Detail.Contains("0.012") && w.flowHint.Text.Contains("全部"), "everything done: " + States() + " " + w.workflow[7].Detail + " / " + Hint());
            // A stop undoes the sequence from the reference on.
            w.TestLine("STOP: DEADMAN host heartbeat timeout; position invalid, release keys then C to enable");
            w.TestLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1");
            w.UpdateControls();
            Check(w.workflow[1].State == StepState.Current && w.workflow[2].State == StepState.Pending, "a stop must send the strip back to enabling: " + States());
            // A single point has no upload and no run.
            w.xyPreview.SelectShapeForSelfTest(0); w.xyPreview.LoadExampleForSelfTest(); w.UpdateControls();
            Check(w.workflow[5].State == StepState.Skipped && w.workflow[6].State == StepState.Skipped, "a point skips upload and run: " + States());
            w.Disconnect("workflow self-test");
            Check(w.workflow[0].State == StepState.Current && w.lastExperimentOutcome is null && !w.trajectoryRanOnce, "disconnect must start the sequence over: " + States());
        }
        finally { w.Close(); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal int ShapeIndex => shape.SelectedIndex;
    // Where the device has to be before anything else: the first node of the plan, or the single target.
    internal JointPose? WorkflowStartPose => execution is { } plan ? plan.Nodes[0].Pose : shape.SelectedIndex == 0 ? pointTarget is { } p ? PtpProtocol.Quantize(p) : null : null;
    // Has what is on screen been checked: no pending edits, a valid plan, every sample inside the joint ranges.
    internal (bool Ok, string Text) PlanCheck()
    {
        if (parametersDirty) return (false, Lang.T("flow.check.dirty"));
        if (shape.SelectedIndex == 0)
            return pointTarget is null ? (false, Truncate(result.Text)) :
                planningLimits is { } range && !range.Contains(pointTarget.Value) ? (false, Lang.T("flow.check.outRange")) : (true, Lang.T("flow.check.okPoint"));
        return execution is null ? (false, Truncate(timing is null || executionInfo.Text.Length == 0 ? result.Text : executionInfo.Text)) : (true, Lang.T("flow.check.ok"));
    }
    private static string Truncate(string text) { text = text.Trim(); int end = text.IndexOf('\n'); if (end >= 0) text = text[..end]; return text.Length > 160 ? text[..160] + "…" : text; }
    internal void SetParameterForSelfTest(string field, string value) { (field == "a" ? a : field == "b" ? b : x).Text = value; }
}
