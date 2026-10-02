using System.Windows;
using System.Windows.Controls;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    // Every shape through the buttons a person presses and the real host state machine; only the device is simulated.
    // Upload calls RequestTrajectory directly: the button click would also overwrite the saved parameters file.
    private static void ShapeFlowSelfTest()
    {
        static void Check(bool ok, string shape, string why) { if (!ok) throw new InvalidOperationException($"Shape flow ({shape}): {why}"); }
        string[] names = ["单点", "直线", "矩形", "圆"];
        for (int shape = 0; shape < 4; shape++)
        {
            string name = names[shape];
            var w = new ControlWindow { selfTesting = true };
            try
            {
                var panel = w.xyPreview;
                panel.SelectShapeForSelfTest(shape); panel.LoadExampleForSelfTest();
                Check(panel.TargetForSelfTest is not null && (shape == 0 || panel.Plan is not null), name, "example is not a valid, executable plan: " + panel.ResultForSelfTest);
                const uint epoch = 4; uint seq = 1;
                void Device(JointPose at) {
                    w.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
                    w.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch={epoch} seq={seq} min1=-3000 max1=3000 min2=-6000 max2=6000");
                    if (!w.protocolMatches) w.TestLine($"LINK fw=stage2-v7-absolute-q2 q2_absolute=1 proto={DeviceContract.ProtocolVersion} motion_rates=1");
                    w.TestLine($"TELEM t=10 epoch={epoch} seq={seq} ref=1 source=2 p1={ExecutionTrajectory.Counts(0, at)} p2={ExecutionTrajectory.Counts(1, at)} e1=0 err=0 inputs=0 drops=0");
                    w.UpdateControls();
                }
                void Click(Button button) => button.RaiseEvent(new RoutedEventArgs(Button.ClickEvent));
                void MoveThere(JointPose target) {
                    Click(panel.MoveButton);
                    Check(w.pendingPoint != 0 && w.pointWire.Count == 36, name, "低速移到起点 did not queue a move");
                    uint moving = w.pendingPoint;
                    w.TestLine($"PTP_STATE lock=0 ref=1 active=1 epoch={epoch} seq={moving} min1=-3000 max1=3000 min2=-6000 max2=6000");
                    Check(w.pendingPoint == 0, name, "the move was not acknowledged by its PTP_STATE");
                    seq = moving; Device(target);   // arrived: idle again at the target with matching telemetry
                }
                var start = shape == 0 ? panel.TargetForSelfTest!.Value : panel.Plan!.Nodes[0].Pose;
                var far = new JointPose(start.Q1 - 3, start.Q2 - 3);   // somewhere else inside the default range
                Device(far);
                Check(panel.MoveButton.IsEnabled && !panel.UploadButton.IsEnabled && !panel.RunButton.IsEnabled, name, "away from the start only the move may be enabled");
                MoveThere(start);
                if (shape == 0) { Check(panel.MoveButton.IsEnabled && w.pendingPoint == 0, name, "single point did not finish"); continue; }

                var plan = panel.Plan!; int n = plan.Nodes.Length;
                Check(panel.UploadButton.IsEnabled && !panel.RunButton.IsEnabled && panel.MoveButton.IsEnabled, name, "at the start only upload may be added");
                w.RequestTrajectory(plan);
                Check(w.uploading is not null && w.pendingPoint != 0, name, "upload did not start");
                uint last = 0;
                for (int i = 0; i < n; i++) {
                    uint s = last = w.pendingPoint;
                    w.TestLine($"TRAJ_STATE state={(i + 1 == n ? 2 : 1)} count={i + 1} total={n} index=0 seq={s} epoch={epoch}");
                }
                seq = last; Device(start);
                Check(w.uploaded == plan && w.uploading is null && !w.executing && w.pendingPoint == 0, name, "upload did not end in a held plan");
                Check(panel.RunButton.IsEnabled && panel.UploadButton.IsEnabled && !panel.MoveButton.IsEnabled, name, "held plan: run and re-upload open, single moves closed");
                Click(panel.RunButton);
                Check(w.uploading is not null && w.pendingPoint == last + 1 && w.uploaded is null, name, "执行图形 did not send RUN as the next sequence");
                w.TestLine($"TRAJ_STATE state=3 count={n} total={n} index=1 seq={w.pendingPoint} epoch={epoch}");
                Check(w.executing && w.uploading is null, name, "RUN was not acknowledged");
                uint run = w.trajectoryRunSequence;
                w.TestLine($"TRAJ_STATE state=4 count={n} total={n} index={n} seq={run} epoch={epoch}");
                Check(!w.executing, name, "completion was not recognized");
                var end = plan.Nodes[^1].Pose; seq = run; Device(end);
                bool closed = ExecutionTrajectory.Counts(0, end) == ExecutionTrajectory.Counts(0, start) && ExecutionTrajectory.Counts(1, end) == ExecutionTrajectory.Counts(1, start);
                Check(w.uploaded == plan && w.uploadedRan, name, "a finished plan must stay held for a repeat");
                if (closed) Check(panel.UploadButton.IsEnabled && panel.RunButton.IsEnabled, name, "a closed shape ends at its start: run again without moving");
                else {
                    Check(!panel.UploadButton.IsEnabled && !panel.RunButton.IsEnabled && panel.MoveButton.IsEnabled, name, "an open shape ends away from its start: only the move is open");
                    MoveThere(start);
                    Check(panel.UploadButton.IsEnabled && panel.RunButton.IsEnabled && w.uploaded == plan, name, "back at the start the held plan must run again");
                }
                // Repeat: a fresh RUN on the next sequence, no new upload.
                Click(panel.RunButton);
                Check(w.uploading is not null && w.uploaded is null && w.pendingPoint != 0, name, "repeat did not send RUN");
                w.TestLine($"TRAJ_STATE state=3 count={n} total={n} index=1 seq={w.pendingPoint} epoch={epoch}");
                run = w.trajectoryRunSequence;
                w.TestLine($"TRAJ_STATE state=4 count={n} total={n} index={n} seq={run} epoch={epoch}");
                Check(!w.executing && w.uploaded == plan && w.uploadedRan, name, "second run was not recognized as complete");            }
            finally { w.Stop(); w.Close(); }
        }
    }
}

public sealed partial class XYPreviewPanel
{
    internal Button UploadButton => upload;
    internal Button RunButton => run;
    internal Button MoveButton => movePoint;
    internal ExecutionTrajectory? Plan => execution;
    internal JointPose? TargetForSelfTest => pointTarget;
    internal string ResultForSelfTest => result.Text;
    internal void SelectShapeForSelfTest(int index) => shape.SelectedIndex = index;
    internal void LoadExampleForSelfTest() => LoadExample();
}
