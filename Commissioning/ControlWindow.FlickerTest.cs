using System.Windows;
using System.Windows.Controls;
using System.Windows.Shapes;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    // Replays a real device's line order -- a status block every 500 ms, TELEM running ahead of PTP_STATE as each node is
    // taken -- through connect, enable, upload, hold, run and finish, and fails if any visible property goes A -> B -> A
    // within a few updates. Progress only moves forward and a drawn shape returns to its start slowly, so a quick return is a flash.
    private static void FlickerSelfTest()
    {
        var w = new ControlWindow { selfTesting = true };
        var panel = w.xyPreview;
        var history = new Dictionary<string, List<(string Value, int At)>>();
        int snaps = 0;
        const int FlashSnaps = 8;   // a status block is about 5 lines; the flashes found here lasted 1-2
        var flashes = new List<string>();
        string phase = "";
        static IEnumerable<(string Key, string Value)> Props(DependencyObject node, string path)
        {
            if (node is FrameworkElement e) {
                yield return (path + ".IsEnabled", e.IsEnabled.ToString());
                yield return (path + ".Visibility", e.Visibility.ToString());
                if (e.ToolTip is string tip) yield return (path + ".ToolTip", tip);
            }
            if (node is TextBlock t) yield return (path + ".Text", t.Text);
            if (node is ContentControl c && c.Content is string s) yield return (path + ".Content", s);
            if (node is Control k) { yield return (path + ".Background", k.Background?.ToString() ?? ""); yield return (path + ".Foreground", k.Foreground?.ToString() ?? ""); }
            if (node is Border b) yield return (path + ".Background", b.Background?.ToString() ?? "");
            if (node is Shape h) yield return (path + ".Fill", h.Fill?.ToString() ?? "");
            if (node is Canvas) yield break;   // drawn content is checked as one text below
            int i = 0;
            foreach (var child in LogicalTreeHelper.GetChildren(node))
                if (child is DependencyObject d) foreach (var p in Props(d, path + "/" + d.GetType().Name + i++)) yield return p;
        }
        void Snap(string step)
        {
            w.UpdateControls();
            panel.RedrawForSelfTest(); snaps++;
            var now = Props(w, "W").Append(("viewport", panel.ViewportTextForSelfTest));
            foreach (var (key, value) in now) {
                if (!history.TryGetValue(key, out var seen)) history[key] = seen = [];
                if (seen.Count > 0 && seen[^1].Value == value) continue;
                if (seen.Count > 1 && seen[^2].Value == value && snaps - seen[^1].At <= FlashSnaps && flashes.Count < 40)
                    flashes.Add($"[{phase}] {step}: {key} back to \"{value}\" after \"{seen[^1].Value}\"");
                seen.Add((value, snaps));
            }
        }
        void Phase(string name) => phase = name;   // labels only: a flash across a phase boundary counts too
        const uint epoch = 4;
        uint statusSeq = 1;
        bool on = false;
        JointPose at = default;
        string Telem(uint seq, JointPose pose) => $"TELEM t=10 epoch={epoch} seq={seq} ref={(on ? 1 : 0)} source=2 p1={ExecutionTrajectory.Counts(0, pose)} p2={ExecutionTrajectory.Counts(1, pose)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0";
        void Line(string line, string step) { w.TestLine(line); Snap(step); }
        void Block(string step)
        {
            Line($"STATE fw=stage2-v7 enabled={(on ? 1 : 0)} home=0 homed1={(on ? 1 : 0)} homed2={(on ? 1 : 0)} fault=0 enc=1 dir=1", step + " STATE");
            Line($"CONTROL seq=1", step + " CONTROL");
            Line($"PTP_STATE lock=0 ref={(on ? 1 : 0)} active=0 epoch={epoch} seq={statusSeq} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500", step + " PTP_STATE");
            Line($"LINK uart=hw baud=115200 fw=stage2-v7-test short_hb=1 joint_limits=1 motion_rates=1 rates_live=1 q2_absolute=1 traj_repeat=1 proto={DeviceContract.ProtocolVersion}", step + " LINK");
            Line(Telem(statusSeq, at), step + " TELEM");
        }
        try {
            panel.SelectShapeForSelfTest(3); panel.LoadExampleForSelfTest();
            var plan = panel.Plan!; int n = plan.Nodes.Length;
            at = plan.Nodes[0].Pose;
            Phase("connected, disabled");
            for (int b = 0; b < 4; b++) Block("block " + b);
            Phase("enabled, at start");
            on = true;
            for (int b = 0; b < 4; b++) Block("block " + b);
            Phase("upload");
            w.RequestTrajectory(plan); Snap("upload start");
            if (w.uploading is null) throw new InvalidOperationException("Flicker: upload did not start");
            for (int i = 0; i < n; i++) {
                uint s = w.pendingPoint;
                if (i % 2 == 0) Line(Telem(s, at), $"node {i} TELEM ahead");
                Line($"TRAJ_STATE state={(i + 1 == n ? 2 : 1)} count={i + 1} total={n} index=0 seq={s} epoch={epoch}", $"node {i} ack");
                if (i % 10 == 9) { statusSeq = s; Block($"node {i}"); }
            }
            Phase("held");
            statusSeq = w.pointSequence;
            for (int b = 0; b < 3; b++) Block("block " + b);
            if (w.uploaded != plan) throw new InvalidOperationException("Flicker: upload did not end in a held plan");
            Phase("run");
            w.RequestExecute(); Snap("run requested");
            uint run = w.pendingPoint;
            if (w.uploading is null || run == 0) throw new InvalidOperationException("Flicker: RUN was not sent");
            Line(Telem(run, at), "RUN TELEM ahead");
            Line($"TRAJ_STATE state=3 count={n} total={n} index=0 seq={run} epoch={epoch}", "RUN ack");
            for (int i = 1; i < n; i++) {
                at = plan.Nodes[i].Pose;
                if (i % 2 == 0) Line(Telem(run, at), $"index {i} TELEM");
                Line($"TRAJ_STATE state=3 count={n} total={n} index={i} seq={run} epoch={epoch}", $"index {i}");
                if (i % 10 == 9) { statusSeq = run; Block($"index {i}"); }
            }
            Line($"TRAJ_STATE state=4 count={n} total={n} index={n} seq={run} epoch={epoch}", "complete");
            Phase("finished");
            for (int b = 0; b < 3; b++) Block("block " + b);
            if (w.executing || w.uploaded != plan) throw new InvalidOperationException("Flicker: run did not finish into a held plan");
            if (flashes.Count > 0) throw new InvalidOperationException("Flicker: " + flashes.Count + " flash(es)\n" + string.Join("\n", flashes));
        }
        finally { w.Stop(); w.Close(); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal void RedrawForSelfTest() => RedrawViewport();
    internal string ViewportTextForSelfTest =>
        $"device={(startFeedback is null ? "none" : "shown")} note={deviceNote} | " + string.Join(" | ", canvas.Children.OfType<TextBlock>().Select(t => t.Text));
}
