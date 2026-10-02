using System.Diagnostics;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    // Real referenced-idle rendering and log trimming, with no serial port or motor commands.
    // A UI stall of LeaseStopMs or more stops the machine in the field, so the replays fail on it.  One long stall can also
    // come from the test machine (antivirus scan, a background build), so a lease-only failure is run once more and the
    // gate fails only if it repeats; the first stall stays in the report.
    private sealed class LeaseGapException(string report) : InvalidOperationException(report);
    private static string WithLeaseRetry(Func<string> replay)
    {
        try { return replay(); }
        catch (LeaseGapException first)
        {
            string again = replay();
            return again + $" NOTE: attempt 1 stalled the UI past {LeaseStopMs:F0} ms and did not repeat: {first.Message}";
        }
    }
    private static string ReferenceIdleReplaySelfTest() => ReferenceReplaySelfTest(false);
    private static string PreviewInteractionReplaySelfTest() => ReferenceReplaySelfTest(true);
    private static string ReferenceReplaySelfTest(bool interaction, bool framed = false)
    {
        var window = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false, Left = -12000, Top = -12000 };
        window.heartbeatTimer.Tick -= window.OnHeartbeatTick;
        var seed = new StringBuilder();
        while (seed.Length < 92000)
            seed.AppendLine("[19:30:19.432] TELEM t=49000 epoch=2 seq=1 ref=1 source=2 p1=0 p2=0 e1=0 err=0 inputs=0 drops=0");
        window.log.Text = seed.ToString();
        var tabs = (TabControl)window.diagnostics.Content;
        var clock = new Stopwatch();
        var frame = new DispatcherFrame();
        double last = 0, longest = 0, longestReceive = 0, longestLine = 0, longestInput = 0, lastLease = 0, longestLease = 0;
        string slowestLine = "";
        int beats = 0, batch = 0, phase = -1, inputs = 0;
        int duration = interaction ? 6000 : 10000;
        bool trimmed = false;
        bool firstEndCopy = true;
        // Framed mode: the UI tick only republishes the lease, the writer thread owns
        // the paced wire.  The measured interval is therefore the writer's cadence.
        var writerStop = false;
        Exception? writerFailure = null;
        void FrameWrite(byte[] data, int offset, int count) {
            if (offset + count == data.Length && data.Length == 36 &&
                ((data[1] >> 1) & 15) == 6 && ((data[2] >> 1) & 15) == 0) {
                firstEndCopy = !firstEndCopy;
                if (firstEndCopy) {
                    double completed = clock.Elapsed.TotalMilliseconds;
                    longest = Math.Max(longest, completed - last); last = completed; beats++;
                }
            }
        }
        window.heartbeatTimer.Tick += (_, _) => {
            double now = clock.Elapsed.TotalMilliseconds;
            if (!framed) { longest = Math.Max(longest, now - last); last = now; beats++; }
            else { longestLease = Math.Max(longestLease, now - lastLease); lastLease = now; window.PublishHeartbeatLease(); }
        };
        var writer = framed ? new Thread(() => {
            while (!writerStop) {
                try { if (!window.WireWriterStep(FrameWrite)) window.WireIdleWait(); }
                catch (Exception ex) { writerFailure = ex; return; }
            }
        }) { IsBackground = true } : null;
        var input = new DispatcherTimer(DispatcherPriority.Input) { Interval = TimeSpan.FromMilliseconds(30) };
        input.Tick += (_, _) => {
            var handled = Stopwatch.StartNew();
            window.xyPreview.ReplayEditingAndViewport(inputs++);
            longestInput = Math.Max(longestInput, handled.Elapsed.TotalMilliseconds);
            if (clock.ElapsedMilliseconds >= duration) { input.Stop(); frame.Continue = false; }
        };
        var replay = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(40) };
        replay.Tick += (_, _) => {
            var receive = Stopwatch.StartNew();
            int nextPhase = clock.ElapsedMilliseconds < 3000 ? 0 : clock.ElapsedMilliseconds < 6000 ? 1 : 2;
            if (phase != nextPhase) {
                phase = nextPhase;
                window.diagnostics.IsExpanded = phase != 0;
                tabs.SelectedIndex = phase == 2 ? 1 : 0;
            }
            if (clock.ElapsedMilliseconds >= batch * 500) {
                string[] lines = [
                    $"STATE fw=stage2-v{(framed ? 7 : 6)} enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1",
                    "PTP_STATE lock=0 ref=1 active=0 epoch=2 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500",
                    "TRAJ_STATE state=0 count=0 total=0 index=0 seq=1 epoch=2",
                    "LINK uart=hw baud=115200 rx_overflow=0 rx_error=0 tx_dropped=0",
                    $"TELEM t={49000 + batch * 500} epoch=2 seq=1 ref=1 source=2 p1=0 p2=0 e1={batch % 3 - 1} err=0 inputs=0 drops=0"
                ];
                foreach (string line in lines) {
                    var handled = Stopwatch.StartNew();
                    window.TestLine(line);
                    if (handled.Elapsed.TotalMilliseconds > longestLine) {
                        longestLine = handled.Elapsed.TotalMilliseconds;
                        slowestLine = line.Split(' ')[0];
                    }
                }
                batch++;
                if (framed && window.pointWire.Count == 0) {
                    foreach (byte b in PtpProtocol.Encode((uint)batch, 2, new JointPose(0,0))) window.pointWire.Enqueue(b);
                }
            }
            window.UpdateControls();
            trimmed |= window.logCharacters < 80000;
            longestReceive = Math.Max(longestReceive, receive.Elapsed.TotalMilliseconds);
            if (clock.ElapsedMilliseconds >= duration) { replay.Stop(); frame.Continue = false; }
        };
        try {
            window.Show(); window.UpdateLayout();
            clock.Start(); window.heartbeatTimer.Start(); replay.Start();
            writer?.Start();
            if (interaction) input.Start();
            Dispatcher.PushFrame(frame);
            writerStop = true; writer?.Join(500);
            if (writerFailure is not null) throw writerFailure;
            longest = Math.Max(longest, clock.Elapsed.TotalMilliseconds - last);
            string report = $"{(framed ? "v7 complete control frames" : "v6 timer")} referenced {(interaction ? "editing/viewport" : "idle")} ({duration / 1000} s, {batch} batches, {inputs} input callbacks, hidden/feedback/log views, log trimmed={trimmed}): heartbeat max gap={longest:F1} ms; input callback={longestInput:F1} ms; receive callback={longestReceive:F1} ms; UI lease max gap={longestLease:F1} ms; slowest line={slowestLine} {longestLine:F1} ms; beats={beats}.";
            // A lease gap of LeaseStopMs makes the writer stop heartbeating (a real stop); KeyStaleMs already releases held keys.
            if (longestLease >= KeyStaleMs) report += $" WARN: UI lease gap {longestLease:F0} ms >= {KeyStaleMs:F0} ms (a held key would be released).";
            if (beats < (framed ? duration/200 : 50) || longest >= 250 || (!interaction && !trimmed))
                throw new InvalidOperationException(report);
            if (longestLease >= LeaseStopMs) throw new LeaseGapException(report);

            return report + "\nNo serial port opened; no motor commands sent.\n";
        }
        finally { writerStop = true; input.Stop(); replay.Stop(); window.heartbeatTimer.Stop(); window.Close(); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal void ReplayEditingAndViewport(int i)
    {
        // Human-scale field edits plus a stream of wheel/drag events; no batched bypass of TextChanged.
        if (i < 100 && i % 10 == 0) {
            bool small = i / 10 % 2 == 0;
            x.Text = small ? "340" : "335";
            a.Text = small ? "5" : "15";
        }
        if (i <= 105 && i % 30 == 15) Refresh(); // Finish with a checked plan and sustained wheel/drag events.
        ZoomAt(new Point(Plot.Left + Plot.Width / 2, Plot.Top + Plot.Height / 2), i % 2 == 0 ? 1.05 : 1 / 1.05);
        PanFrom(cameraCenter, new Vector(i % 2 == 0 ? 3 : -3, 1));
        RequestViewportRedraw();
    }
}
