using System.Diagnostics;
using System.IO.Ports;
using System.Windows;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private static string BackgroundMotionSelfTest()
    {
        var window = new ControlWindow { selfTesting = true };
        using var unopenedPort = new SerialPort(); // Exercise the connected event path without opening hardware.
        window.serial = unopenedPort;
        var limits = new JointLimits(-30, 30, -60, 60);
        var path = DrawingTrajectory.Create(new PlanarArm(180, 176.38), DrawingShape.Circle, 340, 0, 5, 0, 0, limits);
        var plan = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(5, 2, 1.5, 2, 1.5)), limits);
        try {
            foreach (string phase in new[] { "home request", "homing", "point request", "point active", "upload", "drawing", "jog" }) {
                window.stopPending = false;
                window.pendingControl = phase == "home request" ? "回零" : null;
                window.homing = phase == "homing";
                window.pendingPoint = phase is "point request" or "upload" ? 7u : 0;
                window.ptp = new PtpState(false, true, phase == "point active" ? 1 : 0, 2, 7, limits);
                window.uploading = phase == "upload" ? plan : null;
                window.executing = phase == "drawing";
                window.keyboardMask = phase == "jog" ? 1 : 0;
                window.mouseMask = phase == "jog" ? 4 : 0;
                window.pointWire.Clear();
                if (window.pendingPoint != 0) window.pointWire.Enqueue(0xa5);
                var before = (window.pendingControl, window.homing, window.pendingPoint, window.ptp,
                    window.uploading, window.executing, window.pointWire.Count, window.pointSequence);
                window.OnDeactivated(EventArgs.Empty);
                window.OnActivated(EventArgs.Empty);
                var after = (window.pendingControl, window.homing, window.pendingPoint, window.ptp,
                    window.uploading, window.executing, window.pointWire.Count, window.pointSequence);
                if (before != after || window.stopPending || window.keyboardMask != 0 || window.mouseMask != 0)
                    throw new InvalidOperationException($"Window focus change cancelled {phase} or retained/restarted jog input.");
            }
            return "Focus events: home request, homing, point request/active, upload and drawing retained; jog inputs released and not restarted.\nNo serial port opened; no motor commands sent.\n";
        }
        finally { window.serial = null; window.Close(); }
    }

    // Exercise the production timer on the real WPF dispatcher, without opening a port.
    private static string HeartbeatSchedulingSelfTest()
    {
        var window = new ControlWindow { selfTesting = true };
        window.heartbeatTimer.Tick -= window.OnHeartbeatTick;
        var clock = Stopwatch.StartNew();
        var frame = new DispatcherFrame();
        double last = 0, longest = 0;
        int beats = 0;
        window.heartbeatTimer.Tick += (_, _) => {
            double now = clock.Elapsed.TotalMilliseconds;
            longest = Math.Max(longest, now - last); last = now; beats++;
        };
        void InputWork()
        {
            // Small, bounded mouse/input jobs must not starve the 40 ms heartbeat.
            window.UpdateControls();
            Thread.Sleep(4);
            if (clock.ElapsedMilliseconds < 700)
                window.Dispatcher.BeginInvoke(DispatcherPriority.Input, new Action(InputWork));
            else window.Dispatcher.BeginInvoke(DispatcherPriority.ApplicationIdle, new Action(() => frame.Continue = false));
        }
        try
        {
            window.heartbeatTimer.Start();
            window.Dispatcher.BeginInvoke(DispatcherPriority.Input, new Action(InputWork));
            Dispatcher.PushFrame(frame);
            longest = Math.Max(longest, clock.Elapsed.TotalMilliseconds - last);
            if (beats < 5 || longest >= 250)
                throw new InvalidOperationException($"Heartbeat starved by input: ticks={beats}, max gap={longest:F0} ms (firmware stops at 250 ms).");
            string report = $"Input load: {beats} heartbeat ticks, max gap={longest:F1} ms (<250 ms).\n";
            // Replay the real HOME UI, including WPF rendering and serial-line handlers.
            window.ShowActivated = false; window.ShowInTaskbar = false;
            window.Left = -12000; window.Top = -12000; window.Show(); window.UpdateLayout();
            clock.Restart(); last = longest = 0; beats = 0; frame = new DispatcherFrame();
            int batch = 0; double longestReceive = 0;
            var replay = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(40) };
            replay.Tick += (_, _) => {
                var receive = Stopwatch.StartNew();
                if (clock.ElapsedMilliseconds >= batch * 500) {
                    window.TestLine("STATE fw=stage2-v6 enabled=1 home=3 homed1=0 homed2=0 fault=0 enc=1 dir=1");
                    window.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=20 seq=6 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
                    window.TestLine("TRAJ_STATE state=0 count=0 total=0 index=0 seq=6 epoch=20");
                    window.TestLine("LINK uart=hw baud=115200 rx_overflow=0 rx_error=2 tx_dropped=0");
                    window.TestLine($"TELEM t={1123000 + batch * 500} epoch=20 seq=6 ref=0 source=0 p1={-249 - batch * 900} p2=0 e1={-620 - batch * 2300} err=15 inputs=0 drops=2");
                    batch++;
                }
                window.UpdateControls();
                longestReceive = Math.Max(longestReceive, receive.Elapsed.TotalMilliseconds);
                if (clock.ElapsedMilliseconds >= 4000) { replay.Stop(); frame.Continue = false; }
            };
            try { replay.Start(); Dispatcher.PushFrame(frame); }
            finally { replay.Stop(); }
            longest = Math.Max(longest, clock.Elapsed.TotalMilliseconds - last);
            if (beats < 20 || longest >= 250)
                throw new InvalidOperationException($"Rendered HOME heartbeat gap={longest:F1} ms, receive callback={longestReceive:F1} ms, ticks={beats}.");
            report += $"Rendered HOME (4 s, {batch} received batches): max heartbeat gap={longest:F1} ms, receive callback={longestReceive:F1} ms.\n";
            window.WindowState = WindowState.Minimized;
            window.compatible = true; window.lastStateAt = DateTime.UtcNow;
            window.OnHeartbeatTick(null, EventArgs.Empty);
            if (window.IsActive || !window.homing || !window.heartbeatTimer.IsEnabled)
                throw new InvalidOperationException("Minimizing the window cancelled homing or heartbeat.");
            clock.Restart(); last = longest = 0; beats = 0; frame = new DispatcherFrame();
            var finish = new DispatcherTimer { Interval = TimeSpan.FromSeconds(1) };
            finish.Tick += (_, _) => { finish.Stop(); frame.Continue = false; };
            try { finish.Start(); Dispatcher.PushFrame(frame); }
            finally { finish.Stop(); window.Hide(); }
            longest = Math.Max(longest, clock.Elapsed.TotalMilliseconds - last);
            if (beats < 10 || longest >= 250 || !window.homing || !window.heartbeatTimer.IsEnabled)
                throw new InvalidOperationException($"Minimized HOME heartbeat gap={longest:F1} ms, ticks={beats}.");
            report += $"Minimized HOME (1 s, inactive window): {beats} heartbeat ticks, max gap={longest:F1} ms; homing retained.\n";
            // A genuinely blocked UI must still stop producing heartbeats.
            int before = beats;
            Thread.Sleep(300);
            if (beats != before) throw new InvalidOperationException("Heartbeat survived a blocked UI.");
            // If rendering prevents stop/key-release input, it must also prevent keepalive.
            clock.Restart(); frame = new DispatcherFrame();
            void BlockInput()
            {
                if (beats != before) throw new InvalidOperationException("Heartbeat outran blocked stop/key-release input.");
                Thread.Sleep(4);
                if (clock.ElapsedMilliseconds < 300)
                    window.Dispatcher.BeginInvoke(DispatcherPriority.Render, new Action(BlockInput));
                else frame.Continue = false;
            }
            window.Dispatcher.BeginInvoke(DispatcherPriority.Render, new Action(BlockInput));
            Dispatcher.PushFrame(frame);
            window.timer.Start();
            window.compatible = true; window.lastStateAt = DateTime.UtcNow;
            window.OnHeartbeatTick(null, EventArgs.Empty);
            if (!window.compatible || !window.heartbeatTimer.IsEnabled || !window.timer.IsEnabled)
                throw new InvalidOperationException("Fresh device state unexpectedly stopped communication timers.");
            window.lastStateAt = DateTime.UtcNow.AddSeconds(-3);
            window.OnHeartbeatTick(null, EventArgs.Empty);
            if (window.compatible || window.heartbeatTimer.IsEnabled || window.timer.IsEnabled)
                throw new InvalidOperationException("Stale device state did not disconnect and stop both timers.");
            return report + "Blocked UI/input: no heartbeat during each 300 ms block; stale device state disconnects and stops both timers.\nNo serial port opened; no motor commands sent.\n";
        }
        finally { window.heartbeatTimer.Stop(); window.Close(); }
    }
}
