using System.Diagnostics;
using System.Reflection;
using System.Text;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    // Field stops 2026-10-14/15: the operator worked in other programs while an automation script re-planned (检查并预览) and read
    // the window through UI Automation; the UI lease stalled 360-700 ms and the firmware deadman stopped an idle, referenced arm.
    // This replays that desktop on the real dispatcher: referenced-idle status traffic, re-plans and field edits every 0.7 s, a UIA
    // client that listens for events and reads every element, and `cpuThreads` other processes spinning at normal priority.  No serial port,
    // no motor commands.  `external` runs the UIA client as another process (--uia-client), as the field script was.
    internal static string DesktopLoadReplay(int cpuThreads, bool uia, int seconds, bool external = false, StringBuilder? profile = null)
    {
        // The profile shows the window on screen (real composition, as in the field); the gate keeps it off screen.
        var window = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false, Left = profile is null ? -12000 : 0, Top = profile is null ? -12000 : 0, Width = 1380, Height = 930 };
        window.heartbeatTimer.Tick -= window.OnHeartbeatTick;
        window.heartbeatTimer.Interval = TimeSpan.FromMilliseconds(8);   // framed mode (ResetControlTransport)
        var clock = new Stopwatch();
        var frame = new DispatcherFrame();
        double lastLease = 0, longestLease = 0, longestAction = 0;
        int over120 = 0, actions = 0, reads = 0, elements = 0, events = 0, batch = 0, telem = 0;
        string slowestAction = "";
        bool stop = false;
        Exception? uiaFailure = null;
        using var firstRead = new ManualResetEventSlim(false);
        var counts = new int[3];   // full reads, elements, events: live, so a client stalled at the end still reports
        // Profile only: every dispatcher operation by delegate, and what filled each gap over 100 ms.
        var started = new Dictionary<DispatcherOperation, long>();
        var timeline = new List<(double Start, double End, string Name)>();
        var gaps = new List<(double Gap, string Text)>();
        var messages = new List<(double At, int Msg)>();
        var methodField = typeof(DispatcherOperation).GetField("_method", BindingFlags.Instance | BindingFlags.NonPublic);
        var argsField = typeof(DispatcherOperation).GetField("_args", BindingFlags.Instance | BindingFlags.NonPublic);
        void Started(object? s, DispatcherHookEventArgs e) => started[e.Operation] = Stopwatch.GetTimestamp();
        void Completed(object? s, DispatcherHookEventArgs e)
        {
            if (!started.Remove(e.Operation, out long at) || !clock.IsRunning) return;
            var d = methodField?.GetValue(e.Operation) as Delegate;
            double end = clock.Elapsed.TotalMilliseconds;
            string timer = argsField?.GetValue(e.Operation) is DispatcherTimer t ? $" ({t.Interval.TotalMilliseconds:F0} ms timer)" : "";
            timeline.Add((end - Stopwatch.GetElapsedTime(at).TotalMilliseconds, end, $"{e.Operation.Priority} {d?.Method.DeclaringType?.Name}.{d?.Method.Name}{timer}"));
            if (timeline.Count > 20000) timeline.RemoveRange(0, 10000);
        }
        if (profile is not null) { window.Dispatcher.Hooks.OperationStarted += Started; window.Dispatcher.Hooks.OperationCompleted += Completed; }
        window.heartbeatTimer.Tick += (_, _) => {
            if (!clock.IsRunning) { window.PublishHeartbeatLease(); return; }
            double now = clock.Elapsed.TotalMilliseconds, gap = now - lastLease;
            lastLease = now;
            longestLease = Math.Max(longestLease, gap);
            if (gap > SlowLeaseLogMs) over120++;
            if (profile is not null && gap > 100) {
                double from = now - gap;
                var parts = timeline.Where(r => r.End > from && r.Start < now).GroupBy(r => r.Name)
                    .Select(g => (Name: g.Key, Count: g.Count(), Ms: g.Sum(r => Math.Min(r.End, now) - Math.Max(r.Start, from))))
                    .OrderByDescending(x => x.Ms).ToList();
                var inGap = timeline.Where(r => r.End > from && r.Start < now).OrderBy(r => r.Start).ToList();
                double holeAt = from, hole = 0, cursor = from;
                foreach (var r in inGap) { if (r.Start - cursor > hole) { hole = r.Start - cursor; holeAt = cursor; } cursor = Math.Max(cursor, r.End); }
                if (now - cursor > hole) { hole = now - cursor; holeAt = cursor; }
                var msgs = messages.Where(m => m.At >= holeAt - 1 && m.At <= holeAt + hole).GroupBy(m => m.Msg).Select(g => $"0x{g.Key:X} x{g.Count()} first +{g.Min(m => m.At) - holeAt:F0}");
                gaps.Add((gap, $"  gap {gap:F0} ms: in operations {parts.Sum(x => x.Ms):F0} ms; " + string.Join("; ", parts.Take(4).Select(x => $"{x.Name} x{x.Count} {x.Ms:F0} ms")) +
                    $"; largest hole {hole:F0} ms, messages there: {string.Join(", ", msgs.Take(8))}"));
            }
            window.PublishHeartbeatLease();
        };
        // The 20:48 status block of the field log, then 10 Hz telemetry.
        string[] block = [
            "STATE fw=stage2-v11 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1 enc2=1 dir2=1 arm2fb=1",
            "CONTROL seq=35",
            "PTP_STATE lock=0 ref=1 active=0 epoch=15 seq=1689 min1=-12000 max1=12000 min2=-15000 max2=15000 emin=-16960 emax=16450",
            "PTP_CHECK final_lsb=228 follow_lsb=328",
            "TRAJ_STATE state=0 count=0 total=0 index=0 seq=1689 epoch=15",
            "LINK uart=hw baud=115200 link=usb rx_overflow=0 rx_error=0 tx_dropped=1 short_hb=1 joint_limits=1 q2_absolute=1 dual_encoder=1 telem_hz=10 proto=6",
            "PTP_RX short_ok=260380 drop_crc=0 drop_short=0 drop_timeout=0 rejected=7 epoch_rejected=7 last_rejected_byte=0x00",
            "TIMING loop_max_ms=24 tick_lag_max_ms=23 clock=hardware wdt=256-384ms key_age_ms=0 key_max_gap_ms=122 status_max_us=702 traj_lag_max_us=11 traj_slip_us=0",
            "LAST_STOP id=2 t=3609258 epoch=9 seq=1353 phase=traj_run home=0 traj=3 idx=250/485 ref=1 p1=-8525 p2=27198 e1=-21824 err=0 e2=69629 err2=3 inputs=0 hb_age_ms=31",
        ];
        var feed = new DispatcherTimer(DispatcherPriority.Input) { Interval = TimeSpan.FromMilliseconds(100) };
        feed.Tick += (_, _) => {
            if (telem++ % 5 == 0) {
                if (window.xyPreview.RequestedProfile is { } p) window.TestLine($"MOTION_RATES epoch=15 r1={p.R1} r2={p.R2} a1={p.A1} a2={p.A2}");
                foreach (string line in block) window.TestLine(line);
                batch++;
            }
            window.TestLine($"TELEM t={8892499 + telem * 100} epoch=15 seq=1689 ref=1 source=1 p1=0 p2=0 e1={telem % 3 - 1} err=0 e2=0 err2=0 inputs=0 drops=0");
            window.UpdateControls();
            if (clock.ElapsedMilliseconds >= seconds * 1000) frame.Continue = false;
        };
        var panel = window.xyPreview;
        var act = new DispatcherTimer(DispatcherPriority.Input) { Interval = TimeSpan.FromMilliseconds(700) };
        act.Tick += (_, _) => {
            var t = Stopwatch.StartNew();
            string what = panel.ReplayDesktopAction(actions++);
            if (t.Elapsed.TotalMilliseconds > longestAction) { longestAction = t.Elapsed.TotalMilliseconds; slowestAction = what; }
        };
        var load = new List<Process>();
        Thread? reader = null;
        Process? client = null;
        string clientError = "";
        void Pump(Func<bool> until, int ms)
        {
            var wait = Stopwatch.StartNew();
            while (!until() && wait.ElapsedMilliseconds < ms) {
                var f = new DispatcherFrame();
                window.Dispatcher.BeginInvoke(DispatcherPriority.Background, () => f.Continue = false);
                Dispatcher.PushFrame(f);
                Thread.Sleep(5);
            }
        }
        try {
            window.Show(); window.UpdateLayout();
            if (profile is not null) ((System.Windows.Interop.HwndSource)PresentationSource.FromVisual(window)).AddHook((IntPtr h, int msg, IntPtr w, IntPtr l, ref bool handled) => {
                if (clock.IsRunning) { messages.Add((clock.Elapsed.TotalMilliseconds, msg)); if (messages.Count > 50000) messages.RemoveRange(0, 25000); }
                return IntPtr.Zero;
            });
            panel.PrepareDesktopReplay();
            Pump(() => AutomationWarmUp is { IsCompleted: true }, 10000);   // start-up work of the host, before any port is open
            window.heartbeatTimer.Start(); feed.Start(); act.Start();
            if (uia) {
                var handle = new System.Windows.Interop.WindowInteropHelper(window).Handle;
                if (external) {
                    // Another program connecting for the first time is part of what the host must survive (see WarmUpAutomation).
                    clock.Start();
                    client = Process.Start(new ProcessStartInfo(Environment.ProcessPath!, $"--uia-client {handle} {seconds * 1000}") { RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false });
                } else {
                    reader = new Thread(() => {
                        try { UiaClient(handle, () => Volatile.Read(ref stop), counts, firstRead.Set); }
                        catch (Exception ex) { uiaFailure = ex; firstRead.Set(); }
                        finally { Automation.RemoveAllEventHandlers(); }   // (another process just exits)
                    }) { IsBackground = true, Name = "UIA reader" };
                    reader.SetApartmentState(ApartmentState.MTA);
                    reader.Start();
                    // Not measured: this process's client subscribing and walking for the first time (the JIT of its client side, which
                    // no other program brings into the host).  Measured is the steady state of a window a client keeps listening to.
                    Pump(() => firstRead.IsSet, 8000);
                    Pump(() => false, 500);
                }
            }
            // Other programs keep the CPUs busy, as on the field PC; spinning threads inside this process would also delay its GC.
            for (int i = 0; i < cpuThreads; i++)
                load.Add(Process.Start(new ProcessStartInfo("cmd.exe", "/d /c for /l %i in (1,0,2) do rem") { CreateNoWindow = true, UseShellExecute = false })!);
            clock.Start();
            Dispatcher.PushFrame(frame);
            longestLease = Math.Max(longestLease, clock.Elapsed.TotalMilliseconds - lastLease);
            // However often they were redrawn, the plots are one automation element each: no per-label events (see PlotCanvas).
            if (!panel.PlotHidesLabelsFromAutomation() || System.Windows.Automation.Peers.UIElementAutomationPeer.CreatePeerForElement(window.errorPlot).GetChildren() is { Count: > 0 })
                throw new InvalidOperationException("A redrawn plot exposes its shapes and labels to UI Automation.");
        }
        finally {
            Volatile.Write(ref stop, true);
            feed.Stop(); act.Stop(); window.heartbeatTimer.Stop();
            foreach (var spinner in load) { try { spinner.Kill(); } catch (InvalidOperationException) { } spinner.Dispose(); }
            // The client marshals into this dispatcher; keep pumping until it has left.
            if (reader is not null) Pump(() => !reader.IsAlive, 5000);
            if (client is not null) {
                Pump(() => client.HasExited, 10000);
                bool hung = !client.HasExited;
                if (hung) client.Kill();   // a UIA call of its own can hang for its 20 s timeout; it printed its counts after every read
                if (client.StandardOutput.ReadToEnd().Split(Environment.NewLine, StringSplitOptions.RemoveEmptyEntries).LastOrDefault()?.Split(' ') is [var r, var n, var e]) { reads = int.Parse(r); elements = int.Parse(n); events = int.Parse(e); }
                else if (hung) clientError = " (client hung and was killed)";
                else clientError = $" (client exit {client.ExitCode}: {client.StandardError.ReadToEnd().Split(Environment.NewLine)[0]})";
                client.Dispose();
            }
            if (profile is not null) { window.Dispatcher.Hooks.OperationStarted -= Started; window.Dispatcher.Hooks.OperationCompleted -= Completed; }
            window.Close();
        }
        if (uiaFailure is not null) throw new InvalidOperationException("UIA client failed: " + uiaFailure.Message, uiaFailure);
        if (reader is not null) (reads, elements, events) = (counts[0], counts[1], counts[2]);
        string report = $"Desktop load ({seconds} s, {cpuThreads} spinning processes on {Environment.ProcessorCount} CPUs, " +
            $"UIA {(uia ? $"{(external ? "other process" : "in process")}: {reads} full reads of {elements} elements, {events} events" : "off")}, " +
            $"{batch} status blocks, {actions} edits/previews): UI lease max gap={longestLease:F1} ms, gaps>{SlowLeaseLogMs:F0}ms={over120}; slowest action={slowestAction} {longestAction:F1} ms.";
        if (profile is not null) {
            profile.AppendLine(report);
            foreach (var g in gaps.OrderByDescending(g => g.Gap).Take(3)) profile.AppendLine(g.Text);
            foreach (var g in timeline.Where(r => r.End - r.Start >= 30).GroupBy(r => r.Name).OrderByDescending(g => g.Max(r => r.End - r.Start)).Take(5))
                profile.AppendLine($"  {g.Max(r => r.End - r.Start),7:F1} ms max {g.Sum(r => r.End - r.Start),7:F0} ms total x{g.Count()} {g.Key}");
        }
        if (uia && reads == 0) throw new InvalidOperationException("UIA client never read the window" + clientError + ": " + report);
        if (actions < seconds) throw new InvalidOperationException(report);
        CheckLeaseGap(longestLease, report);
        return report + "\n";
    }

    // The automation script of the field stops: listens for structure, name, value, enabled, bounds and text events on the whole
    // window, then reads every element (name, bounds, state, value, text) again and again until told to stop; counts: reads, elements, events.
    internal static void UiaClient(IntPtr handle, Func<bool> stop, int[] counts, Action? afterRead = null)
    {
        var root = AutomationElement.FromHandle(handle);
        Automation.AddStructureChangedEventHandler(root, TreeScope.Subtree, (_, _) => Interlocked.Increment(ref counts[2]));
        Automation.AddAutomationPropertyChangedEventHandler(root, TreeScope.Subtree, (_, _) => Interlocked.Increment(ref counts[2]),
            AutomationElement.NameProperty, ValuePattern.ValueProperty, AutomationElement.IsEnabledProperty, AutomationElement.BoundingRectangleProperty);
        Automation.AddAutomationEventHandler(TextPattern.TextChangedEvent, root, TreeScope.Subtree, (_, _) => Interlocked.Increment(ref counts[2]));
        while (!stop()) {
            AutomationElementCollection all;
            // The window changes under the walk; a script retries when an element vanishes.
            try { all = root.FindAll(TreeScope.Descendants, System.Windows.Automation.Condition.TrueCondition); }
            catch (Exception ex) when (ex is ElementNotAvailableException or System.Runtime.InteropServices.COMException) { continue; }
            foreach (AutomationElement e in all) {
                if (stop()) break;
                try {
                    var c = e.Current;
                    _ = (c.Name, c.BoundingRectangle, c.IsOffscreen, c.HelpText, c.ControlType, c.AutomationId, c.IsEnabled);
                    if (e.TryGetCurrentPattern(ValuePattern.Pattern, out object v)) _ = ((ValuePattern)v).Current.Value;
                    if (e.TryGetCurrentPattern(TextPattern.Pattern, out object tp)) _ = ((TextPattern)tp).DocumentRange.GetText(-1);
                } catch (Exception ex) when (ex is ElementNotAvailableException or InvalidOperationException or System.Runtime.InteropServices.COMException) { }
            }
            counts[1] = all.Count;
            Interlocked.Increment(ref counts[0]);
            afterRead?.Invoke();
        }
    }

    // --desktop-load-profile: the replay idle and loaded, without and with a UIA client, at normal priority and, loaded, at the
    // above-normal priority the host takes while a port is open; with what filled each long gap.
    internal static void DesktopLoadProfile(string output)
    {
        var report = new StringBuilder();
        int n = Environment.ProcessorCount;
        using var self = Process.GetCurrentProcess();
        foreach (var (cpu, uia, external, above) in new[] { (0, false, false, false), (n, false, false, false), (0, true, false, false), (n, true, false, false),
                     (0, true, true, false), (n, true, true, false), (n, false, false, true), (n, true, false, true), (n, true, true, true) }) {
            self.PriorityClass = above ? ProcessPriorityClass.AboveNormal : ProcessPriorityClass.Normal;
            report.Append(above ? "[above normal] " : "[normal] ");
            try { DesktopLoadReplay(cpu, uia, 15, external, report); }
            catch (InvalidOperationException ex) { report.AppendLine("  FAILED: " + ex.Message.Split(Environment.NewLine)[0]); }
            System.IO.File.WriteAllText(output, report.ToString());
        }
        self.PriorityClass = ProcessPriorityClass.Normal;
    }
}

public sealed partial class XYPreviewPanel
{
    // The field session's range and shape (parameters.json): circle at (230, 0), r = 50.
    internal void PrepareDesktopReplay()
    {
        min1.Text = "-120"; max1.Text = "120"; min2.Text = "-150"; max2.Text = "150"; elbowMin.Text = "-169.6"; elbowMax.Text = "164.5";
        shape.SelectedIndex = 3; x.Text = "230"; y.Text = "0"; a.Text = "50";
        ClickPreview();
    }
    internal bool PlotHidesLabelsFromAutomation() => canvas.Children.OfType<System.Windows.Controls.TextBlock>().Any() &&
        System.Windows.Automation.Peers.UIElementAutomationPeer.CreatePeerForElement(canvas).GetChildren() is null or { Count: 0 };
    private void ClickPreview() => previewButton!.RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Primitives.ButtonBase.ClickEvent));
    // 检查并预览, then edits of position, speed and shape (circle <-> polygon), each invalidating the plan, between further 检查并预览.
    internal string ReplayDesktopAction(int i)
    {
        switch (i % 6) {
            case 1: x.Text = i % 12 < 6 ? "231" : "230"; return "x";
            case 2: v1.Text = i % 12 < 6 ? "14" : "14.5"; return "v1";
            case 4: shape.SelectedIndex = shape.SelectedIndex == 3 ? 8 : 3; return "shape";
            default: ClickPreview(); return "preview";
        }
    }
}
