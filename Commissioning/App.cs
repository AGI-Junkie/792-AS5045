using System.IO;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

public sealed class App : Application
{
    [STAThread]
    public static int Main(string[] args)
    {
        if (args.Contains("--self-test") || args.Contains("--language-self-test") || args.Contains("--upload-language-test") || args.Contains("--log-level-self-test"))
        {
            try
            {
                Lang.PersistChanges = false;
                int snapshot = Array.IndexOf(args, "--snapshot");
                string? path = snapshot >= 0 && snapshot + 1 < args.Length ? args[snapshot + 1] : null;
                if (args.Contains("--log-level-self-test")) ControlWindow.LoggingSelfTest();
                else if (args.Contains("--upload-language-test")) ControlWindow.UploadLanguageTest();
                else if (args.Contains("--language-self-test")) { Lang.SelfTest(); ControlWindow.LanguageSelfTest(path); }
                else ControlWindow.SelfTest(path);
                return 0;
            }
            catch (Exception ex) { File.WriteAllText("commissioning-self-test-error.txt", ex.ToString()); return 1; }
        }
        // The pure geometry and planning checks alone (a few seconds, no window): quick feedback while changing shapes or timing.
        if (args.Contains("--geometry-self-test"))
        {
            try { Lang.PersistChanges = false; Lang.Set(false, persist: false); PlanarArm.SelfTest(); DrawingTrajectory.SelfTest(); TimedTrajectory.SelfTest(); ExecutionTrajectory.SelfTest(); ReachabilityMask.SelfTest(); ExperimentArchiveTests.SelfTest(); return 0; }
            catch (Exception ex) { File.WriteAllText("commissioning-self-test-error.txt", ex.ToString()); return 1; }
        }
        // The experiment table of any folder as a CSV, with the plan/sample alignment check; no window.
        int report = Array.IndexOf(args, "--experiments-report");
        if (report >= 0)
        {
            if (report + 2 >= args.Length) return 2;
            var (runs, unreadable) = ExperimentArchive.List(args[report + 1]);
            File.WriteAllText(args[report + 2], ExperimentArchive.ReportCsv(runs), new UTF8Encoding(true));
            Console.Error.WriteLine($"{runs.Count} experiments, {unreadable} unreadable");
            return 0;
        }
        // Line-quality experiment against a real firmware on a serial port; see docs/UART_DIAGNOSIS.md.
        int stress = Array.IndexOf(args, "--uart-stress");
        if (stress >= 0) return UartStress.Run(args[(stress + 1)..]);
        // The release gate compares these numbers with main.c (see DeviceContract.cs).
        int contract = Array.IndexOf(args, "--print-contract");
        if (contract >= 0)
        {
            if (contract + 1 >= args.Length) return 2;
            File.WriteAllLines(args[contract + 1], DeviceContract.Lines());
            return 0;
        }
        Lang.Load();
        new App().Run(new ControlWindow(restoreParameters: true));
        return 0;
    }
}

public sealed partial class ControlWindow : Window
{
    private readonly ComboBox ports = new() { Width = 130, Margin = new Thickness(4) };
    private readonly TextBlock state = new() { Text = Lang.T("state.disconnected"), FontSize = 22, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4, 12, 4, 8) };
    private readonly TextBlock inputs = new() { Text = Lang.T("state.inputsEmpty"), Margin = new Thickness(4), TextWrapping = TextWrapping.Wrap };
    private readonly TextBlock logPath = new() { TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4) };
    private readonly TextBlock lastStop = new() { TextWrapping = TextWrapping.Wrap, Foreground = Brushes.Firebrick, Margin = new Thickness(4) };
    private readonly TextBox log = new() { IsReadOnly = true, AcceptsReturn = true, VerticalScrollBarVisibility = ScrollBarVisibility.Auto,
        HorizontalScrollBarVisibility = ScrollBarVisibility.Auto, FontFamily = new FontFamily("Consolas"), FontSize = 12,
        Language = System.Windows.Markup.XmlLanguage.GetLanguage("en-US") }; // Raw protocol text does not need language-dependent reformatting.
    private readonly Button enable = MakeButton(Lang.T("btn.enable"));
    private readonly Button home = MakeButton(Lang.T("btn.home"));
    private readonly Button manualReference = MakeButton(Lang.T("btn.manualRef"));
    private readonly Button connect = MakeButton(Lang.T("btn.connect"));
    private readonly Button langToggle = MakeButton(Lang.T("lang.toggle"));
    private readonly XYPreviewPanel xyPreview = new();
    private readonly List<Button> jogButtons = [];
    private readonly DispatcherTimer timer = new() { Interval = TimeSpan.FromMilliseconds(40) };
    // Same priority as stop/key-release input; rendering or a hung UI must still stop motion.
    private readonly DispatcherTimer heartbeatTimer = new(DispatcherPriority.Input) { Interval = TimeSpan.FromMilliseconds(40) };
    private SerialPort? serial;
    private StreamWriter? recording;
    private int logCharacters;
    private readonly StringBuilder pending = new();
    private int keyboardMask, mouseMask;
    private bool compatible, enabled, homing, stopPending;
    private DateTime openedAt, lastStateAt;
    private long lastHeartbeatSent;
    private PtpState? ptp;
    private DateTime lastPtpAt, pointRequestedAt;
    private bool stage2, homed, feedbackReady, hardwareReady, supportsManualReference, manualReferencePending;
    private bool arm2FeedbackEnabled = true;
    private uint pointSequence, pendingPoint;
    private uint lastMoveSequence;   // the last point move or RUN in this reference epoch; 0 while disabled (see AtStart)
    private readonly Queue<byte> pointWire = new();
    private byte[] pointPacket = [];
    private int pointAttempts;
    private bool supportsPacketRetry;
    private bool supportsTelemetry;
    private bool hardwareUart;
    private bool supportsShortHeartbeat;   // firmware advertises short_hb=1 in LINK
    private TelemetrySample? telemetry;
    private DateTime lastTelemetryAt, controlRequestedAt;
    private string? pendingControl;
    private uint requestEpoch;
    private ExperimentRecorder? experiment;
    private string? experimentCompleting;
    private string experimentPhase = "idle";
    internal static string ExperimentRoot => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "Experiments");
    private ExecutionTrajectory? uploading;
    private ExecutionTrajectory? uploaded;   // complete plan the device holds, waiting for 执行图形
    private ExecutionTrajectory? runPlan;    // plan of the RUN in flight; becomes `uploaded` again when it completes
    private bool uploadedRan;              // the held plan has already been drawn once; 执行图形 repeats it until the plan changes
    private int uploadNext, trajectoryIndex, trajectoryTotal;
    private bool supportsTrajectory, executing;
    private uint trajectoryEpoch, trajectoryRunSequence;
    private DateTime lastTrajectoryAt;
    private DateTime uploadStartedAt;
    private static readonly Regex TrajectoryPattern = new(@"^TRAJ_STATE state=([0-4]) count=(\d+) total=(\d+) index=(\d+) seq=(\d+) epoch=(\d+)$", RegexOptions.CultureInvariant);
    private string? lastLinkLine;
    private Func<string> regularStateText = () => Lang.T("state.disconnected");
    private static readonly Regex StatePattern = new(@"^STATE fw=(?:stage1-v3|stage2-v(?:[1-9]|10|11)) enabled=([01]) home=([0-7]) homed1=([01]) homed2=([01]) fault=([01]) enc=([01]) dir=([01])(?: enc2=([01]) dir2=([01]))?(?: arm2fb=([01]))?$", RegexOptions.CultureInvariant);

    public ControlWindow(bool restoreParameters = false)
    {
        log.TextChanged += (_, e) => {
            foreach (var change in e.Changes) logCharacters += change.AddedLength - change.RemovedLength;
        };
        BuildLayout();
        Lang.Changed += OnLanguageChanged;
        Closed += (_, _) => Lang.Changed -= OnLanguageChanged;
        if (restoreParameters) { xyPreview.RestoreSavedParameters(); xyPreview.RefreshRecentRuns(); }
        timer.Tick += (_, _) => Tick();
        heartbeatTimer.Tick += OnHeartbeatTick;
        xyPreview.PointMoveRequested += RequestPoint;
        xyPreview.TrajectoryRequested += RequestTrajectory;
        xyPreview.ExecuteRequested += RequestExecute;
        HookBoardPanel();
        xyPreview.LimitsChanged += OnLimitsChanged;
        PreviewKeyDown += OnKeyDown; PreviewKeyUp += OnKeyUp;
        Deactivated += (_, _) => {
            xyPreview.PausePlayback();
            keyboardMask = mouseMask = 0;
            foreach (var button in jogButtons) if (button.IsMouseCaptured) button.ReleaseMouseCapture();
            SafeAction(Heartbeat);
        };
        Closing += (_, _) => Disconnect(() => Lang.T("fault.windowClose"));
        RefreshPorts(); UpdateControls();
    }

    private static Button MakeButton(string title) => new() { Content = title, Margin = new Thickness(2), Padding = new Thickness(12, 3, 12, 3), MinHeight = 28 };
    private void OnLanguageChanged()
    {
        enable.Content = Lang.T("btn.enable");
        home.Content = Lang.T("btn.home");
        manualReference.Content = Lang.T("btn.manualRef");
        manualReference.ToolTip = Lang.T("btn.manualRef.tip");
        connect.Content = serial is null ? Lang.T("btn.connect") : Lang.T("btn.disconnect");
        langToggle.Content = Lang.T("lang.toggle");
        readInputs.Content = Lang.T("btn.readInputs");
        Title = Lang.T("app.title");
        RefreshLanguageLayout();
        xyPreview.RefreshLanguage();
        langToggle.ToolTip = Lang.T("lang.toggle.tip");
        System.Windows.Automation.AutomationProperties.SetName(langToggle, Lang.T("lang.toggle.tip"));
        foreach (var element in new FrameworkElement[] { state, lastStop, inputs, liveValues, experimentResult, logPath }) Lang.RefreshText(element);
        if (lastLinkLine is { } link) connectionBadge.ToolTip = Lang.T("link.diag") + link;
        if (compatible) ShowDeviceState();
        UpdateControls(synchronize: false);
        Language = System.Windows.Markup.XmlLanguage.GetLanguage(Lang.IsEn ? "en-NZ" : "zh-CN");
    }
    private void AddJog(Panel row, string title, int bit)
    {
        var button = MakeButton(title); WorkbenchTheme.Primary(button); jogButtons.Add(button); row.Children.Add(button);
        button.PreviewMouseLeftButtonDown += (_, e) =>
        {
            if (!button.IsEnabled || !CanJog) return;
            mouseMask = bit; button.CaptureMouse(); e.Handled = true;
        };
        button.PreviewMouseLeftButtonUp += (_, e) => { mouseMask = 0; button.ReleaseMouseCapture(); e.Handled = true; SafeAction(Heartbeat); };
        button.LostMouseCapture += (_, _) => { mouseMask = 0; };
    }
    private static int KeyBit(Key key) => key switch { Key.A => 1, Key.D => 2, Key.Left => 4, Key.Right => 8, _ => 0 };
    private static bool IsTextInteraction(ModifierKeys modifiers, bool textFocused) => modifiers != ModifierKeys.None || textFocused;
    private void OnKeyDown(object sender, KeyEventArgs e)
    {
        if (e.Key is Key.Space or Key.Escape) { Stop(); e.Handled = true; return; }
        // Ctrl+C/Ctrl+A and navigation inside the log are native text operations.
        if (IsTextInteraction(Keyboard.Modifiers, IsEditingControls)) return;
        if (e.Key == Key.C && !e.IsRepeat) { RequestEnable(); e.Handled = true; return; }
        if (e.Key == Key.R && !e.IsRepeat) { RequestHome(); e.Handled = true; return; }
        int bit = KeyBit(e.Key);
        if (bit != 0) { if (CanJog && !e.IsRepeat) keyboardMask |= bit; e.Handled = true; }
    }
    private void OnKeyUp(object sender, KeyEventArgs e)
    {
        int bit = KeyBit(e.Key);
        if (bit != 0)
        {
            keyboardMask &= ~bit;
            if (IsTextInteraction(Keyboard.Modifiers, IsEditingControls)) return;
            e.Handled = true; SafeAction(Heartbeat);
        }
    }
    private bool PointBusy => pendingControl is not null || pendingPoint != 0 || ptp is { Active: > 0 } || uploading is not null || executing || StoreBusy || pendingBoardSave is not null;
    private bool CanPoint => LimitsReady && stage2 && compatible && enabled && homed && feedbackReady && !homing && !stopPending && !PointBusy &&
        keyboardMask == 0 && mouseMask == 0 && ptp is { Locked: false, Referenced: true, Epoch: > 0 } &&
        (DateTime.UtcNow - lastPtpAt).TotalSeconds < 2 && (!supportsTelemetry ||
        telemetry is { Referenced: true, Inputs: 0 } && telemetry.Epoch == ptp.Epoch && (DateTime.UtcNow-lastTelemetryAt).TotalSeconds < 2);
    private bool CanManualReference => LimitsReady && supportsManualReference && compatible && enabled && hardwareReady && !homing && !stopPending && !PointBusy && (uploaded is null || uploadedRan) &&   // firmware refuses only a plan still waiting for RUN (TRAJ_READY), not a finished one (TRAJ_DONE)
        keyboardMask == 0 && mouseMask == 0 && ptp is { Locked: false, Epoch: > 0 } &&
        (DateTime.UtcNow - lastPtpAt).TotalSeconds < 2;
    private bool CanJog => LimitsReady && compatible && enabled && !homing && !stopPending && !PointBusy && IsActive && (DateTime.UtcNow - lastStateAt).TotalSeconds < 2;
    // A focused slider takes the arrow keys itself; they must not also start a jog.
    private bool IsEditingControls => Keyboard.FocusedElement is TextBoxBase or ComboBox or Slider || xyPreview.IsKeyboardFocusWithin;
    private void RefreshPorts()
    {
        if (serial is not null) return;
        ports.ItemsSource = SerialPort.GetPortNames().Order().ToArray();
        if (ports.Items.Count > 0) ports.SelectedIndex = 0;
    }
    private void Connect()
    {
        if (ports.SelectedItem is not string port) { Lang.Text(state, () => Lang.T("state.needPort")); return; }
        try
        {
            string directory = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "Logs");
            Directory.CreateDirectory(directory);
            string path = Path.Combine(directory, $"stage1-{DateTime.Now:yyyyMMdd-HHmmss-fff}.log");
            recording = new StreamWriter(path, false, new UTF8Encoding(false)) { AutoFlush = true };
            Lang.Text(logPath, () => Lang.T("state.logSaved") + path);
            // The firmware answers on USB only while the host holds DTR; the KitProg bridge keeps the old DTR-off setting.
            usbLink = UsbLink.IsFirmwareCdc(port);
            serial = new SerialPort(port, 115200, Parity.None, 8, StopBits.One) { WriteTimeout = 100, ReadTimeout = 10, DtrEnable = usbLink, RtsEnable = false };
            serial.DataReceived += (_, _) => WakeSerialPump();
            serial.Open(); openedAt = DateTime.UtcNow; lastStateAt = DateTime.MinValue;
            lastHeartbeatSent = 0;
            ResetWireSession();
            lastFirmwareBuild = "unknown";
            protocolMatches = false; protocolVersion = 0;
            ResetControlTransport();
            hardwareUart = false;
            pending.Clear(); compatible = enabled = homing = false; stopPending = true;
            keyboardMask = mouseMask = 0;
            Send(0x79); Heartbeat(); Send((byte)'s'); Send((byte)'i');
            Lang.Text(state, () => Lang.T("state.connected"));
            connect.Content = Lang.T("btn.disconnect"); ports.IsEnabled = false; timer.Start(); heartbeatTimer.Start();
            StartWireWriter();
            Append("HOST connected " + port + (usbLink ? " (USB CDC: one copy per byte, one write per frame)" : " (UART)"));
            Append("HOST UI " + Title + "; executable=" + Environment.ProcessPath);
        }
        catch (Exception ex) { Disconnect(() => Lang.T("state.connectFail") + ex.Message); }
        UpdateControls();
    }
    private void Send(byte command, double gapMs = 1)
    {
        if (serial is not { IsOpen: true }) return;
        // Framed mode: queue the raw byte so the writer thread emits it between
        // frames. A byte injected inside a frame is discarded by the firmware
        // parser, and two threads writing the same port would interleave anyway.
        if (framedControl) { lock (wireSync) { urgentWire.Enqueue(command); } return; }
        byte[] bytes = [command];
        if (hardwareUart) { serial.Write(bytes, 0, 1); return; }
        for (int i = 0; i < 3; i++)
        {
            serial.Write(bytes, 0, 1);
            // Sleep(1) can round up to a Windows timer quantum for EACH copy.
            // Keep the required wire gap bounded (~3 ms total per packet).
            long start = Stopwatch.GetTimestamp();
            while (Stopwatch.GetElapsedTime(start).TotalMilliseconds < gapMs) Thread.SpinWait(20);
        }
    }
    private void Heartbeat()
    {
        if (serial is not { IsOpen: true }) return;
        long now = Stopwatch.GetTimestamp();
        if (framedControl) {
            // Publish, then ask the writer for a frame now: key release and window
            // deactivation must reach the firmware without waiting for the cadence.
            PublishHeartbeatLease();
            ForceHeartbeatNow();
            return;
        }
        double gap = lastHeartbeatSent == 0 ? 0 : Stopwatch.GetElapsedTime(lastHeartbeatSent, now).TotalMilliseconds;
        Send((byte)(0x41 | ((CanJog ? (keyboardMask | mouseMask) : 0) << 1)));
        lastHeartbeatSent = now;
        if (gap > 150) Append($"HOST heartbeat send gap={gap:F0} ms", LogLevel.Warning);
    }
    private void OnHeartbeatTick(object? sender, EventArgs e)
    {
        SafeAction(() => {
            // This callback only republishes the lease; the writer thread does all
            // framed I/O. It stays on the UI thread on purpose: an unresponsive
            // window stops republishing, the lease goes stale, the writer stops
            // sending, and the firmware deadman stops the machine.
            double age = (DateTime.UtcNow - (compatible ? lastStateAt : openedAt)).TotalSeconds;
            if (age > (compatible ? 2 : 8)) { Disconnect(() => Lang.T("state.fwTimeout")); return; }
            PublishHeartbeatLease();
        });
    }
    private void PumpSerial()
    {
        if (serial is not { IsOpen: true }) return;
        pending.Append(serial.ReadExisting());
        string text = pending.ToString(); int end;
        while ((end = text.IndexOf('\n')) >= 0)
        {
            HandleLine(text[..end].TrimEnd('\r')); text = text[(end + 1)..];
        }
        pending.Clear();
        if (text.Length <= 4096) pending.Append(text);
        else Append("HOST discarded oversized/unterminated serial line", LogLevel.Warning);
    }
    // Each trajectory node waits for its ACK, so a 40 ms poll adds ~20 ms per node. Read as soon as bytes
    // arrive (still on the UI thread); the Tick keeps polling as the fallback if the driver never raises this.
    private int pumpQueued;
    private void WakeSerialPump()
    {
        if (Interlocked.Exchange(ref pumpQueued, 1) == 1) return;
        Dispatcher.BeginInvoke(DispatcherPriority.Input, () => { Volatile.Write(ref pumpQueued, 0); SafeAction(PumpSerial); });
    }
    private void Tick()
    {
        SafeAction(() =>
        {
            if (serial is not { IsOpen: true }) throw new IOException(Lang.T("fault.ioClosed"));
            PumpSerial();
            RetryPendingPoint(DateTime.UtcNow);
            PollStoreJob(DateTime.UtcNow);
            CheckControlTimeout(DateTime.UtcNow);
            CheckTelemetryTimeout(DateTime.UtcNow);
            if ((uploading is not null || executing) && ((DateTime.UtcNow - lastPtpAt).TotalSeconds > 2 ||
                (executing && (DateTime.UtcNow - lastTrajectoryAt).TotalSeconds > 2))) {
                Append("HOST trajectory status timeout; stopping", LogLevel.Error); Stop();
            }
            if (framedControl) { /* The 8 ms control timer owns all framed writes. */ }
            else if (LimitsReady && hardwareUart && pointWire.Count > 0 && serial is { IsOpen: true })
            {
                FlushHardwarePacket(serial.Write);
            }
            else
            {
                // Legacy GPIO UART still needs triplication and inter-byte gaps.
                for (int i = 0; LimitsReady && i < 8 && pointWire.Count > 0; i++) Send(pointWire.Dequeue(), 1.3);
            }
            UpdateControls();
        });
    }
    private void HandleLine(string line)
    {
        Append(line);
        var match = StatePattern.Match(line);
        if (match.Success)
        {
            compatible = true; lastStateAt = DateTime.UtcNow;
            stage2 = line.StartsWith("STATE fw=stage2-");
            framedControl = line.StartsWith("STATE fw=stage2-v7 ") || line.StartsWith("STATE fw=stage2-v8 ") || line.StartsWith("STATE fw=stage2-v9 ") || line.StartsWith("STATE fw=stage2-v10 ") || line.StartsWith("STATE fw=stage2-v11 ");
            heartbeatTimer.Interval = TimeSpan.FromMilliseconds(framedControl ? 8 : 40);
            bool fastLink = framedControl || line.StartsWith("STATE fw=stage2-v6 ");
            if (hardwareUart != fastLink)
                Append(framedControl ? $"HOST transport: v7 hardware UART 115200; one dedicated writer thread owns the wire, " + (usbLink ? "1 copy/byte (USB CDC)" : $"2 copies/byte, >={ControlByteGapMs.ToString(System.Globalization.CultureInfo.InvariantCulture)} ms spacing") :
                    fastLink ? "HOST transport: hardware UART 115200; paced chunks <=8 bytes/40ms, CRC/ACK/retry retained" :
                    "HOST transport: legacy UART; paced triplicate writes");
            hardwareUart = fastLink;
            supportsTelemetry = hardwareUart || line.StartsWith("STATE fw=stage2-v5 ");
            supportsPacketRetry = supportsTelemetry || line.StartsWith("STATE fw=stage2-v4 ");
            supportsManualReference = supportsPacketRetry || line.StartsWith("STATE fw=stage2-v3 ");
            supportsTrajectory = supportsManualReference || line.StartsWith("STATE fw=stage2-v2 ");
            if (!stage2) { ptp = null; reportedLimits = null; reportedProfile = null; }
            homed = match.Groups[3].Value == "1" && match.Groups[4].Value == "1";
            arm2FeedbackEnabled = !match.Groups[10].Success || match.Groups[10].Value == "1";
            hardwareReady = match.Groups[5].Value == "0" && match.Groups[6].Value == "1" &&
                (!arm2FeedbackEnabled || !match.Groups[8].Success || match.Groups[8].Value == "1");
            feedbackReady = hardwareReady && match.Groups[7].Value == "1" &&
                (!arm2FeedbackEnabled || !match.Groups[8].Success || match.Groups[9].Value == "1");
            bool wasEnabled = enabled;
            enabled = match.Groups[1].Value == "1"; homing = match.Groups[2].Value != "0";
            if ((pendingControl == "使能" && enabled) || (pendingControl == "回零" && homing)) pendingControl = null;
            if (pendingPoint != 0 && (!enabled || !hardwareReady || homing)) Stop();
            if ((uploading is not null || executing) && (!supportsTrajectory || !enabled || !homed || !feedbackReady || homing)) Stop();
            if (!enabled) {
                stopPending = false; lastMoveSequence = 0;   // nothing moves while disabled; a reboot restarts the sequences
                // A reboot may reuse epoch 1 while restoring default ranges. Every
                // disabled STATE requires the following full range report again.
                // The report follows in the same status block, so a device that stays disabled holds the display meanwhile (see UpdateControls).
                if (!wasEnabled && reportedLimits is not null && limitsReportSince == default) limitsReportSince = DateTime.UtcNow;
                reportedLimits = null; reportedProfile = null;
            }
            regularStateText = () =>
                $"{(match.Groups[1].Value == "1" ? Lang.T("state.enabled") : Lang.T("state.motionOff"))} · {Lang.T("phase." + match.Groups[2].Value)}" +
                $"\n{Lang.T("status.ref")}arm1 {(match.Groups[3].Value == "1" ? Lang.T("state.refEstablished") : Lang.T("state.refMissing"))} / arm2 {(match.Groups[4].Value == "1" ? Lang.T("state.refEstablished") : Lang.T("state.refMissing"))}" +
                $" · {Lang.T("lbl.encoder")} {(match.Groups[6].Value == "1" ? Lang.T("state.encOk") : Lang.T("state.encBad"))} · {Lang.T("lbl.driveFault")}{(Lang.IsEn ? ": " : "：")}{(match.Groups[5].Value == "1" ? Lang.T("state.faultOn") : Lang.T("state.faultOff"))}";
            ShowDeviceState();
            UpdateControls();
        }
        else if (line.StartsWith("PTP_STATE ")) {
            ptp = stage2 ? PtpState.Parse(line) : null; lastPtpAt = DateTime.UtcNow;
            if (ptp is null && PointBusy) { Lang.Text(lastStop, () => Lang.T("fault.badPtp")); Stop(); return; }
            if (ptp is not null) {
                ObserveControlEpoch(ptp.Epoch);
                ObserveJointLimits(ptp);
                pointSequence = Math.Max(pointSequence, ptp.Sequence);
                if (pendingPoint != 0 && ptp.Epoch != requestEpoch &&
                    (!manualReferencePending || ptp.Epoch != (requestEpoch == uint.MaxValue ? 1u : requestEpoch+1u))) {
                    Lang.Text(lastStop, () => Lang.T("fault.epochChanged")); Stop(); return;
                }
                if (uploading is null && pendingPoint != 0 && ptp.Sequence == pendingPoint &&
                    ptp.Epoch == (manualReferencePending ? (requestEpoch == uint.MaxValue ? 1u : requestEpoch+1u) : requestEpoch)) {
                    if (manualReferencePending) {
                        if (ptp.Locked || !ptp.Referenced || ptp.Active!=0) {
                            Lang.Text(lastStop, () => Lang.T("fault.manualRefAck")); Stop(); return;
                        }
                        manualReferenceConfirmedEpoch=ptp.Epoch;
                    }
                    pendingPoint = 0; pointWire.Clear(); manualReferencePending = false;
                }
            }
            if ((uploading is not null || executing) && (ptp is null || ptp.Locked || !ptp.Referenced || ptp.Epoch != trajectoryEpoch)) Stop();
            ShowDeviceState();
            UpdateControls();
        }
        else if (line.StartsWith("PTP_EPOCH ") && uint.TryParse(line.AsSpan(10), out uint announcedEpoch) && announcedEpoch != 0) {
            // Compact re-announcement: the firmware repeats this while we are still on
            // a previous reference epoch, so a status block lost to a full transmit
            // queue cannot leave us stuck on the old epoch until the next 500 ms poll.
            ObserveControlEpoch(announcedEpoch);
            if (ptp is not null) ptp = ptp with { Epoch = announcedEpoch };
            UpdateControls();
        }
        else if (line.StartsWith("CONTROL seq=") && uint.TryParse(line.AsSpan(12), out uint lastControl))
            lock (wireSync) { controlSequence = Math.Max(controlSequence, lastControl); }
        else if (line.StartsWith("MOTION_RATES ")) { ObserveMotionRates(line); }
        else if (line.StartsWith("TELEM_RATE ")) { ObserveTelemetryRate(line); }
        else if (line.StartsWith("TELEM_DIAG ")) { ObserveTelemetryDiagnostics(line); }
        else if (line.StartsWith("FAULT_SNAPSHOT ")) { HandleFaultSnapshot(line); }
        else if (line.StartsWith("LAST_STOP ")) { HandleLatchedStop(line); }
        else if (line.StartsWith("PTP_LIMITS ") || line.StartsWith("PTP_RATES ")) { HandleLimitsAck(line); UpdateControls(); }
        else if (line.StartsWith("TRAJ_SLOT ") || line.StartsWith("TRAJ_STORE ")) ObserveStoreLine(line);
        else if (line.StartsWith("TRAJ_STATE ")) HandleTrajectoryState(line);
        else if (line.StartsWith("TELEM ")) HandleTelemetry(line);
        else if (line.StartsWith("LINK ")) {
            lastLinkLine = line; connectionBadge.ToolTip = Lang.T("link.diag") + line;
            // The compact heartbeat is only sent to firmware that advertises it; an
            // older firmware keeps receiving the full 17-byte heartbeat frame.  Say so
            // in the log, because "which heartbeat is running" decides the margin.
            lastFirmwareBuild = line.Split(' ').FirstOrDefault(w => w.StartsWith("fw=", StringComparison.Ordinal))?[3..] ?? "unknown";
            RefreshFirmwareStatus();
            string answeredOn = line.Split(' ').FirstOrDefault(w => w.StartsWith("link=", StringComparison.Ordinal))?[5..] ?? "uart";
            string linkWarning = usbLink && answeredOn != "usb" ? "HOST link: the USB CDC port is open but the firmware answers on the UART (DTR not seen)"
                : !usbLink && answeredOn == "usb" ? "HOST link: the firmware answers on USB but this port is not its USB CDC device" : "";
            if (linkWarning != linkWarned) { linkWarned = linkWarning; if (linkWarning.Length > 0) Append(linkWarning, LogLevel.Warning); }
            bool compact = line.Contains("short_hb=1");
            if (compact != supportsShortHeartbeat)
                Append(compact ? "HOST heartbeat: compact 6 wire bytes per beat, 20 ms idle target"
                               : "HOST heartbeat: legacy 36 wire bytes per beat, 40 ms idle target");
            supportsShortHeartbeat = compact;
            supportsMotionRates = line.Split(' ').Contains("motion_rates=1");
            supportsLiveRates = line.Split(' ').Contains("rates_live=1");
            supportsJointLimits = line.Split(' ').Contains("joint_limits=1");
            ReadCapabilities(line.Split(' '));
            // Compatibility is the numeric wire-contract version, not the build id: a new build with the same
            // contract stays usable, and a changed contract is refused even if the build id did not change.
            protocolVersion = line.Split(' ').Where(w => w.StartsWith("proto=", StringComparison.Ordinal))
                .Select(w => int.TryParse(w[6..], out int v) ? v : 0).FirstOrDefault();
            protocolMatches = protocolVersion == DeviceContract.ProtocolVersion;
            UpdateControls();
        }
        else if (line.StartsWith("PTP done ")) {
            if (experimentPhase == "point") experimentCompleting = "complete";
        }
        else if (loadingSlot is not null && uploading is not null && line.StartsWith("PTP refused:")) FallBackToUpload(line);
        else if (line.StartsWith("PTP refused:")) {
            // A queued packet may be refused after STOP invalidates its reference.
            // Keep the stop cause unless this refusal belongs to a pending request.
            if (pendingPoint != 0 || lastStop.Text.Length == 0) Lang.Text(lastStop, () => ExplainFault(line));
            if (pendingPoint != 0) Stop();
        }
        else if (line.StartsWith("INPUTS:")) Lang.Text(inputs, () => Lang.T("inputs.raw") + line[7..]);
        else if (line.StartsWith("STOP:") || line.StartsWith("ENABLE refused:") || line.StartsWith("HOME refused:")) {
            if (!(stopPending && line.Contains("operator stop") && lastStop.Text.Length>0)) Lang.Text(lastStop, () => ExplainFault(line));
            InvalidateManualReferenceFeedback();
            if (PointBusy) Stop();
        }
        else if (line.StartsWith("ENABLE OK:")) Lang.Text(lastStop, () => "");
    }
    private void RequestEnable()
    {
        if (!enable.IsEnabled) return;
        // Clicked while the display holds through a status block: act once the block has confirmed (or not) the ranges.
        if (!LimitsReady) { enableAfterReport = limitsReportSince != default; return; }
        SafeAction(() => { keyboardMask = mouseMask = 0; pendingControl = "使能"; controlRequestedAt = DateTime.UtcNow;
            if (framedControl) { lock (wireSync) { pendingControlType = 7; lastControlHeartbeat = 0; } }
            else { Send(0x41); Send(0x63); }
            Lang.Text(state, () => Lang.T("state.enableReq")); UpdateControls(); });
    }
    private void RequestHome()
    {
        if (!home.IsEnabled || !LimitsReady || keyboardMask != 0 || mouseMask != 0) return;
        SafeAction(() => { pendingControl = "回零"; controlRequestedAt = DateTime.UtcNow;
            if (framedControl) { lock (wireSync) { pendingControlType = 8; lastControlHeartbeat = 0; } }
            else { Send(0x41); Send(0x71); }
            Lang.Text(state, () => Lang.T("state.homeReq")); UpdateControls(); });
    }
    private void RequestManualReference()
    {
        if (!CanManualReference || ptp is null) return;
        SafeAction(() => {
            if (pointSequence == uint.MaxValue) throw new InvalidOperationException(Lang.T("fault.seqExhausted"));
            pendingPoint = ++pointSequence; pointRequestedAt = DateTime.UtcNow; manualReferencePending = true;
            manualReferenceConfirmedEpoch=0;
            QueuePointPacket(PtpProtocol.EncodePacket(5, pendingPoint, ptp.Epoch, new JointPose(0, 0), 0));
            Append("HOST manual reference: operator declares both links straight at q1=q2=0; no movement requested");
            ShowDeviceState(); UpdateControls();
        });
    }
    private void ShowDeviceState()
    {
        if (!stopPending && manualReferencePending) { Lang.Text(state, () => Lang.T("state.manualRef")); return; }
        if (!stopPending && uploading is not null) { Lang.Text(state, () => uploadNext >= uploading.Nodes.Length ? Lang.T("state.uploadStart") : Lang.T("state.uploading", Math.Min(uploadNext, uploading.Nodes.Length), uploading.Nodes.Length)); return; }
        if (!stopPending && uploaded is not null) { Lang.Text(state, () => uploadedRan ? Lang.T("state.uploadedRan", uploaded.Nodes.Length) : Lang.T("state.uploaded", uploaded.Nodes.Length)); return; }
        if (!stopPending && executing) { Lang.Text(state, () => Lang.T("state.executing", trajectoryIndex, trajectoryTotal)); return; }
        Lang.Text(state, () => stopPending ? Lang.T("state.stopReq") : PointBusy ?
            (ptp is { Active: 2 } ? Lang.T("state.pointSettle") : ptp is { Active: 1 } ? Lang.T("state.pointMove") : Lang.T("state.pointWait")) +
            Lang.T("state.pointNote") : regularStateText());
    }
    private void RequestTrajectory(ExecutionTrajectory plan) => RequestTrajectory(plan, allowBoard: true);
    // allowBoard: a plan the board already stores is loaded from its slot (seconds) instead of uploaded node by node (a minute).
    private void RequestTrajectory(ExecutionTrajectory plan, bool allowBoard)
    {
        if (!CanPoint || !supportsTrajectory || ptp is null) return;
        SafeAction(() => {
            ExecutionTrajectory.Validate(plan.Nodes, ptp.Limits,xyPreview.RequestedProfile);
            if (!AtStart(plan.Nodes[0].Pose)) { Lang.Text(lastStop, () => Lang.T("fault.notAtStart")); UpdateControls(); return; }
            if ((ulong)pointSequence + (uint)plan.Nodes.Length + 1 > uint.MaxValue) throw new InvalidOperationException(Lang.T("fault.seqExhaustedHome"));
            StartExperiment("trajectory",pointSequence+1,plan.Nodes);
            uploading = plan; uploaded = null; uploadedRan = false; trajectoryRanOnce = false; uploadNext = 0; trajectoryEpoch = ptp.Epoch; uploadStartedAt = DateTime.UtcNow;
            xyPreview.PausePlayback();
            if (allowBoard && StoreAvailable && TrajectoryArchive.SlotFor(boardSlots, plan.Nodes) is { } stored)
            {
                loadingSlot = stored.Slot;
                Append($"HOST trajectory load from board slot {stored.Slot} nodes={plan.Nodes.Length} duration={plan.Duration:F3}s; no automatic approach");
                SendLoadPacket();
                return;
            }
            Append($"HOST trajectory upload start nodes={plan.Nodes.Length} duration={plan.Duration:F3}s; no automatic approach");
            SendNextTrajectoryPacket();
        });
    }
    private void RequestExecute()
    {
        if (uploaded is not { } plan || !CanPoint || ptp is null || ptp.Epoch != trajectoryEpoch || !AtStart(plan.Nodes[0].Pose)) return;
        SafeAction(() => {
            // The first run continues the upload's recording; a repeat starts its own (the previous one finished with "complete").
            if (experiment is null || experimentPhase != "trajectory") StartExperiment("trajectory", pointSequence + 1, plan.Nodes);
            uploading = runPlan = plan; uploaded = null; uploadNext = plan.Nodes.Length;
            Append("HOST trajectory run requested; the device checks the start position again");
            SendNextTrajectoryPacket();
        });
    }
    private void SendNextTrajectoryPacket()
    {
        if (uploading is not { } plan) return;
        bool run = uploadNext == plan.Nodes.Length;
        var node = run ? new ExecutionNode(new JointPose(0, 0), 0) : plan.Nodes[uploadNext];
        pendingPoint = ++pointSequence; pointRequestedAt = DateTime.UtcNow;
        QueuePointPacket(PtpProtocol.EncodePacket(run ? (byte)4 : uploadNext == 0 ? (byte)2 : (byte)3,
            pendingPoint, trajectoryEpoch, node.Pose, uploadNext == 0 ? (ushort)plan.Nodes.Length : node.Milliseconds));
        if (run) trajectoryRunSequence = lastMoveSequence = pendingPoint;
        ShowDeviceState(); UpdateControls();
    }
    private void HandleTrajectoryState(string line)
    {
        var m = TrajectoryPattern.Match(line); var values = new uint[6];
        bool valid = m.Success;
        for (int i = 0; valid && i < 6; i++) valid = uint.TryParse(m.Groups[i + 1].Value, out values[i]);
        if (!valid || values[1] > DeviceContract.TrajCapacity || values[2] > DeviceContract.TrajCapacity || values[3] > values[1]) {
            if (uploading is not null || executing) { Append("HOST malformed trajectory status", LogLevel.Error); Stop(); }
            return;
        }
        uint mode = values[0], count = values[1], total = values[2], seq = values[4], epoch = values[5];
        if (CompleteBoardLoad(seq, epoch, mode, count, total)) return;
        if (uploading is { } plan && seq == pendingPoint && epoch == trajectoryEpoch)
        {
            bool run = uploadNext == plan.Nodes.Length;
            uint expected = (uint)(run ? plan.Nodes.Length : uploadNext + 1);
            if (count != expected || total != plan.Nodes.Length || (run ? mode is not (3 or 4) : mode != (count == total ? 2 : 1))) {
                Append("HOST trajectory acknowledgement mismatch", LogLevel.Error); Stop(); return;
            }
            pendingPoint = 0; lock (wireSync) { pointWire.Clear(); }
            if (run) { uploading = null; executing = true; }
            else if (++uploadNext == plan.Nodes.Length) {
                // Fully loaded: the device holds the plan and does not move until the operator asks for RUN.
                uploading = null; uploaded = plan; uploadedRan = false;
                Append("HOST trajectory upload complete; waiting for the operator to start it (no automatic run)");
                ContinueBoardSave(plan);
            }
            else SendNextTrajectoryPacket();
        }
        if (executing && seq == trajectoryRunSequence && epoch == trajectoryEpoch) {
            if (mode is not (3 or 4) || count != total || total < 2) { Stop(); return; }
            lastTrajectoryAt = DateTime.UtcNow; trajectoryIndex = (int)values[3]; trajectoryTotal = (int)total - 1;
            if (mode == 4) {
                // The device keeps the nodes after a run (state 4): hold the plan so 执行图形 can repeat it.
                executing = false; experimentCompleting = "complete"; uploaded = supportsTrajRepeat ? runPlan : null; uploadedRan = uploaded is not null; trajectoryRanOnce = true; Append(arm2FeedbackEnabled ? "HOST trajectory complete; firmware checks passed for both motor encoders" : "HOST trajectory complete; firmware checks passed for the arm1 motor encoder (arm2 has no feedback)");
                // A firmware that does not declare traj_repeat=1 refuses a second RUN of the plan it holds (and stops): upload again instead.
                if (!supportsTrajRepeat) Append("HOST this firmware cannot repeat a held plan (its LINK line has no traj_repeat=1); upload the plan again to draw it again");
            }
        }
        // The device dropped the plan it held (new speeds invalidate it): forget it too, or 执行图形 would offer a plan that is gone.
        if (mode == 0 && uploaded is not null && uploading is null && !executing) { uploaded = null; uploadedRan = false; Append("HOST the device dropped the held plan; upload it again"); }
        ShowDeviceState(); UpdateControls();
    }
    private void RequestPoint(JointPose pose)
    {
        if (!CanPoint || (uploaded is not null && !uploadedRan) || ptp is null || !ptp.Limits.Contains(PtpProtocol.Quantize(pose))) return;
        SafeAction(() => {
            if (pointSequence == uint.MaxValue) throw new InvalidOperationException(Lang.T("fault.seqExhaustedRef"));
            if (xyPreview.RequestedProfile is not { } profile || telemetry is null) return;
            double rate=Math.Min(DeviceContract.PlanRateMax,Math.Min(profile.R1,profile.R2));
            StartExperiment("point",pointSequence+1,[new(pose,0)]);
            pendingPoint = lastMoveSequence = ++pointSequence; pointRequestedAt = DateTime.UtcNow;
            QueuePointPacket(PtpProtocol.Encode(pendingPoint, ptp.Epoch, pose,(ushort)Math.Floor(rate)));
            xyPreview.PausePlayback();
            Append($"HOST PTP request seq={pendingPoint} q1={pose.Q1:F2} q2={pose.Q2:F2} rate={Math.Floor(rate)} usteps/s; global joint limits; joint-space move, not a straight XY path");
            ShowDeviceState();
            UpdateControls();
        });
    }
    private void QueuePointPacket(byte[] packet)
    {
        pointPacket = packet; pointAttempts = 1;
        requestEpoch = ptp?.Epoch ?? 0;
        lock (wireSync) {
            pointWire.Clear();
            foreach (byte b in packet) pointWire.Enqueue(b);
        }
        wireWake.Set();
    }
    private void FlushHardwarePacket(Action<byte[], int, int> write)
    {
        // Leave receive/encoder and heartbeat time between USB UART bursts.
        byte[] chunk = pointWire.Take(8).ToArray();
        if (chunk.Length == 0) return;
        write(chunk, 0, chunk.Length);
        for (int i = 0; i < chunk.Length; i++) pointWire.Dequeue();
    }
    private void RetryPendingPoint(DateTime now)
    {
        if (pendingPoint == 0) return;
        double age = (now - pointRequestedAt).TotalSeconds;
        if (age >= 3) {
            uint sequence = pendingPoint; int attempts = pointAttempts;
            Lang.Text(lastStop, () => Lang.T("fault.pointTimeout", sequence, attempts));
            Append($"HOST PTP acknowledgement timeout seq={pendingPoint} attempts={pointAttempts}; stopping", LogLevel.Error); Stop(); return;
        }
        // Same seq/epoch/payload: firmware only acknowledges duplicates, never restarts motion.
        bool wireEmpty;
        lock (wireSync) { wireEmpty = pointWire.Count == 0; }
        if (!supportsPacketRetry || pointAttempts >= 3 || age < pointAttempts || !wireEmpty) return;
        if (!enabled || !hardwareReady || homing || stopPending || (now - lastPtpAt).TotalSeconds >= 2) { Stop(); return; }
        pointAttempts++;
        lock (wireSync) { foreach (byte b in pointPacket) pointWire.Enqueue(b); }
        Append($"HOST PTP retry seq={pendingPoint} attempt={pointAttempts}/3; identical packet", LogLevel.Warning);
    }
    private void Stop()
    {
        CancelLimitSync(requireState: true);
        lock (wireSync) {
            waitingStopEpoch = framedControl && reportedEpoch != 0;
            Volatile.Write(ref leaseEpoch, 0u);
        }
        ResetControlTransport();
        InvalidateManualReferenceFeedback();
        xyPreview.PausePlayback();
        lock (wireSync) { pointWire.Clear(); }
        pendingPoint = 0; ptp = null;
        manualReferencePending = false;
        uploading = uploaded = runPlan = null; executing = false;
        pendingBoardSave = null; loadingSlot = null;
        keyboardMask = mouseMask = 0;
        pendingControl = null; telemetry = null; limitsReportSince = default; enableAfterReport = false;
        if (serial is not null) {
            stopPending = true;
            Lang.Text(state, () => Lang.T("state.stopReq"));
            SafeAction(() => { Send(0x79); Send(0x41); });
        }
        PublishHeartbeatLease();   // disabled confirmation first, then idle epoch probes
        try { FinishExperiment("aborted"); } catch(Exception ex) { Lang.Text(experimentResult, () => Lang.T("fault.saveSummary") + ex.Message); }
        UpdateControls();
    }
    private void UpdateControls(bool synchronize = true)
    {
        if (synchronize) { SynchronizeJointLimits(DateTime.UtcNow); SynchronizeTelemetryRate(DateTime.UtcNow); SynchronizeBoard(DateTime.UtcNow); }
        Volatile.Write(ref limitsMotionAllowed, LimitsReady ? 1 : 0);
        if (synchronize && manualReferenceConfirmedEpoch!=0 && (!enabled || homing || ptp is not { Referenced:true } || ptp.Epoch!=manualReferenceConfirmedEpoch))
            InvalidateManualReferenceFeedback();
        // The device forgets a loaded plan when its reference epoch changes or it is disabled.
        if (synchronize && uploaded is not null && (!enabled || homing || ptp is not { Referenced: true } || ptp.Epoch != trajectoryEpoch)) uploaded = null;
        if (synchronize && trajectoryRanOnce && (!enabled || homing || ptp is not { Referenced: true } || ptp.Epoch != trajectoryEpoch)) trajectoryRanOnce = false;
        // A disabled STATE clears the reported ranges/speeds until PTP_STATE and MOTION_RATES of the same block arrive
        // (tens of ms later). The gates above already use the cleared values; only the display waits, or every 500 ms
        // block would flash the enable button and status between "ready" and "waiting for the device".
        if (limitsReportSince != default) {
            bool reported = reportedLimits is not null && (reportedProfile is not null || !supportsMotionRates);
            if (!reported && (DateTime.UtcNow - limitsReportSince).TotalMilliseconds < LimitsReportHoldMs) return;
            limitsReportSince = default;
        }
        if (synchronize) xyPreview.SetEditingAllowed(!PointBusy && !homing);
        enable.IsEnabled = LimitsReady && compatible && !enabled && !stopPending && !PointBusy;
        manualReference.IsEnabled = CanManualReference;
        manualReference.Background=manualReference.BorderBrush=manualReferenceConfirmedEpoch!=0?WorkbenchTheme.Teal:WorkbenchTheme.Blue;
        home.IsEnabled = LimitsReady && compatible && enabled && !homing && !stopPending && !PointBusy;
        foreach (var button in jogButtons) button.IsEnabled = LimitsReady && compatible && enabled && !homing && !stopPending && !PointBusy;
        string reason = !stage2 ? Lang.T("reason.needStage2") :
            ptp is { Locked: true } ? Lang.T("reason.lockedFw") :
            PointBusy ? Lang.T("reason.busy") :
            stopPending ? Lang.T("reason.waitStop") :
            !LimitsReady ? LimitsStatus :
            !enabled ? Lang.T("reason.notEnabled") :
            homing ? Lang.T("reason.homing") :
            !homed ? (supportsManualReference ? Lang.T("reason.needManualRef") : Lang.T("reason.needHome")) :
            !feedbackReady ? Lang.T("reason.notReady") :
            ptp is not { Referenced: true, Epoch: > 0 } ? Lang.T("reason.badRef") :
            (DateTime.UtcNow - lastPtpAt).TotalSeconds >= 2 ? Lang.T("reason.stalePtp") :
            supportsTelemetry && (telemetry is null || (DateTime.UtcNow-lastTelemetryAt).TotalSeconds >= 2) ? Lang.T("reason.waitTelem") :
            supportsTelemetry && telemetry is { Inputs: > 0 } ? Lang.T("reason.inputsHit") :
            keyboardMask != 0 || mouseMask != 0 ? Lang.T("reason.keysDown") :
            CanPoint ? (supportsTrajectory ? Lang.T("reason.readyTrajectory") :
                Lang.T("reason.pointOnly")) :
            Lang.T("reason.needRef");
        pointBlockReason = reason;
        xyPreview.SetParameterConfirmation(RangeConfirmed, SpeedConfirmed);
        xyPreview.SetPointMoveState(CanPoint, reportedLimits, reason, supportsTrajectory, LimitsReady, uploaded, uploadedRan);
        var devicePose = supportsTelemetry && telemetry is { Referenced: true } && ptp is { Referenced: true } &&
            telemetry.Epoch == ptp.Epoch && (DateTime.UtcNow-lastTelemetryAt).TotalSeconds < 2 ? telemetry : null;   // a live position; AtStart is stricter
        // The canvas draws the device arm whenever there is a pose; otherwise it says why it shows the offline plan.
        xyPreview.SetStartFeedback(devicePose, devicePose is not null || !compatible ? null :
            homing ? Lang.T("why.homing") : ptp is not { Referenced: true } ? Lang.T("why.noRef") : Lang.T("why.waitFeed"));
        readInputs.IsEnabled = compatible && !PointBusy && !homing;
        referenceStatus.Text = Lang.T("status.ref") + (ptp is { Referenced: true } ? Lang.T("status.ref.on") + (telemetry is { Referenced: true } ? " · " + telemetry.ReferenceName : "") : Lang.T("status.ref.off"));
        rangeStatus.Text = Lang.T("status.range") + (LimitsReady ? Lang.T("status.range.on") : Lang.T("status.range.off"));
        leds[0].Fill = enabled ? WorkbenchTheme.Ok : WorkbenchTheme.Idle;
        leds[1].Fill = !compatible ? WorkbenchTheme.Idle : ptp is { Referenced: true } ? WorkbenchTheme.Ok : WorkbenchTheme.Warn;
        connectionBadge.Text = compatible ? (framedControl ? Lang.T("badge.v7") : hardwareUart ? Lang.T("badge.v6") : supportsTelemetry ? Lang.T("badge.v5") : Lang.T("badge.old")) : Lang.T("badge.offline");
        bool statusFault=lastStop.Text.Length>0 && (!enabled || stopPending);
        bool statusBusy=PointBusy || homing || stopPending;
        string? statusMessage=stopPending?Lang.T("msg.stopSent"):
            statusFault?lastStop.Text.Split('\n')[0]:
            uploading is {} plan?(uploadNext>=plan.Nodes.Length?Lang.T("msg.uploadRun"):Lang.T("msg.uploadPoint", Math.Min(uploadNext,plan.Nodes.Length), plan.Nodes.Length)):
            executing?Lang.T("msg.drawing", trajectoryIndex, trajectoryTotal):
            manualReferencePending?Lang.T("msg.manualRef"):
            pendingControl is not null?Lang.T("msg.ctl", pendingControl=="使能"?Lang.T("ctl.enable"):Lang.T("ctl.home")):
            homing?Lang.T("msg.homing"):PointBusy?Lang.T("msg.slowMove"):
            !compatible?Lang.T("msg.connectFirst"):!stage2?Lang.T("msg.pointOnly"):
            !LimitsReady?LimitsStatus:
            !enabled?Lang.T("msg.enableBtn"):!homed?Lang.T("msg.homeOrRef"):
            CanPoint?null:reason;
        if(uploading is {} upload && uploadNext>0 && uploadNext<upload.Nodes.Length && !statusFault)
            statusMessage+=Lang.T("msg.eta", Math.Max(0,(DateTime.UtcNow-uploadStartedAt).TotalSeconds*(upload.Nodes.Length-uploadNext)/uploadNext));
        double percent=uploading is {} p?uploadNext*100.0/p.Nodes.Length:executing?trajectoryIndex*100.0/Math.Max(1,trajectoryTotal):0;
        xyPreview.SetDeviceStatus(statusMessage,statusFault,statusBusy,percent,
            stopPending || pendingControl is not null || homing || ptp is { Active: > 0 } && !executing);
        UpdateWorkflow();
        if (enableAfterReport) { enableAfterReport = false; RequestEnable(); }
    }
    private void SafeAction(Action action)
    {
        try { action(); }
        catch (Exception ex) {
            Trace.TraceError(ex.ToString());
            try { Append("HOST exception: " + ex, LogLevel.Error); } catch { /* Preserve fail-stop even if logging failed. */ }
            Disconnect(() => Lang.T("fault.opFail") + ex.GetType().Name + "：" + ex.Message + Lang.T("fault.opFailSuffix"));
        }
    }
    private void Disconnect(string reason) => Disconnect(() => reason);
    private void Disconnect(Func<string> reason)
    {
        // Stop the writer before the port closes so it cannot write to a dead handle,
        // then report the session while the capability flags still describe it.
        StopWireWriter();
        try { var summary = WireSessionSummary(); if (summary.Length > 0) Append(summary); } catch { }
        ResetWireSession();
        ResetControlTransport();
        ResetDiagnostics();
        CancelLimitSync();
        reportedLimits = null; reportedProfile = null; supportsJointLimits = supportsMotionRates = supportsLiveRates = protocolMatches = limitsAwaitingState = false; protocolVersion = 0; reportedProfile=null; limitSyncError = "";
        lock (wireSync) { reportedEpoch = 0; waitingStopEpoch = false; }
        framedControl = false; heartbeatTimer.Interval = TimeSpan.FromMilliseconds(40);
        InvalidateManualReferenceFeedback();
        heartbeatTimer.Stop(); timer.Stop(); keyboardMask = mouseMask = 0;
        pendingControl = null; telemetry = null; supportsTelemetry = supportsPacketRetry = false; limitsReportSince = default; enableAfterReport = false;
        lock (wireSync) { pointWire.Clear(); }
        pendingPoint = lastMoveSequence = 0; ptp = null; stage2 = homed = feedbackReady = false;
        uploading = uploaded = null; executing = supportsTrajectory = false;
        manualReferencePending = supportsManualReference = hardwareReady = false;
        try { Send(0x79); } catch { /* Firmware heartbeat timeout remains the fallback. */ }
        hardwareUart = false; supportsShortHeartbeat = false; connectionBadge.ToolTip = null; lastLinkLine = null;
        RefreshFirmwareStatus();
        try { FinishExperiment("disconnected: " + reason()); } catch(Exception ex) { Lang.Text(experimentResult, () => Lang.T("fault.saveSummary2") + ex.Message); }
        lastExperimentOutcome = null; trajectoryRanOnce = false;
        try { serial?.Dispose(); } catch { }
        serial = null;
        compatible = enabled = homing = stopPending = false;
        pending.Clear(); Lang.Text(state, reason); regularStateText = reason; Lang.Text(inputs, () => Lang.T("fault.inputsInvalid"));
        ShowFeedback(null, () => Lang.T("fault.unplugReason")); errorHistory.Clear(); errorHistory2.Clear(); DrawErrorPlot();
        connect.Content = Lang.T("btn.connect"); ports.IsEnabled = true; UpdateControls();
        try { Append("HOST " + reason()); } catch { }
        try { recording?.Dispose(); } catch { }
        recording = null;
    }
    private void Append(string line, LogLevel? level = null)
    {
        string entry = $"[{DateTime.Now:HH:mm:ss.fff}] {line}{Environment.NewLine}";
        recording?.Write(entry);
        LogLevel severity = level ?? LogLevels.Classify(line);
        string displayed = $"[{DateTime.Now:HH:mm:ss.fff}] [{severity}] {line}{Environment.NewLine}";
        recentLog.Enqueue((severity, displayed));
        // ponytail: retain 1000 lines for instant filtering; use the raw file for older history.
        while (recentLog.Count > 1000) recentLog.Dequeue();
        if (severity < minimumLogLevel) return;
        bool reading = log.IsKeyboardFocusWithin || log.SelectionLength > 0;
        // Reading Text after every append materializes the whole log on the large-object heap.
        if (logCharacters > 100000 && !reading) log.Text = log.Text[^50000..];
        log.AppendText(displayed);
        // TextBox.ScrollToEnd forces layout per line; the native viewer queues the scroll.
        if (!reading && log.Template?.FindName("PART_ContentHost", log) is ScrollViewer scroll) scroll.ScrollToEnd();
    }
    public static void SelfTest(string? snapshotPath = null)
    {
        Lang.Set(false, persist: false);   // assertions below match the Chinese UI copy
        Lang.SelfTest();
        LanguageSelfTest(snapshotPath);
        MotionProfileSelfTest();
        JointLimitsSyncSelfTest();
        LiveRatesSyncSelfTest();
        StatusBlockHoldSelfTest();
        FlickerSelfTest();
        LoggingSelfTest();
        DiagnosticsSelfTest();
        StoreSelfTest();
        WorkflowSelfTest();
        StopRecoverySelfTest();
        ShapeFlowSelfTest();
        // Accumulate step by step: a later failure used to discard every earlier
        // measurement, which is exactly the evidence needed to diagnose the failure.
        var heartbeatReport = new StringBuilder();
        try
        {
            heartbeatReport.Append(BackgroundMotionSelfTest()).Append(HeartbeatSchedulingSelfTest()).Append(ExperimentFinishHeartbeatSelfTest()).Append(UsbLinkSelfTest())
                .Append(WithLeaseRetry(ReferenceIdleReplaySelfTest)).Append(WithLeaseRetry(PreviewInteractionReplaySelfTest))
                .Append(WithLeaseRetry(UploadReplaySelfTest))
                .Append(WithLeaseRetry(() => ReferenceReplaySelfTest(false, true))).Append(WithLeaseRetry(() => ReferenceReplaySelfTest(true, true)));
        }
        finally
        {
            if (snapshotPath is not null && heartbeatReport.Length > 0)
                File.WriteAllText(Path.Combine(Path.GetDirectoryName(snapshotPath)!, "heartbeat-tests.txt"), heartbeatReport.ToString());
        }
        PlanarArm.SelfTest();
        DrawingTrajectory.SelfTest();
        TimedTrajectory.SelfTest();
        PtpProtocol.SelfTest();
        BoardLedger.SelfTest();
        UartStress.SelfTest();
        ControlTransportSelfTest(snapshotPath);
        ShortHeartbeatSelfTest();
        ExecutionTrajectory.SelfTest();
        ExperimentRecorder.SelfTest();
        ExperimentArchiveTests.SelfTest();
        FeedbackSelfTest(snapshotPath);
        XYPreviewPanel.SelfTest(snapshotPath);
        ExperimentWindow.SelfTest(snapshotPath);
        Calibrations.SelfTest();
        CalibrationWindow.SelfTest(snapshotPath);
        var window = new ControlWindow { selfTesting = true };
        static void Check(bool condition, [System.Runtime.CompilerServices.CallerArgumentExpression(nameof(condition))] string? expression = null)
        { if (!condition) throw new InvalidOperationException("UI state check failed: " + expression); }
        // One-click language toggle rewrites live chrome without rebuilding the window.
        Check((string)window.langToggle.Content! == "EN" && (string)window.connect.Content! == "连接" && window.Title.Contains("轨迹工作台"));
        Lang.Toggle();
        Check((string)window.langToggle.Content! == "中" && (string)window.connect.Content! == "Connect" && window.Title.Contains("Trajectory"));
        Lang.Toggle();
        Check((string)window.langToggle.Content! == "EN" && (string)window.connect.Content! == "连接");
        // CanPoint also needs the device to report rates equal to the request and telemetry matching the
        // reference epoch AND sequence (HandleTelemetry drops anything else), as real firmware does periodically.
        void Arm(int epoch, uint seq = 0) {
            var r = window.xyPreview.RequestedProfile!.Value;
            window.TestLine($"MOTION_RATES epoch={epoch} r1={r.R1} r2={r.R2} a1={r.A1} a2={r.A2}");
            window.TestLine($"TELEM t=10 epoch={epoch} seq={seq} ref=1 source=1 p1=0 p2=0 e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
        }
        Check(!window.enable.IsEnabled && !window.home.IsEnabled);
        window.TestLine("STATE fw=stage2-v6 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
        Check(window.hardwareUart && window.supportsTelemetry && window.supportsPacketRetry && !window.enabled);
        window.TestLine("LINK uart=hw baud=115200 rx_overflow=2 rx_error=1 tx_dropped=0");
        Check(window.connectionBadge.ToolTip?.ToString()?.Contains("rx_overflow=2") == true);
        window.Disconnect("transport reset self-test");
        Check(!window.hardwareUart && !window.compatible && window.connectionBadge.ToolTip is null);
        byte[] pacedPacket = PtpProtocol.EncodePacket(5, 7, 4, new JointPose(0, 0), 0);
        window.QueuePointPacket(pacedPacket);
        var pacedWrites = new List<byte>();
        window.FlushHardwarePacket((bytes, offset, count) => {
            Check(offset == 0 && count == 8);
            pacedWrites.AddRange(bytes);
        });
        Check(window.pointWire.Count == 28 && pacedWrites.SequenceEqual(pacedPacket.Take(8)));
        window.pendingPoint = 7; window.supportsPacketRetry = window.enabled = window.hardwareReady = true;
        window.pointRequestedAt = DateTime.UtcNow; window.lastPtpAt = DateTime.UtcNow;
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(1100));
        Check(window.pointAttempts == 1 && window.pointWire.Count == 28); // Do not duplicate a partly sent frame.
        byte[] remainder = window.pointWire.ToArray();
        try { window.FlushHardwarePacket((_, _, _) => throw new IOException("mock write failure")); }
        catch (IOException) { }
        Check(window.pointWire.SequenceEqual(remainder));
        while (window.pointWire.Count > 0) window.FlushHardwarePacket((bytes, _, count) => {
            Check(count <= 8); pacedWrites.AddRange(bytes);
        });
        Check(pacedWrites.SequenceEqual(pacedPacket));
        window.Disconnect("paced transport self-test complete");
        Check(IsTextInteraction(ModifierKeys.Control, false));
        Check(IsTextInteraction(ModifierKeys.None, true));
        Check(!IsTextInteraction(ModifierKeys.None, false));
        window.log.Text = "copy this line";
        window.log.Select(0, 4);
        window.Append("new line must not steal the selection");
        Check(window.log.SelectedText == "copy");
        window.TestLine("STOP: P0[3] global limit");
        window.TestLine("STATE enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(!window.compatible); // old firmware must never unlock motion controls
        window.TestLine("STATE fw=stage1-v2 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(!window.compatible && !window.home.IsEnabled);
        window.TestLine("STATE fw=stage1-v3 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
        Check(!window.enable.IsEnabled && !window.home.IsEnabled); // No reported joint range, no motion.
        Check(window.lastStop.Text.Contains("STOP: P0[3] global limit"));
        window.TestLine("STATE fw=stage1-v3 enabled=1 home=6 homed1=1 homed2=0 fault=0 enc=1 dir=1");
        Check(window.homing && !window.home.IsEnabled && window.jogButtons.All(b => !b.IsEnabled));
        window.TestLine("STATE fw=stage1-v3 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(!window.home.IsEnabled);
        window.stopPending = true;
        window.TestLine("STATE fw=stage1-v3 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(!window.home.IsEnabled); // stale enabled report cannot cancel a pending stop
        window.TestLine("STATE fw=stage1-v3 enabled=0 home=0 homed1=0 homed2=0 fault=1 enc=1 dir=1");
        Check(!window.enable.IsEnabled && !window.home.IsEnabled && !window.stopPending);
        window.xyPreview.RestoreLimitsForSelfTest("-60", "60", "-90", "90");
        window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.TestLine("PTP_STATE lock=1 ref=1 active=0 epoch=7 seq=0 min1=0 max1=0 min2=0 max2=0 emin=0 emax=0");
        Check(!window.CanPoint);
        window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=7 seq=0 min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        Check(!window.CanPoint); // matching limits cannot override missing q2 capability
        window.TestLine($"LINK fw=stage2-v7-absolute-q2 q2_absolute=1 proto={DeviceContract.ProtocolVersion} motion_rates=1");
        Check(!window.CanPoint); // the device has not confirmed the requested rates yet
        { var r = window.xyPreview.RequestedProfile!.Value; window.TestLine($"MOTION_RATES epoch=7 r1={r.R1} r2={r.R2} a1={r.A1} a2={r.A2}"); }
        Check(!window.CanPoint); // no telemetry for this reference yet
        window.TestLine("TELEM t=10 epoch=7 seq=0 ref=1 source=1 p1=0 p2=0 e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
        Check(window.CanPoint);
        window.RequestPoint(new JointPose(1.25, -2.5));
        Check(window.pendingPoint==1 && window.pointWire.Count==36 && !window.CanPoint && !window.home.IsEnabled);
        window.Stop(); Check(window.pendingPoint==0 && window.pointWire.Count==0 && !window.CanPoint);
        window.TestLine("PTP_STATE lock=0 ref=1 active=1 epoch=7 seq=1 min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        Check(!window.CanPoint && !window.home.IsEnabled);
        window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=7 seq=1 min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        Arm(7, 1);
        Check(window.CanPoint);
        // Compact re-announcement: the firmware repeats this after a reference epoch
        // change, so the host must adopt the new epoch without a full status block.
        window.TestLine("PTP_EPOCH 9");
        Check(window.ptp is { Epoch: 9 }, "compact epoch announcement was not adopted");
        window.TestLine("PTP_EPOCH nonsense");
        Check(window.ptp is { Epoch: 9 }, "malformed epoch announcement changed the epoch");
        window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=9 seq=1 min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        window.lastPtpAt = DateTime.UtcNow.AddSeconds(-3); Check(!window.CanPoint);
        window.TestLine("PTP_STATE malformed"); Check(!window.CanPoint);
        window.xyPreview.RestoreLimitsForSelfTest("-30", "30", "-60", "60");
        window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
        window.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=20 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
        Check(window.CanManualReference && window.manualReference.IsEnabled && !window.CanPoint);
        window.RequestManualReference();
        uint manualSeq = window.pendingPoint;
        Check(window.manualReferencePending && window.pointWire.Count == 36 && !window.CanManualReference);
        window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=21 seq={manualSeq} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
        Arm(21, manualSeq);
        Check(window.CanPoint && !window.manualReferencePending && window.pointWire.Count == 0);
        window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.RequestPoint(new JointPose(-21.95, 44.37));
        uint retrySeq = window.pendingPoint;
        byte[] original = window.pointWire.ToArray();
        window.pointWire.Clear(); // entire first transmission/ACK lost
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(900));
        Check(window.pointWire.Count == 0 && window.pointAttempts == 1);
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(1100));
        Check(window.pendingPoint == retrySeq && window.pointAttempts == 2 && window.pointWire.SequenceEqual(original));
        window.TestLine("PTP RX dropped: incomplete nibbles=30/34");
        Check(window.pendingPoint == retrySeq); // transport drop is not a motion refusal
        window.pointWire.Clear();
        window.lastPtpAt = window.pointRequestedAt.AddSeconds(2);
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(2100));
        Check(window.pointAttempts == 3 && window.pointWire.SequenceEqual(original));
        window.RetryPendingPoint(window.pointRequestedAt.AddSeconds(3));
        Check(window.pendingPoint == 0 && window.pointWire.Count == 0);
        window.RetryPendingPoint(window.pointRequestedAt.AddSeconds(4));
        Check(window.pointWire.Count == 0); // timeout cannot restart anything
        window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=21 seq={retrySeq} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
        window.RequestPoint(new JointPose(-21.95, 44.37));
        retrySeq = window.pendingPoint;
        window.TestLine($"PTP_STATE lock=0 ref=1 active=1 epoch=21 seq={retrySeq} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(1100));
        Check(window.pendingPoint == 0 && window.pointWire.Count == 0); // ACK cancels retransmission
        window.TestLine("STOP: driver fault");
        Check(window.ptp is null && window.pointWire.Count == 0);
        window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=22 seq={retrySeq} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
        window.RequestPoint(new JointPose(-21.95, 44.37));
        window.TestLine("PTP refused: joint range/rate");
        window.RetryPendingPoint(window.pointRequestedAt.AddSeconds(1));
        Check(window.pendingPoint == 0 && window.pointWire.Count == 0); // genuine refusal never retries
        window.Stop(); Check(!window.CanPoint && !window.CanManualReference);
        window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=0 homed2=0 fault=0 enc=0 dir=0");
        window.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=22 seq=2 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
        Check(!window.CanManualReference);
        var path = DrawingTrajectory.Create(new PlanarArm(180, 176.38), DrawingShape.Circle, 340, 0, 5, 0, 0, null);
        var execution = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(20, 5, 3, 10, 6)), new JointLimits(-30, 30, -60, 60));
        window.xyPreview.RestoreLimitsForSelfTest("-60", "60", "-90", "90");
        window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=8 seq=1 min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        void StartSample(uint epoch, uint seq) => window.TestLine($"TELEM t=1000 epoch={epoch} seq={seq} ref=1 source=2 p1={ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose)} p2={ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
        window.RequestTrajectory(execution); Check(window.uploading is null); // missing position feedback
        StartSample(8,1);
        window.RequestTrajectory(execution);
        Check(window.uploading is not null && window.pointWire.Count == 36 && !window.CanPoint && !window.home.IsEnabled);
        uint first = window.pendingPoint;
        window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=8 seq={first} min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        Check(window.pendingPoint == first && window.uploadNext == 0); // PTP status is not a node acknowledgement
        uint lastSeq = 0;
        for (int i = 0; i < execution.Nodes.Length; i++) {
            uint seq = lastSeq = window.pendingPoint;
            window.TestLine($"TRAJ_STATE state={(i + 1 == execution.Nodes.Length ? 2 : 1)} count={i + 1} total={execution.Nodes.Length} index=0 seq={seq} epoch=8");
            Check(window.uploadNext == i + 1 && window.pendingPoint == (i + 1 == execution.Nodes.Length ? 0 : seq + 1)); // the last ACK ends the upload; RUN is a separate request
            if (i == 0) { // duplicate ack cannot enqueue the next node twice
                window.TestLine($"TRAJ_STATE state=1 count=1 total={execution.Nodes.Length} index=0 seq={seq} epoch=8");
                Check(window.uploadNext == 1 && window.pointWire.Count == 36);
            }
        }
        Check(window.uploading is null && window.uploaded == execution && !window.executing && window.pointWire.Count == 0 && window.CanPoint); // fully loaded, nothing moves
        window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=8 seq={lastSeq} min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500"); StartSample(8, lastSeq); // the device reports the last node as its sequence
        window.RequestPoint(new JointPose(1, 1)); window.RequestManualReference();
        Check(window.pendingPoint == 0 && window.pointWire.Count == 0 && !window.CanManualReference); // the device refuses other moves while it holds a plan
        window.RequestExecute(); Check(window.uploading is not null && window.uploaded is null && window.pendingPoint != 0 && window.pointWire.Count == 36);
        window.TestLine($"TRAJ_STATE state=3 count={execution.Nodes.Length} total={execution.Nodes.Length} index=1 seq={window.pendingPoint} epoch=8");
        Check(window.uploading is null && window.executing && window.pointWire.Count == 0 && window.pendingPoint == 0);
        window.TestLine($"TRAJ_STATE state=4 count={execution.Nodes.Length} total={execution.Nodes.Length} index={execution.Nodes.Length} seq={window.trajectoryRunSequence} epoch=8");
        Check(!window.executing && window.uploaded == execution && window.uploadedRan); // the device keeps a finished plan
        window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=8 seq={window.trajectoryRunSequence} min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        StartSample(8,window.trajectoryRunSequence);
        Check(window.CanManualReference && window.manualReference.IsEnabled); // a finished plan (device TRAJ_DONE) does not block the manual zero; only one still waiting for RUN does
        window.RequestExecute(); Check(window.uploading is not null && window.uploaded is null && window.pointWire.Count == 36); // repeat: RUN only, no upload
        window.TestLine($"TRAJ_STATE state=3 count={execution.Nodes.Length} total={execution.Nodes.Length} index=1 seq={window.pendingPoint} epoch=8");
        window.TestLine($"TRAJ_STATE state=4 count={execution.Nodes.Length} total={execution.Nodes.Length} index={execution.Nodes.Length} seq={window.trajectoryRunSequence} epoch=8");
        Check(!window.executing && window.uploaded == execution && window.uploadedRan);
        window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=8 seq={window.trajectoryRunSequence} min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        StartSample(8,window.trajectoryRunSequence);
        window.RequestTrajectory(execution); Check(window.uploading is not null);
        window.Stop(); Check(window.uploading is null && window.pointWire.Count == 0 && window.pendingPoint == 0);
        window.uploaded = execution; window.UpdateControls(); Check(window.uploaded is null); // no reference, no held plan
        window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=9 seq=9999 min1=-6000 max1=6000 min2=-9000 max2=9000 emin=-15500 emax=15500");
        StartSample(9,9999);
        window.RequestTrajectory(execution);
        window.TestLine($"TRAJ_STATE state=1 count=2 total={execution.Nodes.Length} index=0 seq={window.pendingPoint} epoch=9");
        Check(window.uploading is null && window.pointWire.Count == 0); // wrong count stops upload
        window.xyPreview.RestoreLimitsForSelfTest("-30", "30", "-60", "60");
        if (snapshotPath is not null) {
            using var binary = new BinaryWriter(File.Create(Path.ChangeExtension(snapshotPath, ".execution.bin")));
            binary.Write((ushort)execution.Nodes.Length);
            foreach (var n in execution.Nodes) { binary.Write((short)Math.Round(n.Pose.Q1 * 100)); binary.Write((short)Math.Round(n.Pose.Q2 * 100)); binary.Write(n.Milliseconds); }
            foreach(var kind in new[]{DrawingShape.Line,DrawingShape.Rectangle}) {
                var extraPath=DrawingTrajectory.Create(new PlanarArm(180,176.38),kind,338,-4,kind==DrawingShape.Line?344:6,kind==DrawingShape.Line?4:8,0,null);
                var extraPlan=new ExecutionTrajectory(new TimedTrajectory(extraPath,new PlanningRates(20,5,3,10,6)),new JointLimits(-30,30,-60,60));
                using var extra=new BinaryWriter(File.Create(Path.Combine(Path.GetDirectoryName(snapshotPath)!,kind.ToString().ToLowerInvariant()+".execution.bin")));
                extra.Write((ushort)extraPlan.Nodes.Length);
                foreach(var n in extraPlan.Nodes) { extra.Write((short)Math.Round(n.Pose.Q1*100)); extra.Write((short)Math.Round(n.Pose.Q2*100)); extra.Write(n.Milliseconds); }
            }
            window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=30 seq=10000 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            int p1=ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose), p2=ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose);
            window.errorHistory.Clear();
            for(int i=0;i<40;i++) window.TestLine($"TELEM t={i*500} epoch=30 seq=10000 ref=1 source=2 p1={p1} p2={p2} e1={(int)Math.Round(p1*2.56)+i%9-4} err=0 e2=0 err2=0 inputs=0 drops=0");
            Lang.Text(window.lastStop, () => ""); window.connectionBadge.Text="离线界面检查 · 模拟数据";
            string dir=Path.GetDirectoryName(snapshotPath)!;
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench.png"),1380,930);
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-compact.png"),1100,760);
            window.diagnostics.IsExpanded=true;
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-feedback.png"),1380,930);
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-feedback-compact.png"),1100,760);
            window.diagnostics.IsExpanded=false; window.xyPreview.FitView(false);
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-full-arm.png"),1380,930);
            window.xyPreview.FitView(true);
            window.TestLine("STATE fw=stage2-v5 enabled=0 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            window.connectionBadge.Text="离线界面检查 · 模拟未使能";
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-waiting.png"),1100,760);
            // Every group folds: collapsed, only the header row may remain in the layout; expanded, the body is back.
            var sections = window.Sections().ToList();
            if (sections.Count < 8) throw new InvalidOperationException("Collapsible groups missing: " + sections.Count);
            foreach (var section in sections) section.IsExpanded = false;
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-collapsed.png"),1380,930);
            var headerOnly = sections.Select(section => section.ActualHeight).ToList();
            if (headerOnly.Any(height => height > 40)) throw new InvalidOperationException("A collapsed group kept its body in the layout: " + string.Join(", ", headerOnly));
            foreach (var section in sections) section.IsExpanded = true;
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-expanded.png"),1380,930);
            if (sections.Where((section, i) => section.ActualHeight <= headerOnly[i] + 10).Any()) throw new InvalidOperationException("An expanded group has no body");
            // The side columns are draggable: a drag on each splitter must move exactly that column's width.
            var splitters = Descendants(window).OfType<GridSplitter>().ToList();
            if (splitters.Count != 2) throw new InvalidOperationException("Expected two column splitters, found " + splitters.Count);
            window.Show(); window.UpdateLayout();
            var columns = ((Grid)splitters[0].Parent).ColumnDefinitions;
            double leftWidth = columns[0].ActualWidth, rightWidth = columns[4].ActualWidth;
            void Drag(GridSplitter splitter, double dx) {
                splitter.RaiseEvent(new System.Windows.Controls.Primitives.DragStartedEventArgs(0, 0));
                splitter.RaiseEvent(new System.Windows.Controls.Primitives.DragDeltaEventArgs(dx, 0));
                splitter.RaiseEvent(new System.Windows.Controls.Primitives.DragCompletedEventArgs(dx, 0, false));
                window.UpdateLayout();
            }
            Drag(splitters[0], 40); Drag(splitters[1], -30);
            if (Math.Abs(columns[0].ActualWidth - (leftWidth + 40)) > 1 || Math.Abs(columns[4].ActualWidth - (rightWidth + 30)) > 1)
                throw new InvalidOperationException($"Splitters did not resize the side columns: left {leftWidth}->{columns[0].ActualWidth}, right {rightWidth}->{columns[4].ActualWidth}");
            Drag(splitters[0], -10000); Drag(splitters[1], 10000);
            if (columns[0].ActualWidth < columns[0].MinWidth - 1 || columns[4].ActualWidth < columns[4].MinWidth - 1)
                throw new InvalidOperationException("Splitters ignored the minimum column widths");
            window.Hide();
            // The solid arm tracks the device even when it differs from the plan; the plan stays as a dashed ghost.
            window.xyPreview.FitView(false);
            var away = new JointPose(10, 25);
            window.TestLine($"TELEM t=30000 epoch=30 seq=10000 ref=1 source=2 p1={ExecutionTrajectory.Counts(0,away)} p2={ExecutionTrajectory.Counts(1,away)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-device-moved.png"),1380,930);
        }
        window.Close();
    }
}
