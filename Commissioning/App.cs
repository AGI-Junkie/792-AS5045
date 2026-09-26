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
        if (args.Contains("--self-test"))
        {
            try
            {
                int snapshot = Array.IndexOf(args, "--snapshot");
                ControlWindow.SelfTest(snapshot >= 0 && snapshot + 1 < args.Length ? args[snapshot + 1] : null);
                return 0;
            }
            catch (Exception ex) { File.WriteAllText("commissioning-self-test-error.txt", ex.ToString()); return 1; }
        }
        new App().Run(new ControlWindow());
        return 0;
    }
}

public sealed partial class ControlWindow : Window
{
    private readonly ComboBox ports = new() { Width = 130, Margin = new Thickness(4) };
    private readonly TextBlock state = new() { Text = "未连接", FontSize = 22, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4, 12, 4, 8) };
    private readonly TextBlock inputs = new() { Text = "输入尚未读取", Margin = new Thickness(4), TextWrapping = TextWrapping.Wrap };
    private readonly TextBlock logPath = new() { TextWrapping = TextWrapping.Wrap, Margin = new Thickness(4) };
    private readonly TextBlock lastStop = new() { TextWrapping = TextWrapping.Wrap, Foreground = Brushes.Firebrick, Margin = new Thickness(4) };
    private readonly TextBox log = new() { IsReadOnly = true, AcceptsReturn = true, VerticalScrollBarVisibility = ScrollBarVisibility.Auto,
        HorizontalScrollBarVisibility = ScrollBarVisibility.Auto, FontFamily = new FontFamily("Consolas"), FontSize = 12 };
    private readonly Button enable = MakeButton("使能 / 重新检查 (C)");
    private readonly Button home = MakeButton("开始回零 (R)");
    private readonly Button manualReference = MakeButton("人工确认：当前位置为伸直零位");
    private readonly Button connect = MakeButton("连接");
    private readonly XYPreviewPanel xyPreview = new();
    private readonly List<Button> jogButtons = [];
    private readonly DispatcherTimer timer = new() { Interval = TimeSpan.FromMilliseconds(40) };
    private SerialPort? serial;
    private StreamWriter? recording;
    private readonly StringBuilder pending = new();
    private int keyboardMask, mouseMask;
    private bool compatible, enabled, homing, stopPending;
    private DateTime openedAt, lastStateAt;
    private long lastHeartbeatSent;
    private PtpState? ptp;
    private DateTime lastPtpAt, pointRequestedAt;
    private bool stage2, homed, feedbackReady, hardwareReady, supportsManualReference, manualReferencePending;
    private uint pointSequence, pendingPoint;
    private readonly Queue<byte> pointWire = new();
    private byte[] pointPacket = [];
    private int pointAttempts;
    private bool supportsPacketRetry;
    private bool supportsTelemetry;
    private bool hardwareUart;
    private TelemetrySample? telemetry;
    private DateTime lastTelemetryAt, controlRequestedAt;
    private string? pendingControl;
    private uint requestEpoch;
    private ExperimentRecorder? experiment;
    private string? experimentCompleting;
    private string experimentPhase = "idle";
    private static string ExperimentRoot => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "Experiments");
    private ExecutionTrajectory? uploading;
    private int uploadNext, trajectoryIndex, trajectoryTotal;
    private bool supportsTrajectory, executing;
    private uint trajectoryEpoch, trajectoryRunSequence;
    private DateTime lastTrajectoryAt;
    private DateTime uploadStartedAt;
    private static readonly Regex TrajectoryPattern = new(@"^TRAJ_STATE state=([0-4]) count=(\d+) total=(\d+) index=(\d+) seq=(\d+) epoch=(\d+)$", RegexOptions.CultureInvariant);
    private string regularStateText = "未连接";
    private static readonly Regex StatePattern = new(@"^STATE fw=(?:stage1-v3|stage2-v[123456]) enabled=([01]) home=([0-7]) homed1=([01]) homed2=([01]) fault=([01]) enc=([01]) dir=([01])$", RegexOptions.CultureInvariant);

    public ControlWindow()
    {
        BuildLayout();
        timer.Tick += (_, _) => Tick();
        xyPreview.PointMoveRequested += RequestPoint;
        xyPreview.TrajectoryRequested += RequestTrajectory;
        PreviewKeyDown += OnKeyDown; PreviewKeyUp += OnKeyUp;
        Deactivated += (_, _) => { xyPreview.PausePlayback(); if (serial is not null && (PointBusy || homing || pendingControl is not null || keyboardMask != 0 || mouseMask != 0)) Stop(); };
        Closing += (_, _) => Disconnect("窗口关闭");
        RefreshPorts(); UpdateControls();
    }

    private static Button MakeButton(string title) => new() { Content = title, Margin = new Thickness(4), Padding = new Thickness(12, 8, 12, 8), MinHeight = 38 };
    private void AddJog(Panel row, string title, int bit)
    {
        var button = MakeButton(title); WorkbenchTheme.Primary(button); jogButtons.Add(button); row.Children.Add(button);
        button.PreviewMouseLeftButtonDown += (_, e) =>
        {
            if (!button.IsEnabled) return;
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
    private bool PointBusy => pendingControl is not null || pendingPoint != 0 || ptp is { Active: > 0 } || uploading is not null || executing;
    private bool CanPoint => stage2 && compatible && enabled && homed && feedbackReady && !homing && !stopPending && !PointBusy &&
        keyboardMask == 0 && mouseMask == 0 && ptp is { Locked: false, Referenced: true, Epoch: > 0 } &&
        (DateTime.UtcNow - lastPtpAt).TotalSeconds < 2 && (!supportsTelemetry ||
        telemetry is { Referenced: true, Inputs: 0 } && telemetry.Epoch == ptp.Epoch && (DateTime.UtcNow-lastTelemetryAt).TotalSeconds < 2);
    private bool CanManualReference => supportsManualReference && compatible && enabled && hardwareReady && !homing && !stopPending && !PointBusy &&
        keyboardMask == 0 && mouseMask == 0 && ptp is { Locked: false, Epoch: > 0 } &&
        (DateTime.UtcNow - lastPtpAt).TotalSeconds < 2;
    private bool CanJog => compatible && enabled && !homing && !stopPending && !PointBusy && IsActive && (DateTime.UtcNow - lastStateAt).TotalSeconds < 2;
    private bool IsEditingControls => Keyboard.FocusedElement is TextBoxBase or ComboBox || xyPreview.IsKeyboardFocusWithin;
    private void RefreshPorts()
    {
        if (serial is not null) return;
        ports.ItemsSource = SerialPort.GetPortNames().Order().ToArray();
        if (ports.Items.Count > 0) ports.SelectedIndex = 0;
    }
    private void Connect()
    {
        if (ports.SelectedItem is not string port) { state.Text = "请选择 KitProg 的 COM 口"; return; }
        try
        {
            string directory = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "Logs");
            Directory.CreateDirectory(directory);
            string path = Path.Combine(directory, $"stage1-{DateTime.Now:yyyyMMdd-HHmmss-fff}.log");
            recording = new StreamWriter(path, false, new UTF8Encoding(false)) { AutoFlush = true };
            logPath.Text = "日志自动保存：" + path;
            serial = new SerialPort(port, 115200, Parity.None, 8, StopBits.One) { WriteTimeout = 100, ReadTimeout = 10, DtrEnable = false, RtsEnable = false };
            serial.Open(); openedAt = DateTime.UtcNow; lastStateAt = DateTime.MinValue;
            lastHeartbeatSent = 0;
            hardwareUart = false;
            pending.Clear(); compatible = enabled = homing = false; stopPending = true;
            keyboardMask = mouseMask = 0;
            Send(0x79); Heartbeat(); Send((byte)'s'); Send((byte)'i');
            state.Text = "已连接，等待 stage1-v3 / stage2 固件状态；不会自动使能";
            connect.Content = "断开"; ports.IsEnabled = false; timer.Start();
            Append("HOST connected " + port);
        }
        catch (Exception ex) { Disconnect("连接失败：" + ex.Message); }
        UpdateControls();
    }
    private void Send(byte command, double gapMs = 1)
    {
        if (serial is not { IsOpen: true }) return;
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
        double gap = lastHeartbeatSent == 0 ? 0 : Stopwatch.GetElapsedTime(lastHeartbeatSent, now).TotalMilliseconds;
        Send((byte)(0x41 | ((CanJog ? (keyboardMask | mouseMask) : 0) << 1)));
        lastHeartbeatSent = now;
        if (gap > 150) Append($"HOST heartbeat send gap={gap:F0} ms");
    }
    private void Tick()
    {
        SafeAction(() =>
        {
            if (serial is not { IsOpen: true }) throw new IOException("串口已关闭");
            // The UI thread owns the heartbeat: a hung UI cannot keep motion alive.
            Heartbeat();
            pending.Append(serial.ReadExisting());
            string text = pending.ToString(); int end;
            while ((end = text.IndexOf('\n')) >= 0)
            {
                HandleLine(text[..end].TrimEnd('\r')); text = text[(end + 1)..];
            }
            pending.Clear();
            if (text.Length <= 4096) pending.Append(text);
            else Append("HOST discarded oversized/unterminated serial line");
            double age = (DateTime.UtcNow - (compatible ? lastStateAt : openedAt)).TotalSeconds;
            if (age > (compatible ? 2 : 8)) Disconnect("固件状态超时 / 版本不匹配；已请求停止，请检查设备");
            RetryPendingPoint(DateTime.UtcNow);
            CheckControlTimeout(DateTime.UtcNow);
            if (supportsTelemetry && (DateTime.UtcNow-lastTelemetryAt).TotalSeconds >= 2) {
                telemetry = null; liveValues.Text = "反馈已过期；等待新的设备数据。";
                if (PointBusy || executing) { Append("HOST telemetry timeout; stopping"); Stop(); }
            }
            if ((uploading is not null || executing) && ((DateTime.UtcNow - lastPtpAt).TotalSeconds > 2 ||
                (executing && (DateTime.UtcNow - lastTrajectoryAt).TotalSeconds > 2))) {
                Append("HOST trajectory status timeout; stopping"); Stop();
            }
            if (hardwareUart && pointWire.Count > 0 && serial is { IsOpen: true })
            {
                // One bounded frame, then wait for its existing sequence/epoch ACK.
                byte[] frame = pointWire.ToArray();
                serial.Write(frame, 0, frame.Length);
                pointWire.Clear();
            }
            else
            {
                // Legacy GPIO UART still needs triplication and inter-byte gaps.
                for (int i = 0; i < 8 && pointWire.Count > 0; i++) Send(pointWire.Dequeue(), 1.3);
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
            bool fastLink = line.StartsWith("STATE fw=stage2-v6 ");
            if (hardwareUart != fastLink)
                Append(fastLink ? "HOST transport: hardware UART 115200; whole-frame writes, CRC/ACK/retry retained" :
                    "HOST transport: legacy UART; paced triplicate writes");
            hardwareUart = fastLink;
            supportsTelemetry = hardwareUart || line.StartsWith("STATE fw=stage2-v5 ");
            supportsPacketRetry = supportsTelemetry || line.StartsWith("STATE fw=stage2-v4 ");
            supportsManualReference = supportsPacketRetry || line.StartsWith("STATE fw=stage2-v3 ");
            supportsTrajectory = supportsManualReference || line.StartsWith("STATE fw=stage2-v2 ");
            if (!stage2) ptp = null;
            homed = match.Groups[3].Value == "1" && match.Groups[4].Value == "1";
            hardwareReady = match.Groups[5].Value == "0" && match.Groups[6].Value == "1";
            feedbackReady = hardwareReady && match.Groups[7].Value == "1";
            enabled = match.Groups[1].Value == "1"; homing = match.Groups[2].Value != "0";
            if ((pendingControl == "使能" && enabled) || (pendingControl == "回零" && homing)) pendingControl = null;
            if (pendingPoint != 0 && (!enabled || !hardwareReady || homing)) Stop();
            if ((uploading is not null || executing) && (!supportsTrajectory || !enabled || !homed || !feedbackReady || homing)) Stop();
            if (!enabled) stopPending = false;
            string[] phases = ["待机", "释放开关", "额外退开", "快速触碰", "慢速触碰", "确认零点", "移到伸直位（轴号见日志）", "确认伸直位置"];
            regularStateText =
                $"{(enabled ? "已使能" : "运动禁用")} · {phases[int.Parse(match.Groups[2].Value)]}" +
                $"\n坐标参考：arm1 {(match.Groups[3].Value == "1" ? "已建立" : "未建立")} / arm2 {(match.Groups[4].Value == "1" ? "已建立" : "未建立")}" +
                $" · 编码器 {(match.Groups[6].Value == "1" ? "可用" : "不可用")} · 驱动故障：{(match.Groups[5].Value == "1" ? "有（锁定）" : "无")}";
            ShowDeviceState();
            UpdateControls();
        }
        else if (line.StartsWith("PTP_STATE ")) {
            ptp = stage2 ? PtpState.Parse(line) : null; lastPtpAt = DateTime.UtcNow;
            if (ptp is null && PointBusy) { lastStop.Text="运动状态回执格式无效，已取消请求；检查通信与固件版本。"; Stop(); return; }
            if (ptp is not null) {
                pointSequence = Math.Max(pointSequence, ptp.Sequence);
                if (pendingPoint != 0 && ptp.Epoch != requestEpoch &&
                    (!manualReferencePending || ptp.Epoch != (requestEpoch == uint.MaxValue ? 1u : requestEpoch+1u))) {
                    lastStop.Text="等待指令期间坐标参考已改变；已取消请求，请重新确认零位。"; Stop(); return;
                }
                if (uploading is null && pendingPoint != 0 && ptp.Sequence == pendingPoint &&
                    ptp.Epoch == (manualReferencePending ? (requestEpoch == uint.MaxValue ? 1u : requestEpoch+1u) : requestEpoch)) {
                    pendingPoint = 0; pointWire.Clear(); manualReferencePending = false;
                }
            }
            if ((uploading is not null || executing) && (ptp is null || ptp.Locked || !ptp.Referenced || ptp.Epoch != trajectoryEpoch)) Stop();
            ShowDeviceState();
            UpdateControls();
        }
        else if (line.StartsWith("TRAJ_STATE ")) HandleTrajectoryState(line);
        else if (line.StartsWith("TELEM ")) HandleTelemetry(line);
        else if (line.StartsWith("LINK ")) connectionBadge.ToolTip = "通信诊断（累计计数，重启设备清零）：\n" + line;
        else if (line.StartsWith("PTP done ")) {
            if (experimentPhase == "point") experimentCompleting = "complete";
        }
        else if (line.StartsWith("PTP refused:")) {
            lastStop.Text = ExplainFault(line);
            if (pendingPoint != 0) Stop();
        }
        else if (line.StartsWith("INPUTS:")) inputs.Text = line;
        else if (line.StartsWith("STOP:") || line.StartsWith("ENABLE refused:") || line.StartsWith("HOME refused:")) {
            if (!(stopPending && line.Contains("operator stop") && lastStop.Text.Length>0)) lastStop.Text = ExplainFault(line);
            if (PointBusy) Stop();
        }
        else if (line.StartsWith("ENABLE OK:")) lastStop.Text = "";
    }
    private void RequestEnable()
    {
        if (!enable.IsEnabled) return;
        SafeAction(() => { keyboardMask = mouseMask = 0; pendingControl = "使能"; controlRequestedAt = DateTime.UtcNow;
            Send(0x41); Send(0x63); state.Text = "使能已请求，等待固件检查结果"; UpdateControls(); });
    }
    private void RequestHome()
    {
        if (!home.IsEnabled || keyboardMask != 0 || mouseMask != 0) return;
        SafeAction(() => { pendingControl = "回零"; controlRequestedAt = DateTime.UtcNow;
            Send(0x41); Send(0x71); state.Text = "回零已请求，等待固件报告；Space / Esc 停止"; UpdateControls(); });
    }
    private void RequestManualReference()
    {
        if (!CanManualReference || ptp is null) return;
        SafeAction(() => {
            if (pointSequence == uint.MaxValue) throw new InvalidOperationException("运动序号用尽，请重启设备。");
            pendingPoint = ++pointSequence; pointRequestedAt = DateTime.UtcNow; manualReferencePending = true;
            QueuePointPacket(PtpProtocol.EncodePacket(5, pendingPoint, ptp.Epoch, new JointPose(0, 0), 0));
            Append("HOST manual reference: operator declares both links straight at q1=q2=0; no movement requested");
            ShowDeviceState(); UpdateControls();
        });
    }
    private void ShowDeviceState()
    {
        if (!stopPending && manualReferencePending) { state.Text = "正在确认人工伸直零位；此指令不驱动电机。"; return; }
        if (!stopPending && uploading is not null) { state.Text = $"轨迹上传：{Math.Min(uploadNext, uploading.Nodes.Length)} / {uploading.Nodes.Length} 点；等待确认，上传时不运动"; return; }
        if (!stopPending && executing) { state.Text = $"图形执行：{trajectoryIndex} / {trajectoryTotal} 段\narm2 为命令步数；末端位置未实测。"; return; }
        state.Text = stopPending ? "停止已请求，等待固件确认" : PointBusy ?
            (ptp is { Active: 2 } ? "XY 单点：停稳并检查到位" : ptp is { Active: 1 } ? "XY 单点：双轴协调移动中" : "XY 单点：等待固件接收") +
            "\narm1 有编码器反馈；arm2 为命令步数，末端位置未实测。" : regularStateText;
    }
    private void RequestTrajectory(ExecutionTrajectory plan)
    {
        if (!CanPoint || !supportsTrajectory || ptp is null) return;
        SafeAction(() => {
            ExecutionTrajectory.Validate(plan.Nodes, ptp.Limits);
            if (!AtStart(plan.Nodes[0].Pose)) { lastStop.Text = "尚未确认到达这条轨迹的起点。请先低速移到起点，等待到位反馈，再执行。"; UpdateControls(); return; }
            if ((ulong)pointSequence + (uint)plan.Nodes.Length + 1 > uint.MaxValue) throw new InvalidOperationException("运动序号用尽，请重启设备后回零。");
            StartExperiment("trajectory",pointSequence+1,plan.Nodes);
            uploading = plan; uploadNext = 0; trajectoryEpoch = ptp.Epoch; uploadStartedAt = DateTime.UtcNow;
            xyPreview.PausePlayback();
            Append($"HOST trajectory upload start nodes={plan.Nodes.Length} duration={plan.Duration:F3}s; no automatic approach");
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
        if (run) trajectoryRunSequence = pendingPoint;
        ShowDeviceState(); UpdateControls();
    }
    private void HandleTrajectoryState(string line)
    {
        var m = TrajectoryPattern.Match(line); var values = new uint[6];
        bool valid = m.Success;
        for (int i = 0; valid && i < 6; i++) valid = uint.TryParse(m.Groups[i + 1].Value, out values[i]);
        if (!valid || values[1] > 2048 || values[2] > 2048 || values[3] > values[1]) {
            if (uploading is not null || executing) { Append("HOST malformed trajectory status"); Stop(); }
            return;
        }
        uint mode = values[0], count = values[1], total = values[2], seq = values[4], epoch = values[5];
        if (uploading is { } plan && seq == pendingPoint && epoch == trajectoryEpoch)
        {
            bool run = uploadNext == plan.Nodes.Length;
            uint expected = (uint)(run ? plan.Nodes.Length : uploadNext + 1);
            if (count != expected || total != plan.Nodes.Length || (run ? mode is not (3 or 4) : mode != (count == total ? 2 : 1))) {
                Append("HOST trajectory acknowledgement mismatch"); Stop(); return;
            }
            pendingPoint = 0; pointWire.Clear();
            if (run) { uploading = null; executing = true; }
            else { uploadNext++; SendNextTrajectoryPacket(); }
        }
        if (executing && seq == trajectoryRunSequence && epoch == trajectoryEpoch) {
            if (mode is not (3 or 4) || count != total || total < 2) { Stop(); return; }
            lastTrajectoryAt = DateTime.UtcNow; trajectoryIndex = (int)values[3]; trajectoryTotal = (int)total - 1;
            if (mode == 4) { executing = false; experimentCompleting = "complete"; Append("HOST trajectory complete; firmware arm1 check passed, arm2 commanded only"); }
        }
        ShowDeviceState(); UpdateControls();
    }
    private void RequestPoint(JointPose pose)
    {
        if (!CanPoint || ptp is null || !ptp.Limits.Contains(PtpProtocol.Quantize(pose))) return;
        SafeAction(() => {
            if (pointSequence == uint.MaxValue) throw new InvalidOperationException("PTP 序号用尽，请重启设备后重新建立坐标参考。");
            StartExperiment("point",pointSequence+1,[new(pose,0)]);
            pendingPoint = ++pointSequence; pointRequestedAt = DateTime.UtcNow;
            QueuePointPacket(PtpProtocol.Encode(pendingPoint, ptp.Epoch, pose));
            xyPreview.PausePlayback();
            Append($"HOST PTP request seq={pendingPoint} q1={pose.Q1:F2} q2={pose.Q2:F2}; joint-space move, not a straight XY path");
            ShowDeviceState();
            UpdateControls();
        });
    }
    private void QueuePointPacket(byte[] packet)
    {
        pointPacket = packet; pointAttempts = 1;
        requestEpoch = ptp?.Epoch ?? 0;
        pointWire.Clear();
        foreach (byte b in packet) pointWire.Enqueue(b);
    }
    private void RetryPendingPoint(DateTime now)
    {
        if (pendingPoint == 0) return;
        double age = (now - pointRequestedAt).TotalSeconds;
        if (age >= 3) {
            lastStop.Text=$"设备未确认指令 {pendingPoint}（已发送 {pointAttempts} 次）。已停止；请检查串口和丢包日志，再重新使能。";
            Append($"HOST PTP acknowledgement timeout seq={pendingPoint} attempts={pointAttempts}; stopping"); Stop(); return;
        }
        // Same seq/epoch/payload: firmware only acknowledges duplicates, never restarts motion.
        if (!supportsPacketRetry || pointAttempts >= 3 || age < pointAttempts || pointWire.Count != 0) return;
        if (!enabled || !hardwareReady || homing || stopPending || (now - lastPtpAt).TotalSeconds >= 2) { Stop(); return; }
        pointAttempts++;
        foreach (byte b in pointPacket) pointWire.Enqueue(b);
        Append($"HOST PTP retry seq={pendingPoint} attempt={pointAttempts}/3; identical packet");
    }
    private void Stop()
    {
        xyPreview.PausePlayback();
        pointWire.Clear(); pendingPoint = 0; ptp = null;
        manualReferencePending = false;
        uploading = null; executing = false;
        keyboardMask = mouseMask = 0;
        pendingControl = null; telemetry = null;
        if (serial is not null) {
            stopPending = true;
            state.Text = "停止已请求，等待固件确认";
            SafeAction(() => { Send(0x79); Send(0x41); });
        }
        try { FinishExperiment("aborted"); } catch(Exception ex) { experimentResult.Text = "已请求停止；结果摘要保存失败：" + ex.Message; }
        UpdateControls();
    }
    private void UpdateControls()
    {
        xyPreview.SetEditingAllowed(!PointBusy && !homing);
        enable.IsEnabled = compatible && !enabled && !stopPending && !PointBusy;
        manualReference.IsEnabled = CanManualReference;
        home.IsEnabled = compatible && enabled && !homing && !stopPending && !PointBusy;
        foreach (var button in jogButtons) button.IsEnabled = compatible && enabled && !homing && !stopPending && !PointBusy;
        string reason = !stage2 ? "图形执行需要 stage2-v2 固件；当前只可预览。" :
            ptp is { Locked: true } ? "当前板上仍是锁定固件；请烧录 stage2-manual-reference 版本，再人工确认零位或回零。" :
            PointBusy ? "运动指令发送 / 执行中；总停止按钮可中止。" :
            stopPending ? "等待停止确认；随后重新使能，再人工确认零位或回零。" :
            !enabled ? "未使能：请点“使能 / 重新检查”，再人工确认零位或回零。" :
            homing ? "正在回零：等待两轴完成并报告 HOME OK。" :
            !homed ? (supportsManualReference ? "坐标参考未建立：把两杆摆直，点击上方“人工确认：当前位置为伸直零位”；也可选择开关回零。" : "坐标参考未建立：当前固件需开关回零；人工确认功能需要 stage2-v3。") :
            !feedbackReady ? "驱动、编码器或方向状态未就绪，请查看状态和停止原因。" :
            ptp is not { Referenced: true, Epoch: > 0 } ? "XY 参考无效：重新摆直并人工确认零位，或执行开关回零。" :
            (DateTime.UtcNow - lastPtpAt).TotalSeconds >= 2 ? "XY 固件状态超过 2 秒未更新，请检查通信。" :
            supportsTelemetry && (telemetry is null || (DateTime.UtcNow-lastTelemetryAt).TotalSeconds >= 2) ? "等待 v5 当前角度反馈；超过 2 秒请检查通信。" :
            supportsTelemetry && telemetry is { Inputs: > 0 } ? "传感器或驱动输入触发；请查看左侧设备状态。" :
            keyboardMask != 0 || mouseMask != 0 ? "请松开所有点动按键。" :
            CanPoint ? (supportsTrajectory ? "设备已就绪：目标须通过下方范围检查，再移到轨迹起点并上传执行。固件范围为临时软件限制。" :
                "当前固件仅支持单点移动；图形执行需要 stage2-v2 固件。") :
            "单点执行：等待有效坐标参考、编码器反馈和新鲜固件状态。";
        xyPreview.SetPointMoveState(CanPoint, ptp?.Limits, reason, supportsTrajectory);
        xyPreview.SetStartFeedback(supportsTelemetry && telemetry is { Referenced: true } && ptp is { Referenced: true } &&
            telemetry.Epoch == ptp.Epoch && telemetry.Sequence == ptp.Sequence && (DateTime.UtcNow-lastTelemetryAt).TotalSeconds < 2 ? telemetry : null);
        readInputs.IsEnabled = compatible && !PointBusy && !homing;
        connectionBadge.Text = compatible ? (hardwareUart ? "在线 · v6 硬件串口" : supportsTelemetry ? "在线 · v5 兼容慢速串口" : "在线 · 旧固件，建议更新") : "未连接 · 离线预览";
        bool statusFault=lastStop.Text.Length>0 && (!enabled || stopPending);
        bool statusBusy=PointBusy || homing || stopPending;
        string? statusMessage=stopPending?"停止请求已发送，等待设备确认":
            statusFault?lastStop.Text.Split('\n')[0]:
            uploading is {} plan?$"上传 {Math.Min(uploadNext,plan.Nodes.Length)} / {plan.Nodes.Length} 点，上传期间不运动":
            executing?$"正在绘制 {trajectoryIndex} / {trajectoryTotal} 段":
            pendingControl is not null?$"正在{pendingControl}，等待设备确认":
            homing?"正在开关回零":PointBusy?"正在低速移动 / 到位检查":
            !compatible?"连接设备；离线可预览":!stage2?"当前固件仅支持点动，完整绘图需 v5":
            !enabled?"请在设备页使能 / 重新检查":!homed?"请在设备页确认人工零位或回零":
            CanPoint?null:reason;
        if(uploading is {} upload && uploadNext>0 && !statusFault)
            statusMessage+=$" · 预计剩余 {Math.Max(0,(DateTime.UtcNow-uploadStartedAt).TotalSeconds*(upload.Nodes.Length-uploadNext)/uploadNext):F0} 秒";
        double percent=uploading is {} p?uploadNext*100.0/p.Nodes.Length:executing?trajectoryIndex*100.0/Math.Max(1,trajectoryTotal):0;
        xyPreview.SetDeviceStatus(statusMessage,statusFault,statusBusy,percent,
            stopPending || pendingControl is not null || homing || ptp is { Active: > 0 } && !executing);
    }
    private void SafeAction(Action action)
    {
        try { action(); }
        catch (Exception ex) { Disconnect("通信 / 日志错误：" + ex.Message + "；请确认下位机已停止"); }
    }
    private void Disconnect(string reason)
    {
        timer.Stop(); keyboardMask = mouseMask = 0;
        pendingControl = null; telemetry = null; supportsTelemetry = supportsPacketRetry = false;
        pointWire.Clear(); pendingPoint = 0; ptp = null; stage2 = homed = feedbackReady = false;
        uploading = null; executing = supportsTrajectory = false;
        manualReferencePending = supportsManualReference = hardwareReady = false;
        try { Send(0x79); } catch { /* Firmware heartbeat timeout remains the fallback. */ }
        hardwareUart = false; connectionBadge.ToolTip = null;
        try { FinishExperiment("disconnected: " + reason); } catch(Exception ex) { experimentResult.Text = "实验摘要保存失败：" + ex.Message; }
        try { serial?.Dispose(); } catch { }
        serial = null;
        compatible = enabled = homing = stopPending = false;
        pending.Clear(); state.Text = reason; inputs.Text = "输入状态已失效，需重新连接读取";
        liveValues.Text="设备已断开；当前位置反馈无效。"; errorHistory.Clear(); DrawErrorPlot();
        connect.Content = "连接"; ports.IsEnabled = true; UpdateControls();
        try { Append("HOST " + reason); } catch { }
        try { recording?.Dispose(); } catch { }
        recording = null;
    }
    private void Append(string line)
    {
        string entry = $"[{DateTime.Now:HH:mm:ss.fff}] {line}{Environment.NewLine}";
        recording?.Write(entry);
        bool reading = log.IsKeyboardFocusWithin || log.SelectionLength > 0;
        if (log.Text.Length > 100000 && !reading) log.Text = log.Text[^50000..];
        log.AppendText(entry);
        if (!reading) log.ScrollToEnd();
    }
    public static void SelfTest(string? snapshotPath = null)
    {
        PlanarArm.SelfTest();
        DrawingTrajectory.SelfTest();
        TimedTrajectory.SelfTest();
        PtpProtocol.SelfTest();
        ExecutionTrajectory.SelfTest();
        ExperimentRecorder.SelfTest();
        FeedbackSelfTest();
        XYPreviewPanel.SelfTest(snapshotPath);
        var window = new ControlWindow { selfTesting = true };
        static void Check(bool condition, [System.Runtime.CompilerServices.CallerArgumentExpression(nameof(condition))] string? expression = null)
        { if (!condition) throw new InvalidOperationException("UI state check failed: " + expression); }
        Check(!window.enable.IsEnabled && !window.home.IsEnabled);
        window.HandleLine("STATE fw=stage2-v6 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
        Check(window.hardwareUart && window.supportsTelemetry && window.supportsPacketRetry && !window.enabled);
        window.HandleLine("LINK uart=hw baud=115200 rx_overflow=2 rx_error=1 tx_dropped=0");
        Check(window.connectionBadge.ToolTip?.ToString()?.Contains("rx_overflow=2") == true);
        window.Disconnect("transport reset self-test");
        Check(!window.hardwareUart && !window.compatible && window.connectionBadge.ToolTip is null);
        Check(IsTextInteraction(ModifierKeys.Control, false));
        Check(IsTextInteraction(ModifierKeys.None, true));
        Check(!IsTextInteraction(ModifierKeys.None, false));
        window.log.Text = "copy this line";
        window.log.Select(0, 4);
        window.Append("new line must not steal the selection");
        Check(window.log.SelectedText == "copy");
        window.HandleLine("STOP: P0[3] global limit");
        window.HandleLine("STATE enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(!window.compatible); // old firmware must never unlock motion controls
        window.HandleLine("STATE fw=stage1-v2 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(!window.compatible && !window.home.IsEnabled);
        window.HandleLine("STATE fw=stage1-v3 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
        Check(window.enable.IsEnabled && !window.home.IsEnabled);
        Check(window.lastStop.Text.Contains("STOP: P0[3] global limit"));
        window.HandleLine("STATE fw=stage1-v3 enabled=1 home=6 homed1=1 homed2=0 fault=0 enc=1 dir=1");
        Check(window.homing && !window.home.IsEnabled && window.jogButtons.All(b => !b.IsEnabled));
        window.HandleLine("STATE fw=stage1-v3 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(window.home.IsEnabled);
        window.stopPending = true;
        window.HandleLine("STATE fw=stage1-v3 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        Check(!window.home.IsEnabled); // stale enabled report cannot cancel a pending stop
        window.HandleLine("STATE fw=stage1-v3 enabled=0 home=0 homed1=0 homed2=0 fault=1 enc=1 dir=1");
        Check(window.enable.IsEnabled && !window.home.IsEnabled && !window.stopPending);
        window.HandleLine("STATE fw=stage2-v1 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.HandleLine("PTP_STATE lock=1 ref=1 active=0 epoch=7 seq=0 min1=0 max1=0 min2=0 max2=0");
        Check(!window.CanPoint);
        window.HandleLine("PTP_STATE lock=0 ref=1 active=0 epoch=7 seq=0 min1=-6000 max1=6000 min2=-9000 max2=9000");
        Check(window.CanPoint);
        window.RequestPoint(new JointPose(1.25, -2.5));
        Check(window.pendingPoint==1 && window.pointWire.Count==36 && !window.CanPoint && !window.home.IsEnabled);
        window.Stop(); Check(window.pendingPoint==0 && window.pointWire.Count==0 && !window.CanPoint);
        window.HandleLine("PTP_STATE lock=0 ref=1 active=1 epoch=7 seq=1 min1=-6000 max1=6000 min2=-9000 max2=9000");
        Check(!window.CanPoint && !window.home.IsEnabled);
        window.HandleLine("PTP_STATE lock=0 ref=1 active=0 epoch=7 seq=1 min1=-6000 max1=6000 min2=-9000 max2=9000");
        Check(window.CanPoint);
        window.lastPtpAt = DateTime.UtcNow.AddSeconds(-3); Check(!window.CanPoint);
        window.HandleLine("PTP_STATE malformed"); Check(!window.CanPoint);
        window.HandleLine("STATE fw=stage2-v3 enabled=1 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
        window.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=20 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000");
        Check(window.CanManualReference && window.manualReference.IsEnabled && !window.CanPoint);
        window.RequestManualReference();
        uint manualSeq = window.pendingPoint;
        Check(window.manualReferencePending && window.pointWire.Count == 36 && !window.CanManualReference);
        window.HandleLine("STATE fw=stage2-v3 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.HandleLine($"PTP_STATE lock=0 ref=1 active=0 epoch=21 seq={manualSeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        Check(window.CanPoint && !window.manualReferencePending && window.pointWire.Count == 0);
        window.HandleLine("STATE fw=stage2-v4 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.RequestPoint(new JointPose(-21.95, 44.37));
        uint retrySeq = window.pendingPoint;
        byte[] original = window.pointWire.ToArray();
        window.pointWire.Clear(); // entire first transmission/ACK lost
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(900));
        Check(window.pointWire.Count == 0 && window.pointAttempts == 1);
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(1100));
        Check(window.pendingPoint == retrySeq && window.pointAttempts == 2 && window.pointWire.SequenceEqual(original));
        window.HandleLine("PTP RX dropped: incomplete nibbles=30/34");
        Check(window.pendingPoint == retrySeq); // transport drop is not a motion refusal
        window.pointWire.Clear();
        window.lastPtpAt = window.pointRequestedAt.AddSeconds(2);
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(2100));
        Check(window.pointAttempts == 3 && window.pointWire.SequenceEqual(original));
        window.RetryPendingPoint(window.pointRequestedAt.AddSeconds(3));
        Check(window.pendingPoint == 0 && window.pointWire.Count == 0);
        window.RetryPendingPoint(window.pointRequestedAt.AddSeconds(4));
        Check(window.pointWire.Count == 0); // timeout cannot restart anything
        window.HandleLine($"PTP_STATE lock=0 ref=1 active=0 epoch=21 seq={retrySeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        window.RequestPoint(new JointPose(-21.95, 44.37));
        retrySeq = window.pendingPoint;
        window.HandleLine($"PTP_STATE lock=0 ref=1 active=1 epoch=21 seq={retrySeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        window.RetryPendingPoint(window.pointRequestedAt.AddMilliseconds(1100));
        Check(window.pendingPoint == 0 && window.pointWire.Count == 0); // ACK cancels retransmission
        window.HandleLine("STOP: driver fault");
        Check(window.ptp is null && window.pointWire.Count == 0);
        window.HandleLine($"PTP_STATE lock=0 ref=1 active=0 epoch=22 seq={retrySeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        window.RequestPoint(new JointPose(-21.95, 44.37));
        window.HandleLine("PTP refused: joint range/rate");
        window.RetryPendingPoint(window.pointRequestedAt.AddSeconds(1));
        Check(window.pendingPoint == 0 && window.pointWire.Count == 0); // genuine refusal never retries
        window.Stop(); Check(!window.CanPoint && !window.CanManualReference);
        window.HandleLine("STATE fw=stage2-v3 enabled=1 home=0 homed1=0 homed2=0 fault=0 enc=0 dir=0");
        window.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=22 seq=2 min1=-3000 max1=3000 min2=-6000 max2=6000");
        Check(!window.CanManualReference);
        var path = DrawingTrajectory.Create(new PlanarArm(180, 176.38), DrawingShape.Circle, 340, 0, 5, 0, 0, null);
        var execution = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(20, 5, 3, 10, 6)), new JointLimits(-30, 30, -60, 60));
        window.HandleLine("STATE fw=stage2-v5 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
        window.HandleLine("PTP_STATE lock=0 ref=1 active=0 epoch=8 seq=1 min1=-6000 max1=6000 min2=-9000 max2=9000");
        void StartSample(uint epoch, uint seq) => window.HandleLine($"TELEM t=1000 epoch={epoch} seq={seq} ref=1 source=2 p1={ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose.Q1)} p2={ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose.Q2)} e1=0 err=0 inputs=0 drops=0");
        window.RequestTrajectory(execution); Check(window.uploading is null); // missing position feedback
        StartSample(8,1);
        window.RequestTrajectory(execution);
        Check(window.uploading is not null && window.pointWire.Count == 36 && !window.CanPoint && !window.home.IsEnabled);
        uint first = window.pendingPoint;
        window.HandleLine($"PTP_STATE lock=0 ref=1 active=0 epoch=8 seq={first} min1=-6000 max1=6000 min2=-9000 max2=9000");
        Check(window.pendingPoint == first && window.uploadNext == 0); // PTP status is not a node acknowledgement
        for (int i = 0; i < execution.Nodes.Length; i++) {
            uint seq = window.pendingPoint;
            window.HandleLine($"TRAJ_STATE state={(i + 1 == execution.Nodes.Length ? 2 : 1)} count={i + 1} total={execution.Nodes.Length} index=0 seq={seq} epoch=8");
            Check(window.uploadNext == i + 1 && window.pendingPoint == seq + 1);
            if (i == 0) { // duplicate ack cannot enqueue the next node twice
                window.HandleLine($"TRAJ_STATE state=1 count=1 total={execution.Nodes.Length} index=0 seq={seq} epoch=8");
                Check(window.uploadNext == 1 && window.pointWire.Count == 36);
            }
        }
        Check(window.uploading is not null && !window.executing); // waits for explicit run acknowledgement
        window.HandleLine($"TRAJ_STATE state=3 count={execution.Nodes.Length} total={execution.Nodes.Length} index=1 seq={window.pendingPoint} epoch=8");
        Check(window.uploading is null && window.executing && window.pointWire.Count == 0 && window.pendingPoint == 0);
        window.HandleLine($"TRAJ_STATE state=4 count={execution.Nodes.Length} total={execution.Nodes.Length} index={execution.Nodes.Length} seq={window.trajectoryRunSequence} epoch=8");
        Check(!window.executing);
        window.HandleLine($"PTP_STATE lock=0 ref=1 active=0 epoch=8 seq={window.trajectoryRunSequence} min1=-6000 max1=6000 min2=-9000 max2=9000");
        StartSample(8,window.trajectoryRunSequence);
        window.RequestTrajectory(execution); Check(window.uploading is not null);
        window.Stop(); Check(window.uploading is null && window.pointWire.Count == 0 && window.pendingPoint == 0);
        window.HandleLine("PTP_STATE lock=0 ref=1 active=0 epoch=9 seq=9999 min1=-6000 max1=6000 min2=-9000 max2=9000");
        StartSample(9,9999);
        window.RequestTrajectory(execution);
        window.HandleLine($"TRAJ_STATE state=1 count=2 total={execution.Nodes.Length} index=0 seq={window.pendingPoint} epoch=9");
        Check(window.uploading is null && window.pointWire.Count == 0); // wrong count stops upload
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
            window.HandleLine("STATE fw=stage2-v5 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            window.HandleLine("PTP_STATE lock=0 ref=1 active=0 epoch=30 seq=10000 min1=-3000 max1=3000 min2=-6000 max2=6000");
            int p1=ExecutionTrajectory.Counts(0,execution.Nodes[0].Pose.Q1), p2=ExecutionTrajectory.Counts(1,execution.Nodes[0].Pose.Q2);
            window.errorHistory.Clear();
            for(int i=0;i<40;i++) window.HandleLine($"TELEM t={i*500} epoch=30 seq=10000 ref=1 source=2 p1={p1} p2={p2} e1={(int)Math.Round(p1*2.56)+i%9-4} err=0 inputs=0 drops=0");
            window.lastStop.Text=""; window.connectionBadge.Text="离线界面检查 · 模拟数据";
            string dir=Path.GetDirectoryName(snapshotPath)!;
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench.png"),1380,930);
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-compact.png"),1100,760);
            window.sideTabs.SelectedIndex=1; window.diagnostics.IsExpanded=true;
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-feedback.png"),1380,930);
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-feedback-compact.png"),1100,760);
            window.diagnostics.IsExpanded=false; window.xyPreview.FitView(false);
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-full-arm.png"),1380,930);
            window.xyPreview.FitView(true); window.sideTabs.SelectedIndex=0;
            window.HandleLine("STATE fw=stage2-v5 enabled=0 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            window.connectionBadge.Text="离线界面检查 · 模拟未使能";
            window.SaveWorkbenchSnapshot(Path.Combine(dir,"workbench-waiting.png"),1100,760);
        }
        window.Close();
    }
}
