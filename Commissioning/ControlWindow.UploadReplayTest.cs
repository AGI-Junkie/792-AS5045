using System.Buffers.Binary;
using System.Diagnostics;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    internal static void UploadLanguageTest() => System.IO.File.WriteAllText("language-upload-tests.txt", WithLeaseRetry(UploadReplaySelfTest));
    // The real upload/ACK queue and UI dispatcher; only the serial peer is simulated.
    private static string UploadReplaySelfTest()
    {
        bool initialLanguage = Lang.IsEn;
        var window = new ControlWindow { selfTesting = true, ShowActivated = false, ShowInTaskbar = false, Left = -12000, Top = -12000 };
        window.heartbeatTimer.Tick -= window.OnHeartbeatTick;
        var limits = new JointLimits(-30, 30, -60, 60);
        var path = DrawingTrajectory.Create(new PlanarArm(180, 176.38), DrawingShape.Circle, 340, 0, 5, 0, 0, limits);
        var plan = new ExecutionTrajectory(new TimedTrajectory(path, new PlanningRates(0.65, 2, 1.5, 2, 1.5)), limits);
        if (plan.Nodes.Length < 362) throw new InvalidOperationException("Upload replay must exercise at least 362 nodes.");
        var seed = new StringBuilder();
        while (seed.Length < 92000)
            seed.AppendLine("[19:30:19.432] TELEM t=49000 epoch=2 seq=1 ref=1 source=2 p1=0 p2=0 e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
        window.logLevelBox.SelectedIndex = (int)LogLevel.Trace; // Exercise full-output trimming even though the UI defaults to Info.
        window.log.Text = seed.ToString();
        window.diagnostics.IsExpanded = true;
        var tabs = (TabControl)window.diagnostics.Content;
        tabs.SelectedIndex = 1;
        var clock = new Stopwatch();
        var frame = new DispatcherFrame();
        var incoming = new Queue<string>();
        var wire = new List<byte>();
        var heartbeatTimes = new List<double>();
        var sentFrames = new List<(double Start, double End, byte Type, int Node)>();
        var heartbeatSchedule = new List<(double Gap, double LeaseGap, int Node, bool Run, int G0, int G1, int G2)>();
        int copies = 0, received = 0, batches = 0, view = 1, progressChecks = 0;
        int completedAtBeat = -1, initialG0 = 0, initialG1 = 0, initialG2 = 0;
        uint sequence = 1, control = 0;
        bool runAccepted = false, trimmed = false, runRequested = false;
        double readySince = 0;
        double longestReceive = 0, nextStatus = 0;
        double sendStarted = 0, lastDispatchEnd = 0, leaseGap = 0, longestLease = 0, runSent = 0, runAck = 0;
        Exception? failure = null;
        // Frames now leave on the writer thread, so everything the simulated peer
        // touches is taken under one lock and failures are carried back to the UI.
        var peerSync = new object();
        bool writerStop = false;
        Exception? writerFailure = null;
        void Require(bool condition, string reason)
        {
            if (!condition) throw new InvalidOperationException("Upload replay: " + reason);
        }
        Require(window.logCharacters == seed.Length, "log length must track text replacement without reading Text");
        void Status()
        {
            // Firmware emits one coherent status block; the simulated writer may advance meanwhile.
            uint statusSequence;
            lock (peerSync) { statusSequence = sequence; }
            window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            window.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=2 seq={statusSequence} min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            window.TestLine($"TELEM t={49000 + batches * 500} epoch=2 seq={statusSequence} ref=1 source=2 p1={ExecutionTrajectory.Counts(0, plan.Nodes[0].Pose)} p2={ExecutionTrajectory.Counts(1, plan.Nodes[0].Pose)} e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            // Advertise the compact heartbeat so the replay exercises the configuration
            // that actually ships, not a legacy one.
            window.TestLine($"LINK uart=hw baud=115200 rx_overflow=0 rx_error=0 tx_dropped=0 fw=stage2-v7-absolute-q2 short_hb=1 motion_rates=1 joint_limits=1 q2_absolute=1 proto={DeviceContract.ProtocolVersion}");
            batches++;
        }
        void Write(byte[] data, int offset, int count)
        {
          lock (peerSync)
          {
            Require(count == 1 && offset == copies / 2, "nonconsecutive paced byte");
            if (copies == 0) sendStarted = clock.Elapsed.TotalMilliseconds;
            if (copies++ % 2 == 0) wire.Add(data[offset]);
            else Require(wire[^1] == data[offset], "byte copies differ");
            // Frames are either the 17-byte control/trajectory envelope or the compact
            // 6-byte heartbeat; the delimiter says which.
            if (wire.Count == 0) return;
            int expectedWire = wire[0] == 0xd5 ? 6 : 36;
            if (copies != expectedWire * 2) return;
            double completed = clock.Elapsed.TotalMilliseconds;
            copies = 0;
            if (wire[0] == 0xd5) {
                Require(wire.Count == 6 && wire[^1] == 0xd7, "incomplete compact heartbeat");
                byte keys = (byte)(((wire[1] >> 1) & 15) | (((wire[2] >> 1) & 15) << 4));
                byte check = (byte)(((wire[3] >> 1) & 15) | (((wire[4] >> 1) & 15) << 4));
                Require(keys == 0 && check == PtpProtocol.Crc8(keys), "compact heartbeat payload invalid");
                // A compact heartbeat carries no sequence; if it consumed one, the
                // consecutive-sequence check on trajectory frames below would fail.
                wire.Clear();
                sentFrames.Add((sendStarted, completed, (byte)6, received));
                heartbeatSchedule.Add((heartbeatTimes.Count == 0 ? 0 : completed - heartbeatTimes[^1],
                    longestLease, received, runAccepted, GC.CollectionCount(0), GC.CollectionCount(1), GC.CollectionCount(2)));
                heartbeatTimes.Add(completed);
                return;
            }
            Require(wire.Count == 36 && wire[0] == 0xd1 && wire[^1] == 0xd3, "incomplete/interleaved frame");
            var payload = new byte[17];
            for (int i = 0; i < payload.Length; i++) {
                Require((wire[1 + i * 2] & 0xe1) == 0x81 && (wire[2 + i * 2] & 0xe1) == 0xa1, "invalid nibble marker");
                payload[i] = (byte)(((wire[1 + i * 2] >> 1) & 15) | (((wire[2 + i * 2] >> 1) & 15) << 4));
            }
            byte[] packet = wire.ToArray(); wire.Clear();
            uint seq = BinaryPrimitives.ReadUInt32LittleEndian(payload.AsSpan(1));
            Require(BinaryPrimitives.ReadUInt32LittleEndian(payload.AsSpan(5)) == 2, "reference epoch changed");
            Require(BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(15)) == PtpProtocol.Crc(payload.AsSpan(0, 15)), "CRC mismatch");
            byte type = payload[0];
            sentFrames.Add((sendStarted, completed, type, received));
            if (type == 6) {
                Require(seq > control && payload.AsSpan(9, 6).IndexOfAnyExcept((byte)0) < 0, "heartbeat sequence/keys invalid");
                heartbeatSchedule.Add((heartbeatTimes.Count == 0 ? 0 : completed - heartbeatTimes[^1],
                    longestLease, received, runAccepted, GC.CollectionCount(0), GC.CollectionCount(1), GC.CollectionCount(2)));
                control = seq; heartbeatTimes.Add(completed); return;
            }
            Require(seq == sequence + 1, "trajectory sequence skipped or duplicated");
            bool run = received == plan.Nodes.Length;
            byte expectedType = run ? (byte)4 : received == 0 ? (byte)2 : (byte)3;
            Require(type == expectedType && !runAccepted, "BEGIN/NODE/RUN order invalid");
            Require(!run || runRequested, "RUN left before the operator asked for 执行图形");
            var node = run ? new ExecutionNode(new JointPose(0, 0), 0) : plan.Nodes[received];
            ushort value = received == 0 ? (ushort)plan.Nodes.Length : node.Milliseconds;
            Require(packet.SequenceEqual(PtpProtocol.EncodePacket(expectedType, seq, 2, node.Pose, value)), "trajectory payload differs from checked plan");
            sequence = seq;
            if (run) { runAccepted = true; runSent = completed; }
            else received++;
            incoming.Enqueue($"TRAJ_STATE state={(run ? 4 : received == plan.Nodes.Length ? 2 : 1)} count={received} total={plan.Nodes.Length} index={(run ? received : 0)} seq={seq} epoch=2");
          }
        }
        // The UI tick only republishes the lease; the writer thread owns the wire.
        // Lease lateness is reported next to the heartbeat gap on purpose: the whole
        // point of this build is that dispatcher lateness no longer lands in it.
        window.heartbeatTimer.Tick += (_, _) => {
            double now = clock.Elapsed.TotalMilliseconds;
            leaseGap = now - lastDispatchEnd;
            longestLease = Math.Max(longestLease, leaseGap);
            lastDispatchEnd = now;
            try { window.PublishHeartbeatLease(); }
            catch (Exception ex) { failure = ex; frame.Continue = false; }
        };
        var writer = new Thread(() => {
            while (!writerStop) {
                try { if (!window.WireWriterStep(Write)) window.WireIdleWait(); }
                catch (Exception ex) { writerFailure = ex; return; }
            }
        }) { IsBackground = true };
        var replay = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(40) };
        replay.Tick += (_, _) => {
            var receive = Stopwatch.StartNew();
            try {
                string[] lines;
                lock (peerSync) { lines = incoming.ToArray(); incoming.Clear(); }
                foreach (string line in lines) window.TestLine(line);
                if (clock.Elapsed.TotalMilliseconds >= nextStatus) {
                    Status(); nextStatus = clock.Elapsed.TotalMilliseconds + 500;
                    Lang.Set(batches % 2 == 0, persist: false);
                }
                if (received >= plan.Nodes.Length / 2 && view != 0) { view = 0; tabs.SelectedIndex = 0; }
                if (!runAccepted && (progressChecks == 0 && received > 0 || progressChecks == 1 && view == 0)) {
                    window.xyPreview.AssertUploadProgressTemplate(); progressChecks++;
                }
                window.RetryPendingPoint(DateTime.UtcNow);
                window.UpdateControls();
                trimmed |= window.logCharacters < 80000;
                Require(window.uploading is not null || window.uploaded is not null || runAccepted, "upload cancelled: " + window.lastStop.Text);
                // Upload ends in a held plan; nothing may move until 执行图形, then RUN goes out on its own frame.
                if (window.uploaded is not null && !runRequested) {
                    if (readySince == 0) readySince = clock.Elapsed.TotalMilliseconds;
                    else if (clock.Elapsed.TotalMilliseconds - readySince >= 600) {
                        lock (peerSync) { Require(received == plan.Nodes.Length && !runAccepted && sentFrames.All(f => f.Type != 4), "the device was started without 执行图形"); }
                        runRequested = true; window.RequestExecute();
                        Require(window.uploading is not null && window.uploaded is null, "执行图形 did not queue RUN: " + window.lastStop.Text);
                    }
                }
                Require(clock.Elapsed.TotalSeconds < 90, "upload did not finish within 90 seconds");
                if (runAccepted && window.uploading is null && !window.executing) {
                    int beats;
                    lock (peerSync) { beats = heartbeatTimes.Count; }
                    if (completedAtBeat < 0) { completedAtBeat = beats; runAck = clock.Elapsed.TotalMilliseconds; }
                    if (beats >= completedAtBeat + 2) frame.Continue = false;
                }
            }
            catch (Exception ex) { failure = ex; frame.Continue = false; }
            longestReceive = Math.Max(longestReceive, receive.Elapsed.TotalMilliseconds);
        };
        try {
            window.Show(); window.UpdateLayout(); Status();
            Require(window.log.Text.Contains("HOST heartbeat: compact 6 wire bytes per beat"),
                "connection log does not state which heartbeat form was used");
            initialG0 = GC.CollectionCount(0); initialG1 = GC.CollectionCount(1); initialG2 = GC.CollectionCount(2);
            clock.Start();
            window.RequestTrajectory(plan);
            Require(window.uploading is not null && window.pointWire.Count == 36, "real upload did not queue BEGIN");
            window.heartbeatTimer.Start(); replay.Start(); writer.Start();
            Dispatcher.PushFrame(frame);
            writerStop = true; writer.Join(500);
            if (failure is not null) throw failure;
            if (writerFailure is not null) throw writerFailure;
            Require(runAccepted && received == plan.Nodes.Length && window.pendingPoint == 0 && view == 0 && trimmed && progressChecks == 2, "incomplete upload or missing view/log/progress coverage");
            Require(heartbeatTimes.Count > plan.Nodes.Length / 2, "heartbeat starvation");
            // Include synchronous upload start and two real heartbeats after the final ACK.
            // Time zero stands for a valid idle heartbeat just before the upload request.
            double[] timeline = [0, .. heartbeatTimes];
            double longest = timeline.Zip(timeline.Skip(1), (a, b) => b - a).Max();
            // Explicitly discard the complete heartbeat whose loss gives the worst receive gap.
            int drop = Enumerable.Range(1, heartbeatTimes.Count - 1)
                .MaxBy(i => timeline[i + 1] - timeline[i - 1]);
            double[] accepted = timeline.Where((_, i) => i != drop).ToArray();
            double lostGap = accepted.Zip(accepted.Skip(1), (a, b) => b - a).Max();
            string report = $"v7 real upload: {received} nodes + RUN ACK, {clock.Elapsed.TotalSeconds:F1} s, {batches} status/telemetry batches, raw log/feedback views, log trimmed={trimmed}, visible static progress checks={progressChecks}; complete heartbeat max gap={longest:F1} ms (first={heartbeatTimes[0]:F1} ms); dropped complete heartbeat #{drop}, receive max gap={lostGap:F1} ms; receive callback={longestReceive:F1} ms; UI lease max gap={longestLease:F1} ms; beats={heartbeatTimes.Count}.";
            if (longestLease >= KeyStaleMs) report += $" WARN: UI lease gap {longestLease:F0} ms >= {KeyStaleMs:F0} ms (a held key would be released).";
            report += $"\nRUN fully sent at {runSent:F1} ms, ACK handled at {runAck:F1} ms; {heartbeatTimes.Count - completedAtBeat} real heartbeats after completion.";
            foreach (int i in Enumerable.Range(0, heartbeatTimes.Count)
                .OrderByDescending(i => timeline[i + 1] - timeline[i]).Take(5)) {
                var schedule = heartbeatSchedule[i];
                var before = i == 0 ? (initialG0, initialG1, initialG2) : (heartbeatSchedule[i - 1].G0, heartbeatSchedule[i - 1].G1, heartbeatSchedule[i - 1].G2);
                string frames = string.Join(", ", sentFrames.Where(f => f.End > timeline[i] && f.End <= timeline[i + 1])
                    .Select(f => $"type{f.Type}/node{f.Node}={f.End - f.Start:F1}ms"));
                report += $"\nHB#{i + 1} at {timeline[i + 1]:F1}ms: gap={timeline[i + 1] - timeline[i]:F1}ms, node={schedule.Node}, run={schedule.Run}, writer gap={schedule.Gap:F1}ms, max UI lease gap={schedule.LeaseGap:F1}ms, GC deltas={schedule.G0 - before.Item1}/{schedule.G1 - before.Item2}/{schedule.G2 - before.Item3}; frames [{frames}].";
            }
            if (longest >= 125 || lostGap >= 250) throw new InvalidOperationException(report);
            if (longestLease >= LeaseStopMs) throw new LeaseGapException(report);
            Require(window.logCharacters == window.log.Text.Length, "log length drifted after append/trim");
            // The saved log has to state the configuration and the run's worst case on
            // its own, so a field log can be read without asking the operator anything.
            string tail = window.log.Text;
            // The initial capability announcement may have scrolled out of the
            // bounded UI log; the on-disk log retains it and the summary repeats it.
            window.Disconnect("upload replay complete");
            tail = window.log.Text;
            int at = tail.LastIndexOf("HOST session:", StringComparison.Ordinal);
            Require(at >= 0, "session summary line missing from the log");
            string summary = tail[at..];
            Require(summary.Contains("fw=stage2-v7-absolute-q2") && summary.Contains("heartbeat=compact6B/20ms") && summary.Contains("dataFrames=") &&
                summary.Contains("worstHeartbeatGap=") && summary.Contains("worstUiLeaseGap=") &&
                summary.Contains("heartbeats="),
                "session summary is incomplete: " + summary.Split('\n')[0]);
            return report + $"\nLanguage switched {batches} times during upload/held/run.\n" + summary.Split('\n')[0] + "\nNo serial port opened; no motor commands sent.\n";
        }
        finally { writerStop = true; replay.Stop(); window.heartbeatTimer.Stop(); window.Close(); Lang.Set(initialLanguage, persist: false); }
    }
}

public sealed partial class XYPreviewPanel
{
    internal void AssertUploadProgressTemplate()
    {
        actionProgress.ApplyTemplate(); actionProgress.UpdateLayout();
        if (!actionProgress.IsVisible || actionProgress.IsIndeterminate ||
            actionProgress.Template.FindName("PART_GlowRect", actionProgress) is not null ||
            actionProgress.Template.FindName("PART_Track", actionProgress) is not FrameworkElement track ||
            actionProgress.Template.FindName("PART_Indicator", actionProgress) is not FrameworkElement indicator ||
            track.ActualWidth <= 0 || indicator.HasAnimatedProperties ||
            Math.Abs(indicator.ActualWidth - track.ActualWidth * (actionProgress.Value - actionProgress.Minimum) /
                (actionProgress.Maximum - actionProgress.Minimum)) > 1)
            throw new InvalidOperationException("Visible upload progress must be static and its indicator width must follow Value.");
    }
}
