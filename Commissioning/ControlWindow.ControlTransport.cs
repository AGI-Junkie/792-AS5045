using System.Diagnostics;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private bool framedControl;
    private uint controlSequence, controlEpoch;
    private uint reportedEpoch;
    private bool waitingStopEpoch;
    private byte pendingControlType;
    private long lastControlHeartbeat;
    private bool controlDataSent;

    // --- v7 writer thread ---------------------------------------------------
    // The paced frame write used to run inside a UI dispatcher callback, so every
    // heartbeat interval paid the 35.5 ms frame cost twice: once for the send and
    // once for the dispatcher lateness that blocking callback caused.  Field log
    // stage1-20260927-220423-569.log shows 150-249 ms send gaps against a 250 ms
    // firmware deadman.  One dedicated thread now owns the wire; the UI thread only
    // queues frames and republishes a liveness lease, so an unresponsive window
    // still stops the heartbeat and the firmware still stops the machine.
    private const double ShortHeartbeatIdleMs = 20;  // idle after a compact (6-byte) heartbeat
    private const double LongHeartbeatIdleMs = 40;   // idle after a legacy 36-byte heartbeat
    private const double KeyStaleMs = 100;           // stop honouring held keys this long after the UI stops
    private const double LeaseStopMs = 200;          // stop heartbeating this long after the UI stops
    private const double SlowHeartbeatLogMs = 100;   // report a gap before it becomes a stop
    private const double SlowLeaseLogMs = 120;       // report a stalled UI dispatcher turn
    private const double SlowLeaseLogPeriodMs = 1000; // ...at most this often
    private readonly object wireSync = new();
    private readonly Queue<byte> urgentWire = new();
    private Thread? wireThread;
    private volatile bool wireStop;
    private long leasePublishedAt;      // Stopwatch ticks; 0 = never published
    private uint leaseEpoch;            // 0 = no motion lease, send nothing (Volatile.Read/Write)
    private int leaseKeys;              // jog mask the firmware should honour (Volatile.Read/Write)
    private double pendingHeartbeatGap; // written by the writer, logged by the UI tick
    private double worstLeaseGap;       // max UI dispatcher gap in the current log period
    private long lastLeaseLogAt;
    // Session totals, reported as one line when the link closes so a saved log always
    // carries the run's worst case without having to scan every 500 ms status block.
    private long sessionHeartbeats, sessionFrames, sessionSlowHeartbeats, sessionSlowLeases;
    private double sessionWorstHeartbeatGap, sessionWorstLeaseGap;
    private string lastFirmwareBuild = "unknown";

    private void ResetControlTransport()
    {
        lock (wireSync)
        {
            pendingControlType = 0;
            controlEpoch = 0; lastControlHeartbeat = 0;
            controlDataSent = false;
        }
        if (framedControl) heartbeatTimer.Interval = TimeSpan.FromMilliseconds(8);
    }

    private void ObserveControlEpoch(uint epoch)
    {
        lock (wireSync) {
            if (epoch != reportedEpoch) {
                waitingStopEpoch = false;
                reportedLimits = null; reportedProfile = null;
                Volatile.Write(ref limitsMotionAllowed, 0);
            }
            reportedEpoch = epoch;
        }
    }

    // Runs on the UI dispatcher thread only.  Publishing stops the moment this stops
    // running, which is exactly the previous "a hung UI cannot keep motion alive"
    // behaviour; the writer refuses to send once the lease is older than LeaseStopMs.
    private void PublishHeartbeatLease()
    {
        long now = Stopwatch.GetTimestamp();
        long previous = Volatile.Read(ref leasePublishedAt);
        // The lease window (LeaseStopMs) is the one safety timing number left that is
        // still chosen from simulation alone.  Logging the real dispatcher gap here is
        // what lets it be tightened from field evidence instead of a guess -- but it is
        // throttled, because appending a log line during a stall extends the stall and
        // would turn the measurement into its own cause.
        if (previous != 0) {
            double leaseGap = Stopwatch.GetElapsedTime(previous, now).TotalMilliseconds;
            if (leaseGap > worstLeaseGap) worstLeaseGap = leaseGap;
            if (leaseGap > sessionWorstLeaseGap) sessionWorstLeaseGap = leaseGap;
            if (leaseGap > SlowLeaseLogMs) sessionSlowLeases++;
            if (leaseGap > SlowLeaseLogMs && Stopwatch.GetElapsedTime(lastLeaseLogAt, now).TotalMilliseconds >= SlowLeaseLogPeriodMs) {
                lastLeaseLogAt = now;
                Append($"HOST UI lease gap max={worstLeaseGap:F0} ms");
                worstLeaseGap = 0;
            }
        }
        Volatile.Write(ref leaseKeys, CanJog ? keyboardMask | mouseMask : 0);
        // If raw STOP is lost, silence must still trip the firmware deadman. Only
        // resume epoch probes once STATE confirms that motion is disabled.
        lock (wireSync) { Volatile.Write(ref leaseEpoch, framedControl && !stopPending ? reportedEpoch : 0u); }
        Volatile.Write(ref leasePublishedAt, now);
        double gap = Interlocked.Exchange(ref pendingHeartbeatGap, 0);
        if (gap > 0) Append($"HOST control heartbeat gap={gap:F0} ms");
    }

    // Asks the writer for a heartbeat now: used when a key is released, when the
    // window is deactivated, and before enable/home changes the reference epoch.
    private void ForceHeartbeatNow()
    {
        lock (wireSync) { lastControlHeartbeat = 0; }
    }

    // Two windows, because the two hazards differ.  A held jog key must stop as soon
    // as the UI stops responding (KeyStaleMs).  Heartbeats themselves keep going for
    // the firmware deadman window (LeaseStopMs), so an ordinary long dispatcher job
    // cannot by itself become a false stop; a truly hung UI still stops the machine,
    // at most LeaseStopMs + one frame + the 250 ms firmware deadline after the hang.
    // The idle target has to stay proportional to what a beat costs on the wire.  A
    // 36-byte frame spends 35.5 ms of paced writes, so asking for a 20 ms idle would
    // keep the writer busy ~65% of the time and let it fight the UI for the CPU; the
    // compact frame costs 5.5 ms and can afford the tighter beat.
    private double HeartbeatIdleTarget() => supportsShortHeartbeat ? ShortHeartbeatIdleMs : LongHeartbeatIdleMs;

    private long LeaseAgeMs()
    {
        long published = Volatile.Read(ref leasePublishedAt);
        return published == 0 ? long.MaxValue : (long)Stopwatch.GetElapsedTime(published).TotalMilliseconds;
    }

    private uint LeasedEpoch() => LeaseAgeMs() > LeaseStopMs ? 0u : Volatile.Read(ref leaseEpoch);

    private ushort LeasedKeys() => LeaseAgeMs() > KeyStaleMs ? (ushort)0 : (ushort)Volatile.Read(ref leaseKeys);

    private static void WriteControlFrame(Action<byte[], int, int> write, byte[] packet)
    {
        // KitProg bench A/B: bursts caused framing errors; single paced copies
        // still lost bytes. Existing firmware deduplication accepts two copies.
        // One frame per writer step, never interleaved with another frame.
        for (int i = 0; i < packet.Length * 2; i++) {
            write(packet, i / 2, 1);
            if (i + 1 < packet.Length * 2) {
                long sent = Stopwatch.GetTimestamp();
                while (Stopwatch.GetElapsedTime(sent).TotalMilliseconds < 0.5) Thread.SpinWait(20);
            }
        }
    }

    // ---- the single writer -------------------------------------------------
    private void StartWireWriter()
    {
        if (wireThread is not null) return;
        wireStop = false;
        // AboveNormal so UI load cannot delay a beat, but the writer only ever sleeps
        // (never spins): a spinning AboveNormal writer starved the UI dispatcher in the
        // offline replay, which is the opposite failure.
        wireThread = new Thread(WireWriterLoop) { IsBackground = true, Name = "v7 wire writer", Priority = ThreadPriority.AboveNormal };
        wireThread.Start();
    }

    private void StopWireWriter()
    {
        wireStop = true;
        wireThread?.Join(500);
        wireThread = null;
        lock (wireSync) { urgentWire.Clear(); }
    }

    private void WireWriterLoop()
    {
        while (!wireStop)
        {
            try
            {
                if (serial is not { IsOpen: true }) { Thread.Sleep(5); continue; }
                // One complete frame per step, never interleaved.
                if (WireWriterStep(serial.Write)) continue;
                // Nothing to send.  Thread.Sleep(1) would be quantised to the ~15.6 ms
                // system tick, and a sleeping thread can be rescheduled tens of ms late
                // under load: the offline gate measured 94 ms of idle wait where 40 ms
                // was intended.  Sleep only while far from the deadline and yield for
                // the rest, so the frame leaves on time.
                WireIdleWait();
            }
            catch (Exception ex)
            {
                Trace.TraceError("v7 wire writer: " + ex);
                Thread.Sleep(5);
            }
        }
    }

    // Shared by the production loop and the offline replays so a replay measures the
    // policy that actually ships.  Sleep only, never spin: the delay is recomputed
    // from the deadline each pass, so a late wake costs one bounded overshoot instead
    // of the compounding 15.6 ms quantisation that Thread.Sleep(1) produced.
    // A queued trajectory packet cuts the wait short (wireWake) instead of sitting out up to 17 ms.
    private readonly AutoResetEvent wireWake = new(false);
    private void WireIdleWait()
    {
        if (LeasedEpoch() == 0) { Thread.Sleep(5); return; }
        double remaining = MsUntilHeartbeatDue();
        wireWake.WaitOne(remaining > 4 ? (int)(remaining - 3) : 1);
    }

    private string WireSessionSummary()
    {
        if (sessionHeartbeats == 0 && sessionFrames == 0) return "";
        return $"HOST session: fw={lastFirmwareBuild} heartbeat={(supportsShortHeartbeat ? "compact6B/20ms" : "legacy36B/40ms")} " +
               $"framesTotal={sessionFrames} dataFrames={sessionFrames - sessionHeartbeats} heartbeats={sessionHeartbeats} " +
               $"worstHeartbeatGap={sessionWorstHeartbeatGap:F0}ms gapsOver{SlowHeartbeatLogMs:F0}ms={sessionSlowHeartbeats} " +
                $"worstUiLeaseGap={sessionWorstLeaseGap:F0}ms leaseGapsOver{SlowLeaseLogMs:F0}ms={sessionSlowLeases}" +
               (lastTelemDiag.Length > 0 ? " " + lastTelemDiag : "");
    }

    private void ResetWireSession()
    {
        sessionHeartbeats = sessionFrames = sessionSlowHeartbeats = sessionSlowLeases = 0;
        sessionWorstHeartbeatGap = sessionWorstLeaseGap = 0;
    }

    private double MsUntilHeartbeatDue()
    {
        lock (wireSync)
        {
            if (lastControlHeartbeat == 0) return 0;
            return HeartbeatIdleTarget() - Stopwatch.GetElapsedTime(lastControlHeartbeat).TotalMilliseconds;
        }
    }

    // One step: at most one complete frame.  Raw bytes (stop) go out between frames
    // only, because the firmware drops anything that lands inside a frame.
    private bool WireWriterStep(Action<byte[], int, int> write)
    {
        if (!framedControl) return false;
        byte[]? raw = null;
        lock (wireSync) { if (urgentWire.Count > 0) raw = [urgentWire.Dequeue()]; }
        if (raw is not null) { write(raw, 0, 1); return true; }
        return FlushControlTransport(write, Stopwatch.GetTimestamp(), LeasedKeys(), LeasedEpoch());
    }

    // Frame order is preserved because this is the only writer.  The heartbeat is
    // chosen before queued payload so an upload can never starve the lease.
    private bool FlushControlTransport(Action<byte[], int, int> write, long now, ushort keys, uint epoch)
    {
        byte[] packet;
        bool heartbeat;
        lock (wireSync)
        {
            if (epoch == 0) return false;
            // STOP discards the motion reference, but idle full heartbeats still
            // solicit PTP_EPOCH if its status was dropped. No command may use the
            // previous epoch, including a lease published just before a new reply.
            bool epochReady = !waitingStopEpoch && epoch == reportedEpoch;
            bool motionAllowed = Volatile.Read(ref limitsMotionAllowed) != 0;
            if (!epochReady || !motionAllowed) keys = 0;
            if (controlEpoch != epoch) {
                controlEpoch = epoch; lastControlHeartbeat = 0;
            }
            heartbeat = controlDataSent || lastControlHeartbeat == 0 || Stopwatch.GetElapsedTime(lastControlHeartbeat, now).TotalMilliseconds >= HeartbeatIdleTarget();
            if (heartbeat && supportsShortHeartbeat && epochReady) {
                // Six wire bytes instead of 36: one beat costs 5.5 ms of paced writes
                // instead of 35.5 ms.  No control sequence is consumed, so command
                // ordering is unaffected.
                packet = PtpProtocol.EncodeShortHeartbeat((byte)keys);
            } else if (heartbeat || (epochReady && pendingControlType != 0 && (motionAllowed || pendingControlType is 9 or 10 or 11))) {
                bool limits = !heartbeat && pendingControlType is 9 or 10;
                bool telemRate = !heartbeat && pendingControlType == 11;   // output only: taken whether or not motion is allowed
                if (limits && limitPacket.Length > 0) packet = limitPacket;
                else {
                    if (controlSequence == uint.MaxValue) throw new InvalidOperationException(Lang.T("err.seqExhaustedCtrl"));
                    packet = PtpProtocol.EncodePacket(heartbeat ? (byte)6 : pendingControlType, ++controlSequence,
                        controlEpoch, limits ? limitBounds : new JointPose(0, 0), heartbeat ? keys : limits ? (ushort)limitAxis : telemRate ? (ushort)telemRequestedHz : (ushort)0);
                    if (limits) { limitSequence = controlSequence; limitPacket = packet; }
                }
                if (!heartbeat) pendingControlType = 0;
            } else if (epochReady && motionAllowed && pointWire.Count > 0) {
                packet = pointWire.ToArray(); pointWire.Clear();
            } else return false;
        }
        long sendStarted = Stopwatch.GetTimestamp();
        WriteControlFrame(write, packet);
        sessionFrames++;
        if (heartbeat) {
            lock (wireSync) {
                controlDataSent = false;
                sessionHeartbeats++;
                now += Stopwatch.GetTimestamp() - sendStarted;
                double gap = lastControlHeartbeat == 0 ? 0 : Stopwatch.GetElapsedTime(lastControlHeartbeat, now).TotalMilliseconds;
                if (gap > SlowHeartbeatLogMs) { pendingHeartbeatGap = gap; sessionSlowHeartbeats++; }
                if (gap > sessionWorstHeartbeatGap) sessionWorstHeartbeatGap = gap;
                lastControlHeartbeat = now;
            }
        } else {
            lock (wireSync) { controlDataSent = !heartbeat; } // never two motion packets between heartbeats
        }
        return true;
    }

    private static void StopRecoverySelfTest()
    {
        foreach (bool epochFirst in new[] { false, true })
        {
            var window = new ControlWindow { selfTesting = true };
            var frames = new List<byte[]>();
            void Write(byte[] data, int offset, int count) {
                if (offset == 0 && (frames.Count == 0 || !ReferenceEquals(frames[^1], data))) frames.Add(data);
            }
            static byte[] Decode(byte[] frame) => Enumerable.Range(0, 17)
                .Select(i => (byte)(((frame[1 + i * 2] >> 1) & 15) | (((frame[2 + i * 2] >> 1) & 15) << 4))).ToArray();
            static void Check(bool condition, string reason) {
                if (!condition) throw new InvalidOperationException("STOP recovery: " + reason);
            }
            try {
                window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
                window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=9 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
                window.supportsShortHeartbeat = true;
                // A closed port exercises the connected STOP state without opening hardware.
                window.serial = new System.IO.Ports.SerialPort();
                window.pendingControlType = 8;
                foreach (byte b in PtpProtocol.Encode(2, 9, new JointPose(1, 2))) window.pointWire.Enqueue(b);
                window.Stop();
                Check(window.ptp is null && window.pointWire.Count == 0 && window.pendingControlType == 0 && !window.enable.IsEnabled,
                    "STOP retained motion/reference or accepted enable before stop confirmation");
                Check(!window.WireWriterStep(Write), "unconfirmed STOP renewed the motion lease");
                window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
                window.PublishHeartbeatLease();
                Check(!window.WireWriterStep(Write), "stale enabled state cancelled STOP silence");
                if (epochFirst) {
                    window.TestLine("PTP_EPOCH 10");
                    window.PublishHeartbeatLease();
                    Check(!window.enable.IsEnabled && !window.WireWriterStep(Write), "epoch alone confirmed STOP");
                }
                window.TestLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1");
                window.PublishHeartbeatLease();
                Check(window.WireWriterStep(Write), "confirmed STOP must resume idle heartbeat without PTP_STATE");
                if (!epochFirst) {
                    Check(frames[^1].Length == 36, "epoch recovery requires a full heartbeat");
                    byte[] probe = Decode(frames[^1]);
                    Check(probe[0] == 6 && probe[13] == 0 && System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(probe.AsSpan(5)) == 9,
                        "epoch recovery probe must be an idle heartbeat, never a command");
                }
                window.RequestEnable();
                Check(window.pendingControlType == 0 && !window.enable.IsEnabled, "enable bypassed fresh range confirmation");
                if (!epochFirst) {
                    window.TestLine("PTP_EPOCH 9");
                    window.ForceHeartbeatNow(); window.PublishHeartbeatLease(); window.WireWriterStep(Write);
                    Check(frames[^1].Length == 36 && Decode(frames[^1])[0] == 6 && window.pendingControlType == 0,
                        "stale epoch allowed the queued enable to leave");
                    window.TestLine("PTP_EPOCH 10");
                }
                window.PublishHeartbeatLease(); window.ForceHeartbeatNow(); window.WireWriterStep(Write);
                Check(frames[^1].Length == 6 && window.ptp is null && !window.enable.IsEnabled,
                    "heartbeat did not resume independently of motion/range confirmation");
                window.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=10 seq=1 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
                window.TestLine($"LINK fw=stage2-v7-absolute-q2 q2_absolute=1 proto={DeviceContract.ProtocolVersion} motion_rates=1 short_hb=1");
                window.RequestEnable();
                Check(window.pendingControlType == 7, "confirmed range did not permit enable");
                window.lastTelemetryAt = DateTime.UtcNow.AddSeconds(-3);
                window.CheckTelemetryTimeout(DateTime.UtcNow);
                Check(window.pendingControlType == 7, "old telemetry timestamp cancelled the enable request");
                window.TestLine("TELEM t=10 epoch=10 seq=1 ref=0 source=0 p1=0 p2=0 e1=0 err=0 inputs=0 drops=0");
                Check(window.pendingControlType == 7, "unreferenced telemetry cancelled the enable request");
                window.PublishHeartbeatLease(); window.WireWriterStep(Write);
                Check(window.WireWriterStep(Write), "enable failed after full range confirmation");
                byte[] enable = Decode(frames[^1]);
                Check(enable[0] == 7 && System.Buffers.Binary.BinaryPrimitives.ReadUInt32LittleEndian(enable.AsSpan(5)) == 10 && window.ptp is { Referenced: false },
                    "enable did not use the confirmed new epoch or restored an invalid reference");
                window.leasePublishedAt = Stopwatch.GetTimestamp() - Stopwatch.Frequency;
                Check(!window.WireWriterStep(Write), "epoch recovery bypassed the UI liveness lease");
                window.pendingControl = null; window.pendingPoint = 99;
                window.lastTelemetryAt = DateTime.UtcNow.AddSeconds(-3);
                window.CheckTelemetryTimeout(DateTime.UtcNow);
                Check(window.pendingPoint == 0 && window.stopPending, "telemetry timeout stopped protecting point motion");
            }
            finally { window.Close(); }
        }
    }

    private static void ControlTransportSelfTest(string? snapshotPath)
    {
        var window = new ControlWindow { selfTesting = true };
        var frames = new List<byte[]>();
        var bytes = new List<byte>();
        var received = new List<byte>();
        var heartbeatTimes = new List<int>();
        long start = Stopwatch.GetTimestamp();
        long previousByte = 0;
        int wireBytes = 0;
        int elapsed = 0;
        static byte Type(byte[] frame) => (byte)(((frame[1] >> 1) & 15) | (((frame[2] >> 1) & 15) << 4));
        void Write(byte[] data, int offset, int count)
        {
            long now = Stopwatch.GetTimestamp();
            if (count != 1 || offset != wireBytes / 2 ||
                (wireBytes != 0 && Stopwatch.GetElapsedTime(previousByte, now).TotalMilliseconds < 0.5))
                throw new InvalidOperationException("v7 byte pacing or frame order violated");
            previousByte = now;
            var chunk = data.AsSpan(offset, count).ToArray();
            bytes.AddRange(chunk);
            if (wireBytes++ % 2 == 0) received.AddRange(chunk);
            else if (chunk[0] != received[^1]) throw new InvalidOperationException("Byte copies differ");
            if (wireBytes == 72) {
                wireBytes = 0;
                byte[] frame = received.ToArray(); received.Clear(); frames.Add(frame);
                if (frame[0] != 0xd1 || frame[^1] != 0xd3) throw new InvalidOperationException("Interleaved control frame");
                if (Type(frame) == 6) heartbeatTimes.Add(elapsed);
            }
        }
        try {
            window.TestLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
            window.TestLine("CONTROL seq=9");
            window.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=1 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            window.TestLine($"LINK fw=stage2-v7-absolute-q2 q2_absolute=1 proto={DeviceContract.ProtocolVersion} motion_rates=1");
            if (!window.framedControl || !window.hardwareUart || window.heartbeatTimer.Interval.TotalMilliseconds != 8)
                throw new InvalidOperationException("v7 transport not selected");
            window.pendingControlType = 7;
            var point = PtpProtocol.Encode(1, 1, new JointPose(1, 2));
            foreach (byte b in point) window.pointWire.Enqueue(b);
            for (elapsed = 0; elapsed < 400; elapsed += 8)
                window.FlushControlTransport(Write, start + elapsed * Stopwatch.Frequency / 1000, 0, 1);
            if (!frames.Any(f => Type(f) == 7) || !frames.Any(f => f.SequenceEqual(point)) ||
                heartbeatTimes.Count < 4 || heartbeatTimes.Zip(heartbeatTimes.Skip(1), (a,b) => b-a).Any(gap => gap >= 150))
                throw new InvalidOperationException("Control/trajectory starvation or heartbeat deadline missed");
            if (!frames[0].SequenceEqual(PtpProtocol.EncodePacket(6, 10, 1, new JointPose(0,0), 0)))
                throw new InvalidOperationException("Reconnect did not advance control sequence");
            window.ResetControlTransport();
            window.pointWire.Clear();
            frames.Clear(); received.Clear(); bytes.Clear();
            // This exact WPF-encoded stream is replayed through the actual C parser.
            window.controlSequence = 0;
            window.pendingControlType = 7;
            for (elapsed = 0; elapsed <= 8; elapsed += 8)
                window.FlushControlTransport(Write, start + elapsed * Stopwatch.Frequency / 1000, 0, 1);
            if (frames.Count != 2 || Type(frames[0]) != 6 || Type(frames[1]) != 7)
                throw new InvalidOperationException("Enable did not follow a complete idle heartbeat");
            window.pendingControlType = 8;
            window.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=2 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            for (elapsed = 16; elapsed <= 24; elapsed += 8)
                window.FlushControlTransport(Write, start + elapsed * Stopwatch.Frequency / 1000, 0, 2);
            if (frames.Count != 4 || Type(frames[2]) != 6 || Type(frames[3]) != 8)
                throw new InvalidOperationException("Home epoch or ordering failed");
            if (snapshotPath is not null) System.IO.File.WriteAllBytes(System.IO.Path.ChangeExtension(snapshotPath, ".control.bin"), bytes.ToArray());
            window.pendingControl = null; window.pointWire.Clear(); window.UpdateControls();
            window.lastControlHeartbeat = Stopwatch.GetTimestamp();
            window.RequestEnable();
            if (window.pendingControlType != 7 || window.lastControlHeartbeat != 0)
                throw new InvalidOperationException("Enable must refresh the idle heartbeat before changing epoch");
            window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1");
            window.lastControlHeartbeat = Stopwatch.GetTimestamp();
            window.RequestHome();
            if (window.pendingControlType != 8 || window.lastControlHeartbeat != 0)
                throw new InvalidOperationException("Home must refresh the idle heartbeat before changing epoch");
            window.pendingControlType = 8;
            window.Stop();
            if (window.pendingControlType != 0 || window.pointWire.Count != 0)
                throw new InvalidOperationException("STOP retained queued motion");
            int beforeEmpty = frames.Count;
            if (!window.WireWriterStep(Write) || frames.Count != beforeEmpty + 1 || Type(frames[^1]) != 6 ||
                window.WireWriterStep(Write))
                throw new InvalidOperationException("STOP must retain paced idle heartbeats without a busy loop");
        }
        finally { window.Close(); }
    }

    // The compact heartbeat is only used when the firmware advertises it, costs 12
    // paced writes instead of 72, and must not consume a control sequence number.
    private static void ShortHeartbeatSelfTest()
    {
        var window = new ControlWindow { selfTesting = true };
        try {
            window.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            window.TestLine("PTP_STATE lock=0 ref=1 active=0 epoch=4 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
            if (window.supportsShortHeartbeat)
                throw new InvalidOperationException("Compact heartbeat selected without a capability line");
            window.TestLine($"LINK uart=hw baud=115200 rx_overflow=0 rx_error=0 tx_dropped=0 fw=stage2-v7-absolute-q2 short_hb=1 q2_absolute=1 proto={DeviceContract.ProtocolVersion}");
            if (!window.supportsShortHeartbeat)
                throw new InvalidOperationException("Compact heartbeat capability not detected");
            var wire = new List<byte>();
            int copies = 0;
            uint sequenceBefore = window.controlSequence;
            void Write(byte[] data, int offset, int count) {
                if (count != 1 || offset != copies / 2) throw new InvalidOperationException("Compact frame pacing violated");
                if (copies++ % 2 == 0) wire.Add(data[offset]);
                else if (wire[^1] != data[offset]) throw new InvalidOperationException("Compact byte copies differ");
            }
            if (!window.FlushControlTransport(Write, Stopwatch.GetTimestamp(), 5, 4))
                throw new InvalidOperationException("Compact heartbeat not produced");
            if (copies != 12 || wire.Count != 6)
                throw new InvalidOperationException($"Compact heartbeat used {copies} writes / {wire.Count} wire bytes");
            if (wire[0] != 0xd5 || wire[^1] != 0xd7) throw new InvalidOperationException("Compact frame delimiters wrong");
            byte recovered = (byte)(((wire[1] >> 1) & 15) | (((wire[2] >> 1) & 15) << 4));
            byte check = (byte)(((wire[3] >> 1) & 15) | (((wire[4] >> 1) & 15) << 4));
            if (recovered != 5 || check != PtpProtocol.Crc8(5))
                throw new InvalidOperationException("Compact heartbeat payload wrong");
            // Adjacent wire bytes must never be equal, or the receiver's duplicate-copy
            // rule would swallow a real byte.
            for (int i = 0; i + 1 < wire.Count; i++)
                if (wire[i] == wire[i + 1]) throw new InvalidOperationException("Compact frame has adjacent equal bytes");
            if (window.controlSequence != sequenceBefore)
                throw new InvalidOperationException("Compact heartbeat consumed a control sequence number");
        }
        finally { window.StopWireWriter(); window.Close(); }
    }
}
