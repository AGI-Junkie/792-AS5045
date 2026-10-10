using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;

namespace TwoLinkCommissioning;

// Line-quality experiment for the host -> firmware UART path (docs/UART_DIAGNOSIS.md):
//
//   TwoLinkCommissioning.exe --uart-stress <COM port> <seconds per pattern> <report file> [label]
//
// It sends nothing but type-6 heartbeats with an empty key mask (no frame that can start a motor) in several byte-timing
// patterns and reports how the firmware's own error counters moved.  It refuses to run while the firmware says enabled=1.
public static class UartStress
{
    public enum Pattern { Production, BurstSingle, BurstDouble, PacedSingle1ms, PacedDouble025ms, PacedDouble1ms, CompactProduction, PacedDouble05ms }

    public sealed record Counters(long Overflow, long Error, long Dropped, long ShortOk, long DropCrc, long DropShort, long DropTimeout, long Rejected, long EpochRejected)
    {
        private static readonly Regex Link = new(@"^LINK uart=hw baud=\d+ rx_overflow=(\d+) rx_error=(\d+) tx_dropped=(\d+)", RegexOptions.CultureInvariant);
        private static readonly Regex Rx = new(@"^PTP_RX short_ok=(\d+) drop_crc=(\d+) drop_short=(\d+) drop_timeout=(\d+) rejected=(\d+) epoch_rejected=(\d+)", RegexOptions.CultureInvariant);
        public static Counters Empty { get; } = new(0, 0, 0, 0, 0, 0, 0, 0, 0);
        // Merges one firmware line into the running totals; other lines are ignored.
        public Counters With(string line)
        {
            var m = Link.Match(line);
            if (m.Success) return this with { Overflow = long.Parse(m.Groups[1].Value), Error = long.Parse(m.Groups[2].Value), Dropped = long.Parse(m.Groups[3].Value) };
            m = Rx.Match(line);
            if (m.Success) return this with { ShortOk = long.Parse(m.Groups[1].Value), DropCrc = long.Parse(m.Groups[2].Value), DropShort = long.Parse(m.Groups[3].Value),
                DropTimeout = long.Parse(m.Groups[4].Value), Rejected = long.Parse(m.Groups[5].Value), EpochRejected = long.Parse(m.Groups[6].Value) };
            return this;
        }
        public string Delta(Counters before) =>
            $"rx_error+{Error - before.Error} rx_overflow+{Overflow - before.Overflow} drop_crc+{DropCrc - before.DropCrc} " +
            $"drop_short+{DropShort - before.DropShort} drop_timeout+{DropTimeout - before.DropTimeout} tx_dropped+{Dropped - before.Dropped}";
    }

    // Sends one frame with the byte timing of `pattern`.  `write` is the port, `gap` waits the given milliseconds.
    public static void SendFrame(Pattern pattern, byte[] frame, Action<byte[], int, int> write, Action<double> gap)
    {
        switch (pattern)
        {
            case Pattern.BurstSingle: write(frame, 0, frame.Length); return;
            case Pattern.BurstDouble: write(frame.SelectMany(b => new[] { b, b }).ToArray(), 0, frame.Length * 2); return;
        }
        (int copies, double gapMs) = pattern switch
        {
            Pattern.PacedSingle1ms => (1, 1.0),
            Pattern.PacedDouble025ms => (2, 0.25),
            Pattern.PacedDouble1ms => (2, 1.0),
            Pattern.PacedDouble05ms => (2, 0.5),
            _ => (2, ControlWindow.ControlByteGapMs), // Production and CompactProduction follow the workbench.
        };
        for (int i = 0; i < frame.Length * copies; i++)
        {
            write(frame, i / copies, 1);
            if (i + 1 < frame.Length * copies) gap(gapMs);
        }
    }

    public static int Run(string[] args)
    {
        if (args.Length < 3 || !double.TryParse(args[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double seconds) || !double.IsFinite(seconds) || seconds < 1)
            return 2;
        string label = args.Length > 3 ? args[3] : "";
        var report = new StringBuilder();
        void Say(string s) { report.AppendLine(s); File.WriteAllText(args[2], report.ToString()); }
        var lines = new System.Collections.Concurrent.ConcurrentQueue<string>();
        using var port = new SerialPort(args[0], 115200, Parity.None, 8, StopBits.One) { WriteTimeout = 1000, ReadTimeout = 10, DtrEnable = false, RtsEnable = false };
        try { port.Open(); } catch (Exception ex) { Say("open failed: " + ex.Message); return 1; }
        bool stop = false;
        var reader = new Thread(() =>
        {
            var text = new StringBuilder();
            while (!Volatile.Read(ref stop))
            {
                try
                {
                    text.Append(port.ReadExisting());
                    int at;
                    while ((at = text.ToString().IndexOf('\n')) >= 0) { lines.Enqueue(text.ToString(0, at).TrimEnd('\r')); text.Remove(0, at + 1); }
                }
                catch (TimeoutException) { }
                catch (Exception) { return; }
                Thread.Sleep(2);
            }
        }) { IsBackground = true };
        reader.Start();

        var totals = Counters.Empty; uint epoch = 1, seq = 1; bool enabled = false, sawLink = false;
        void Drain()
        {
            while (lines.TryDequeue(out string? line))
            {
                totals = totals.With(line);
                if (line.StartsWith("LINK ")) sawLink = true;
                if (line.StartsWith("STATE ")) enabled = line.Contains(" enabled=1");
                var e = Regex.Match(line, @"^PTP_STATE .* epoch=(\d+)"); if (e.Success) epoch = uint.Parse(e.Groups[1].Value);
                var p = Regex.Match(line, @"^PTP_EPOCH (\d+)"); if (p.Success) epoch = uint.Parse(p.Groups[1].Value);
                var c = Regex.Match(line, @"^CONTROL seq=(\d+)"); if (c.Success) seq = Math.Max(seq, uint.Parse(c.Groups[1].Value) + 1);
            }
        }
        try
        {
            Say($"uart-stress {DateTime.Now:yyyy-MM-dd HH:mm:ss} port={args[0]} label='{label}' seconds/pattern={seconds}");
            Thread.Sleep(1600); Drain();   // one status period: learn epoch, sequence and the counters
            if (!sawLink) { Say("no LINK line from the firmware; is it powered and on this port?"); return 1; }
            if (enabled) { Say("firmware reports enabled=1; stop the machine (Space) and disable before running this test"); return 1; }
            Say($"firmware epoch={epoch}; heartbeats carry keys=0 only");
            foreach (Pattern pattern in Enum.GetValues<Pattern>())
            {
                Thread.Sleep(1200); Drain();
                var before = totals; long frames = 0, bytes = 0;
                var clock = Stopwatch.StartNew();
                while (clock.Elapsed.TotalSeconds < seconds)
                {
                    byte[] frame = pattern == Pattern.CompactProduction ? PtpProtocol.EncodeShortHeartbeat(0)
                        : PtpProtocol.EncodePacket(6, seq++, epoch, new JointPose(0, 0), 0);
                    SendFrame(pattern, frame, (b, o, n) => { port.Write(b, o, n); bytes += n; }, ms =>
                    {
                        long t0 = Stopwatch.GetTimestamp();
                        while (Stopwatch.GetElapsedTime(t0).TotalMilliseconds < ms) Thread.SpinWait(20);
                    });
                    frames++;
                    Thread.Sleep(40); Drain();
                    if (enabled) { Say("firmware became enabled; aborting"); return 1; }
                }
                Thread.Sleep(1200); Drain();
                Say($"{pattern,-18} frames={frames,5} wire_bytes={bytes,7}  {totals.Delta(before)}  (accepted heartbeats +{totals.ShortOk - before.ShortOk}+epoch_rejected +{totals.EpochRejected - before.EpochRejected})");
            }
            Say("done; cumulative since power-up: " + totals.Delta(Counters.Empty));
            return 0;
        }
        finally { Volatile.Write(ref stop, true); reader.Join(300); }
    }

    public static void SelfTest()
    {
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("UartStress: " + why); }
        foreach (string seconds in new[] { "NaN", "Infinity", "-Infinity", "1e309", "0", "0.99", "-1", "invalid" })
            Check(UartStress.Run(["invalid-port", seconds, "unused-report"]) == 2, "invalid duration reached the serial port: " + seconds);
        var t = Counters.Empty
            .With("LINK uart=hw baud=115200 rx_overflow=1 rx_error=129 tx_dropped=5 fw=x short_hb=1 proto=1")
            .With("PTP_RX short_ok=7 drop_crc=2 drop_short=3 drop_timeout=4 rejected=5 epoch_rejected=6 last_rejected_byte=0x00")
            .With("noise");
        Check(t == new Counters(1, 129, 5, 7, 2, 3, 4, 5, 6), "counters parsed wrongly");
        Check(t.Delta(Counters.Empty).StartsWith("rx_error+129 rx_overflow+1 drop_crc+2 drop_short+3 drop_timeout+4"), "delta text changed");
        var frame = new byte[] { 0xd1, 0x81, 0xa1, 0xd3 };
        (int writes, int bytes, int gaps, List<double> gapMs) Run(Pattern p, byte[] packet)
        {
            int w = 0, b = 0, g = 0; var ms = new List<double>();
            SendFrame(p, packet, (data, offset, n) => { w++; b += n; }, m => { g++; ms.Add(m); });
            return (w, b, g, ms);
        }
        var heartbeat = PtpProtocol.EncodePacket(6, 1, 1, new JointPose(0, 0), 0);
        var prod = Run(Pattern.Production, heartbeat);
        Check(prod.writes == 72 && prod.bytes == 72 && prod.gaps == 71 && prod.gapMs.All(m => m == ControlWindow.ControlByteGapMs), "production heartbeat must use the workbench's gap and count both copies");
        var compact = Run(Pattern.CompactProduction, PtpProtocol.EncodeShortHeartbeat(0));
        Check(compact.writes == 12 && compact.bytes == 12 && compact.gaps == 11 && compact.gapMs.All(m => m == ControlWindow.ControlByteGapMs), "compact heartbeat must use the workbench's gap and count both copies");
        var baseline = Run(Pattern.PacedDouble05ms, heartbeat);
        Check(baseline.writes == 72 && baseline.bytes == 72 && baseline.gaps == 71 && baseline.gapMs.All(m => m == 0.5), "historical 0.5 ms baseline changed");
        var burst = Run(Pattern.BurstSingle, frame);
        Check(burst.writes == 1 && burst.bytes == 4 && burst.gaps == 0, "burst-single must be one write");
        var burst2 = Run(Pattern.BurstDouble, frame);
        Check(burst2.writes == 1 && burst2.bytes == 8 && burst2.gaps == 0, "burst-double must be one write of doubled bytes");
        var single = Run(Pattern.PacedSingle1ms, frame);
        Check(single.writes == 4 && single.bytes == 4 && single.gaps == 3 && single.gapMs.All(m => m == 1.0), "paced-single is one copy at 1 ms");
        var doubled = new List<byte>();
        SendFrame(Pattern.BurstDouble, frame, (data, offset, n) => doubled.AddRange(data.Skip(offset).Take(n)), _ => { });
        Check(doubled.SequenceEqual(new byte[] { 0xd1, 0xd1, 0x81, 0x81, 0xa1, 0xa1, 0xd3, 0xd3 }), "burst-double bytes");
    }
}
