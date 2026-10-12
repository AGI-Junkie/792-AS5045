using System.IO;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using static TwoLinkCommissioning.TrajectoryArchive;

namespace TwoLinkCommissioning;

// Host side of the device's flash archive of finished trajectories (docs/TRAJ_STORE.md, protocol types 12 and 13).

public enum StoreOp : short { Query = 1, Save = 2, Erase = 3, Meta = 4 }   // Meta: read back one slot's description

// One archive slot as the device reports it (TRAJ_SLOT line).  Crc is the CRC-32 of the node stream, the same value PlanCrc
// computes for a plan on this side, so "is this plan already on the board" is a number comparison.  MetaLen is the size of the
// description stored with the plan (0 = none; see BoardMeta).
public sealed record BoardSlot(int Slot, bool Valid, uint Seq, int Nodes, int DurationMs, int Q1Cd, int Q2Cd, uint Crc, int MetaLen = 0)
{
    private static readonly Regex Pattern = new(@"^TRAJ_SLOT slot=(\d+) valid=([01])(?: seq=(\d+) nodes=(\d+) duration_ms=(\d+) q1=(-?\d+) q2=(-?\d+) crc=([0-9A-F]{8})(?: meta=(\d+))?)?$", RegexOptions.CultureInvariant);
    public double Seconds => DurationMs / 1000.0;
    public static BoardSlot? Parse(string line)
    {
        var m = Pattern.Match(line);
        if (!m.Success || !int.TryParse(m.Groups[1].Value, out int slot) || slot is < 0 or >= DeviceContract.TrajStoreSlots) return null;
        if (m.Groups[2].Value == "0") return m.Groups[3].Success ? null : new BoardSlot(slot, false, 0, 0, 0, 0, 0, 0);
        if (!m.Groups[3].Success || !uint.TryParse(m.Groups[3].Value, out uint seq) || !int.TryParse(m.Groups[4].Value, out int nodes) ||
            !int.TryParse(m.Groups[5].Value, out int duration) || !int.TryParse(m.Groups[6].Value, out int q1) || !int.TryParse(m.Groups[7].Value, out int q2) ||
            !uint.TryParse(m.Groups[8].Value, System.Globalization.NumberStyles.HexNumber, null, out uint crc) ||
            nodes is < 2 or > DeviceContract.TrajCapacity || duration <= 0 ||
            !int.TryParse(m.Groups[9].Success ? m.Groups[9].Value : "0", out int metaLen) || metaLen is < 0 or > BoardMeta.MaxBytes ||
            q1 is < -DeviceContract.JointLimitCdMax or > DeviceContract.JointLimitCdMax || q2 is < -DeviceContract.JointLimitCdMax or > DeviceContract.JointLimitCdMax) return null;   // not Math.Abs: it throws for int.MinValue
        return new BoardSlot(slot, true, seq, nodes, duration, q1, q2, crc, metaLen);
    }
}

// The description a computer keeps with a plan on the board: a short text of the settings that made it (the same "key=value;"
// text the experiment archive records), so another computer can name the figure and restore its settings.  The device stores the
// bytes and gives them back without reading them (docs/TRAJ_STORE.md); it is sent four bytes per frame, type 14.
public sealed record BoardMeta(int Slot, uint Seq, string Text)
{
    public const int MaxBytes = 256;                    // TS_META_MAX; the device advertises the value it really has (traj_meta=<n>)
    private static readonly Regex Pattern = new(@"^TRAJ_META slot=(\d+) seq=(\d+) len=(\d+) crc=([0-9A-F]{8}) hex=([0-9A-F]+)$", RegexOptions.CultureInvariant);
    // The answer to a type 12 op 4 query.  A damaged line (length or CRC off, a byte that is not printable ASCII) gives null.
    public static BoardMeta? Parse(string line)
    {
        var m = Pattern.Match(line);
        if (!m.Success || !int.TryParse(m.Groups[1].Value, out int slot) || slot is < 0 or >= DeviceContract.TrajStoreSlots ||
            !uint.TryParse(m.Groups[2].Value, out uint seq) || !int.TryParse(m.Groups[3].Value, out int len) || len is < 1 or > MaxBytes ||
            !uint.TryParse(m.Groups[4].Value, System.Globalization.NumberStyles.HexNumber, null, out uint crc) || m.Groups[5].Value.Length != len * 2) return null;
        byte[] bytes = Convert.FromHexString(m.Groups[5].Value);
        if (TrajectoryArchive.Crc32(bytes) != crc || bytes.Any(b => b is < 0x20 or > 0x7E)) return null;
        return new BoardMeta(slot, seq, Encoding.ASCII.GetString(bytes));
    }

    // What of a figure's settings is worth keeping on the board: the figure, its transform and speeds (what the recent runs
    // restore), and the link lengths (so a different machine setup can be told).  Null when it does not fit or is not plain text.
    private static readonly string[] Keys = ["shape", "branch", "x", "y", "a", "b", "c", "rot", "scale", "dir", "v1", "v2", "a1", "a2", "L1", "L2"];
    public static string? TextFor(string settings, int max = MaxBytes)
    {
        var kept = settings.Split(';', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries)
            .Where(part => part.IndexOf('=') is int eq and > 0 && Keys.Contains(part[..eq].Trim()));
        string text = string.Join(";", kept);
        return text.Length is > 0 && text.Length <= max && text.All(c => c is >= ' ' and <= '~') ? text : null;
    }

    // The frames' data: the stream the device checks (text length, CRC-32 of the text, CRC-32 of the plan's node stream, the text),
    // cut into four-byte chunks; the chunk number travels in the frame.
    public static byte[][] Chunks(string text, uint planCrc)
    {
        byte[] body = Encoding.ASCII.GetBytes(text);
        var stream = new byte[(12 + body.Length + 3) / 4 * 4];
        System.Buffers.Binary.BinaryPrimitives.WriteUInt16LittleEndian(stream, (ushort)body.Length);
        System.Buffers.Binary.BinaryPrimitives.WriteUInt32LittleEndian(stream.AsSpan(4), TrajectoryArchive.Crc32(body));
        System.Buffers.Binary.BinaryPrimitives.WriteUInt32LittleEndian(stream.AsSpan(8), planCrc);
        body.CopyTo(stream, 12);
        return Enumerable.Range(0, stream.Length / 4).Select(i => stream[(i * 4)..(i * 4 + 4)]).ToArray();
    }
}

// The device's account of the archive job: the receipt of a type 12 command and its progress and end.
public sealed record StoreStatus(string State, int Slot, int RowsDone, int RowsTotal, uint Seq, string Last, int LastSlot, int RowMaxUs)
{
    private static readonly Regex Pattern = new(@"^TRAJ_STORE state=(idle|saving|erasing) slot=(\d+) rows=(\d+)/(\d+) seq=(\d+) last=(none|saved|erased|failed|aborted) last_slot=(\d+) row_max_us=(\d+)$", RegexOptions.CultureInvariant);
    public bool Busy => State != "idle";
    public static StoreStatus? Parse(string line)
    {
        var m = Pattern.Match(line);
        if (!m.Success || !int.TryParse(m.Groups[2].Value, out int slot) || !int.TryParse(m.Groups[3].Value, out int done) || !int.TryParse(m.Groups[4].Value, out int total) ||
            !uint.TryParse(m.Groups[5].Value, out uint seq) || !int.TryParse(m.Groups[7].Value, out int lastSlot) || !int.TryParse(m.Groups[8].Value, out int rowMax) ||
            slot >= DeviceContract.TrajStoreSlots || lastSlot >= DeviceContract.TrajStoreSlots || done > total) return null;
        return new StoreStatus(m.Groups[1].Value, slot, done, total, seq, m.Groups[6].Value, lastSlot, rowMax);
    }
}

public static class TrajectoryArchive
{
    // CRC-32/IEEE (the polynomial of zip and PNG); the firmware's ts_crc32 produces the same values.
    public static uint Crc32(ReadOnlySpan<byte> data)
    {
        uint crc = 0xFFFFFFFFu;
        foreach (byte b in data)
        {
            crc ^= b;
            for (int i = 0; i < 8; i++) crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
        }
        return ~crc;
    }

    // The node stream as the device stores it: per node q1 and q2 in 0.01 degree (int16) and the segment time in ms (uint16), little endian.
    public static byte[] Stream(IReadOnlyList<ExecutionNode> nodes)
    {
        var bytes = new byte[nodes.Count * 6];
        for (int i = 0; i < nodes.Count; i++)
        {
            var span = bytes.AsSpan(i * 6);
            System.Buffers.Binary.BinaryPrimitives.WriteInt16LittleEndian(span, checked((short)Math.Round(nodes[i].Pose.Q1 * 100)));
            System.Buffers.Binary.BinaryPrimitives.WriteInt16LittleEndian(span[2..], checked((short)Math.Round(nodes[i].Pose.Q2 * 100)));
            System.Buffers.Binary.BinaryPrimitives.WriteUInt16LittleEndian(span[4..], nodes[i].Milliseconds);
        }
        return bytes;
    }
    public static uint PlanCrc(IReadOnlyList<ExecutionNode> nodes) => Crc32(Stream(nodes));

    // The slot holding exactly this plan (same node bytes, so the same drawing at the same speeds), if any.
    public static BoardSlot? SlotFor(IReadOnlyList<BoardSlot?> slots, IReadOnlyList<ExecutionNode> nodes)
    {
        uint crc = PlanCrc(nodes);
        int total = nodes.Sum(n => (int)n.Milliseconds);
        return slots.FirstOrDefault(s => s is { Valid: true } && s.Crc == crc && s.Nodes == nodes.Count && s.DurationMs == total);
    }

    // A free slot to save into: the first one the device reported as empty, else none (the operator then picks which to replace).
    // A slot nothing has been heard about is not free: only `count` slots exist, and an unread one may hold a figure.
    public static int? FreeSlot(IReadOnlyList<BoardSlot?> slots, int count = DeviceContract.TrajStoreSlots)
    {
        for (int i = 0; i < Math.Min(count, DeviceContract.TrajStoreSlots); i++) if (slots.ElementAtOrDefault(i) is { Valid: false }) return i;
        return null;
    }
}

// What this computer knows about the figures it put on the board: which settings produced the plan in a slot.  The device only
// keeps nodes, so the picture and its name come from here; a slot saved from another computer is listed as unknown.
public sealed record LedgerEntry(int Slot, uint Crc, string Settings);

public static class BoardLedger
{
    internal static string? PathOverride;   // tests keep the real file untouched
    private static string FilePath => PathOverride ?? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TwoLinkCommissioning", "board-slots.json");

    public static List<LedgerEntry> Load()
    {
        try
        {
            if (!File.Exists(FilePath) || new FileInfo(FilePath).Length > 65_536) return [];
            var entries = JsonSerializer.Deserialize<List<LedgerEntry>>(File.ReadAllText(FilePath)) ?? [];
            return entries.Where(e => e is { Settings: not null } && e.Slot is >= 0 and < DeviceContract.TrajStoreSlots && e.Settings.Length <= 1024).GroupBy(e => e.Slot).Select(g => g.Last()).ToList();
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException) { return []; }
    }

    // Replaces (or, with a null settings text, forgets) one slot's entry.
    public static void Set(int slot, uint crc, string? settings)
    {
        var entries = Load().Where(e => e.Slot != slot).ToList();
        if (settings is not null) entries.Add(new LedgerEntry(slot, crc, settings));
        Directory.CreateDirectory(Path.GetDirectoryName(FilePath)!);
        File.WriteAllText(FilePath + ".tmp", JsonSerializer.Serialize(entries.OrderBy(e => e.Slot)), new UTF8Encoding(false));
        File.Move(FilePath + ".tmp", FilePath, true);
    }

    // What is known about each slot: the description stored on the board wins (it belongs to the plan in that slot by
    // construction); the file on this computer fills in the slots without one (saved by older firmware).
    public static List<LedgerEntry> Merge(IReadOnlyList<BoardSlot?> slots, IReadOnlyList<BoardMeta?> metas, IEnumerable<LedgerEntry> local)
    {
        var merged = new List<LedgerEntry>();
        for (int i = 0; i < slots.Count && i < metas.Count; i++)
            if (slots[i] is { Valid: true } slot && metas[i] is { } meta && meta.Seq == slot.Seq) merged.Add(new LedgerEntry(i, slot.Crc, meta.Text));
        merged.AddRange(local.Where(e => merged.All(m => m.Slot != e.Slot)));
        return merged;
    }

    // The entry for a slot, only while the device still reports the plan it was recorded for.
    public static LedgerEntry? For(BoardSlot? slot, IEnumerable<LedgerEntry> entries) =>
        slot is { Valid: true } ? entries.FirstOrDefault(e => e.Slot == slot.Slot && e.Crc == slot.Crc) : null;

    public static void SelfTest()
    {
        static void Check(bool ok, string what) { if (!ok) throw new InvalidOperationException("Trajectory archive: " + what); }
        Check(Crc32("123456789"u8) == 0xCBF43926u, "CRC-32 of 123456789 must be CBF43926");
        Check(BoardSlot.Parse("TRAJ_SLOT slot=2 valid=0") is { Slot: 2, Valid: false }, "empty slot line");
        Check(BoardSlot.Parse("TRAJ_SLOT slot=1 valid=1 seq=3 nodes=485 duration_ms=60480 q1=-5686 q2=5871 crc=BD3EB746") is { Valid: true, Seq: 3, Nodes: 485, DurationMs: 60480, Q1Cd: -5686, Q2Cd: 5871, Crc: 0xBD3EB746 }, "valid slot line");
        Check(BoardSlot.Parse("TRAJ_SLOT slot=1 valid=1 seq=3 nodes=485 duration_ms=60480 q1=-5686 q2=5871 crc=BD3EB746 meta=105") is { MetaLen: 105, Crc: 0xBD3EB746 } &&
              BoardSlot.Parse("TRAJ_SLOT slot=1 valid=1 seq=3 nodes=485 duration_ms=60480 q1=-5686 q2=5871 crc=BD3EB746") is { MetaLen: 0 }, "slot line with a description size");
        foreach (string bad in new[] { "TRAJ_SLOT slot=1 valid=1 seq=3 nodes=485 duration_ms=60480 q1=0 q2=0 crc=BD3EB746 meta=257", "TRAJ_SLOT slot=1 valid=1 seq=3 nodes=485 duration_ms=60480 q1=0 q2=0 crc=BD3EB746 meta=",
            "TRAJ_SLOT slot=1 valid=0 meta=5", "TRAJ_SLOT slot=4 valid=0", "TRAJ_SLOT slot=0 valid=1", "TRAJ_SLOT slot=0 valid=0 seq=1", "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=1 duration_ms=5 q1=0 q2=0 crc=00000000",
            "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=2049 duration_ms=5 q1=0 q2=0 crc=00000000", "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=5 q1=18001 q2=0 crc=00000000",
            "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=5 q1=0 q2=0 crc=bd3eb746", "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=0 q1=0 q2=0 crc=BD3EB746",
            "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=5 q1=-2147483648 q2=0 crc=BD3EB746", "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=5 q1=0 q2=-2147483648 crc=BD3EB746" })
            Check(BoardSlot.Parse(bad) is null, "bad slot line accepted: " + bad);
        Check(StoreStatus.Parse("TRAJ_STORE state=saving slot=2 rows=8/12 seq=74 last=failed last_slot=0 row_max_us=7000") is { Busy: true, Slot: 2, RowsDone: 8, RowsTotal: 12, Seq: 74, Last: "failed", RowMaxUs: 7000 }, "status line");
        Check(StoreStatus.Parse("TRAJ_STORE state=idle slot=0 rows=0/0 seq=1 last=none last_slot=0 row_max_us=0") is { Busy: false }, "idle status line");
        Check(StoreStatus.Parse("TRAJ_STORE state=idle slot=0 rows=9/3 seq=1 last=saved last_slot=0 row_max_us=0") is null && StoreStatus.Parse("TRAJ_STORE state=idle slot=7 rows=0/0 seq=1 last=saved last_slot=0 row_max_us=0") is null &&
              StoreStatus.Parse("TRAJ_STORE state=busy slot=0 rows=0/0 seq=1 last=saved last_slot=0 row_max_us=0") is null && StoreStatus.Parse("TRAJ_STORE refused: stage not idle") is null, "bad status line accepted");
        // The stream is the device's node format: 6 bytes per node, little endian, node 0 first.
        var nodes = new[] { new ExecutionNode(new JointPose(-1.25, 2.5), 0), new ExecutionNode(new JointPose(0.01, -180), 500) };
        var stream = Stream(nodes);
        Check(stream.Length == 12 && stream.SequenceEqual(new byte[] { 0x83, 0xFF, 0xFA, 0x00, 0, 0, 0x01, 0x00, 0xB0, 0xB9, 0xF4, 0x01 }), "node stream layout: " + Convert.ToHexString(stream));
        Check(PlanCrc(nodes) == Crc32(stream), "plan CRC");
        var slots = new BoardSlot?[] { null, new BoardSlot(1, true, 1, 2, 500, -125, 250, PlanCrc(nodes)), new BoardSlot(2, false, 0, 0, 0, 0, 0, 0), null };
        // slots: unknown, held, reported empty, unknown.  Only a reported-empty slot is free, and only within the advertised count.
        Check(FreeSlot(slots) == 2 && FreeSlot([slots[1], slots[1], slots[1], slots[1]]) is null && FreeSlot([slots[1], slots[1], slots[2], slots[1]]) == 2 && FreeSlot([null, null, null, null]) is null, "free slot choice");
        Check(FreeSlot([slots[1], slots[1], slots[2], slots[2]], 2) is null && FreeSlot([slots[1], slots[2], slots[2], slots[2]], 2) == 1 && FreeSlot([slots[2], slots[2], slots[2], slots[2]], 0) is null, "free slot choice must honour the slot count");
        Check(SlotFor(slots, nodes) is { Slot: 1 } && SlotFor(slots, [nodes[0], nodes[1] with { Milliseconds = 400 }]) is null, "slot lookup by plan");
        // Packets: type 12 query-all, save, erase and type 13 load; refused when out of range.
        static byte[] Decode(byte[] wire) => Enumerable.Range(0, 17).Select(i => (byte)(((wire[1 + i * 2] >> 1) & 15) | (((wire[2 + i * 2] >> 1) & 15) << 4))).ToArray();
        var query = Decode(PtpProtocol.EncodeStore(5, 7, -1, StoreOp.Query));
        Check(query[0] == 12 && BitConverter.ToInt16(query, 9) == -1 && BitConverter.ToInt16(query, 11) == 1 && BitConverter.ToUInt16(query, 13) == 0 && BitConverter.ToUInt16(query, 15) == PtpProtocol.Crc(query.AsSpan(0, 15)), "query packet");
        var save = Decode(PtpProtocol.EncodeStore(6, 7, 3, StoreOp.Save));
        Check(save[0] == 12 && BitConverter.ToInt16(save, 9) == 3 && BitConverter.ToInt16(save, 11) == 2 && BitConverter.ToUInt32(save, 1) == 6 && BitConverter.ToUInt32(save, 5) == 7, "save packet");
        var load = Decode(PtpProtocol.EncodeLoad(9, 7, 1));
        Check(load[0] == 13 && BitConverter.ToInt16(load, 9) == 1 && BitConverter.ToInt16(load, 11) == 0 && BitConverter.ToUInt16(load, 13) == 0, "load packet");
        foreach (var refused in new Action[] { () => PtpProtocol.EncodeStore(0, 7, 0, StoreOp.Save), () => PtpProtocol.EncodeStore(1, 0, 0, StoreOp.Save), () => PtpProtocol.EncodeStore(1, 7, 4, StoreOp.Save),
            () => PtpProtocol.EncodeStore(1, 7, -1, StoreOp.Save), () => PtpProtocol.EncodeStore(1, 7, -1, StoreOp.Erase), () => PtpProtocol.EncodeStore(1, 7, 0, (StoreOp)5), () => PtpProtocol.EncodeStore(1, 7, -1, StoreOp.Meta), () => PtpProtocol.EncodeStore(1, 7, 4, StoreOp.Meta),
            () => PtpProtocol.EncodeMeta(1, 7, 0, new byte[3]), () => PtpProtocol.EncodeMeta(1, 7, 67, new byte[4]), () => PtpProtocol.EncodeMeta(1, 7, -1, new byte[4]), () => PtpProtocol.EncodeMeta(0, 7, 0, new byte[4]), () => PtpProtocol.EncodeLoad(1, 7, 4), () => PtpProtocol.EncodeLoad(1, 7, -1) })
        {
            bool threw = false;
            try { refused(); } catch (ArgumentException) { threw = true; }
            Check(threw, "an invalid archive packet was encoded");
        }
        // The description kept with a plan: only the keys that restore a figure (and the link lengths), as plain text that fits.
        const string settings = "shape=3; branch=0; L1=181.5; L2=176.38; x=340; y=0; a=10; b=6; feed=1; v1=6.5; v2=3; a1=10; a2=6; min1=-1; max1=1; emin=-1; emax=1; c=; rot=20; scale=1.2; dir=cw; custom_chars=0";
        string text = BoardMeta.TextFor(settings) ?? "";
        Check(text == "shape=3;branch=0;L1=181.5;L2=176.38;x=340;y=0;a=10;b=6;v1=6.5;v2=3;a1=10;a2=6;c=;rot=20;scale=1.2;dir=cw", "description text: " + text);
        Check(BoardMeta.TextFor("feed=1; min1=2") is null && BoardMeta.TextFor("shape=\u00e9") is null && BoardMeta.TextFor("x=" + new string('9', 300)) is null && BoardMeta.TextFor(settings, 20) is null, "descriptions that must not be sent");
        // Cut into four-byte chunks: text length, 0, CRC-32 of the text, CRC-32 of the plan, the text; the chunk number rides in the frame.
        var chunks = BoardMeta.Chunks(text, 0x12345678);
        var body = chunks.SelectMany(c => c).ToArray();
        Check(chunks.Length == (12 + text.Length + 3) / 4 && chunks.All(c => c.Length == 4) && BitConverter.ToUInt16(body, 0) == text.Length && BitConverter.ToUInt16(body, 2) == 0 &&
              BitConverter.ToUInt32(body, 4) == Crc32(Encoding.ASCII.GetBytes(text)) && BitConverter.ToUInt32(body, 8) == 0x12345678 && Encoding.ASCII.GetString(body, 12, text.Length) == text, "description stream");
        var chunkFrame = Decode(PtpProtocol.EncodeMeta(8, 7, 2, chunks[2]));
        Check(chunkFrame[0] == 14 && BitConverter.ToUInt32(chunkFrame, 1) == 8 && BitConverter.ToUInt32(chunkFrame, 5) == 7 && chunkFrame[9..13].SequenceEqual(chunks[2]) && BitConverter.ToUInt16(chunkFrame, 13) == 2 &&
              BitConverter.ToUInt16(chunkFrame, 15) == PtpProtocol.Crc(chunkFrame.AsSpan(0, 15)), "description chunk frame");
        var op4 = Decode(PtpProtocol.EncodeStore(9, 7, 2, StoreOp.Meta));
        Check(op4[0] == 12 && BitConverter.ToInt16(op4, 9) == 2 && BitConverter.ToInt16(op4, 11) == 4, "description query frame");
        // What the device answers (type 12 op 4): one hex line, checked for length, CRC and plain text.
        string Answer(string t, int slot = 1, int? len = null, uint? crc = null)
        {
            byte[] b = Encoding.ASCII.GetBytes(t);
            return $"TRAJ_META slot={slot} seq=3 len={len ?? b.Length} crc={crc ?? Crc32(b):X8} hex={Convert.ToHexString(b)}";
        }
        Check(BoardMeta.Parse(Answer(text)) is { Slot: 1, Seq: 3 } meta && meta.Text == text, "description answer");
        foreach (string bad in new[] { Answer(text, len: text.Length + 1), Answer(text, crc: 5), Answer("a\u0001b"), Answer(text, slot: 4), Answer(text).ToLowerInvariant(), "TRAJ_META slot=1 len=0", Answer(new string('A', BoardMeta.MaxBytes + 1)) })
            Check(BoardMeta.Parse(bad) is null, "bad description answer accepted: " + bad);
        // What is known about a slot: the board's own description (for the plan it holds), else the file of this computer.
        var held = new BoardSlot(1, true, 3, 2, 500, 0, 0, 0xBD3EB746);
        var local = new[] { new LedgerEntry(1, 0xBD3EB746, "shape=1"), new LedgerEntry(2, 7, "shape=2") };
        var merged = BoardLedger.Merge([null, held, null, null], [null, new BoardMeta(1, 3, text), null, null], local);
        Check(merged.Count == 2 && merged.Single(e => e.Slot == 1).Settings == text && merged.Single(e => e.Slot == 1).Crc == 0xBD3EB746 && merged.Single(e => e.Slot == 2).Settings == "shape=2", "board description beats the local record");
        merged = BoardLedger.Merge([null, held, null, null], [null, new BoardMeta(1, 2, text), null, null], local);
        Check(merged.Count == 2 && merged.Single(e => e.Slot == 1).Settings == "shape=1", "a description of an older save must not be used");
        // The ledger: recorded per slot, only trusted while the device still reports the same plan, ignored when damaged.
        string file = Path.Combine(Path.GetTempPath(), "arm-ledger-" + Guid.NewGuid().ToString("N") + ".json");
        PathOverride = file;
        try
        {
            Check(Load().Count == 0, "no ledger yet");
            Set(1, 0xBD3EB746, "shape=2; x=340"); Set(3, 5, "shape=3"); Set(1, 0xBD3EB746, "shape=2; x=341");
            var read = Load();
            Check(read.Count == 2 && read.First(e => e.Slot == 1).Settings == "shape=2; x=341", "ledger round trip");
            Check(For(slots[1], read) is null && For(new BoardSlot(1, true, 1, 2, 500, 0, 0, 0xBD3EB746), read) is { Slot: 1 } && For(new BoardSlot(1, true, 1, 2, 500, 0, 0, 1), read) is null && For(null, read) is null, "ledger lookup");
            Set(1, 0, null);
            Check(Load().Count == 1, "forgetting a slot");
            File.WriteAllText(file, "not json");
            Check(Load().Count == 0, "damaged ledger must be ignored");
        }
        finally { PathOverride = null; if (File.Exists(file)) File.Delete(file); }
    }
}
