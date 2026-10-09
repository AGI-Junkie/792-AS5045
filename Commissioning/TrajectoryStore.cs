using System.IO;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using static TwoLinkCommissioning.TrajectoryArchive;

namespace TwoLinkCommissioning;

// Host side of the device's flash archive of finished trajectories (docs/TRAJ_STORE.md, protocol types 12 and 13).

public enum StoreOp : short { Query = 1, Save = 2, Erase = 3 }

// One archive slot as the device reports it (TRAJ_SLOT line).  Crc is the CRC-32 of the node stream, the same value PlanCrc
// computes for a plan on this side, so "is this plan already on the board" is a number comparison.
public sealed record BoardSlot(int Slot, bool Valid, uint Seq, int Nodes, int DurationMs, int Q1Cd, int Q2Cd, uint Crc)
{
    private static readonly Regex Pattern = new(@"^TRAJ_SLOT slot=(\d+) valid=([01])(?: seq=(\d+) nodes=(\d+) duration_ms=(\d+) q1=(-?\d+) q2=(-?\d+) crc=([0-9A-F]{8}))?$", RegexOptions.CultureInvariant);
    public double Seconds => DurationMs / 1000.0;
    public static BoardSlot? Parse(string line)
    {
        var m = Pattern.Match(line);
        if (!m.Success || !int.TryParse(m.Groups[1].Value, out int slot) || slot is < 0 or >= DeviceContract.TrajStoreSlots) return null;
        if (m.Groups[2].Value == "0") return m.Groups[3].Success ? null : new BoardSlot(slot, false, 0, 0, 0, 0, 0, 0);
        if (!m.Groups[3].Success || !uint.TryParse(m.Groups[3].Value, out uint seq) || !int.TryParse(m.Groups[4].Value, out int nodes) ||
            !int.TryParse(m.Groups[5].Value, out int duration) || !int.TryParse(m.Groups[6].Value, out int q1) || !int.TryParse(m.Groups[7].Value, out int q2) ||
            !uint.TryParse(m.Groups[8].Value, System.Globalization.NumberStyles.HexNumber, null, out uint crc) ||
            nodes is < 2 or > DeviceContract.TrajCapacity || duration <= 0 || Math.Abs(q1) > DeviceContract.JointLimitCdMax || Math.Abs(q2) > DeviceContract.JointLimitCdMax) return null;
        return new BoardSlot(slot, true, seq, nodes, duration, q1, q2, crc);
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

    // A free slot to save into: the first empty one, else none (the operator then picks which to replace).
    public static int? FreeSlot(IReadOnlyList<BoardSlot?> slots)
    {
        for (int i = 0; i < DeviceContract.TrajStoreSlots; i++) if (slots.ElementAtOrDefault(i) is not { Valid: true }) return i;
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

    // The entry for a slot, only while the device still reports the plan it was recorded for.
    public static LedgerEntry? For(BoardSlot? slot, IEnumerable<LedgerEntry> entries) =>
        slot is { Valid: true } ? entries.FirstOrDefault(e => e.Slot == slot.Slot && e.Crc == slot.Crc) : null;

    public static void SelfTest()
    {
        static void Check(bool ok, string what) { if (!ok) throw new InvalidOperationException("Trajectory archive: " + what); }
        Check(Crc32("123456789"u8) == 0xCBF43926u, "CRC-32 of 123456789 must be CBF43926");
        Check(BoardSlot.Parse("TRAJ_SLOT slot=2 valid=0") is { Slot: 2, Valid: false }, "empty slot line");
        Check(BoardSlot.Parse("TRAJ_SLOT slot=1 valid=1 seq=3 nodes=485 duration_ms=60480 q1=-5686 q2=5871 crc=BD3EB746") is { Valid: true, Seq: 3, Nodes: 485, DurationMs: 60480, Q1Cd: -5686, Q2Cd: 5871, Crc: 0xBD3EB746 }, "valid slot line");
        foreach (string bad in new[] { "TRAJ_SLOT slot=4 valid=0", "TRAJ_SLOT slot=0 valid=1", "TRAJ_SLOT slot=0 valid=0 seq=1", "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=1 duration_ms=5 q1=0 q2=0 crc=00000000",
            "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=2049 duration_ms=5 q1=0 q2=0 crc=00000000", "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=5 q1=18001 q2=0 crc=00000000",
            "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=5 q1=0 q2=0 crc=bd3eb746", "TRAJ_SLOT slot=0 valid=1 seq=1 nodes=9 duration_ms=0 q1=0 q2=0 crc=BD3EB746" })
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
        Check(FreeSlot(slots) == 0 && FreeSlot([slots[1], slots[1], slots[1], slots[1]]) is null && FreeSlot([slots[1], slots[1], slots[2], slots[1]]) == 2, "free slot choice");
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
            () => PtpProtocol.EncodeStore(1, 7, -1, StoreOp.Save), () => PtpProtocol.EncodeStore(1, 7, -1, StoreOp.Erase), () => PtpProtocol.EncodeStore(1, 7, 0, (StoreOp)4), () => PtpProtocol.EncodeLoad(1, 7, 4), () => PtpProtocol.EncodeLoad(1, 7, -1) })
        {
            bool threw = false;
            try { refused(); } catch (ArgumentException) { threw = true; }
            Check(threw, "an invalid archive packet was encoded");
        }
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
