using System.Buffers.Binary;
using System.Text.RegularExpressions;

namespace TwoLinkCommissioning;

public sealed record PtpState(bool Locked, bool Referenced, int Active, uint Epoch, uint Sequence, JointLimits Limits)
{
    private static readonly Regex Pattern = new(@"^PTP_STATE lock=([01]) ref=([01]) active=([012]) epoch=(\d+) seq=(\d+) min1=(-?\d+) max1=(-?\d+) min2=(-?\d+) max2=(-?\d+)$", RegexOptions.CultureInvariant);
    public static PtpState? Parse(string line)
    {
        var m = Pattern.Match(line);
        if (!m.Success || !uint.TryParse(m.Groups[4].Value, out uint epoch) || !uint.TryParse(m.Groups[5].Value, out uint seq)) return null;
        var values = new int[4];
        for (int i = 0; i < 4; i++) if (!int.TryParse(m.Groups[i + 6].Value, out values[i])) return null;
        bool locked = m.Groups[1].Value == "1";
        if (!locked && (values[0] < -9000 || values[1] > 9000 || values[2] < -18000 || values[3] > 18000 ||
            values[0] >= values[1] || values[2] >= values[3] || values[0] > 0 || values[1] < 0 || values[2] > 0 || values[3] < 0)) return null;
        return new PtpState(locked, m.Groups[2].Value == "1", int.Parse(m.Groups[3].Value), epoch, seq,
            new JointLimits(values[0] / 100.0, values[1] / 100.0, values[2] / 100.0, values[3] / 100.0));
    }
}

public static class PtpProtocol
{
    public static JointPose Quantize(JointPose p) => new(Math.Round(p.Q1 * 100, MidpointRounding.AwayFromZero) / 100,
        Math.Round(p.Q2 * 100, MidpointRounding.AwayFromZero) / 100);
    public static ushort Crc(ReadOnlySpan<byte> data)
    {
        ushort crc = 0xffff;
        foreach (byte b in data)
        {
            crc ^= (ushort)(b << 8);
            for (int i = 0; i < 8; i++) crc = (ushort)((crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1);
        }
        return crc;
    }
    public static byte[] Encode(uint sequence, uint epoch, JointPose pose, ushort rate = 200)
    {
        if (sequence == 0 || epoch == 0 || !double.IsFinite(pose.Q1) || !double.IsFinite(pose.Q2) ||
            Math.Abs(pose.Q1) > 180 || Math.Abs(pose.Q2) > 180 || rate is < 100 or > 600)
            throw new ArgumentException("单点目标/序号/速度无效。");
        return EncodePacket(1, sequence, epoch, pose, rate);
    }
    public static byte[] EncodePacket(byte type, uint sequence, uint epoch, JointPose pose, ushort value)
    {
        if (type is < 1 or > 5 || sequence == 0 || epoch == 0 || !double.IsFinite(pose.Q1) ||
            !double.IsFinite(pose.Q2) || Math.Abs(pose.Q1) > 180 || Math.Abs(pose.Q2) > 180)
            throw new ArgumentException("运动数据包无效。");
        pose = Quantize(pose);
        byte[] packet = new byte[17]; packet[0] = type;
        BinaryPrimitives.WriteUInt32LittleEndian(packet.AsSpan(1), sequence);
        BinaryPrimitives.WriteUInt32LittleEndian(packet.AsSpan(5), epoch);
        BinaryPrimitives.WriteInt16LittleEndian(packet.AsSpan(9), checked((short)Math.Round(pose.Q1 * 100)));
        BinaryPrimitives.WriteInt16LittleEndian(packet.AsSpan(11), checked((short)Math.Round(pose.Q2 * 100)));
        BinaryPrimitives.WriteUInt16LittleEndian(packet.AsSpan(13), value);
        BinaryPrimitives.WriteUInt16LittleEndian(packet.AsSpan(15), Crc(packet.AsSpan(0, 15)));
        var wire = new List<byte> { 0xd1 };
        foreach (byte b in packet) { wire.Add((byte)(0x81 | ((b & 15) << 1))); wire.Add((byte)(0xa1 | ((b >> 4) << 1))); }
        wire.Add(0xd3); return wire.ToArray();
    }
    public static void SelfTest()
    {
        static void Check(bool b) { if (!b) throw new InvalidOperationException("PTP protocol check failed"); }
        Check(Crc("123456789"u8) == 0x29b1);
        var wire = Encode(1, 7, new JointPose(-1.25, 2.5));
        Check(wire.Length == 36 && wire.All(b => (b & 1) == 1));
        var decoded = new byte[17];
        for (int i = 0; i < 17; i++) decoded[i] = (byte)(((wire[1 + i * 2] >> 1) & 15) | (((wire[2 + i * 2] >> 1) & 15) << 4));
        Check(BinaryPrimitives.ReadUInt32LittleEndian(decoded.AsSpan(5)) == 7);
        Check(BinaryPrimitives.ReadInt16LittleEndian(decoded.AsSpan(9)) == -125);
        Check(BinaryPrimitives.ReadInt16LittleEndian(decoded.AsSpan(11)) == 250);
        Check(BinaryPrimitives.ReadUInt16LittleEndian(decoded.AsSpan(15)) == Crc(decoded.AsSpan(0, 15)));
        Check(PtpState.Parse("PTP_STATE lock=0 ref=1 active=0 epoch=7 seq=0 min1=-6000 max1=6000 min2=-9000 max2=9000") is { Locked: false, Referenced: true });
        Check(PtpState.Parse("PTP_STATE lock=0 ref=1 active=0 epoch=7 seq=0 min1=0 max1=0 min2=0 max2=0") is null);
        Check(PtpState.Parse("PTP_STATE lock=1 ref=0 active=0 epoch=999999999999999 seq=0 min1=0 max1=0 min2=0 max2=0") is null);
    }
}
