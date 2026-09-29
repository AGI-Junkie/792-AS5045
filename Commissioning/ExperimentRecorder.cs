using System.Globalization;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;

namespace TwoLinkCommissioning;

public sealed record TelemetrySample(uint Milliseconds, uint Epoch, uint Sequence, bool Referenced, int Source,
    int Steps1, int Steps2, int Encoder1, int Slip, int Inputs, uint Drops)
{
    private static readonly Regex Pattern = new(@"^TELEM t=(\d+) epoch=(\d+) seq=(\d+) ref=([01]) source=([012]) p1=(-?\d+) p2=(-?\d+) e1=(-?\d+) err=(-?\d+) inputs=(\d+) drops=(\d+)$", RegexOptions.CultureInvariant);
    public JointPose Command {
        get {
            double q1=Steps1*360.0/DeviceContract.Arm1StepsPerRev;
            return new(q1,q1+Steps2*360.0/DeviceContract.Arm2StepsPerRev);
        }
    }
    public double Measured1 => Encoder1 * 360.0 / (DeviceContract.EncoderCounts * DeviceContract.Arm1GearRatio);
    public double Error1 => Measured1 - Command.Q1;
    public string ReferenceName => Source == 2 ? "人工伸直零位" : Source == 1 ? "开关回零" : "未建立";
    public static TelemetrySample? Parse(string line)
    {
        var m = Pattern.Match(line);
        if (!m.Success) return null;
        var v = new long[11];
        for (int i = 0; i < v.Length; i++) if (!long.TryParse(m.Groups[i + 1].Value, out v[i])) return null;
        if (v[0] > uint.MaxValue || v[1] > uint.MaxValue || v[2] > uint.MaxValue || v[10] > uint.MaxValue || v[9] > 15 ||
            v.Skip(5).Take(4).Any(n => n < int.MinValue || n > int.MaxValue) || (v[3] == 1 && (v[1] == 0 || v[4] == 0))) return null;
        return new((uint)v[0], (uint)v[1], (uint)v[2], v[3] == 1, (int)v[4], (int)v[5], (int)v[6], (int)v[7], (int)v[8], (int)v[9], (uint)v[10]);
    }
}

// Raw session logs and experiment CSV have separate purposes. Never label model XY as measured XY.
public sealed class ExperimentRecorder : IDisposable
{
    public string DirectoryPath { get; }
    public uint Epoch { get; }
    public int Samples { get; private set; }
    public int MotionSamples { get; private set; }
    public double Rmse => MotionSamples == 0 ? 0 : Math.Sqrt(squares / MotionSamples);
    public double Peak { get; private set; }
    private double squares;
    private readonly PlanarArm model;
    private readonly StreamWriter csv;
    private readonly DateTime started = DateTime.UtcNow;
    private bool finished;
    public ExperimentRecorder(string root, string kind, uint sequence, uint epoch, PlanarArm model, string settings,
        string reference, ExecutionNode[] nodes)
    {
        this.model = model; Epoch = epoch;
        DirectoryPath = Path.Combine(root, $"{DateTime.Now:yyyyMMdd-HHmmss-fff}-{kind}-{sequence}");
        Directory.CreateDirectory(DirectoryPath);
        File.WriteAllText(Path.Combine(DirectoryPath, "metadata.json"), JsonSerializer.Serialize(new {
            schema = 1, kind, sequence, epoch, reference, started_utc = started, model.L1, model.L2, settings,
            completion_file = "summary.json; absence means recording was interrupted before finalization",
            feedback = "arm1 motor encoder through nominal 40:1 ratio; arm2 commanded only; XY model estimate; no measured pen XY",
            sampling = "approximately 2 Hz plus state transitions; not a high-rate tracking accuracy test"
        }, new JsonSerializerOptions { WriteIndented = true }), new UTF8Encoding(false));
        var plan = new StringBuilder("index,time_ms,q1_target_deg,q2_target_deg,p1_ccw_microsteps,p2_ccw_microsteps\n");
        int time = 0;
        for (int i = 0; i < nodes.Length; i++) {
            time += nodes[i].Milliseconds;
            plan.AppendLine(FormattableString.Invariant($"{i},{time},{nodes[i].Pose.Q1:F2},{nodes[i].Pose.Q2:F2},{ExecutionTrajectory.Counts(0,nodes[i].Pose)},{ExecutionTrajectory.Counts(1,nodes[i].Pose)}"));
        }
        File.WriteAllText(Path.Combine(DirectoryPath, "plan.csv"), plan.ToString(), new UTF8Encoding(true));
        csv = new StreamWriter(Path.Combine(DirectoryPath, "feedback.csv"), false, new UTF8Encoding(true)) { AutoFlush = true };
        csv.WriteLine("host_utc,device_ms,epoch,seq,phase,segment,q1_command_deg,q1_encoder_deg,q1_error_deg,q2_command_deg,q2_measured_deg,x_command_model_mm,y_command_model_mm,x_hybrid_estimate_mm,y_hybrid_estimate_mm,slip_motor_lsb,inputs_active_mask,rx_drops");
    }
    public void Record(TelemetrySample s, string phase, int segment)
    {
        if (finished || !s.Referenced || s.Epoch != Epoch) return;
        var command = model.Forward(s.Command).Tip;
        var hybrid = model.Forward(new(s.Measured1, s.Measured1 + s.Command.Q2 - s.Command.Q1)).Tip;
        csv.WriteLine(FormattableString.Invariant($"{DateTime.UtcNow:O},{s.Milliseconds},{s.Epoch},{s.Sequence},{phase},{segment},{s.Command.Q1:F6},{s.Measured1:F6},{s.Error1:F6},{s.Command.Q2:F6},,{command.X:F6},{command.Y:F6},{hybrid.X:F6},{hybrid.Y:F6},{s.Slip},{s.Inputs},{s.Drops}"));
        Samples++;
        if(phase is "point" or "drawing") { MotionSamples++; squares += s.Error1 * s.Error1; Peak = Math.Max(Peak, Math.Abs(s.Error1)); }
    }
    public void Finish(string outcome)
    {
        if (finished) return;
        finished = true; csv.Dispose();
        File.WriteAllText(Path.Combine(DirectoryPath, "summary.json"), JsonSerializer.Serialize(new {
            outcome, started_utc = started, ended_utc = DateTime.UtcNow, samples = Samples,
            motion_samples = MotionSamples,
            q1_rmse_deg = MotionSamples > 0 ? (double?)Rmse : null, q1_peak_abs_error_deg = MotionSamples > 0 ? (double?)Peak : null,
            measured_xy_error_available = false
        }, new JsonSerializerOptions { WriteIndented = true }), new UTF8Encoding(false));
    }
    public void Dispose() => Finish("interrupted");
    public static void SelfTest()
    {
        var s = TelemetrySample.Parse("TELEM t=500 epoch=9 seq=3 ref=1 source=2 p1=1600 p2=-1600 e1=4096 err=-2 inputs=0 drops=3")!;
        if (s is null || s.Command.Q1 != 9 || Math.Abs(s.Command.Q2-(9-1600*360.0/DeviceContract.Arm2StepsPerRev))>1e-8 || s.Measured1 != 9 || s.Error1 != 0 || s.ReferenceName != "人工伸直零位") throw new Exception("Telemetry units/sign invalid");
        if (TelemetrySample.Parse("TELEM t=0 epoch=0 seq=0 ref=1 source=0 p1=0 p2=0 e1=0 err=0 inputs=16 drops=0") is not null) throw new Exception("Malformed telemetry accepted");
        string root = Path.Combine(Path.GetTempPath(), "arm-recorder-" + Guid.NewGuid().ToString("N"));
        try {
            using var r = new ExperimentRecorder(root, "test", 3, 9, new PlanarArm(180,176.38), "test", "manual", [new(new(0,0),0)]);
            r.Record(s, "point", 0); r.Record(s with { Encoder1=s.Encoder1+16 },"point",0);
            r.Record(s with { Encoder1=s.Encoder1+320 },"upload",0);
            r.Record(s with { Epoch = 10 }, "point", 0); r.Record(s with { Referenced = false }, "point", 0);
            r.Finish("complete"); r.Finish("aborted");
            if (r.Samples != 3 || r.MotionSamples!=2 || Math.Abs(r.Rmse-16*360.0/(4096*40)/Math.Sqrt(2))>1e-9 ||
                File.ReadAllLines(Path.Combine(r.DirectoryPath,"feedback.csv")).Length != 4 ||
                !File.ReadAllText(Path.Combine(r.DirectoryPath,"summary.json")).Contains("complete")) throw new Exception("Experiment persistence failed");
        } finally {
            if (Directory.Exists(root) && Path.GetDirectoryName(Path.GetFullPath(root)) == Path.GetFullPath(Path.GetTempPath()).TrimEnd(Path.DirectorySeparatorChar) &&
                Path.GetFileName(root).StartsWith("arm-recorder-",StringComparison.Ordinal)) Directory.Delete(root,true);
        }
    }
}
