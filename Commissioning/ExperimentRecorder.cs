using System.Globalization;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;

namespace TwoLinkCommissioning;

public sealed record TelemetrySample(uint Milliseconds, uint Epoch, uint Sequence, bool Referenced, int Source,
    int Steps1, int Steps2, int Encoder1, int Slip, int Inputs, uint Drops, int? Encoder2 = null, int? Slip2 = null)
{
    private static readonly Regex Pattern = new(@"^TELEM t=(\d+) epoch=(\d+) seq=(\d+) ref=([01]) source=([012]) p1=(-?\d+) p2=(-?\d+) e1=(-?\d+) err=(-?\d+)(?: e2=(-?\d+) err2=(-?\d+))? inputs=(\d+) drops=(\d+)$", RegexOptions.CultureInvariant);
    public JointPose Command {
        get {
            double q1=Steps1*360.0/DeviceContract.Arm1StepsPerRev;
            return new(q1,q1+Steps2*360.0/DeviceContract.Arm2StepsPerRev);
        }
    }
    public double Measured1 => Encoder1 * 360.0 / (DeviceContract.EncoderCounts * DeviceContract.Arm1GearRatio);
    public double? MeasuredElbow => Encoder2 * 360.0 / (DeviceContract.EncoderCounts * (DeviceContract.Arm2GearRatioX100 / 100.0));
    public double? Measured2 => MeasuredElbow + Measured1;
    public double? Error2 => Measured2 - Command.Q2;
    public double Error1 => Measured1 - Command.Q1;
    // arm2's own following error: elbow encoder minus commanded elbow (q2-q1); unlike Error2 it excludes arm1's error.
    public double? ElbowError => MeasuredElbow - (Command.Q2 - Command.Q1);
    public string ReferenceName => Source == 2 ? Lang.T("ref.manual") : Source == 1 ? Lang.T("ref.home") : Lang.T("ref.none");
    public static TelemetrySample? Parse(string line)
    {
        var m = Pattern.Match(line);
        if (!m.Success) return null;
        var v = new long[13];
        for (int i = 0; i < v.Length; i++) {
            if (i is 9 or 10 && !m.Groups[i + 1].Success) continue;
            if (!long.TryParse(m.Groups[i + 1].Value, out v[i])) return null;
        }
        if (v[0] > uint.MaxValue || v[1] > uint.MaxValue || v[2] > uint.MaxValue || v[12] > uint.MaxValue || v[11] > 15 ||
            v.Skip(5).Take(6).Any(n => n < int.MinValue || n > int.MaxValue) || (v[3] == 1 && (v[1] == 0 || v[4] == 0))) return null;
        return new((uint)v[0], (uint)v[1], (uint)v[2], v[3] == 1, (int)v[4], (int)v[5], (int)v[6], (int)v[7], (int)v[8], (int)v[11], (uint)v[12],
            m.Groups[10].Success ? (int)v[9] : null, m.Groups[11].Success ? (int)v[10] : null);
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
    private double squares2;
    public int MotionSamples2 { get; private set; }
    public double Rmse2 => MotionSamples2 > 0 ? Math.Sqrt(squares2 / MotionSamples2) : 0;
    public double Peak2 { get; private set; }
    private double squaresElbow;
    public double RmseElbow => MotionSamples2 > 0 ? Math.Sqrt(squaresElbow / MotionSamples2) : 0;
    public double PeakElbow { get; private set; }
    private double squares;
    private readonly PlanarArm model;
    private readonly StreamWriter csv;
    private readonly DateTime started = DateTime.UtcNow;
    private bool finished;
    public ExperimentRecorder(string root, string kind, uint sequence, uint epoch, PlanarArm model, string settings,
        string reference, ExecutionNode[] nodes, int telemetryHz = 0, string? calibrationId = null, bool arm2Feedback = true)
    {
        this.model = model; Epoch = epoch;
        DirectoryPath = Path.Combine(root, $"{DateTime.Now:yyyyMMdd-HHmmss-fff}-{kind}-{sequence}");
        Directory.CreateDirectory(DirectoryPath);
        File.WriteAllText(Path.Combine(DirectoryPath, "metadata.json"), JsonSerializer.Serialize(new {
            schema = 2, kind, sequence, epoch, reference, started_utc = started, model.L1, model.L2, settings,
            completion_file = "summary.json; absence means recording was interrupted before finalization",
            feedback = arm2Feedback ? "both motor encoders: arm1 nominal 40:1, arm2 nominal 51.06:1; q2=q1+measured elbow; XY kinematic estimate; no measured pen XY" : "arm1 motor encoder; arm2 commanded steps without feedback; hybrid XY model estimate; no measured pen XY",
            telemetry_hz = telemetryHz,
            calibration_id = calibrationId,   // the calibration that was current when this ran (null = none saved)
            sampling = telemetryHz > 0 ? $"streamed telemetry at {telemetryHz} Hz (firmware skips a line rather than crowd its transmit queue; see TELEM_DIAG skipped= in the log)"
                                       : "approximately 2 Hz plus state transitions; not a high-rate tracking accuracy test"
        }, new JsonSerializerOptions { WriteIndented = true }), new UTF8Encoding(false));
        var plan = new StringBuilder("index,time_ms,q1_target_deg,q2_target_deg,p1_ccw_microsteps,p2_ccw_microsteps\n");
        int time = 0;
        for (int i = 0; i < nodes.Length; i++) {
            time += nodes[i].Milliseconds;
            plan.AppendLine(FormattableString.Invariant($"{i},{time},{nodes[i].Pose.Q1:F2},{nodes[i].Pose.Q2:F2},{ExecutionTrajectory.Counts(0,nodes[i].Pose)},{ExecutionTrajectory.Counts(1,nodes[i].Pose)}"));
        }
        File.WriteAllText(Path.Combine(DirectoryPath, "plan.csv"), plan.ToString(), new UTF8Encoding(true));
        csv = new StreamWriter(Path.Combine(DirectoryPath, "feedback.csv"), false, new UTF8Encoding(true)) { AutoFlush = true };
        csv.WriteLine("host_utc,device_ms,epoch,seq,phase,segment,q1_command_deg,q1_encoder_deg,q1_error_deg,q2_command_deg,q2_measured_deg,x_command_model_mm,y_command_model_mm,x_hybrid_estimate_mm,y_hybrid_estimate_mm,slip_motor_lsb,inputs_active_mask,rx_drops,q2_error_deg,elbow_encoder_deg,slip2_motor_lsb,elbow_error_deg");
    }
    public void Record(TelemetrySample s, string phase, int segment)
    {
        if (finished || !s.Referenced || s.Epoch != Epoch) return;
        var command = model.Forward(s.Command).Tip;
        double? q2Measured = s.Measured2, error2 = s.Error2;
        var hybrid = model.Forward(new(s.Measured1, q2Measured ?? s.Measured1 + s.Command.Q2 - s.Command.Q1)).Tip;
        csv.WriteLine(FormattableString.Invariant($"{DateTime.UtcNow:O},{s.Milliseconds},{s.Epoch},{s.Sequence},{phase},{segment},{s.Command.Q1:F6},{s.Measured1:F6},{s.Error1:F6},{s.Command.Q2:F6},{q2Measured:F6},{command.X:F6},{command.Y:F6},{hybrid.X:F6},{hybrid.Y:F6},{s.Slip},{s.Inputs},{s.Drops},{error2:F6},{s.MeasuredElbow:F6},{s.Slip2},{s.ElbowError:F6}"));
        Samples++;
        if(phase is "point" or "drawing") { MotionSamples++; squares += s.Error1 * s.Error1; Peak = Math.Max(Peak, Math.Abs(s.Error1)); if(error2 is double e2) { MotionSamples2++; squares2 += e2 * e2; Peak2 = Math.Max(Peak2, Math.Abs(e2)); if(s.ElbowError is double ee) { squaresElbow += ee * ee; PeakElbow = Math.Max(PeakElbow, Math.Abs(ee)); } } }
    }
    public void Finish(string outcome)
    {
        if (finished) return;
        finished = true; csv.Dispose();
        File.WriteAllText(Path.Combine(DirectoryPath, "summary.json"), JsonSerializer.Serialize(new {
            outcome, started_utc = started, ended_utc = DateTime.UtcNow, samples = Samples,
            motion_samples = MotionSamples,
            q1_rmse_deg = MotionSamples > 0 ? (double?)Rmse : null, q1_peak_abs_error_deg = MotionSamples > 0 ? (double?)Peak : null,
            q2_rmse_deg = MotionSamples2 > 0 ? (double?)Rmse2 : null, q2_peak_abs_error_deg = MotionSamples2 > 0 ? (double?)Peak2 : null,
            arm2_follow_rmse_deg = MotionSamples2 > 0 ? (double?)RmseElbow : null, arm2_follow_peak_abs_error_deg = MotionSamples2 > 0 ? (double?)PeakElbow : null,
            measured_xy_error_available = false
        }, new JsonSerializerOptions { WriteIndented = true }), new UTF8Encoding(false));
    }
    public void Dispose() => Finish("interrupted");
    public static void SelfTest()
    {
        var s = TelemetrySample.Parse("TELEM t=500 epoch=9 seq=3 ref=1 source=2 p1=1600 p2=-1600 e1=4096 err=-2 e2=-4096 err2=0 inputs=0 drops=3")!;
        if (s is null || s.Command.Q1 != 9 || Math.Abs(s.Command.Q2-(9-1600*360.0/DeviceContract.Arm2StepsPerRev))>1e-8 || s.Measured1 != 9 || s.Error1 != 0 || s.ReferenceName != "人工伸直零位" || s.Measured2 is not double measured2 || Math.Abs(measured2-s.Command.Q2)>1e-8 || Math.Abs(s.ElbowError!.Value)>1e-8 || Math.Abs((s with { Encoder1=s.Encoder1+16 }).ElbowError!.Value)>1e-8) throw new Exception("Telemetry units/sign invalid");
        if (TelemetrySample.Parse("TELEM t=0 epoch=0 seq=0 ref=1 source=0 p1=0 p2=0 e1=0 err=0 e2=0 err2=0 inputs=16 drops=0") is not null) throw new Exception("Malformed telemetry accepted");
        string root = Path.Combine(Path.GetTempPath(), "arm-recorder-" + Guid.NewGuid().ToString("N"));
        try {
            using var r = new ExperimentRecorder(root, "test", 3, 9, new PlanarArm(180,176.38), "test", "manual", [new(new(0,0),0)]);
            r.Record(s, "point", 0); r.Record(s with { Encoder1=s.Encoder1+16 },"point",0);
            r.Record(s with { Encoder1=s.Encoder1+320 },"upload",0);
            r.Record(s with { Epoch = 10 }, "point", 0); r.Record(s with { Referenced = false }, "point", 0);
            r.Finish("complete"); r.Finish("aborted");
            if (r.Samples != 3 || r.MotionSamples!=2 || Math.Abs(r.Rmse2-r.Rmse)>1e-9 || r.RmseElbow!=0 || Math.Abs(r.Rmse-16*360.0/(4096*40)/Math.Sqrt(2))>1e-9 ||
                File.ReadAllLines(Path.Combine(r.DirectoryPath,"feedback.csv")).Length != 4 ||
                !File.ReadAllText(Path.Combine(r.DirectoryPath,"summary.json")).Contains("complete")) throw new Exception("Experiment persistence failed");
            using var optional = new ExperimentRecorder(root,"optional",3,9,new PlanarArm(180,176.38),"test","manual",[],arm2Feedback:false);
            optional.Record(s with { Encoder2=null, Slip2=null },"point",0); optional.Finish("complete");
            using var summary=JsonDocument.Parse(File.ReadAllText(Path.Combine(optional.DirectoryPath,"summary.json")));
            if(optional.Samples!=1 || optional.MotionSamples2!=0 || summary.RootElement.GetProperty("q2_rmse_deg").ValueKind!=JsonValueKind.Null ||
                File.ReadAllLines(Path.Combine(optional.DirectoryPath,"feedback.csv"))[1].Split(',')[10]!="") throw new Exception("Optional arm2 feedback must stay missing");
        } finally {
            if (Directory.Exists(root) && Path.GetDirectoryName(Path.GetFullPath(root)) == Path.GetFullPath(Path.GetTempPath()).TrimEnd(Path.DirectorySeparatorChar) &&
                Path.GetFileName(root).StartsWith("arm-recorder-",StringComparison.Ordinal)) Directory.Delete(root,true);
        }
    }
}
