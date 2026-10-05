namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private static void MotionProfileSelfTest()
    {
        var w=new ControlWindow { selfTesting=true };
        static void Check(bool b,string why) { if(!b) throw new InvalidOperationException("Global profile: "+why); }
        void State() => w.HandleLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1");
        void Range() => w.HandleLine("PTP_STATE lock=0 ref=0 active=0 epoch=7 seq=0 min1=-3000 max1=3000 min2=-6000 max2=6000 emin=-15500 emax=15500");
        void Rates(MotionProfile p) => w.HandleLine($"MOTION_RATES epoch=7 r1={p.R1} r2={p.R2} a1={p.A1} a2={p.A2}");
        var frames=new List<byte[]>();
        void Write(byte[] b,int offset,int count) { if(offset==0 && (frames.Count==0 || !ReferenceEquals(frames[^1],b)))frames.Add(b); }
        byte[] Send() {
            w.PublishHeartbeatLease();
            for(int i=0;i<6;i++) { w.WireWriterStep(Write); if(frames.Count>0 && frames[^1].Length==36 && ((frames[^1][1]>>1)&15)==10) return frames[^1]; }
            throw new InvalidOperationException("Global profile config not emitted");
        }
        void Ack() => w.HandleLine($"PTP_RATES epoch=7 seq={w.limitSequence} axis={w.limitAxis} min={(int)Math.Round(w.limitBounds.Q1*100)} max={(int)Math.Round(w.limitBounds.Q2*100)}");
        try {
            State();Range();
            Check(!w.enable.IsEnabled,"missing speed status enabled movement");
            w.HandleLine("LINK fw=stage2-v7-limits1 joint_limits=1 short_hb=1");
            Check(!w.enable.IsEnabled && w.LimitsStatus.Contains("proto"),"old firmware without proto= silently accepted");
            w.HandleLine("LINK fw=stage2-v7-global1 joint_limits=1 motion_rates=1 short_hb=1");
            Check(!w.LimitsReady && !w.enable.IsEnabled, "old global1 firmware silently accepted relative q2");
            w.HandleLine($"LINK fw=stage2-v7-absolute-q2 joint_limits=1 motion_rates=1 short_hb=1 q2_absolute=1 proto={DeviceContract.ProtocolVersion + 1}");
            Check(!w.LimitsReady && !w.enable.IsEnabled && w.LimitsStatus.Contains("proto"), "a firmware with another protocol version was accepted");
            // Only the protocol version decides compatibility; a different build id with the same contract is fine.
            w.HandleLine($"LINK fw=stage2-v7-other-build joint_limits=1 motion_rates=1 short_hb=1 proto={DeviceContract.ProtocolVersion}");
            var wanted=w.xyPreview.RequestedProfile!.Value;
            var actual=new MotionProfile(800,600,20000,12000);
            Rates(actual);Check(w.limitType==10 && w.limitAxis==0 && !w.LimitsReady,"axis0 speed request missing");
            var first=Send(); w.SynchronizeJointLimits(w.limitRequestedAt.AddSeconds(1.1)); var retry=Send();
            Check(first.SequenceEqual(retry),"retry changed profile packet");
            Ack();Check(!w.LimitsReady,"ACK alone confirmed profile");
            actual=actual with { R1=wanted.R1,A1=wanted.A1 };State();Range();Rates(actual);
            Check(w.limitAxis==1 && !w.LimitsReady,"axis1 not serialized");
            Send();Ack();State();Range();Rates(wanted);
            Check(w.LimitsReady && w.enable.IsEnabled,"full profile confirmation did not enable");
            w.HandleLine("PTP_EPOCH 8");Check(!w.LimitsReady && w.limitsMotionAllowed==0,"epoch kept speed authorization");
        } finally { w.StopWireWriter();w.Close(); }
    }
    // Existing transport/feedback fixtures model a peer with unchanged default
    // global settings. Send the new real status line instead of bypassing a gate.
    private void TestLine(string line)
    {
        supportsMotionRates=true;
        if(line.StartsWith("LINK ") && !line.Contains("motion_rates=")) line+=" motion_rates=1";
        if(line.StartsWith("LINK ") && !line.Contains("traj_repeat=")) line+=" traj_repeat=1";   // fixtures model a firmware that repeats a held plan
        HandleLine(line);
        if(line.StartsWith("PTP_STATE ") && PtpState.Parse(line) is { } state) {
            var p=MotionProfile.From(new PlanningRates(20,5,3,10,6));
            HandleLine($"MOTION_RATES epoch={state.Epoch} r1={p.R1} r2={p.R2} a1={p.A1} a2={p.A2}");
            // Legacy transport fixtures predate TELEM; supply their stationary
            // simulated origin for the new feed-limited point calculation.
            if(!supportsTelemetry) {
                telemetry=TelemetrySample.Parse($"TELEM t=100 epoch={state.Epoch} seq={state.Sequence} ref=1 source=2 p1=0 p2=0 e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
                lastTelemetryAt=DateTime.UtcNow;
            }
        }
    }
}
