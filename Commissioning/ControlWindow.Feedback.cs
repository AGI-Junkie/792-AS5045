using System.IO;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private bool selfTesting;
    private uint experimentSequence;
    private uint manualReferenceConfirmedEpoch;
    private void InvalidateManualReferenceFeedback()
    {
        manualReferenceConfirmedEpoch=0;
        manualReference.Background=manualReference.BorderBrush=WorkbenchTheme.Blue;
    }
    private void CheckControlTimeout(DateTime now)
    {
        if(pendingControl is null || (now-controlRequestedAt).TotalSeconds<3) return;
        string control = pendingControl;
        Lang.Text(lastStop, () => Lang.T("fault.controlTimeout", control=="使能"?Lang.T("ctl.enable"):Lang.T("ctl.home")));
        Append("HOST control acknowledgement timeout: "+pendingControl); Stop();
    }
    private void CheckTelemetryTimeout(DateTime now)
    {
        if (!supportsTelemetry || (now-lastTelemetryAt).TotalSeconds < 2) return;
        telemetry = null; ShowFeedback(null, () => Lang.T("fault.telemetryStale"));
        if (pendingControl is null && (PointBusy || executing)) { Append("HOST telemetry timeout; stopping"); Stop(); }
    }
    private bool AtStart(JointPose pose) => supportsTelemetry && ptp is { Active: 0, Referenced: true } &&
        telemetry is { Referenced: true, Inputs: 0 } && telemetry.Epoch == ptp.Epoch && telemetry.Sequence == ptp.Sequence &&
        (DateTime.UtcNow-lastTelemetryAt).TotalSeconds < 2 &&
        telemetry.Steps1 == ExecutionTrajectory.Counts(0,pose) && telemetry.Steps2 == ExecutionTrajectory.Counts(1,pose);
    private void HandleTelemetry(string line)
    {
        var sample = TelemetrySample.Parse(line);
        if (!supportsTelemetry || sample is null || ptp is null || sample.Epoch != ptp.Epoch || sample.Sequence != ptp.Sequence) {
            // Enable/home establish their own reference; telemetry can arrive before PTP_STATE.
            if (supportsTelemetry && pendingControl is null && (PointBusy || executing)) { Lang.Text(lastStop, () => Lang.T("fault.badTelemetry")); Stop(); }
            return;
        }
        if (telemetry?.Epoch != sample.Epoch) errorHistory.Clear();
        telemetry = sample; lastTelemetryAt = DateTime.UtcNow;
        if (sample.Referenced) {
            ShowFeedback(sample, () => Lang.T("status.ref") + sample.ReferenceName);
            errorHistory.Enqueue(sample.Error1); while(errorHistory.Count > 120) errorHistory.Dequeue(); DrawErrorPlot();
            if (experiment is { } recorder) {
                recorder.Record(sample,uploading is not null ? "upload" : uploaded is not null ? "ready" : experimentPhase=="trajectory" ? "drawing" : "point",trajectoryIndex);
                Lang.Text(experimentResult, () => recorder.MotionSamples==0 ? Lang.T("exp.savedWaiting", recorder.Samples) :
                    Lang.T("exp.live", recorder.Samples, recorder.Rmse, recorder.Peak));
                if(experimentPhase=="point" && pendingPoint==0 && ptp.Active==0 && sample.Sequence==experimentSequence) experimentCompleting="complete";
                if (experimentCompleting is { } outcome) FinishExperiment(outcome);
            }
        } else {
            ShowFeedback(sample, () => Lang.T("live.noRefYet"));
            if (experiment is not null) FinishExperiment("reference_invalidated");
            if (!manualReferencePending && (pendingPoint != 0 || uploading is not null || executing || ptp.Active != 0)) Stop();
        }
        UpdateControls();
    }
    private void StartExperiment(string kind, uint sequence, ExecutionNode[] nodes)
    {
        if (selfTesting || !supportsTelemetry) return;
        if (telemetry is not { Referenced: true } || ptp is null || telemetry.Epoch != ptp.Epoch ||
            (DateTime.UtcNow-lastTelemetryAt).TotalSeconds >= 2) throw new IOException(Lang.T("fault.experimentMissingFeedback"));
        FinishExperiment("replaced"); errorHistory.Clear(); DrawErrorPlot();
        experiment = new ExperimentRecorder(ExperimentRoot,kind,sequence,ptp.Epoch,xyPreview.Model,xyPreview.SettingsDescription,telemetry.ReferenceName,nodes);
        experimentPhase = kind; experimentSequence=sequence; experimentCompleting = null; lastExperimentDirectory = experiment.DirectoryPath;
        Append("HOST experiment recording: " + lastExperimentDirectory);
    }
    private void FinishExperiment(string outcome)
    {
        var recorder = experiment; experiment = null; experimentCompleting = null;
        if (recorder is null) return;
        recorder.Finish(outcome);
        Lang.Text(experimentResult, () => Lang.T("exp.finished", outcome=="complete"?Lang.T("exp.complete"):Lang.T("exp.aborted"), recorder.Samples) +
            (recorder.MotionSamples>0?Lang.T("exp.rmse", recorder.Rmse):Lang.T("exp.noMotion")) +
            Lang.T("exp.savedNote"));
        diagnostics.IsExpanded = true;
        Append("HOST experiment " + outcome + ": " + recorder.DirectoryPath);
    }
    private static string ExplainFault(string line)
    {
        string action = line.Contains("UART receive") ? Lang.T("explain.uart") :
            line.Contains("joint range") ? Lang.T("explain.range") :
            line.Contains("start") && line.Contains("mismatch") ? Lang.T("explain.start") :
            line.Contains("encoder") || line.Contains("following") || line.Contains("slip") ? Lang.T("explain.encoder") :
            line.Contains("limit") || line.Contains("sensor") || line.Contains("NPN") ? Lang.T("explain.limit") :
            line.Contains("heartbeat") || line.Contains("deadman") ? Lang.T("explain.heartbeat") :
            line.Contains("stale") ? Lang.T("explain.stale") :
            line.Contains("operator stop") ? Lang.T("explain.operatorStop") :
            line.Contains("fault") ? Lang.T("explain.fault") :
            Lang.T("explain.other");
        return action + "\n" + line;
    }
    private static void FeedbackSelfTest(string? snapshotPath = null)
    {
        var w=new ControlWindow { selfTesting=true };
        static void Check(bool b,[System.Runtime.CompilerServices.CallerArgumentExpression(nameof(b))] string? expression=null) { if(!b) throw new Exception("Feedback guard: "+expression); }
        void Snapshot(string name) {
            if (snapshotPath is null) return;
            w.SaveWorkbenchSnapshot(Path.Combine(Path.GetDirectoryName(snapshotPath)!,name),1100,760);
        }
        void Ready(uint epoch=4, uint seq=1) {
            w.TestLine("STATE fw=stage2-v7 enabled=1 home=0 homed1=1 homed2=1 fault=0 enc=1 dir=1");
            w.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch={epoch} seq={seq} min1=-3000 max1=3000 min2=-6000 max2=6000");
            if(!w.protocolMatches) w.TestLine($"LINK fw=stage2-v7-absolute-q2 q2_absolute=1 proto={DeviceContract.ProtocolVersion} motion_rates=1");
            w.TestLine($"TELEM t=10 epoch={epoch} seq={seq} ref=1 source=2 p1=0 p2=0 e1=0 err=0 inputs=0 drops=0");
        }
        Ready(); Check(w.CanPoint && w.AtStart(new(0,0)) && !w.AtStart(new(1,0)));
        w.lastTelemetryAt=DateTime.UtcNow.AddSeconds(-3); Check(!w.CanPoint && !w.AtStart(new(0,0)));
        Ready(); w.RequestPoint(new(1,1)); uint seq=w.pendingPoint; int n=w.pointWire.Count;
        w.RequestPoint(new(2,2)); Check(w.pendingPoint==seq && w.pointWire.Count==n); // double click
        w.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=99 seq={seq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        Check(w.pendingPoint==0 && w.pointWire.Count==0 && w.ptp is null); // wrong reference cancels, never ACKs
        Ready(5,3); w.RequestPoint(new(1,1)); w.TestLine("TELEM broken"); Check(w.pendingPoint==0 && w.ptp is null);
        Ready(6,4); w.TestLine("TELEM t=11 epoch=6 seq=4 ref=1 source=2 p1=0 p2=0 e1=0 err=0 inputs=1 drops=0"); Check(!w.CanPoint);
        Ready(7,5); w.RequestPoint(new(1,1)); w.Disconnect("test unplug"); Check(w.pendingPoint==0 && w.pointWire.Count==0 && !w.CanPoint && !w.compatible);
        w.TestLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=0");
        w.TestLine("PTP_STATE lock=0 ref=0 active=0 epoch=7 seq=5 min1=-3000 max1=3000 min2=-6000 max2=6000");
        w.TestLine($"LINK fw=stage2-v7-absolute-q2 q2_absolute=1 proto={DeviceContract.ProtocolVersion} motion_rates=1");
        w.RequestEnable(); Check(w.pendingControl=="使能" && !w.enable.IsEnabled && !w.home.IsEnabled);
        w.CheckControlTimeout(w.controlRequestedAt.AddSeconds(3)); Check(w.pendingControl is null && w.lastStop.Text.Contains("未收到确认"));
        Ready(8,6); w.RequestHome(); Check(w.pendingControl=="回零" && !w.manualReference.IsEnabled);
        w.TestLine("STATE fw=stage2-v7 enabled=1 home=3 homed1=0 homed2=0 fault=0 enc=1 dir=1");
        Check(w.pendingControl is null && w.homing && !w.home.IsEnabled);
        Ready(20,10); w.RequestManualReference(); uint manualSeq=w.pendingPoint;
        Check(!w.manualReference.IsEnabled && w.manualReference.Background==WorkbenchTheme.Blue &&
            w.manualReference.Content.ToString()=="人工确认当前位置为伸直零位");
        Snapshot("manual-reference-waiting.png");
        w.RequestManualReference(); Check(w.pendingPoint==manualSeq && w.pointWire.Count==36); // repeated click never re-zeroes
        w.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=20 seq={manualSeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        Check(w.manualReferencePending && !w.manualReference.IsEnabled && w.manualReference.Background==WorkbenchTheme.Blue); // matching seq alone is not an ACK
        w.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=21 seq={manualSeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        Check(!w.manualReferencePending && w.manualReference.IsEnabled && w.manualReference.Background==WorkbenchTheme.Teal &&
            w.manualReference.BorderBrush==WorkbenchTheme.Teal && w.manualReference.Content.ToString()=="人工确认当前位置为伸直零位");
        Ready(21,manualSeq); Check(w.manualReference.Background==WorkbenchTheme.Teal); Check(w.xyPreview.FollowsDevice); // normal telemetry does not flash away success
        Snapshot("manual-reference-confirmed.png");
        Ready(22,manualSeq); Check(w.manualReference.Background==WorkbenchTheme.Blue); // a new reference is not the acknowledged one
        w.RequestManualReference(); manualSeq=w.pendingPoint;
        w.TestLine($"PTP_STATE lock=0 ref=1 active=0 epoch=23 seq={manualSeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        w.TestLine("STOP: UART receive error/overflow"); Check(w.manualReference.Background==WorkbenchTheme.Blue);
        Snapshot("manual-reference-invalid.png");
        Ready(24,manualSeq); w.RequestManualReference(); w.TestLine("PTP refused: not homed/idle/healthy");
        Check(!w.manualReferencePending && !w.manualReference.IsEnabled && w.manualReference.Background==WorkbenchTheme.Blue && w.lastStop.Text.Contains("refused"));
        Ready(25,manualSeq); w.RequestManualReference(); w.RetryPendingPoint(w.pointRequestedAt.AddSeconds(3));
        Check(!w.manualReferencePending && !w.manualReference.IsEnabled && w.manualReference.Background==WorkbenchTheme.Blue && w.lastStop.Text.Contains("未确认"));
        Ready(26,manualSeq); w.RequestManualReference(); w.Disconnect("test unplug");
        Check(!w.manualReference.IsEnabled && w.manualReference.Background==WorkbenchTheme.Blue);
        Ready(27,manualSeq); w.RequestManualReference(); manualSeq=w.pendingPoint;
        w.TestLine($"PTP_STATE lock=0 ref=0 active=0 epoch=28 seq={manualSeq} min1=-3000 max1=3000 min2=-6000 max2=6000");
        Check(!w.manualReferencePending && !w.manualReference.IsEnabled && w.manualReference.Background==WorkbenchTheme.Blue);
        Ready(29,manualSeq); w.RequestPoint(new(1,1));
        w.TestLine("STOP: DEADMAN host heartbeat timeout; position invalid, release keys then C to enable");
        string firstStop=w.lastStop.Text;
        w.TestLine("STATE fw=stage2-v7 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1");
        w.TestLine("PTP refused: stale reference; establish reference first");
        Check(w.pendingPoint==0 && w.lastStop.Text==firstStop && firstStop.Contains("DEADMAN"));
        Ready(30,manualSeq); w.RequestPoint(new(1,1));
        w.TestLine("PTP refused: not homed/idle/healthy");
        Check(w.pendingPoint==0 && w.lastStop.Text.Contains("not homed/idle/healthy"));
        w.Stop(); w.Close();
    }
}
