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
        lastStop.Text=$"{pendingControl}未收到确认。已停止；检查串口和输入状态后重新使能。";
        Append("HOST control acknowledgement timeout: "+pendingControl); Stop();
    }
    private void CheckTelemetryTimeout(DateTime now)
    {
        if (!supportsTelemetry || (now-lastTelemetryAt).TotalSeconds < 2) return;
        telemetry = null; liveValues.Text = "反馈已过期；等待新的设备数据。";
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
            if (supportsTelemetry && pendingControl is null && (PointBusy || executing)) { lastStop.Text = "设备反馈无效或参考代次不匹配；已停止，请重新建立坐标。"; Stop(); }
            return;
        }
        if (telemetry?.Epoch != sample.Epoch) errorHistory.Clear();
        telemetry = sample; lastTelemetryAt = DateTime.UtcNow;
        inputs.Text = $"全局限位：{((sample.Inputs & 1)==0?"正常":"触发")}  arm1：{((sample.Inputs & 2)==0?"释放":"触发")}\n" +
            $"arm2 NPN：{((sample.Inputs & 4)==0?"释放":"触发")}  驱动：{((sample.Inputs & 8)==0?"正常":"故障")}";
        if (sample.Referenced) {
            liveValues.Text = $"arm1  命令 {sample.Command.Q1:F3}° / 编码器 {sample.Measured1:F3}°\n" +
                $"arm2  命令 {sample.Command.Q2:F3}°   ·   {sample.ReferenceName}\narm1 偏差 {sample.Error1:+0.000;-0.000;0.000}° · 累计丢包 {sample.Drops}";
            errorHistory.Enqueue(sample.Error1); while(errorHistory.Count > 120) errorHistory.Dequeue(); DrawErrorPlot();
            if (experiment is { } recorder) {
                recorder.Record(sample,uploading is not null ? "upload" : experimentPhase=="trajectory" ? "drawing" : "point",trajectoryIndex);
                experimentResult.Text = recorder.MotionSamples==0 ? $"已保存 {recorder.Samples} 条反馈；等待运动样本，暂不计算误差统计。" :
                    $"本次采样 {recorder.Samples} · arm1 RMSE {recorder.Rmse:F4}° · 峰值 {recorder.Peak:F4}°\n数据正在自动保存。";
                if(experimentPhase=="point" && pendingPoint==0 && ptp.Active==0 && sample.Sequence==experimentSequence) experimentCompleting="complete";
                if (experimentCompleting is { } outcome) FinishExperiment(outcome);
            }
        } else {
            liveValues.Text = "坐标参考尚未建立；角度与误差不作有效读数。";
            if (experiment is not null) FinishExperiment("reference_invalidated");
            if (!manualReferencePending && (pendingPoint != 0 || uploading is not null || executing || ptp.Active != 0)) Stop();
        }
        UpdateControls();
    }
    private void StartExperiment(string kind, uint sequence, ExecutionNode[] nodes)
    {
        if (selfTesting || !supportsTelemetry) return;
        if (telemetry is not { Referenced: true } || ptp is null || telemetry.Epoch != ptp.Epoch ||
            (DateTime.UtcNow-lastTelemetryAt).TotalSeconds >= 2) throw new IOException("缺少有效的当前位置反馈；请等待读数后重试。");
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
        experimentResult.Text = $"实验{(outcome=="complete"?"完成":"中止")} · {recorder.Samples} 个样本 · " +
            (recorder.MotionSamples>0?$"RMSE {recorder.Rmse:F4}°\n":"无有效运动样本\n") +
            $"已保存计划、反馈和摘要。arm2 / 末端误差未实测。";
        Append("HOST experiment " + outcome + ": " + recorder.DirectoryPath);
    }
    private static string ExplainFault(string line)
    {
        string action = line.Contains("UART receive") ? "串口接收出错或缓冲溢出，已停止并作废坐标参考。检查 USB 连接及通信诊断，再使能并确认零位。" :
            line.Contains("joint range") ? "目标超出软件范围：缩小图形或调整位置。" :
            line.Contains("start") && line.Contains("mismatch") ? "起点不符：先低速移到当前轨迹起点。" :
            line.Contains("encoder") || line.Contains("following") || line.Contains("slip") ? "编码器或跟随误差异常：检查负载、驱动与编码器，再使能并建立零位。" :
            line.Contains("limit") || line.Contains("sensor") || line.Contains("NPN") ? "限位 / 传感器触发：检查实际位置及输入，再重新使能。" :
            line.Contains("heartbeat") || line.Contains("deadman") ? "心跳超时：保持窗口响应，检查串口后重新使能。" :
            line.Contains("stale") ? "坐标参考或序号失效：重新使能并人工确认零位，也可开关回零。" :
            line.Contains("operator stop") ? "已停止。继续前重新使能，再人工确认零位或回零。" :
            line.Contains("fault") ? "驱动故障：检查驱动供电 / 故障输入，再重新使能。" :
            "请求已停止或被拒绝；检查以下设备原因，解决后再操作。";
        return action + "\n" + line;
    }
    private static void FeedbackSelfTest(string? snapshotPath = null)
    {
        var w=new ControlWindow { selfTesting=true };
        static void Check(bool b,[System.Runtime.CompilerServices.CallerArgumentExpression(nameof(b))] string? expression=null) { if(!b) throw new Exception("Feedback guard: "+expression); }
        void Snapshot(string name) {
            if (snapshotPath is null) return;
            w.sideTabs.SelectedIndex=1;
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
        Ready(21,manualSeq); Check(w.manualReference.Background==WorkbenchTheme.Teal); // normal telemetry does not flash away success
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
