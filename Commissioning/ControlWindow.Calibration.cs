namespace TwoLinkCommissioning;

// The calibration wizard's view of this window. Moves reuse RequestPoint, so they pass the same gate as the Move button
// (enabled, referenced, fresh feedback, range confirmed, nothing else running).
public sealed partial class ControlWindow : ICalibrationHost
{
    private CalibrationWindow? calibrationWindow;
    private void OpenCalibration()
    {
        if (calibrationWindow is { IsLoaded: true } open) { open.Activate(); return; }
        calibrationWindow = new CalibrationWindow(this) { Owner = this };
        calibrationWindow.Closed += (_, _) => calibrationWindow = null;
        calibrationWindow.Show();
    }

    PlanarArm? ICalibrationHost.Model { get { try { return xyPreview.Model; } catch (ArgumentException) { return null; } } }
    // Commanded pose only while the device is at rest with fresh feedback of this epoch: a pose read mid-move would not be an arrival.
    private TelemetrySample? RestingTelemetry => supportsTelemetry && !PointBusy && telemetry is { Referenced: true } t && ptp is { Referenced: true } && t.Epoch == ptp.Epoch &&
        (DateTime.UtcNow - lastTelemetryAt).TotalSeconds < 2 ? t : null;
    JointPose? ICalibrationHost.DeviceCommand => RestingTelemetry?.Command;
    double? ICalibrationHost.DeviceEncoderQ1 => RestingTelemetry?.Measured1;
    string ICalibrationHost.ReferenceName => telemetry is { Referenced: true } t ? t.ReferenceName : "";
    string ICalibrationHost.Firmware => lastFirmwareBuild;
    JointLimits? ICalibrationHost.Limits => xyPreview.RequestedLimits;
    JointPose? ICalibrationHost.PointTarget => xyPreview.PointTargetPose;
    bool ICalibrationHost.CanMove(out string reason) { reason = CanPoint ? "" : pointBlockReason; return CanPoint; }
    bool ICalibrationHost.MoveTo(JointPose pose, out string reason)
    {
        if (!CanPoint || (uploaded is not null && !uploadedRan)) { reason = CanPoint ? Lang.T("cal.status.planHeld") : pointBlockReason; return false; }
        if (ptp is null || !ptp.Limits.Contains(PtpProtocol.Quantize(pose))) { reason = Lang.T("cal.repeat.outRange"); return false; }
        RequestPoint(pose); reason = ""; return true;
    }
    void ICalibrationHost.ApplyLinkLengths(double l1, double l2) => xyPreview.SetLinkLengths(l1, l2);
    void ICalibrationHost.ApplyElbowRange(double min, double max) => xyPreview.SetElbowRange(min, max);
}
