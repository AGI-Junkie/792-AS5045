using System.IO;
using System.Windows;
using System.Windows.Media.Imaging;
using System.Windows.Media;

namespace TwoLinkCommissioning;

internal sealed partial class CalibrationWindow
{
    // A host that records what it is asked to do. The window must never move the machine or change the planner by itself.
    private sealed class FakeHost : ICalibrationHost
    {
        public PlanarArm? Model { get; set; } = new(180, 176.38);
        public JointPose? DeviceCommand { get; set; }
        public double? DeviceEncoderQ1 { get; set; }
        public string ReferenceName { get; set; } = "人工伸直零位";
        public string Firmware { get; set; } = "fw-test";
        public JointLimits? Limits { get; set; } = new(-90, 90, -160, 160, -155, 155);
        public JointPose? PointTarget { get; set; } = new(12, 34);
        public bool Movable { get; set; }
        public List<JointPose> Moves { get; } = [];
        public List<(double, double)> Links { get; } = [];
        public List<(double, double)> Elbows { get; } = [];
        public bool CanMove(out string reason) { reason = Movable ? "" : "未使能"; return Movable; }
        public bool MoveTo(JointPose pose, out string reason) { reason = Movable ? "" : "未使能"; if (Movable) Moves.Add(pose); return Movable; }
        public void ApplyLinkLengths(double l1, double l2) => Links.Add((l1, l2));
        public void ApplyElbowRange(double min, double max) => Elbows.Add((min, max));
    }

    internal static void SelfTest(string? snapshotPath = null)
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Calibration window: " + reason); }
        string temp = Path.Combine(Path.GetTempPath(), "arm-calwin-" + Guid.NewGuid().ToString("N")), previous = Calibrations.Root;
        Calibrations.Root = Path.Combine(temp, "Calibrations");
        try
        {
            Check(!Lang.IsEn, "the test runs in Chinese");
            var host = new FakeHost();
            var w = new CalibrationWindow(host);
            // Nothing is prefilled and nothing is complete: the planner's own numbers are not measurements.
            Check(w.StepLabels.Take(4).All(s => s.StartsWith("○")) && !w.CanSave && w.TrialCount == 0 && w.HistoryCount == 0, "starts empty: " + string.Join("|", w.StepLabels));
            Check(w.LinkText.Contains("不是测量值") && w.ZeroText.Length > 0 && w.StatusText.Contains("现在不能移动：未使能"), "says what to do and why it cannot move: " + w.StatusText);
            // An untouched zero page (offsets 0, no tip) is not a measurement; a typed one is.
            Check(w.StepLabels[1].StartsWith("○"), "all-zero zero page is not a measurement");
            w.Set("q1off", "0.4"); Check(w.StepLabels[1].StartsWith("✓") && w.CanSave, "a typed offset counts");
            w.Set("q1off", "0"); Check(w.StepLabels[1].StartsWith("○") && !w.CanSave, "and removing it uncounts it");
            w.Set("tipx", "356.4"); w.Set("tipy", "1.1"); Check(w.StepLabels[1].StartsWith("✓"), "a measured tip counts");
            // 1. Link lengths: compared with the planner, applied only on request.
            w.Set("l1", "180.4"); w.Set("l2", "176.1");
            Check(w.StepLabels[0].StartsWith("✓") && w.LinkText.Contains("+0.40") && w.LinkText.Contains("-0.28") && host.Links.Count == 0, "differences are shown, nothing is applied: " + w.LinkText);
            w.Set("l1", "-5"); Check(w.StepLabels[0].StartsWith("○"), "a negative length is not a measurement");
            w.Set("l1", "180.4");
            w.ApplyLinks(); Check(host.Links.SequenceEqual([(180.4, 176.1)]) && w.StatusText.Contains("180.4"), "apply puts the lengths into the planner boxes");
            // 3. Stroke: a margin inside the stops; applying needs both elbow ends.
            w.Set("emin", "-150"); w.ApplyElbow(); Check(host.Elbows.Count == 0, "one end is not a range");
            w.Set("emax", "158"); w.Set("margin", "3");
            Check(w.StrokeText.Contains("-147") && w.StrokeText.Contains("155"), "suggestion keeps the margin inside the stops: " + w.StrokeText);
            w.ApplyElbow(); Check(host.Elbows.SequenceEqual([(-147.0, 155.0)]), "apply puts the suggestion into the planner");
            w.Set("margin", "200"); w.ApplyElbow(); Check(host.Elbows.Count == 1, "a margin that eats the stroke applies nothing");
            w.Set("margin", "3");
            // 4. Repeatability: alternating arrivals; moves go through the host, and only when it says it can.
            w.Set("tq1", "5"); w.Set("tq2", "7"); w.Set("approach", "4"); w.Set("n", "1"); w.Generate(); Check(w.TrialCount == 0, "fewer than two arrivals per direction is refused");
            w.Set("n", "3"); w.Generate(); Check(w.TrialCount == 6, "three per direction");
            w.SelectTrial(0); w.Move(true); Check(host.Moves.Count == 0 && w.StatusText.Contains("未使能"), "no move while the host cannot: " + w.StatusText);
            host.Movable = true;
            w.Move(true); w.SelectTrial(1); w.Move(true); w.Move(false);
            Check(host.Moves.SequenceEqual([new JointPose(9, 7), new JointPose(1, 7), new JointPose(5, 7)]), "plus arrives from q1+approach, minus from q1-approach, then the target: " + string.Join(";", host.Moves));
            w.Set("approach", "90"); w.SelectTrial(0); w.Move(true); Check(host.Moves.Count == 3 && w.StatusText.Contains("超出"), "a pre-point outside the ranges is refused: " + w.StatusText);
            w.Set("approach", "4");
            // Recording an arrival reads the device, and says so when there is nothing to read.
            w.SelectTrial(0); w.Record(); Check(w.Selected(0, "q1") == "" && w.StatusText.Contains("没有新鲜"), "no feedback, no record");
            host.DeviceCommand = new JointPose(5.0123, 7.0456); host.DeviceEncoderQ1 = 5.0511;
            w.Record(); Check(w.Selected(0, "q1") == "5.012" && w.Selected(0, "q2") == "7.046" && w.Selected(0, "enc") == "5.051", "the command and the encoder angle are copied from the device");
            // The pen readings are typed by the person: the plus group scatters ±0.1 mm, the minus group ±0.05 mm and sits 0.30 mm away.
            double[] scatter = [0.10, -0.10, 0.0];
            for (int i = 0; i < 3; i++) { w.FillTrial(2 * i, 340.15 + scatter[i], 0.02 * scatter[i] / 0.1); w.FillTrial(2 * i + 1, 339.85 + scatter[i] / 2, 0); }
            var arm = new PlanarArm(180, 176.38); var travel = arm.Forward(new JointPose(5, 7)).Tip - arm.Forward(new JointPose(9, 7)).Tip; travel.Normalize();
            string expected = Math.Abs(0.30 * travel.X).ToString("0.000", System.Globalization.CultureInfo.InvariantCulture);
            Check(w.RepeatSummary.Contains("n=3") && w.RepeatSummary.Contains(expected + " mm") && w.RepeatSummary.Contains("已填 6 次，未填 0 次"), $"statistics along the travel (expect {expected}): " + w.RepeatSummary);
            Check(w.StepLabels[3].StartsWith("✓"), "the step is marked done");
            // 5. Save: dated and versioned, never overwritten, and it becomes the current calibration; nothing was applied by saving.
            int linksBefore = host.Links.Count, elbowsBefore = host.Elbows.Count, movesBefore = host.Moves.Count;
            CalibrationRecord? announced = null; w.Saved += r => announced = r;
            w.Set("note", "第一次"); w.RefreshSummary();
            Check(w.SummaryShown.Contains("v1") && w.SummaryShown.Contains("保存不会修改") && w.SummaryShown.Contains("fw-test"), "the summary names the version and says nothing is applied automatically: " + w.SummaryShown);
            var first = w.SaveNow();
            Check(first is { Version: 1, Firmware: "fw-test", Note: "第一次" } && announced == first && Calibrations.Active()!.Id == first.Id && first.Links!.L1 == 180.4 && first.Stroke!.ElbowMax == 158 && first.Repeatability!.Trials.Count == 6 && first.Zero!.ReferenceMethod == "人工伸直零位", "first version saved with every section");
            Check(w.StatusText.Contains("v1") && w.HistoryCount == 1, "status and history show it: " + w.StatusText);
            var second = w.SaveNow();
            Check(second is { Version: 2 } && second.Id != first!.Id && Calibrations.Active()!.Id == second.Id && w.HistoryCount == 2, "saving again is a new version");
            w.ClickSteps(5); w.ChooseActive(1); Check(Calibrations.Active()!.Id == first!.Id, "an older version can be made current again");
            Check(host.Links.Count == linksBefore && host.Elbows.Count == elbowsBefore && host.Moves.Count == movesBefore, "saving and choosing never touched the machine or the planner");
            // A bad draft cannot be saved.
            var bad = new CalibrationWindow(host); bad.Set("l1", "180"); bad.Set("l2", "176"); bad.Set("emin", "50"); bad.Set("emax", "10");
            Check(!bad.CanSave && bad.SaveNow() is null && Calibrations.List().Count == 2 && bad.StatusText.Contains("保存失败"), "an inverted stroke is refused and nothing is written: " + bad.StatusText);
            bad.Close();
            // Language: every label and note follows, the entries survive.
            Lang.Toggle();
            try
            {
                Check(w.Title.Contains("Calibration") && w.StepLabels[0].Contains("Link lengths") && w.RepeatSummary.Contains("hysteresis") && w.Selected(0, "q1") == "5.012", "English labels, entries kept: " + string.Join("|", w.StepLabels) + " / " + w.RepeatSummary);
                w.ClickSteps(5); Check(w.HistoryCount == 2, "history in English");
            }
            finally { Lang.Toggle(); }
            Check(w.Title.Contains("标定向导") && w.StepLabels[3].StartsWith("✓"), "and back");
            if (snapshotPath is not null)
            {
                string dir = Path.GetDirectoryName(Path.GetFullPath(snapshotPath))!; Directory.CreateDirectory(dir);
                w.Width = 980; w.Height = 760; w.Show(); w.UpdateLayout();
                Check(w.steps.ActualWidth > 120 && w.page.ActualWidth > 500 && w.status.ActualWidth > 500 && w.status.ActualHeight < 120, $"layout: steps {w.steps.ActualWidth:0}, page {w.page.ActualWidth:0}, status {w.status.ActualWidth:0}x{w.status.ActualHeight:0}");
                foreach (var (index, name) in new[] { (0, "links"), (2, "stroke"), (3, "repeat"), (4, "save"), (5, "history") })
                {
                    w.ClickSteps(index); if (index == 4) w.RefreshSummary(); w.UpdateLayout();
                    var content = (FrameworkElement)w.Content;
                    var image = new RenderTargetBitmap((int)content.ActualWidth, (int)content.ActualHeight, 96, 96, PixelFormats.Pbgra32); image.Render(content);
                    var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(image));
                    using var stream = File.Create(Path.Combine(dir, $"calibration-{name}.png")); png.Save(stream);
                }
            }
            w.Close();
        }
        finally { Calibrations.Root = previous; if (Directory.Exists(temp)) Directory.Delete(temp, true); }
    }
}
