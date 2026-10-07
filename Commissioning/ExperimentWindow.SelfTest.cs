using System.Globalization;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace TwoLinkCommissioning;

internal sealed partial class ExperimentWindow
{
    // Two real recordings (written by the recorder from real plans), a session log whose timestamps line up with them, and the window
    // through its list, curves, comparison, replay cursor, log and language switch.
    internal static void SelfTest(string? snapshotPath = null)
    {
        static void Check(bool yes, string reason) { if (!yes) throw new InvalidOperationException("Experiment window: " + reason); }
        string temp = Path.Combine(Path.GetTempPath(), "arm-expwin-" + Guid.NewGuid().ToString("N"));
        string root = Path.Combine(temp, "Experiments"), logs = Path.Combine(temp, "Logs");
        Directory.CreateDirectory(root); Directory.CreateDirectory(logs);
        LogRootOverride = logs;
        try
        {
            const string settings = "shape=2; branch=0; L1=180; L2=176.38; x=320; y=-15; a=30; b=24; feed=136.6; v1=5; v2=3; a1=10; a2=6; min1=-90; max1=90; min2=-160; max2=160; emin=-155; emax=155; c=; rot=0; scale=1; dir=ccw; custom_chars=0";
            string rect = ExperimentArchiveTests.WriteSyntheticRun(root, DrawingShape.Rectangle, 8200, settings, out _);
            Thread.Sleep(15);
            string circle = ExperimentArchiveTests.WriteSyntheticRun(root, DrawingShape.Circle, 40000, settings.Replace("shape=2", "shape=3").Replace("v1=5", "v1=4"), out _);
            // Give the first recording a real host clock (the recorder stamps every row with "now"), a matching start time, and a stop snapshot.
            var baseUtc = DateTime.UtcNow.AddMinutes(-30);
            var lines = File.ReadAllLines(Path.Combine(rect, "feedback.csv"));
            double firstMs = double.Parse(lines[1].Split(',')[1], CultureInfo.InvariantCulture);
            var rewritten = new List<string> { lines[0] };
            foreach (string line in lines.Skip(1)) { var f = line.Split(','); f[0] = baseUtc.AddSeconds((double.Parse(f[1], CultureInfo.InvariantCulture) - firstMs) / 1000).ToString("O", CultureInfo.InvariantCulture); rewritten.Add(string.Join(",", f)); }
            File.WriteAllLines(Path.Combine(rect, "feedback.csv"), rewritten, new UTF8Encoding(true));
            var meta = File.ReadAllText(Path.Combine(rect, "metadata.json"));
            using (var doc = JsonDocument.Parse(meta)) meta = meta.Replace(doc.RootElement.GetProperty("started_utc").GetString()!, baseUtc.ToString("O", CultureInfo.InvariantCulture)).Replace("\"schema\": 1", "\"schema\": 1");
            File.WriteAllText(Path.Combine(rect, "metadata.json"), meta);
            File.WriteAllText(Path.Combine(rect, "fault.json"), JsonSerializer.Serialize(new { schema = 1, snapshot = new { Milliseconds = 8200 + 12000, Phase = "traj_run", Reason = "DRV8825 nFAULT; drivers OFF", Driver = "nfault_undetermined" } }));
            // The seconds before the stop: two samples inside the recording (must not be drawn twice) and three after its last sample.
            double lastMs = double.Parse(lines[^1].Split(',')[1], CultureInfo.InvariantCulture);
            File.WriteAllLines(Path.Combine(rect, "pre_fault.csv"), [
                "host_utc,device_ms,epoch,seq,q1_command_deg,q1_encoder_deg,q1_error_deg,q2_command_deg,q2_encoder_deg,q2_error_deg,slip_motor_lsb,inputs_active_mask,slip2_motor_lsb",
                .. new[] { lastMs - 200, lastMs - 100 }.Select(ms => $"{baseUtc:O},{ms:F0},5,1,10,10,0,0,0,0,0,0,0"),
                .. new[] { lastMs + 500, lastMs + 1000, lastMs + 1500 }.Select(ms => $"{baseUtc:O},{ms:F0},5,1,10,10.5,0.5,0,0,0,0,0,0"), "garbage"], new UTF8Encoding(true));
            // The log of that session: a marker line every second, and status-block noise between them.
            var baseLocal = baseUtc.ToLocalTime();
            var log = new List<string>();
            for (int s = -5; s < 90; s++)
            {
                var at = baseLocal.AddSeconds(s);
                log.Add($"[{at:HH:mm:ss.fff}] HOST marker {s}");
                if (s == 20) log.Add($"[{at.AddMilliseconds(250):HH:mm:ss.fff}] TIMING loop_max_ms=3 tick_lag_max_ms=1 clock=hardware wdt=256-384ms key_age_ms=10 key_max_gap_ms=40 status_max_us=900 traj_lag_max_us=17321 traj_slip_us=6520");
                log.Add($"[{at.AddMilliseconds(500):HH:mm:ss.fff}] TELEM t=1 epoch=5 seq=1 ref=1 source=2 p1=0 p2=0 e1=0 err=0 e2=0 err2=0 inputs=0 drops=0");
            }
            string logFile = Path.Combine(logs, $"stage1-{baseLocal.AddMinutes(-1):yyyyMMdd-HHmmss}-000.log");
            File.WriteAllLines(logFile, log); File.SetLastWriteTime(logFile, DateTime.Now);
            // A log of another day must not be picked.
            File.WriteAllText(Path.Combine(logs, "stage1-20000101-000000-000.log"), "[00:00:00.000] HOST old\n");

            bool zh = !Lang.IsEn;
            Check(zh, "the test runs in Chinese");
            var w = new ExperimentWindow(root);
            Check(w.ListedRuns == 2 && w.ColumnHeaders.Length == 12 && w.ColumnHeaders[0] == "开始时间" && w.ColumnHeaders[8] == "arm2 RMSE °" && w.ColumnHeaders[6] == "RMSE °", "list and localized headers");
            int rectRow = Enumerable.Range(0, 2).First(i => w.RowText(i).Contains("矩形")), circleRow = 1 - rectRow;
            string rowText = w.RowText(rectRow);
            Check(rowText.Contains("|图形|矩形|5 / 3|人工伸直零位|完整|") && rowText.Contains("10 Hz|—|") && rowText.EndsWith("|⚠") && w.RowText(circleRow).Contains("|圆|4 / 3|") && !w.RowText(circleRow).EndsWith("⚠"), "row facts: shape, speeds, reference, outcome, telemetry rate and the fault mark: " + rowText);
            Check(w.ShownRuns == 0 && w.Plots.All(p => p.Series.Count == 0), "nothing is shown until something is selected");

            // One recording: the curves, the events, the written summary.
            w.SelectForSelfTest(rectRow);
            var (q, err, speed, accel) = (w.Plots[0], w.Plots[1], w.Plots[2], w.Plots[3]);
            Check(w.ShownRuns == 1 && q.Series.Count == 7 && err.Series.Count == 3 && speed.Series.Count == 4 && accel.Series.Count == 2, "series per chart");
            Check(err.Marks.Count(m => m.Label is "起步" or "停机" or "拐角 1" or "拐角 2" or "拐角 3") == 5 && err.Marks.Any(m => m.Label.StartsWith("故障 ")), "the plan's events and the stop snapshot are marked on the error chart");
            Check(Math.Abs(err.Marks.First(m => m.Label.StartsWith("故障")).X - 12.0) < 0.02, "the stop snapshot is placed at its device time on the aligned axis");
            Check(new[] { q, speed, accel }.All(p => p.Marks.Any(m => Math.Abs(m.X - 12.0) < 0.02 && m.Color == WorkbenchTheme.Red)), "the stop is marked on every chart");
            Check(q.Series.Count(x => x.Name.StartsWith("停机前遥测")) == 2 && err.Series.Count(x => x.Name.StartsWith("停机前遥测")) == 1, "the pre-stop telemetry is a thin series on the joint and error charts");
            Check(q.Series.First(x => x.Name.StartsWith("停机前遥测")).X.Length == 3 && w.Range.Max >= q.Series.First(x => x.Name.StartsWith("停机前遥测")).X[^1] - 1e-9, "only the pre-stop samples the recording lacks are drawn, and the time range covers them");
            Check(w.SummaryText_.Contains("最大滞后 17321 µs") && w.SummaryText_.Contains("压着走累计 6520 µs") && w.SummaryText_.Contains("3 个停机前遥测样本"), "summary: firmware TIMING from the session log, and the pre-stop samples: " + w.SummaryText_);
            Check(speed.Lines.Count == 4 && speed.Lines.Any(l => l.Label.StartsWith("电机1 上限")) && accel.Lines.Count == 4, "the caps are drawn as reference lines");
            Check(w.Plots.All(p => p.XRange.Min <= 0.001 && p.XRange.Max > 10) && w.Range.Max > 10, "one shared time range");
            Check(err.SymmetricY && q.Source.Contains("40:1") && err.Source.Contains("不是笔尖位置误差") && err.Source.Contains("arm2") && speed.Source.Contains("粗略估计") && w.Plane.Source.Contains("不是实测笔尖位置"), "every chart states where its data comes from, and what it is not");
            Check(w.Plane.Paths.Count == 3 && w.Plane.Paths[0].Name == "规划轨迹（模型）" && w.Plane.Paths.Any(p => p.Name == "混合反馈估算"), "plane: plan, command and hybrid paths");
            Check(w.SummaryText_.Contains("arm2 误差：RMSE") && err.Notes.Any(n => n.StartsWith("arm2 误差")) && err.Lines.Any(l => l.Label.StartsWith("q2 RMSE")) && w.SummaryText_.Contains("分段 arm2 RMSE"), "arm2's error statistics are in the summary, the chart note and as ±RMSE lines");
            Check(q.Series.First(x => x.Name == "q2 编码器").Color != q.Series.First(x => x.Name == "编码器").Color && q.Series.First(x => x.Name == "q2 编码器").Color != q.Series.First(x => x.Name == "q2 命令").Color, "arm2's measured curve has a colour of its own, not arm1's encoder colour");
            Check(w.SummaryText_.Contains("RMSE") && w.SummaryText_.Contains("限速") && w.SummaryText_.Contains("时间对齐") && w.SummaryText_.Contains("停机快照") && w.SummaryText_.Contains("原因待诊断") && w.SummaryText_.Contains("分段 RMSE"), "summary: error, limiter, alignment, stop snapshot (cause undetermined), strokes: " + w.SummaryText_);
            Check(w.Plane.Arms.Count == 2 && w.CursorTime == w.Range.Min, "the replay starts at the beginning, with both arms drawn");

            // The cursor moves every view together.
            w.SetCursor(5.0);
            Check(w.Plots.All(p => p.CursorX == 5.0) && w.CursorTime == 5.0, "the cursor is shared by the four charts");
            var analysis = RunAnalysis.Create(ExperimentArchive.List(root).Runs.First(r => r.Directory == rect), ExperimentArchive.Load(ExperimentArchive.List(root).Runs.First(r => r.Directory == rect)));
            var (elbow, tip) = analysis.Model.Forward(new JointPose(analysis.At(analysis.Q1Command, 5.0), analysis.At(analysis.Q2Command, 5.0)));
            Check((w.Plane.Arms[0].Joints[2] - tip).Length < 1e-9 && (w.Plane.Arms[0].Joints[1] - elbow).Length < 1e-9 && w.Plane.Arms[0].Joints[0] == new Point(0, 0), "the arm drawn is the commanded pose at the cursor");
            double q1At = analysis.At(analysis.Q1Command, 5.0);
            Check(Math.Abs(PlotView.ValueAt(q.Series[0], 5.0) - q1At) < 1e-9, "the legend value at the cursor is the series value there");
            Check(w.LogText.Contains("HOST marker") && !w.LogText.Contains("TELEM t=") && !w.LogText.Contains("old"), "the log shows the lines of the right session around the cursor, status blocks hidden: " + w.LogText);
            double hostSecond = (8200 + 5000 - firstMs) / 1000.0;
            Check(w.LogText.Contains($"marker {(int)Math.Round(hostSecond)}") || w.LogText.Contains($"marker {(int)Math.Floor(hostSecond)}"), $"the log lines are those of the host time of the cursor ({hostSecond:F1} s into the session): " + w.LogText);
            w.QuietLog = false;
            Check(w.LogText.Contains("TELEM t="), "un-hiding the status blocks shows them");
            w.QuietLog = true;
            w.SetCursor(-100); Check(w.CursorTime == w.Range.Min, "the cursor stays inside the data"); w.SetCursor(1e6); Check(w.CursorTime == w.Range.Max, "...at both ends");

            // Playback runs forward and stops at the end.
            w.SetCursor(w.Range.Min); w.TogglePlayForSelfTest(); Check(w.Playing, "playback starts");
            w.Advance(2.0); Check(Math.Abs(w.CursorTime - (w.Range.Min + 2.0)) < 1e-9 && w.Playing, "playback advances");
            w.Advance(1e6); Check(!w.Playing && w.CursorTime == w.Range.Max, "playback stops at the end");
            w.TogglePlayForSelfTest(); Check(w.Playing && w.CursorTime == w.Range.Min, "playing again from the end restarts"); w.PausePlayback();
            // One sample at a time, the keyboard, and looping.
            w.SetCursor(5.0); w.Step(1);
            double nextT = analysis.T.First(t => t > 5.0 + 1e-9), prevT = analysis.T.Last(t => t < nextT - 1e-9);
            Check(Math.Abs(w.CursorTime - nextT) < 1e-9, "stepping forward lands on the next sample");
            w.Step(-1); Check(Math.Abs(w.CursorTime - prevT) < 1e-9 && !w.Playing, "stepping back lands on the previous sample");
            w.SetCursor(w.Range.Min); w.Step(-1); Check(w.CursorTime == w.Range.Min, "stepping back at the start stays there");
            w.HandleKey(System.Windows.Input.Key.End); Check(w.CursorTime == w.Range.Max, "End jumps to the end");
            w.HandleKey(System.Windows.Input.Key.Home); Check(w.CursorTime == w.Range.Min, "Home jumps to the start");
            Check(w.HandleKey(System.Windows.Input.Key.Space) && w.Playing && w.HandleKey(System.Windows.Input.Key.Space) && !w.Playing && !w.HandleKey(System.Windows.Input.Key.A), "Space toggles playback; other keys are left alone");
            w.Loop = true; w.SetCursor(w.Range.Max - 0.01); w.TogglePlayForSelfTest(); w.Advance(1.0);
            Check(w.Playing && w.CursorTime == w.Range.Min, "with Loop on, playback wraps to the start instead of stopping"); w.PausePlayback(); w.Loop = false;
            // Pulling the cursor (a click on a chart) pauses.
            w.TogglePlayForSelfTest(); w.SetCursor(w.Range.Min); w.Plots[1].RaiseCursor(3.0);
            Check(!w.Playing && w.CursorTime == 3.0 && w.Plots.All(p => p.CursorX == 3.0), "clicking a chart pauses and moves every view to that time");

            // Two recordings laid over each other.
            w.SelectForSelfTest(0, 1);
            Check(w.ShownRuns == 2 && q.Series.Count == 12 && err.Series.Count == 6 && speed.Series.Count == 8 && q.Series[0].Name.StartsWith("A ") && q.Series[7].Name.StartsWith("B "), "comparison doubles the series and tags them A and B");
            Check(w.SummaryText_.Contains("B 相对 A") && w.Plane.Paths.Count == 6 && err.Notes.Count == 4 && err.Series.Any(x => x.Name.StartsWith("B−A")), "comparison summary and notes");
            Check(err.Series.First(x => x.Name.StartsWith("B−A")).Y.Length == err.Series.First(x => x.Name.StartsWith("B ")).Y.Length, "the difference curve has a point per sample of the second run");
            Check(w.Range.Max >= 10 && w.Plane.Arms.Count == 4, "both arms at the cursor");
            // The log follows the chosen run; the first by default.
            string logA = w.LogText; w.LogSourceIndex = 1; string logB = w.LogText; w.LogSourceIndex = 0;
            Check(logA != logB && (logA + logB).Contains("HOST marker") && w.LogText == logA, "the log panel follows the run chosen in its selector");
            // A session log that runs past midnight stays in order and can be searched on both sides of it.
            string midnight = Path.Combine(temp, "midnight.log");
            File.WriteAllLines(midnight, ["[23:59:58.000] HOST before", "[23:59:59.500] HOST just before", "[00:00:01.000] HOST just after", "[00:00:01.500] TIMING traj_lag_max_us=7 traj_slip_us=0", "[00:00:02.000] HOST later"]);
            var wrapped = LogIndex.Load(midnight)!;
            Check(wrapped.Seconds.Zip(wrapped.Seconds.Skip(1)).All(p => p.Second > p.First) && wrapped.Seconds[^1] > 86400, "log times keep increasing across midnight");
            Check(wrapped.Around(1.0, 2.0, 10).Count() == 4 && wrapped.Around(86399.0, 1.0, 10).Count() == 2, "lines are found on both sides of midnight, however the time of day is given");
            Check(wrapped.LastMatch("TIMING", 0.5, 25) is { } t1 && t1.Contains("lag_max_us=7") && wrapped.LastMatch("TIMING", 86398, 86410) is not null && wrapped.LastMatch("TIMING", 86398, 86399) is null, "a time window can start before midnight and end after it");
            // Language switch rewrites everything without losing the selection.
            Lang.Toggle();
            try
            {
                Check(w.ColumnHeaders[0] == "Started" && w.Title.Contains("Replay") && w.ShownRuns == 2 && w.SummaryText_.Contains("B vs A") && q.Series[0].Name == "A command" && w.Plane.Source.Contains("not a measured pen position"), "English: headers, summary, series and source note");
                Check(w.RowText(rectRow).Contains("Rectangle") || w.RowText(circleRow).Contains("Rectangle"), "English shape names in the list");
            }
            finally { Lang.Toggle(); }
            Check(w.ColumnHeaders[0] == "开始时间" && q.Series[0].Name.StartsWith("A 命令"), "and back");
            w.Reload2();
            Check(w.ListedRuns == 2 && w.ShownRuns == 2, "refreshing the list keeps the selection");
            // The figure export: the four charts and the plane in one image, as wide as both and as tall as the taller.
            w.Width = 1320; w.Height = 860; w.Show(); w.UpdateLayout();
            string figure = Path.Combine(temp, "figure.png");
            w.RenderImage(figure);
            var decoder = BitmapDecoder.Create(new Uri(figure), BitmapCreateOptions.None, BitmapCacheOption.OnLoad);
            Check(decoder.Frames[0].PixelWidth > w.Plane.ActualWidth + 300 && decoder.Frames[0].PixelHeight >= 700, "the exported figure holds the charts and the plane");
            w.Hide();
            Check(w.ColumnHeaders.Length == 12 && w.ExportImageText == "导出图片 PNG", "the export button is labelled");
            // A stop with nothing recording is saved under "faults": listed, readable, and its pre-stop telemetry and log are replayable.
            string faultDir = Path.Combine(root, "faults", "20261001-000000-000-traj_run"); Directory.CreateDirectory(faultDir);
            var stopAt = baseUtc.AddSeconds(25);
            File.WriteAllText(Path.Combine(faultDir, "fault.json"), JsonSerializer.Serialize(new { schema = 1, host_utc = stopAt, pre_fault_samples = 4, snapshot = new { Milliseconds = 50300, Phase = "traj_run", Reason = "trajectory timing overrun", Driver = "ok" } }));
            File.WriteAllLines(Path.Combine(faultDir, "pre_fault.csv"), [
                "host_utc,device_ms,epoch,seq,q1_command_deg,q1_encoder_deg,q1_error_deg,q2_command_deg,q2_encoder_deg,q2_error_deg,slip_motor_lsb,inputs_active_mask,slip2_motor_lsb",
                .. Enumerable.Range(0, 4).Select(k => $"{stopAt.AddMilliseconds(k * 100 - 400):O},{50000 + k * 100},5,1,{10 + k},{10 + k + 0.1:F1},0.1,0,0,0,0,0,0")], new UTF8Encoding(true));
            Check(ExperimentArchive.List(root).Runs.Count == 2, "stop-only bundles stay out of the plain list (the recent-runs panel)");
            w.Reload2();
            Check(w.ListedRuns == 3, "the stop bundle is listed with the recordings");
            int faultRow = Enumerable.Range(0, 3).First(k => w.RowText(k).Contains("|停机|"));
            Check(w.RowText(faultRow).Contains("停机记录（当时没有录像）") && w.RowText(faultRow).EndsWith("|⚠"), "its row: kind, status and the fault mark: " + w.RowText(faultRow));
            w.SelectForSelfTest(faultRow);
            Check(w.ShownRuns == 1 && q.Series.Count == 2 && q.Series.All(x => x.Name.StartsWith("停机前遥测")) && err.Series.Count == 1 && err.Marks.Any(m => m.Label.StartsWith("故障 trajectory timing overrun")), "a stop-only run shows its pre-stop curves and the stop mark");
            Check(w.SummaryText_.Contains("停机记录：停机时没有在录像") && w.SummaryText_.Contains("4 个样本") && w.SummaryText_.Contains("trajectory timing overrun"), "its summary: " + w.SummaryText_);
            w.SetCursor(w.Range.Min + 0.1);
            Check(w.LogText.Contains("HOST marker"), "the log around the cursor follows the pre-stop telemetry's host clock: " + w.LogText);
            // More than the limit: the first four selected are shown, and the summary says so.
            string extra1 = ExperimentArchiveTests.WriteSyntheticRun(root, DrawingShape.Line, 9000, settings.Replace("shape=2", "shape=0"), out _); Thread.Sleep(15);
            string extra2 = ExperimentArchiveTests.WriteSyntheticRun(root, DrawingShape.Line, 9500, settings.Replace("shape=2", "shape=0"), out _);
            w.Reload2();
            w.SelectForSelfTest(Enumerable.Range(0, w.ListedRuns).ToArray());
            Check(w.ListedRuns == 5 && w.ShownRuns == ExperimentWindow.MaxCompared && w.SummaryText_.Contains("最多比较四次") && w.SummaryText_.Contains("C 相对 A") && q.Series.Any(x => x.Name.StartsWith("D ")), "a fifth selection is not shown, and the others are tagged A to D: " + w.SummaryText_);
            w.SelectForSelfTest(0, 1);
            // Nothing recorded yet: a friendly empty state, not an error.
            var empty = new ExperimentWindow(Path.Combine(temp, "nothing"));
            Check(empty.ListedRuns == 0 && empty.SummaryText_.Contains("还没有实验记录") && empty.ShownRuns == 0, "an empty archive says so");

            if (snapshotPath is not null)
            {
                string dir = Path.GetDirectoryName(snapshotPath)!;
                void Snap(string name)
                {
                    w.Width = 1320; w.Height = 860; w.Show(); w.UpdateLayout(); foreach (var plot in w.Plots) plot.InvalidateVisual(); w.Plane.InvalidateVisual(); w.UpdateLayout();
                    var content = (FrameworkElement)w.Content;
                    var image = new RenderTargetBitmap((int)content.ActualWidth, (int)content.ActualHeight, 96, 96, PixelFormats.Pbgra32); image.Render(content);
                    var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(image));
                    using var stream = File.Create(Path.Combine(dir, name)); png.Save(stream); w.Hide();
                }
                int shotRect = Enumerable.Range(0, w.ListedRuns).First(k => w.RowText(k).Contains("|矩形|")), shotCircle = Enumerable.Range(0, w.ListedRuns).First(k => w.RowText(k).Contains("|圆|"));
                w.SelectForSelfTest(shotRect); w.SetCursor(9.0); Snap("experiment-replay.png");
                w.SelectForSelfTest(shotRect, shotCircle); w.SetCursor(12.0); Snap("experiment-compare.png");
            }
            w.Close();
        }
        finally { LogRootOverride = null; if (Directory.Exists(temp)) Directory.Delete(temp, true); }
    }
    internal void Reload2() => Reload();
    internal string ExportImageText => exportImage.Content?.ToString() ?? "";
}
