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
            // The log of that session: a marker line every second, and status-block noise between them.
            var baseLocal = baseUtc.ToLocalTime();
            var log = new List<string>();
            for (int s = -5; s < 90; s++)
            {
                var at = baseLocal.AddSeconds(s);
                log.Add($"[{at:HH:mm:ss.fff}] HOST marker {s}");
                log.Add($"[{at.AddMilliseconds(500):HH:mm:ss.fff}] TELEM t=1 epoch=5 seq=1 ref=1 source=2 p1=0 p2=0 e1=0 err=0 inputs=0 drops=0");
            }
            string logFile = Path.Combine(logs, $"stage1-{baseLocal.AddMinutes(-1):yyyyMMdd-HHmmss}-000.log");
            File.WriteAllLines(logFile, log); File.SetLastWriteTime(logFile, DateTime.Now);
            // A log of another day must not be picked.
            File.WriteAllText(Path.Combine(logs, "stage1-20000101-000000-000.log"), "[00:00:00.000] HOST old\n");

            bool zh = !Lang.IsEn;
            Check(zh, "the test runs in Chinese");
            var w = new ExperimentWindow(root);
            Check(w.ListedRuns == 2 && w.ColumnHeaders.Length == 10 && w.ColumnHeaders[0] == "开始时间" && w.ColumnHeaders[6] == "RMSE °", "list and localized headers");
            int rectRow = Enumerable.Range(0, 2).First(i => w.RowText(i).Contains("矩形")), circleRow = 1 - rectRow;
            string rowText = w.RowText(rectRow);
            Check(rowText.Contains("|图形|矩形|5 / 3|人工伸直零位|完整|") && rowText.Contains("10 Hz") && rowText.EndsWith("|⚠") && w.RowText(circleRow).Contains("|圆|4 / 3|") && !w.RowText(circleRow).EndsWith("⚠"), "row facts: shape, speeds, reference, outcome, telemetry rate and the fault mark: " + rowText);
            Check(w.ShownRuns == 0 && w.Plots.All(p => p.Series.Count == 0), "nothing is shown until something is selected");

            // One recording: the curves, the events, the written summary.
            w.SelectForSelfTest(rectRow);
            var (q, err, speed, accel) = (w.Plots[0], w.Plots[1], w.Plots[2], w.Plots[3]);
            Check(w.ShownRuns == 1 && q.Series.Count == 3 && err.Series.Count == 1 && speed.Series.Count == 2 && accel.Series.Count == 2, "series per chart");
            Check(err.Marks.Count(m => m.Label is "起步" or "停机" or "拐角 1" or "拐角 2" or "拐角 3") == 5 && err.Marks.Any(m => m.Label.StartsWith("故障 ")), "the plan's events and the stop snapshot are marked on the error chart");
            Check(Math.Abs(err.Marks.First(m => m.Label.StartsWith("故障")).X - 12.0) < 0.02, "the stop snapshot is placed at its device time on the aligned axis");
            Check(speed.Lines.Count == 4 && speed.Lines.Any(l => l.Label.StartsWith("电机1 上限")) && accel.Lines.Count == 4, "the caps are drawn as reference lines");
            Check(w.Plots.All(p => p.XRange.Min <= 0.001 && p.XRange.Max > 10) && w.Range.Max > 10, "one shared time range");
            Check(err.SymmetricY && q.Source.Contains("40:1") && err.Source.Contains("不是笔尖位置误差") && speed.Source.Contains("不是实测速度") && w.Plane.Source.Contains("不是实测笔尖位置"), "every chart states where its data comes from, and what it is not");
            Check(w.Plane.Paths.Count == 3 && w.Plane.Paths[0].Name == "规划轨迹（模型）" && w.Plane.Paths.Any(p => p.Name == "混合反馈估算"), "plane: plan, command and hybrid paths");
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
            // Pulling the cursor (a click on a chart) pauses.
            w.TogglePlayForSelfTest(); w.SetCursor(w.Range.Min); w.Plots[1].RaiseCursor(3.0);
            Check(!w.Playing && w.CursorTime == 3.0 && w.Plots.All(p => p.CursorX == 3.0), "clicking a chart pauses and moves every view to that time");

            // Two recordings laid over each other.
            w.SelectForSelfTest(0, 1);
            Check(w.ShownRuns == 2 && q.Series.Count == 6 && err.Series.Count == 2 && speed.Series.Count == 4 && q.Series[0].Name.StartsWith("A ") && q.Series[3].Name.StartsWith("B "), "comparison doubles the series and tags them A and B");
            Check(w.SummaryText_.Contains("B 相对 A") && w.Plane.Paths.Count == 6 && err.Notes.Count == 2, "comparison summary and notes");
            Check(w.Range.Max >= 10 && w.Plane.Arms.Count == 4, "both arms at the cursor");
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
                w.SelectForSelfTest(rectRow); w.SetCursor(9.0); Snap("experiment-replay.png");
                w.SelectForSelfTest(0, 1); w.SetCursor(12.0); Snap("experiment-compare.png");
            }
            w.Close();
        }
        finally { LogRootOverride = null; if (Directory.Exists(temp)) Directory.Delete(temp, true); }
    }
    internal void Reload2() => Reload();
}
