using System.IO;
using System.Text.Json;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace TwoLinkCommissioning;

public sealed partial class XYPreviewPanel
{
    internal static void FreehandSelfTest(string? snapshotPath = null)
    {
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Freehand UI: " + why); }
        var panel = new XYPreviewPanel();
        int requests = 0;
        panel.PointMoveRequested += _ => requests++; panel.TrajectoryRequested += _ => requests++; panel.ExecuteRequested += () => requests++;
        panel.shape.SelectedIndex = FreehandIndex;
        panel.Measure(new Size(1100, 740)); panel.Arrange(new Rect(0, 0, 1100, 740)); panel.UpdateLayout(); panel.RedrawViewport();
        var start = new Point(330, 0); var end = new Point(334, 0);
        Check(panel.BeginFreehandStroke(panel.ToScreen(start)), "first stroke");
        var plot = panel.Plot; var center = panel.cameraCenter; double zoom = panel.pixelsPerMm;
        panel.ZoomAt(panel.ToScreen(start), 2); panel.FitView(true);
        Check(panel.Plot == plot && panel.cameraCenter == center && panel.pixelsPerMm == zoom, "camera moved while recording");
        panel.AddFreehandPoint(panel.ToScreen(new Point(331, 0)));
        panel.CompleteFreehandStroke(panel.ToScreen(end));
        Check((panel.freehandPoints[^1].Point - end).Length < 1e-8 && panel.freehandStrokes.Count == 1, "terminal point lost");
        panel.RedrawViewport();
        Check(panel.BeginFreehandStroke(panel.ToScreen(end)), "record before capture loss");
        panel.AddFreehandPoint(panel.ToScreen(new Point(336, 0)));
        panel.canvas.RaiseEvent(new MouseEventArgs(Mouse.PrimaryDevice, Environment.TickCount) { RoutedEvent = Mouse.LostMouseCaptureEvent });
        Check(!panel.RecordingFreehand && panel.freehandPoints.Count == 3, "lost capture did not discard the unfinished stroke");
        Check(!panel.BeginFreehandStroke(panel.ToScreen(new Point(310, 0))) && panel.freehandPoints.Count == 3, "far continuation accepted");
        Check(panel.BeginFreehandStroke(panel.ToScreen(new Point(334.1, 0))), "near continuation rejected");
        Check((panel.freehandPoints[^1].Point - end).Length < 1e-8, "continuation must snap, not append a connector");
        panel.CompleteFreehandStroke(panel.ToScreen(new Point(338, 0)));
        Check(panel.freehandStrokes.Count == 2, "stroke boundaries");
        panel.Refresh();
        Check(panel.execution is not null && ReferenceEquals(panel.timing, panel.execution.Timing) && panel.trajectory!.Stops.Length == 2, "continuous execution plan: " + panel.result.Text);
        Check(panel.xRow.Visibility == Visibility.Collapsed && panel.transformSection.Visibility == Visibility.Collapsed, "hidden transforms");
        Check((panel.trajectory!.Samples[0].Target - start).Length < 1e-8 && (panel.trajectory.Samples[^1].Target - new Point(338, 0)).Length < 1e-8, "world coordinates changed");
        var original = panel.freehandPoints.ToArray();
        string saved = panel.SaveParameters();
        var savedRecord = JsonSerializer.Deserialize<SavedParameters>(saved)!;
        Check(panel.BeginFreehandStroke(panel.ToScreen(original[^1].Point)), "edit after preview");
        Check(panel.execution is null && !panel.run.IsEnabled && !panel.upload.IsEnabled && !panel.movePoint.IsEnabled, "stale plan retained");
        panel.AddFreehandPoint(panel.ToScreen(new Point(339, 0)));
        Check(panel.CancelFreehandOnEscape(), "Escape did not cancel recording");
        Check(panel.freehandPoints.SequenceEqual(original), "cancel lost committed points");
        panel.UndoFreehand(); Check((panel.freehandPoints[^1].Point - end).Length < 1e-8, "undo last stroke");
        panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = FreehandTrajectory.RefinementExample }));
        Check(panel.execution is not null && panel.trajectory!.Stops.Length == 2, "automatic corner refinement: " + panel.result.Text);
        Check(panel.freehandPoints.SequenceEqual(FreehandTrajectory.RefinementExample), "refinement rewrote the saved draft");
        var overshoot = Enumerable.Range(0, 81).Select(i => new FreehandPoint(330 + i * 0.4, 3 * Math.Sin(i * 0.2))).ToArray();
        panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = overshoot }));
        Check(panel.execution is not null && panel.freehandPoints.SequenceEqual(overshoot) && panel.result.Text.Contains(Lang.T("fh.repaired", 1.0).TrimStart()[..4]), "automatic repair: " + panel.result.Text);
        panel.RestoreParameters(saved);
        var large = savedRecord with { Freehand = Enumerable.Range(0, 5000).Select(i => new FreehandPoint(330 + i / 1000.0, 0)).ToArray() };
        string largeJson = JsonSerializer.Serialize(large);
        panel.RestoreParameters(largeJson);
        Check(panel.freehandPoints.Count == 5000 && panel.execution is not null, "maximum draft restore");
        Check(panel.BeginFreehandStroke(panel.ToScreen(panel.freehandPoints[^1].Point)), "record at point limit");
        panel.AddFreehandPoint(panel.ToScreen(new Point(336, 0)));
        Check(!panel.RecordingFreehand && panel.freehandPoints.Count == 5000 && panel.execution is null, "point-limit cancellation");
        panel.Refresh();
        string testRoot = Path.Combine(Path.GetTempPath(), "freehand-save-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(testRoot);
        try {
            string file = Path.Combine(testRoot, "parameters.json"); panel.WriteParameters(file);
            panel.ClearFreehand(); panel.ReadSavedParameters(file);
            Check(panel.freehandPoints.Count == 5000 && panel.execution is not null, "large file round trip");
        } finally { File.Delete(Path.Combine(testRoot, "parameters.json")); Directory.Delete(testRoot); }
        panel.RestoreParameters(saved);
        Check(panel.freehandPoints.SequenceEqual(original) && panel.freehandStrokes.SequenceEqual(new[] { 0 }) && panel.uploadedPlan is null, "save/restore");
        Check(panel.BeginFreehandStroke(panel.ToScreen(original[^1].Point)), "record before lock");
        panel.AddFreehandPoint(panel.ToScreen(new Point(339, 0))); panel.SetEditingAllowed(false);
        Check(!panel.RecordingFreehand && panel.freehandPoints.SequenceEqual(original) && !panel.freehandTool.IsEnabled && !panel.freehandUndo.IsEnabled, "motion lock");
        Check(!panel.BeginFreehandStroke(panel.ToScreen(end)), "locked drawing accepted"); panel.SetEditingAllowed(true);
        Check(panel.BeginFreehandStroke(panel.ToScreen(original[^1].Point)), "record before leaving plot");
        panel.AddFreehandPoint(new Point(panel.Plot.Right + 1, panel.Plot.Top));
        Check(!panel.RecordingFreehand && panel.freehandPoints.SequenceEqual(original), "outside cancellation");
        foreach (var bad in new[] { new[] { new FreehandPoint(double.PositiveInfinity, 0) }, new[] { new FreehandPoint(20001, 0) }, new FreehandPoint[5001] }) {
            try { panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = bad }, new JsonSerializerOptions { NumberHandling = System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals })); }
            catch (Exception ex) when (ex is ArgumentException or JsonException) { continue; }
            throw new InvalidOperationException("Freehand UI accepted invalid saved points");
        }
        string missingCoordinate = JsonSerializer.Serialize(new { savedRecord.Schema, savedRecord.Shape, savedRecord.Branch, savedRecord.Fields, savedRecord.Direction, Freehand = new[] { new { X = 330.0 } } });
        try { panel.RestoreParameters(missingCoordinate); throw new InvalidOperationException("Missing Y was replaced by zero"); }
        catch (JsonException) { }
        panel.RestoreParameters(saved);
        panel.freehandPoints.Add(new(340, 0)); panel.UpdateLamps();
        Check(panel.savedFields is null || panel.SnapshotFields()[24] != panel.savedFields[24], "draft absent from saved state");
        panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = null }));
        Check(panel.freehandPoints.Count == 0 && panel.execution is null, "old file inherited a draft");
        Check(RunValues("shape=11; v1=5; v2=3; a1=10; a2=6") is null, "recent run reloaded the wrong drawing");
        panel.LoadExample(); Check(panel.execution is not null, "smooth example: " + panel.result.Text);
        string snapshot = panel.SaveParameters();
        foreach (bool english in new[] { true, false }) {
            Lang.Set(english, false); panel.RefreshLanguage();
            Check(panel.SaveParameters() == snapshot && Equals(panel.freehandTool.Content, Lang.T("fh.draw")), "language changed draft");
            if (snapshotPath is not null) {
                string directory = Path.GetDirectoryName(Path.GetFullPath(snapshotPath))!; Directory.CreateDirectory(directory);
                panel.Measure(new Size(1100, 740)); panel.Arrange(new Rect(0, 0, 1100, 740)); panel.UpdateLayout(); panel.RedrawViewport(); panel.UpdateLayout();
                Check(panel.execution is not null && !panel.parametersDirty && panel.canvas.Children.Count > 20, "snapshot lost plan: " + panel.result.Text);
                var bitmap = new RenderTargetBitmap(1100, 740, 96, 96, PixelFormats.Pbgra32); bitmap.Render(panel);
                var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(bitmap));
                using var file = File.Create(Path.Combine(directory, $"freehand-{(english ? "en" : "zh")}.png")); png.Save(file);
            }
        }
        Check(requests == 0, "editing issued a motion command");
        panel.ClearFreehand(); Check(panel.freehandPoints.Count == 0 && panel.execution is null, "clear");
        if (snapshotPath is not null)
            File.WriteAllText(Path.Combine(Path.GetDirectoryName(Path.GetFullPath(snapshotPath))!, "freehand-tests.txt"), "PASS: automatic corner/reversal refinement within the deviation budget, analytic spline and rate limits, unchanged draft, capture/resume/undo/cancel/lock, plan invalidation, save/restore validation, old files, language, unchanged motion queues.\n");
        panel.viewportRedrawTimer.Stop();
    }
}
