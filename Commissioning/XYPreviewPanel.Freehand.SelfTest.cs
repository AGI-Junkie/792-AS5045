using System.IO;
using System.Diagnostics;
using System.Text.Json;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

public sealed partial class XYPreviewPanel
{
    // Only the offline self-test pumps the dispatcher. Product actions use the asynchronous request completion.
    private static (int Pulses, double MaxGap, double Milliseconds) WaitForFreehand(XYPreviewPanel panel, Action? pulse = null, Func<bool>? finished = null)
    {
        if (!panel.ProcessingFreehand && (finished?.Invoke() ?? true)) return default;
        var clock = Stopwatch.StartNew(); double previous = 0, maxGap = 0; int pulses = 0;
        var frame = new DispatcherFrame();
        var timer = new DispatcherTimer(DispatcherPriority.Normal, panel.Dispatcher) { Interval = TimeSpan.FromMilliseconds(10) };
        timer.Tick += (_, _) => {
            double now = clock.Elapsed.TotalMilliseconds; maxGap = Math.Max(maxGap, now - previous); previous = now; ++pulses;
            pulse?.Invoke();
            if (!panel.ProcessingFreehand && (finished?.Invoke() ?? true) || now > 15000) frame.Continue = false;
        };
        timer.Start();
        try { Dispatcher.PushFrame(frame); } finally { timer.Stop(); }
        if (panel.ProcessingFreehand || !(finished?.Invoke() ?? true)) throw new InvalidOperationException("Freehand worker/action did not finish");
        return (pulses, maxGap, clock.Elapsed.TotalMilliseconds);
    }

    internal static void FreehandSelfTest(string? snapshotPath = null)
    {
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Freehand UI: " + why); }
        var window = new ControlWindow();
        var hosted = (XYPreviewPanel)typeof(ControlWindow).GetField("xyPreview", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)!.GetValue(window)!;
        Check(hosted.freehandTool.IsEnabled, "disconnected draw button disabled");
        hosted.freehandTool.RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Button.ClickEvent));
        hosted.Measure(new Size(1100, 740)); hosted.Arrange(new Rect(0, 0, 1100, 740)); hosted.UpdateLayout(); hosted.RedrawViewport();
        Check(hosted.BeginFreehandStroke(hosted.ToScreen(new Point(330, 0))), "disconnected drawing blocked");
        hosted.CompleteFreehandStroke(hosted.ToScreen(new Point(334, 0))); WaitForFreehand(hosted); hosted.Refresh(); WaitForFreehand(hosted);
        hosted.freehandTool.RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Button.ClickEvent));
        Check(hosted.freehandMode, "draw button turned off drawing for the already-selected freehand shape");
        Check(hosted.execution is not null && !hosted.movePoint.IsEnabled && !hosted.upload.IsEnabled && !hosted.run.IsEnabled, "offline preview opened hardware commands");
        var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
        var updateControls = typeof(ControlWindow).GetMethod("UpdateControls", flags)!;
        foreach (string busyField in new[] { "homing", "executing" }) {
            var field = typeof(ControlWindow).GetField(busyField, flags)!;
            field.SetValue(window, true); updateControls.Invoke(window, [true]);
            Check(!hosted.freehandTool.IsEnabled, "hardware busy gate lost: " + busyField);
            field.SetValue(window, false); updateControls.Invoke(window, [true]);
        }
        Check(hosted.freehandTool.IsEnabled, "offline editing did not recover after busy lock");
        window.Close(); hosted.viewportRedrawTimer.Stop();
        var panel = new XYPreviewPanel();
        Check(panel.freehandSmoothing.Visibility == Visibility.Collapsed && panel.freehandSmoothingLabel.Visibility == Visibility.Collapsed, "smoothing visible for non-freehand shape");
        int requests = 0;
        panel.PointMoveRequested += _ => requests++; panel.TrajectoryRequested += _ => requests++; panel.ExecuteRequested += () => requests++;
        Check(panel.freehandSmoothing.SelectedIndex == 2, "default smoothing");
        panel.freehandSmoothing.SelectedIndex = 0; WaitForFreehand(panel); // Existing capture checks also certify exact legacy Off behavior.
        foreach (int selectedShape in Enumerable.Range(0, panel.shape.Items.Count)) {
            panel.shape.SelectedIndex = selectedShape;
            var visible = selectedShape == FreehandIndex ? Visibility.Visible : Visibility.Collapsed;
            Check(panel.freehandSmoothing.Visibility == visible && panel.freehandSmoothingLabel.Visibility == visible, "shape-specific smoothing visibility");
        }
        panel.shape.SelectedIndex = FreehandIndex;
        panel.Measure(new Size(1100, 740)); panel.Arrange(new Rect(0, 0, 1100, 740)); panel.UpdateLayout(); panel.RedrawViewport();
        var start = new Point(330, 0); var end = new Point(334, 0);
        Check(panel.BeginFreehandStroke(panel.ToScreen(start)), "first stroke");
        var plot = panel.Plot; var center = panel.cameraCenter; double zoom = panel.pixelsPerMm;
        panel.ZoomAt(panel.ToScreen(start), 2); panel.FitView(true);
        Check(panel.Plot == plot && panel.cameraCenter == center && panel.pixelsPerMm == zoom, "camera moved while recording");
        panel.AddFreehandPoint(panel.ToScreen(new Point(331, 0)));
        panel.CompleteFreehandStroke(panel.ToScreen(end)); WaitForFreehand(panel);
        Check((panel.freehandPoints[^1].Point - end).Length < 1e-8 && panel.freehandStrokes.Count == 1, "terminal point lost");
        panel.RedrawViewport();
        Check(panel.BeginFreehandStroke(panel.ToScreen(end)), "record before capture loss");
        panel.AddFreehandPoint(panel.ToScreen(new Point(336, 0)));
        panel.canvas.RaiseEvent(new MouseEventArgs(Mouse.PrimaryDevice, Environment.TickCount) { RoutedEvent = Mouse.LostMouseCaptureEvent });
        Check(!panel.RecordingFreehand && panel.freehandPoints.Count == 3, "lost capture did not discard the unfinished stroke");
        Check(!panel.BeginFreehandStroke(panel.ToScreen(new Point(310, 0))) && panel.freehandPoints.Count == 3, "far continuation accepted");
        Check(panel.BeginFreehandStroke(panel.ToScreen(new Point(334.1, 0))), "near continuation rejected");
        Check((panel.freehandPoints[^1].Point - end).Length < 1e-8, "continuation must snap, not append a connector");
        panel.CompleteFreehandStroke(panel.ToScreen(new Point(338, 0))); WaitForFreehand(panel);
        Check(panel.freehandStrokes.Count == 2, "stroke boundaries");
        panel.Refresh(); WaitForFreehand(panel);
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
        panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = FreehandTrajectory.RefinementExample })); WaitForFreehand(panel);
        Check(panel.execution is not null && panel.trajectory!.Stops.Length == 2, "automatic corner refinement: " + panel.result.Text);
        Check(panel.freehandPoints.SequenceEqual(FreehandTrajectory.RefinementExample), "refinement rewrote the saved draft");
        var overshoot = Enumerable.Range(0, 81).Select(i => new FreehandPoint(330 + i * 0.4, 3 * Math.Sin(i * 0.2))).ToArray();
        panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = overshoot })); WaitForFreehand(panel);
        Check(panel.execution is not null && panel.freehandPoints.SequenceEqual(overshoot) && panel.result.Text.Contains(Lang.T("fh.repaired", 1.0).TrimStart()[..4]), "automatic repair: " + panel.result.Text);
        panel.RestoreParameters(saved); WaitForFreehand(panel);
        var large = savedRecord with { Freehand = Enumerable.Range(0, 5000).Select(i => new FreehandPoint(330 + i / 1000.0, 0)).ToArray() };
        string largeJson = JsonSerializer.Serialize(large);
        panel.RestoreParameters(largeJson); WaitForFreehand(panel);
        Check(panel.freehandPoints.Count == 5000 && panel.execution is not null, "maximum draft restore");
        Check(panel.BeginFreehandStroke(panel.ToScreen(panel.freehandPoints[^1].Point)), "record at point limit");
        panel.AddFreehandPoint(panel.ToScreen(new Point(336, 0)));
        Check(!panel.RecordingFreehand && panel.freehandPoints.Count == 5000 && panel.execution is null, "point-limit cancellation");
        panel.Refresh(); WaitForFreehand(panel);
        string testRoot = Path.Combine(Path.GetTempPath(), "freehand-save-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(testRoot);
        try {
            string file = Path.Combine(testRoot, "parameters.json"); panel.WriteParameters(file);
            panel.ClearFreehand(); panel.ReadSavedParameters(file); WaitForFreehand(panel);
            Check(panel.freehandPoints.Count == 5000 && panel.execution is not null, "large file round trip");
        } finally { File.Delete(Path.Combine(testRoot, "parameters.json")); Directory.Delete(testRoot); }
        panel.RestoreParameters(saved); WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(original) && panel.freehandStrokes.SequenceEqual(new[] { 0 }) && panel.uploadedPlan is null, "save/restore");
        Check(panel.BeginFreehandStroke(panel.ToScreen(original[^1].Point)), "record before lock");
        panel.AddFreehandPoint(panel.ToScreen(new Point(339, 0))); panel.SetEditingAllowed(false);
        Check(!panel.RecordingFreehand && panel.freehandPoints.SequenceEqual(original) && !panel.freehandTool.IsEnabled && !panel.freehandUndo.IsEnabled, "motion lock");
        Check(!panel.BeginFreehandStroke(panel.ToScreen(end)), "locked drawing accepted"); panel.SetEditingAllowed(true);
        Check(panel.BeginFreehandStroke(panel.ToScreen(original[^1].Point)), "record before leaving plot");
        panel.AddFreehandPoint(new Point(panel.Plot.Right + 1, panel.Plot.Top));
        Check(!panel.RecordingFreehand && panel.freehandPoints.SequenceEqual(original), "outside cancellation");
        foreach (var bad in new[] { new[] { new FreehandPoint(double.PositiveInfinity, 0) }, new[] { new FreehandPoint(20001, 0) }, new FreehandPoint[5001] }) {
            try { panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = bad }, new JsonSerializerOptions { NumberHandling = System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals })); WaitForFreehand(panel); }
            catch (Exception ex) when (ex is ArgumentException or JsonException) { continue; }
            throw new InvalidOperationException("Freehand UI accepted invalid saved points");
        }
        string missingCoordinate = JsonSerializer.Serialize(new { savedRecord.Schema, savedRecord.Shape, savedRecord.Branch, savedRecord.Fields, savedRecord.Direction, Freehand = new[] { new { X = 330.0 } } });
        try { panel.RestoreParameters(missingCoordinate); WaitForFreehand(panel); throw new InvalidOperationException("Missing Y was replaced by zero"); }
        catch (JsonException) { }
        panel.RestoreParameters(saved); WaitForFreehand(panel);
        panel.freehandPoints.Add(new(340, 0)); panel.UpdateLamps();
        Check(panel.savedFields is null || panel.SnapshotFields()[24] != panel.savedFields[24], "draft absent from saved state");
        panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = null })); WaitForFreehand(panel);
        Check(panel.freehandPoints.Count == 0 && panel.execution is null, "old file inherited a draft");
        Check(RunValues("shape=11; v1=5; v2=3; a1=10; a2=6") is null, "recent run reloaded the wrong drawing");
        panel.ClearFreehand(); panel.RedrawViewport(); panel.freehandSmoothing.SelectedIndex = 2; WaitForFreehand(panel);
        panel.cameraCenter.Y = 1e-12; // Exercise screen/world roundoff while keeping exact captured-point comparisons.
        Check(panel.BeginFreehandStroke(panel.ToScreen(start)), "smoothed first stroke");
        for (int i = 1; i < 80; i++) panel.AddFreehandPoint(panel.ToScreen(new Point(330 + i * 0.1, 0.4 * Math.Sin(i * 1.7))));
        var captured = panel.freehandPoints.ToArray();
        var releaseScreen = panel.ToScreen(new Point(338, 0)); var releaseWorld = panel.ToWorld(releaseScreen);
        var baseline = captured.Append(new FreehandPoint(releaseWorld.X, releaseWorld.Y)).ToArray();
        panel.CompleteFreehandStroke(releaseScreen); WaitForFreehand(panel);
        Check(panel.freehandRawStrokes.Single().SequenceEqual(baseline) && (releaseWorld - new Point(338, 0)).Length < 1e-8, "release changed captured baseline or endpoint");
        Check(panel.freehandPoints.Count < captured.Length && panel.FreehandFitLevel == 2, "release did not process the stored tail");
        var committed = panel.freehandPoints.ToArray();
        panel.Refresh(); WaitForFreehand(panel); var oldPreview = panel.trajectory;
        panel.freehandSmoothing.SelectedIndex = 1; WaitForFreehand(panel);
        var low = panel.freehandPoints.ToArray();
        Check(panel.FreehandFitLevel == 1 && panel.execution is not null && !ReferenceEquals(oldPreview, panel.trajectory), "selection did not immediately rebuild draft and preview");
        panel.freehandSmoothing.SelectedIndex = 3; WaitForFreehand(panel);
        Check(!panel.freehandPoints.SequenceEqual(low), "high did not change existing draft");
        panel.freehandSmoothing.SelectedIndex = 1; WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(low), "Low-High-Low accumulated filtering");
        panel.freehandSmoothing.SelectedIndex = 0; WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(baseline), "Off did not restore captured baseline");
        panel.freehandSmoothing.SelectedIndex = 2; WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(committed), "Medium did not restore original processed draft");
        panel.RedrawViewport();
        Check(panel.BeginFreehandStroke(panel.ToScreen(committed[^1].Point)), "smoothed continuation");
        for (int i = 1; i < 40; i++) panel.AddFreehandPoint(panel.ToScreen(new Point(338 + i * 0.1, 0.4 * Math.Sin(i * 1.7))));
        panel.CompleteFreehandStroke(panel.ToScreen(new Point(342, 0))); WaitForFreehand(panel);
        Check(panel.freehandPoints.Take(committed.Length).SequenceEqual(committed) && panel.freehandStrokes[^1] == committed.Length, "continuation rewrote prefix/shared endpoint");
        var continuedRaw = panel.freehandRawStrokes.Select(s => s.ToArray()).ToArray();
        panel.freehandSmoothing.SelectedIndex = 0; WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(continuedRaw[0].Concat(continuedRaw[1].Skip(1))), "continued Off baseline");
        panel.freehandSmoothing.SelectedIndex = 1; WaitForFreehand(panel);
        Check(panel.freehandStrokes[^1] == FreehandTrajectory.Smooth(continuedRaw[0], 1).Length, "reprocessed undo boundary");
        var beforeCancel = panel.freehandPoints.ToArray();
        Check(panel.BeginFreehandStroke(panel.ToScreen(beforeCancel[^1].Point)), "reprocessed continuation start");
        panel.AddFreehandPoint(panel.ToScreen(new Point(343, 0))); panel.CancelFreehandStroke();
        Check(panel.freehandPoints.SequenceEqual(beforeCancel) && panel.freehandRawStrokes.Count == 2, "reprocessed cancellation changed baseline");
        panel.freehandSmoothing.SelectedIndex = 2; WaitForFreehand(panel);
        var languagePreview = panel.trajectory; var languageDraft = panel.freehandPoints.ToArray();
        Lang.Set(true, false); panel.RefreshLanguage(); Lang.Set(false, false); panel.RefreshLanguage();
        Check(ReferenceEquals(languagePreview, panel.trajectory) && panel.freehandPoints.SequenceEqual(languageDraft) && panel.FreehandFitLevel == 2, "language switch resmoothed/replanned");
        panel.UndoFreehand(); Check(panel.freehandPoints.SequenceEqual(committed) && panel.FreehandFitLevel == 2, "smoothed undo");
        string processedJson = panel.SaveParameters();
        panel.RestoreParameters(processedJson); WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(committed) && panel.FreehandFitLevel == 0, "processed serialization/legacy fit");
        panel.freehandSmoothing.SelectedIndex = 1; WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(FreehandTrajectory.Smooth(committed, 1)) && panel.FreehandFitLevel == 1 && panel.execution is not null, "loaded baseline not reprocessed on explicit selection");
        panel.freehandSmoothing.SelectedIndex = 0; WaitForFreehand(panel);
        Check(panel.freehandPoints.SequenceEqual(committed), "loaded Off did not restore file baseline");
        var longDraft = Enumerable.Range(0, FreehandTrajectory.MaxPoints)
            .Select(i => new FreehandPoint(335 + 10 * Math.Cos(i / 39.0), 10 * Math.Sin(i / 39.0))).ToArray();
        panel.RestoreParameters(JsonSerializer.Serialize(savedRecord with { Freehand = longDraft })); WaitForFreehand(panel);
        panel.freehandSmoothing.SelectedIndex = 2; WaitForFreehand(panel);
        Check(panel.execution is not null && panel.freehandPoints.Count < longDraft.Length &&
            panel.freehandPoints[0] == longDraft[0] && panel.freehandPoints[^1] == longDraft[^1], "long captured stroke lost its checked plan/endpoints");
        panel.l1.Text = "179";
        Check(panel.execution is null && panel.trajectory is null, "parameter edit retained the previous long-stroke plan");
        panel.Refresh(); WaitForFreehand(panel);
        Check(panel.execution is not null && panel.trajectory!.Model.L1 == 179, "long-stroke refresh used the previous geometry");
        panel.RestoreParameters(saved); WaitForFreehand(panel);
        AsyncFreehandSelfTest(panel, savedRecord, snapshotPath);
        RejectionDiagnosticSelfTest(panel, savedRecord, snapshotPath);
        panel.LoadExample(); WaitForFreehand(panel); Check(panel.execution is not null, "smooth example: " + panel.result.Text);
        string snapshot = panel.SaveParameters();
        foreach (bool english in new[] { true, false }) {
            Lang.Set(english, false); panel.RefreshLanguage();
            Check(panel.SaveParameters() == snapshot && Equals(panel.freehandTool.Content, Lang.T("fh.draw")) && Equals(panel.freehandSmoothing.Items[2], Lang.T("fh.medium")) && System.Windows.Automation.AutomationProperties.GetName(panel.freehandSmoothing) == Lang.T("fh.smoothing"), "language changed draft");
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
        panel.ClearFreehand(); Check(panel.freehandPoints.Count == 0 && panel.freehandRawStrokes.Count == 0 && panel.execution is null, "clear");
        if (snapshotPath is not null)
            File.WriteAllText(Path.Combine(Path.GetDirectoryName(Path.GetFullPath(snapshotPath))!, "freehand-tests.txt"), "PASS: automatic corner/reversal refinement within the deviation budget, analytic spline and rate limits, release smoothing and total budgets, Off unchanged, processed serialization, immediate baseline recalculation, repeat levels and Off restore, disconnected drawing, shape visibility, shared endpoint, capture/resume/undo/cancel/lock, plan invalidation, save/restore validation, old files, language, exact range-failure markers, q1/q2/elbow minimum/maximum and overrun diagnostics, between-sample failure, failed-repair origin, full wrapped details, stale-marker clearing, unchanged motion queues.\n");
        panel.viewportRedrawTimer.Stop();
    }

    private static void AsyncFreehandSelfTest(XYPreviewPanel panel, SavedParameters saved, string? snapshotPath)
    {
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Async freehand: " + why); }
        static double Triangle(double t) => 1 - 4 * Math.Abs(t - Math.Floor(t) - 0.5);
        var draft = Enumerable.Range(0, FreehandTrajectory.MaxPoints)
            .Select(i => new FreehandPoint(335 + 3 * Triangle(i / 40.0), 3 * Triangle(i / 40.0 + 0.25))).ToArray();
        panel.RestoreParameters(JsonSerializer.Serialize(saved with { Freehand = draft })); WaitForFreehand(panel);
        panel.freehandSmoothing.SelectedIndex = 0; WaitForFreehand(panel);
        panel.freehandSmoothing.SelectedIndex = 2;
        Check(panel.ProcessingFreehand && panel.execution is null && panel.pointTarget is null && !panel.upload.IsEnabled && !panel.movePoint.IsEnabled && !panel.saveButton!.IsEnabled,
            "pending request retained motion/save gates");
        try { panel.SaveParameters(); throw new InvalidOperationException("pending raw points were serialized"); }
        catch (Lang.ValidationException ex) { Check(ex.Key == "fh.processing", "wrong pending-save error"); }
        int version = panel.freehandWorkVersion;
        var pulses = WaitForFreehand(panel, () => {
            panel.SetEditingAllowed(true);
            Check(panel.freehandWorkVersion == version, "same-state device polling invalidated the worker");
        });
        Check(panel.execution is not null && pulses.Pulses >= 2 && pulses.MaxGap < 200, "dispatcher heartbeat stalled: " + pulses.MaxGap);
        var expected = FreehandTrajectory.Smooth(draft, 2);
        Check(panel.freehandPoints.SequenceEqual(expected), "background smoothing changed the captured stroke");
        var fixedDraft = FreehandTrajectory.Repair(expected, panel.Model, panel.PreviewLimits(), 0, panel.RequestedRates, 2);
        var expectedPlan = new ExecutionTrajectory(new TimedTrajectory(DrawingTrajectory.Create(panel.Model, DrawingShape.Freehand, 0, 0, 0, 0, 0,
            panel.PreviewLimits(), new ShapeOptions { FreehandPoints = fixedDraft.Points, FreehandFitLevel = 2 }), panel.RequestedRates), panel.PreviewLimits());
        Check(panel.execution!.Nodes.SequenceEqual(expectedPlan.Nodes), "background execution changed the checked nodes");
        panel.Refresh(); var old = panel.freehandRequest!; panel.l1.Text = "179";
        WaitForFreehand(panel);
        Check(!old.GetAwaiter().GetResult() && panel.execution is null && panel.trajectory is null, "late plan survived a geometry edit");
        panel.Refresh(); WaitForFreehand(panel);
        Check(panel.execution is not null && panel.trajectory!.Model.L1 == 179, "new geometry was not used");
        panel.freehandSmoothing.SelectedIndex = 3; var replaced = panel.freehandRequest!;
        panel.freehandSmoothing.SelectedIndex = 1;
        Lang.Set(true, false); panel.RefreshLanguage();
        Check(panel.result.Text == Lang.T("fh.processing"), "pending text did not switch language");
        WaitForFreehand(panel);
        Check(!replaced.GetAwaiter().GetResult() && panel.FreehandFitLevel == 1 && panel.freehandPoints.SequenceEqual(FreehandTrajectory.Smooth(draft, 1)), "older fit overwrote the latest request");
        Lang.Set(false, false); panel.RefreshLanguage();
        panel.freehandSmoothing.SelectedIndex = 2;
        panel.shape.SelectedIndex = 3; panel.Refresh(); var otherPlan = panel.execution; string otherText = panel.result.Text;
        WaitForFreehand(panel);
        Check(otherPlan is not null && ReferenceEquals(otherPlan, panel.execution) && panel.result.Text == otherText, "late draft processing replaced another shape");
        panel.RestoreParameters(JsonSerializer.Serialize(saved with { Freehand = draft })); WaitForFreehand(panel);
        panel.freehandSmoothing.SelectedIndex = 1; panel.SetEditingAllowed(false); int lockedVersion = panel.freehandWorkVersion;
        panel.SetEditingAllowed(false); Check(panel.freehandWorkVersion == lockedVersion, "same-state lock repeatedly canceled work");
        panel.SetEditingAllowed(true); WaitForFreehand(panel);
        Check(!panel.freehandNeedsProcessing && panel.freehandPoints.SequenceEqual(FreehandTrajectory.Smooth(draft, 1)), "editing resume lost the original draft");
        string root = Path.Combine(Path.GetTempPath(), "freehand-async-" + Guid.NewGuid().ToString("N")); Directory.CreateDirectory(root);
        int saves = 0; void Saved() => ++saves; panel.ParametersSaved += Saved;
        try {
            string file = Path.Combine(root, "parameters.json");
            panel.SaveCurrentParameters(file); panel.ClearFreehand(); WaitForFreehand(panel);
            Check(!File.Exists(file) && saves == 0 && !panel.freehandNeedsProcessing, "canceled save wrote an old draft");
            panel.freehandSmoothing.SelectedIndex = 2; WaitForFreehand(panel);
            Check(!panel.ProcessingFreehand && !panel.freehandNeedsProcessing && JsonSerializer.Deserialize<SavedParameters>(panel.SaveParameters())!.Freehand!.Length == 0,
                "empty draft stayed permanently busy");
            panel.RestoreParameters(JsonSerializer.Serialize(saved with { Freehand = [new(500, 0), new(502, 0)] })); WaitForFreehand(panel);
            Check(panel.execution is null && panel.result.Text.Length > 0, "failed worker retained an executable plan");
            string failure = panel.result.Text; Lang.Set(true, false); panel.RefreshLanguage();
            Check(panel.result.Text != failure, "background validation error did not switch language");
            Lang.Set(false, false); panel.RefreshLanguage();
            panel.RestoreParameters(JsonSerializer.Serialize(saved)); WaitForFreehand(panel);
            Check(panel.execution is not null, "failed worker prevented a later valid preview");
            panel.SaveCurrentParameters(file); WaitForFreehand(panel, finished: () => File.Exists(file) && saves == 1);
            Check(File.Exists(file) && saves == 1 && JsonSerializer.Deserialize<SavedParameters>(File.ReadAllText(file))!.Freehand!.SequenceEqual(panel.freehandPoints),
                "completed save did not write exactly the processed points");
            File.Delete(file);
            panel.SaveCurrentParameters(file); panel.RaiseEvent(new RoutedEventArgs(UnloadedEvent)); WaitForFreehand(panel);
            Check(!File.Exists(file) && saves == 1, "unloaded request completed a save");
        } finally { panel.ParametersSaved -= Saved; if (File.Exists(Path.Combine(root, "parameters.json"))) File.Delete(Path.Combine(root, "parameters.json")); Directory.Delete(root); }
        panel.RestoreParameters(JsonSerializer.Serialize(saved)); WaitForFreehand(panel);
        if (snapshotPath is not null) {
            string directory = Path.GetDirectoryName(Path.GetFullPath(snapshotPath))!; Directory.CreateDirectory(directory);
            File.WriteAllText(Path.Combine(directory, "freehand-async-tests.txt"),
                $"PASS: 5000-point crossed stroke, dispatcher heartbeat, same-state device polling, newest geometry/fit, shape switch, lock/resume, pending/canceled save, empty draft, background error localization and recovery, unload. pulses={pulses.Pulses}, maxGapMs={pulses.MaxGap:F1}, elapsedMs={pulses.Milliseconds:F1}\n");
        }
    }

    private static void RejectionDiagnosticSelfTest(XYPreviewPanel panel, SavedParameters saved, string? snapshotPath)
    {
        static void Check(bool ok, string why) { if (!ok) throw new InvalidOperationException("Rejection diagnostic: " + why); }
        var model = panel.Model;
        // Exercise real throw sites, both bounds of each axis, without parsing translated exceptions.
        var probe = model.Inverse(200, 0)[0];
        var wide = new JointLimits(-180, 180, -180, 180, -180, 180);
        var oldLanguage = Lang.IsEn;
        foreach (bool english in new[] { true, false }) {
            Lang.Set(english, false);
            foreach (int axis in new[] { 0, 1, 2 })
            foreach (bool below in new[] { true, false }) {
                double value = axis == 0 ? probe.Q1 : axis == 1 ? probe.Q2 : JointLimits.Elbow(probe);
                double bound = value + (below ? 0.5 : -0.5);
                var limits = axis switch {
                    0 => below ? wide with { Min1 = bound } : wide with { Max1 = bound },
                    1 => below ? wide with { Min2 = bound } : wide with { Max2 = bound },
                    _ => below ? wide with { ElbowMin = bound } : wide with { ElbowMax = bound }
                };
                try {
                    DrawingTrajectory.Create(model, DrawingShape.Line, 200, 0, 201, 0, 0, limits);
                    throw new InvalidOperationException("Range fixture was accepted");
                } catch (Lang.ValidationException ex) {
                    Check(ex.Key == "err.sampleOutOfRange" && ex.RangeFailure is { Target: var point } && point == new Point(200, 0), "typed range failure missing");
                    string name = axis == 0 ? "q1" : axis == 1 ? "q2" : Lang.T("diag.elbow");
                    Check(ex.Message.Contains(Lang.T(below ? "diag.below" : "diag.above", name, value, bound, 0.5)), "wrong cause, bound or overrun");
                }
            }
            var ring = DrawingTrajectory.Create(model, DrawingShape.Circle, 200, 0, 50, 0, 0);
            double sampledElbow = ring.Samples.Max(s => JointLimits.Elbow(s.Pose));
            double exactElbow = JointLimits.Elbow(model.Inverse(150, 0)[0]);
            try {
                DrawingTrajectory.Create(model, DrawingShape.Circle, 200, 0, 50, 0, 0, wide with { ElbowMax = (sampledElbow + exactElbow) / 2 });
                throw new InvalidOperationException("Between-sample fixture was accepted");
            } catch (Lang.ValidationException ex) {
                Check(ex.Key == "err.betweenSamples" && ex.RangeFailure is { } failure &&
                    (failure.Target - new Point(150, 0)).Length < 1e-6 && JointLimits.Elbow(failure.Pose) > failure.Limits.ElbowMax &&
                    ex.Message.Contains(Lang.T("diag.above", Lang.T("diag.elbow"), JointLimits.Elbow(failure.Pose), failure.Limits.ElbowMax,
                        JointLimits.Elbow(failure.Pose) - failure.Limits.ElbowMax)), "between-sample diagnostic wrong: " + ex.Message);
                panel.InvalidPlan(() => ex.Message, ex.RangeFailure); panel.RedrawViewport();
                Check(panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point p && p == ex.RangeFailure!.Target), "shared between-sample marker missing");
            }
        }
        Lang.Set(oldLanguage, false);
        FreehandPoint[] draft = [new(300, 0), new(200, 0)];
        var unrestricted = DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, 0,
            options: new ShapeOptions { FreehandPoints = draft });
        var range = new JointLimits(-40, 180, -180, 180, -180, 180);
        var expected = unrestricted.Samples.First(s => !range.Contains(s.Pose));
        var fields = saved.Fields.ToArray();
        fields[6] = "-40"; fields[7] = "180"; fields[8] = "-180"; fields[9] = "180"; fields[15] = "-180"; fields[16] = "180";
        panel.RestoreParameters(JsonSerializer.Serialize(saved with { Fields = fields, Freehand = draft, Branch = 0 })); WaitForFreehand(panel);
        panel.RedrawViewport();
        Check(panel.trajectory is null && panel.execution is null && !panel.movePoint.IsEnabled && !panel.upload.IsEnabled && !panel.run.IsEnabled, "rejection retained an executable plan");
        var marker = panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>())
            .FirstOrDefault(e => e.Tag is Point);
        Check(marker?.Tag is Point target && target == expected.Target, "first rejected sample marker missing or not exact (including failed repair)");
        var oldCenter = panel.cameraCenter; double oldZoom = panel.pixelsPerMm;
        panel.ZoomAt(panel.ToScreen(expected.Target), 1.2); panel.PanFrom(panel.cameraCenter, new Vector(10, -5)); panel.RedrawViewport();
        marker = panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).First(e => e.Tag is Point);
        Check(((Point)marker.Tag) == expected.Target &&
            (new Point(System.Windows.Controls.Canvas.GetLeft(marker) + marker.Width / 2, System.Windows.Controls.Canvas.GetTop(marker) + marker.Height / 2) - panel.ToScreen(expected.Target)).Length < 1e-8,
            "zoom/pan marker lost world location");
        panel.cameraCenter = oldCenter; panel.pixelsPerMm = oldZoom; panel.RedrawViewport();
        Check(panel.result.Text.Contains("q1") && panel.result.Text.Contains("-40.000000") && panel.result.Text.Contains("超出"), "actual limit and overrun missing");
        Check(panel.actionDetails.Text == panel.actionSummary.Text && panel.actionDetails.TextWrapping == TextWrapping.Wrap &&
            panel.result.TextWrapping == TextWrapping.Wrap && panel.checksSection.IsExpanded,
            "complete reason not accessible without ellipsis");
        var rejectedDraft = panel.freehandPoints.ToArray();
        foreach (bool english in new[] { true, false }) {
            Lang.Set(english, false); panel.RefreshLanguage(); panel.RedrawViewport();
            Check(panel.result.Text.Contains(Lang.T("diag.adjust")) && panel.freehandPoints.SequenceEqual(rejectedDraft) && panel.trajectory is null, "language changed diagnostic state");
            Check(panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Controls.TextBlock>()).Any(t => t.Text == Lang.T("diag.marker")), "localized label missing after redraw");
            if (snapshotPath is not null) {
                string directory = Path.GetDirectoryName(Path.GetFullPath(snapshotPath))!; Directory.CreateDirectory(directory);
                var layout = new System.Windows.Controls.Grid();
                layout.ColumnDefinitions.Add(new() { Width = new GridLength(300) }); layout.ColumnDefinitions.Add(new());
                layout.Children.Add(panel.ParameterContent); System.Windows.Controls.Grid.SetColumn(panel, 1); layout.Children.Add(panel);
                layout.Measure(new Size(1400, 740)); layout.Arrange(new Rect(0, 0, 1400, 740)); layout.UpdateLayout(); panel.RedrawViewport(); layout.UpdateLayout();
                ((System.Windows.Controls.ScrollViewer)((System.Windows.Controls.DockPanel)panel.ParameterContent).Children[1]).ScrollToEnd(); layout.UpdateLayout();
                Check(panel.canvas.Children.Count > 20 && marker is not null, "rejection snapshot lost canvas");
                var bitmap = new RenderTargetBitmap(1400, 740, 96, 96, PixelFormats.Pbgra32); bitmap.Render(layout);
                var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(bitmap));
                using var file = File.Create(Path.Combine(directory, $"rejection-{(english ? "en" : "zh")}.png")); png.Save(file);
                layout.Children.Clear(); System.Windows.Controls.Grid.SetColumn(panel, 0);
            }
        }
        panel.min1.Text = "-41"; panel.RedrawViewport();
        Check(!panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point), "edit retained stale marker");
        panel.Refresh(); WaitForFreehand(panel); panel.RedrawViewport();
        Check(panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point), "new failed preview did not replace marker");
        Check(panel.BeginFreehandStroke(panel.ToScreen(draft[^1].Point)), "cannot edit rejected draft");
        panel.CancelFreehandStroke(); panel.RedrawViewport();
        Check(!panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point), "cancel restored stale marker");
        panel.RestoreParameters(JsonSerializer.Serialize(saved)); WaitForFreehand(panel); panel.RedrawViewport();
        Check(panel.execution is not null && !panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point), "successful preview retained marker");
    }
}
