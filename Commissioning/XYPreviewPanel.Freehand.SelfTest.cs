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
        var window = new ControlWindow();
        var hosted = (XYPreviewPanel)typeof(ControlWindow).GetField("xyPreview", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)!.GetValue(window)!;
        Check(hosted.freehandTool.IsEnabled, "disconnected draw button disabled");
        hosted.freehandTool.RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Button.ClickEvent));
        hosted.Measure(new Size(1100, 740)); hosted.Arrange(new Rect(0, 0, 1100, 740)); hosted.UpdateLayout(); hosted.RedrawViewport();
        Check(hosted.BeginFreehandStroke(hosted.ToScreen(new Point(330, 0))), "disconnected drawing blocked");
        hosted.CompleteFreehandStroke(hosted.ToScreen(new Point(334, 0))); hosted.Refresh();
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
        panel.freehandSmoothing.SelectedIndex = 0; // Existing capture checks also certify exact legacy Off behavior.
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
        panel.ClearFreehand(); panel.RedrawViewport(); panel.freehandSmoothing.SelectedIndex = 2;
        Check(panel.BeginFreehandStroke(panel.ToScreen(start)), "smoothed first stroke");
        for (int i = 1; i < 80; i++) panel.AddFreehandPoint(panel.ToScreen(new Point(330 + i * 0.1, 0.4 * Math.Sin(i * 1.7))));
        var captured = panel.freehandPoints.ToArray();
        panel.CompleteFreehandStroke(panel.ToScreen(new Point(338, 0)));
        Check(panel.freehandPoints.Count < captured.Length && panel.FreehandFitLevel == 2, "release did not process the stored tail");
        var committed = panel.freehandPoints.ToArray();
        panel.Refresh(); var oldPreview = panel.trajectory;
        panel.freehandSmoothing.SelectedIndex = 1;
        var low = panel.freehandPoints.ToArray();
        Check(panel.FreehandFitLevel == 1 && panel.execution is not null && !ReferenceEquals(oldPreview, panel.trajectory), "selection did not immediately rebuild draft and preview");
        panel.freehandSmoothing.SelectedIndex = 3;
        Check(!panel.freehandPoints.SequenceEqual(low), "high did not change existing draft");
        panel.freehandSmoothing.SelectedIndex = 1;
        Check(panel.freehandPoints.SequenceEqual(low), "Low-High-Low accumulated filtering");
        panel.freehandSmoothing.SelectedIndex = 0;
        Check(panel.freehandPoints.SequenceEqual(captured.Append(new FreehandPoint(338, 0))), "Off did not restore captured baseline");
        panel.freehandSmoothing.SelectedIndex = 2;
        Check(panel.freehandPoints.SequenceEqual(committed), "Medium did not restore original processed draft");
        panel.RedrawViewport();
        Check(panel.BeginFreehandStroke(panel.ToScreen(committed[^1].Point)), "smoothed continuation");
        for (int i = 1; i < 40; i++) panel.AddFreehandPoint(panel.ToScreen(new Point(338 + i * 0.1, 0.4 * Math.Sin(i * 1.7))));
        panel.CompleteFreehandStroke(panel.ToScreen(new Point(342, 0)));
        Check(panel.freehandPoints.Take(committed.Length).SequenceEqual(committed) && panel.freehandStrokes[^1] == committed.Length, "continuation rewrote prefix/shared endpoint");
        var continuedRaw = panel.freehandRawStrokes.Select(s => s.ToArray()).ToArray();
        panel.freehandSmoothing.SelectedIndex = 0;
        Check(panel.freehandPoints.SequenceEqual(continuedRaw[0].Concat(continuedRaw[1].Skip(1))), "continued Off baseline");
        panel.freehandSmoothing.SelectedIndex = 1;
        Check(panel.freehandStrokes[^1] == FreehandTrajectory.Smooth(continuedRaw[0], 1).Length, "reprocessed undo boundary");
        var beforeCancel = panel.freehandPoints.ToArray();
        Check(panel.BeginFreehandStroke(panel.ToScreen(beforeCancel[^1].Point)), "reprocessed continuation start");
        panel.AddFreehandPoint(panel.ToScreen(new Point(343, 0))); panel.CancelFreehandStroke();
        Check(panel.freehandPoints.SequenceEqual(beforeCancel) && panel.freehandRawStrokes.Count == 2, "reprocessed cancellation changed baseline");
        panel.freehandSmoothing.SelectedIndex = 2;
        var languagePreview = panel.trajectory; var languageDraft = panel.freehandPoints.ToArray();
        Lang.Set(true, false); panel.RefreshLanguage(); Lang.Set(false, false); panel.RefreshLanguage();
        Check(ReferenceEquals(languagePreview, panel.trajectory) && panel.freehandPoints.SequenceEqual(languageDraft) && panel.FreehandFitLevel == 2, "language switch resmoothed/replanned");
        panel.UndoFreehand(); Check(panel.freehandPoints.SequenceEqual(committed) && panel.FreehandFitLevel == 2, "smoothed undo");
        string processedJson = panel.SaveParameters();
        panel.RestoreParameters(processedJson);
        Check(panel.freehandPoints.SequenceEqual(committed) && panel.FreehandFitLevel == 0, "processed serialization/legacy fit");
        panel.freehandSmoothing.SelectedIndex = 1;
        Check(panel.freehandPoints.SequenceEqual(FreehandTrajectory.Smooth(committed, 1)) && panel.FreehandFitLevel == 1 && panel.execution is not null, "loaded baseline not reprocessed on explicit selection");
        panel.freehandSmoothing.SelectedIndex = 0;
        Check(panel.freehandPoints.SequenceEqual(committed), "loaded Off did not restore file baseline");
        RejectionDiagnosticSelfTest(panel, savedRecord, snapshotPath);
        panel.LoadExample(); Check(panel.execution is not null, "smooth example: " + panel.result.Text);
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
        panel.RestoreParameters(JsonSerializer.Serialize(saved with { Fields = fields, Freehand = draft, Branch = 0 }));
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
        panel.Refresh(); panel.RedrawViewport();
        Check(panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point), "new failed preview did not replace marker");
        Check(panel.BeginFreehandStroke(panel.ToScreen(draft[^1].Point)), "cannot edit rejected draft");
        panel.CancelFreehandStroke(); panel.RedrawViewport();
        Check(!panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point), "cancel restored stale marker");
        panel.RestoreParameters(JsonSerializer.Serialize(saved)); panel.RedrawViewport();
        Check(panel.execution is not null && !panel.canvas.Children.OfType<System.Windows.Controls.Canvas>().SelectMany(c => c.Children.OfType<System.Windows.Shapes.Ellipse>()).Any(e => e.Tag is Point), "successful preview retained marker");
    }
}
