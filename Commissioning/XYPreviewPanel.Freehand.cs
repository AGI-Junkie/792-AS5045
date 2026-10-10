using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

namespace TwoLinkCommissioning;

public sealed partial class XYPreviewPanel
{
    private const int FreehandIndex = (int)DrawingShape.Freehand + 1;
    private readonly List<FreehandPoint> freehandPoints = [];
    private readonly List<int> freehandStrokes = [];
    private readonly List<int> freehandLevels = [];
    private readonly List<FreehandPoint[]> freehandRawStrokes = [];
    private bool refreshingFreehandLanguage;
    private readonly ComboBox freehandSmoothing = new() { MinWidth = 100, SelectedIndex = 2 };
    private readonly TextBlock freehandSmoothingLabel = new() { VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(8, 0, 4, 0) };
    private int FreehandFitLevel => freehandLevels.DefaultIfEmpty(0).OrderBy(FreehandTrajectory.FitTolerance).First();
    private int freehandStrokeStart = -1;
    private bool freehandMode, editingAllowed = true;
    private Rect? freehandPlot;
    private readonly Button freehandTool = new(), freehandUndo = new(), freehandClear = new();
    private Border xRow = null!, yRow = null!;
    private FrameworkElement transformSection = null!;
    private bool RecordingFreehand => freehandStrokeStart >= 0;
    private int freehandWorkVersion;
    private bool freehandWorkerActive, freehandNeedsProcessing, freehandReprocessAll;
    private FreehandWork? freehandPendingWork, freehandRunningWork;
    private Task<bool>? freehandRequest;
    private bool ProcessingFreehand => freehandWorkerActive || freehandPendingWork is not null;
    private sealed record FreehandWork(int Version, FreehandPoint[] Points, int[] Starts, int[] Levels, FreehandPoint[][] Raw,
        bool Process, bool All, PlanarArm? Model, JointLimits Limits, int Branch, PlanningRates Rates, TaskCompletionSource<bool> Done);
    private sealed record FreehandResult(FreehandPoint[] Points, int[] Starts, bool Processed,
        DrawingTrajectory? Path, TimedTrajectory? Time, ExecutionTrajectory? Execution, double Shift, double Asked, ArgumentException? Failure);

    private void CancelFreehandWork()
    {
        ++freehandWorkVersion;
        freehandPendingWork?.Done.TrySetResult(false); freehandPendingWork = null;
        freehandRunningWork?.Done.TrySetResult(false);
    }

    private void QueueFreehandWork(PlanarArm? model = null, JointLimits limits = default, int branch = 0, PlanningRates rates = default)
    {
        freehandPendingWork?.Done.TrySetResult(false);
        var done = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        freehandPendingWork = new(++freehandWorkVersion, freehandPoints.ToArray(), freehandStrokes.ToArray(), freehandLevels.ToArray(),
            freehandRawStrokes.Select(s => s.ToArray()).ToArray(), freehandNeedsProcessing, freehandReprocessAll, model, limits, branch, rates, done);
        freehandRequest = done.Task;
        if (shape.SelectedIndex == FreehandIndex) { Lang.Text(result, () => Lang.T("fh.processing")); result.Foreground = Brushes.DarkSlateGray; }
        StartFreehandWorker(); UpdateFreehandTools(); UpdatePointButton();
    }

    private void StartFreehandWorker()
    {
        if (freehandWorkerActive || freehandPendingWork is not { } work) return;
        freehandPendingWork = null; freehandRunningWork = work; freehandWorkerActive = true;
        _ = FinishFreehandWorkAsync(work);
    }

    private async Task FinishFreehandWorkAsync(FreehandWork work)
    {
        FreehandResult computed;
        try { computed = await Task.Run(() => ComputeFreehand(work)).ConfigureAwait(false); }
        catch (Exception ex) {
            System.Diagnostics.Trace.TraceError("Freehand processing: " + ex);
            computed = new(work.Points, work.Starts, false, null, null, null, 0, 0, Lang.Error("fh.processingFailed", ex.Message));
        }
        try { await Dispatcher.InvokeAsync(() => {
            freehandWorkerActive = false; freehandRunningWork = null;
            bool current = work.Version == freehandWorkVersion;
            if (current) {
                if (computed.Processed) {
                    freehandPoints.Clear(); freehandPoints.AddRange(computed.Points);
                    freehandStrokes.Clear(); freehandStrokes.AddRange(computed.Starts);
                    freehandNeedsProcessing = freehandReprocessAll = false;
                }
                if (computed.Failure is { } failure) {
                    InvalidPlan(() => failure.Message, (failure as Lang.ValidationException)?.RangeFailure, false); checksSection.IsExpanded = true;
                } else if (computed.Path is { } path && computed.Time is { } time && computed.Execution is { } plan) {
                    trajectory = path; timing = time; execution = plan; pointTarget = time.At(0).Pose;
                    ShowExecutionInfo(plan, computed.Asked); ShowDrawingPreview(work.Limits, computed.Shift);
                } else {
                    if (shape.SelectedIndex == FreehandIndex) Lang.Text(result, () => Lang.T("err.needPreview"));
                    UpdateLamps(); RequestViewportRedraw();
                }
            }
            StartFreehandWorker(); UpdateFreehandTools(); UpdatePointButton();
            work.Done.TrySetResult(current);
        }); }
        catch (OperationCanceledException) { work.Done.TrySetResult(false); }
        catch (InvalidOperationException) when (Dispatcher.HasShutdownStarted) { work.Done.TrySetResult(false); }
    }

    private static FreehandResult ComputeFreehand(FreehandWork work)
    {
        var points = work.Points.ToList(); var starts = work.Starts.ToArray(); bool processed = work.Process && work.Raw.Length == 0;
        try {
            if (work.Process && work.Raw.Length > 0) {
                if (work.All) {
                    points.Clear();
                    int remaining = work.Raw.Sum(s => s.Length) - Math.Max(0, work.Raw.Length - 1);
                    for (int i = 0; i < work.Raw.Length; i++) {
                        int start = points.Count, skip = start == 0 ? 0 : 1;
                        remaining -= work.Raw[i].Length - skip;
                        var stroke = FreehandTrajectory.Smooth(work.Raw[i], work.Levels[i]);
                        // Reserve the later raw strokes so Off can still restore every captured point.
                        if (start + stroke.Length - skip + remaining > FreehandTrajectory.MaxPoints) stroke = work.Raw[i];
                        starts[i] = start; points.AddRange(stroke.Skip(skip));
                    }
                } else {
                    int start = starts[^1], first = Math.Max(0, start - 1);
                    var stroke = FreehandTrajectory.Smooth(work.Raw[^1], work.Levels[^1]);
                    if (first + stroke.Length > FreehandTrajectory.MaxPoints) stroke = work.Raw[^1];
                    points.RemoveRange(start, points.Count - start); points.AddRange(stroke.Skip(start == 0 ? 0 : 1));
                }
                processed = true;
            }
            var draft = points.ToArray();
            if (work.Model is not { } model) return new(draft, starts, processed, null, null, null, 0, 0, null);
            int fit = work.Levels.DefaultIfEmpty(0).OrderBy(FreehandTrajectory.FitTolerance).First();
            var repaired = FreehandTrajectory.Repair(draft, model, work.Limits, work.Branch, work.Rates, fit);
            var path = DrawingTrajectory.Create(model, DrawingShape.Freehand, 0, 0, 0, 0, work.Branch, work.Limits,
                new ShapeOptions { FreehandPoints = repaired.Points, FreehandFitLevel = fit });
            var time = new TimedTrajectory(path, work.Rates); double asked = time.Duration;
            var plan = new ExecutionTrajectory(time, work.Limits);
            return new(draft, starts, processed, path, plan.Timing, plan, repaired.Shift, asked, null);
        } catch (ArgumentException ex) { return new(points.ToArray(), starts, processed, null, null, null, 0, 0, ex); }
    }

    private void BuildFreehandTools(Panel bar)
    {
        freehandTool.Click += (_, _) => {
            if (!editingAllowed) return;
            CancelFreehandStroke();
            if (shape.SelectedIndex != FreehandIndex) shape.SelectedIndex = FreehandIndex;
            freehandMode = true;
            boxMode = false; UpdateBoxTool(); UpdateFreehandTools(); RedrawViewport();
        };
        freehandUndo.Click += (_, _) => UndoFreehand();
        freehandClear.Click += (_, _) => ClearFreehand();
        foreach (var button in new[] { freehandTool, freehandUndo, freehandClear }) {
            button.Padding = new Thickness(8, 2, 8, 2); bar.Children.Add(button); editControls.Add(button);
        }
        bar.Children.Add(freehandSmoothingLabel); bar.Children.Add(freehandSmoothing); editControls.Add(freehandSmoothing);
        RefreshFreehandLanguage();
        freehandSmoothing.SelectionChanged += (_, _) => {
            if (refreshingFreehandLanguage || updating || !editingAllowed || RecordingFreehand || shape.SelectedIndex != FreehandIndex || freehandSmoothing.SelectedIndex < 0) return;
            ReprocessFreehand(); InvalidateFreehand(); Refresh(); RedrawViewport();
        };
    }

    private void ReprocessFreehand()
    {
        for (int i = 0; i < freehandLevels.Count; i++) freehandLevels[i] = freehandSmoothing.SelectedIndex;
        freehandNeedsProcessing = freehandReprocessAll = true;
    }

    private void RefreshFreehandLanguage()
    {
        foreach (var (button, name, tip) in new[] {
            (freehandTool, "fh.draw", "fh.draw.tip"), (freehandUndo, "fh.undo", "fh.undo.tip"), (freehandClear, "fh.clear", "fh.clear.tip") }) {
            button.Content = Lang.T(name); button.ToolTip = Lang.T(tip);
            System.Windows.Automation.AutomationProperties.SetName(button, Lang.T(name));
        }
        int selected = freehandSmoothing.SelectedIndex;
        refreshingFreehandLanguage = true;
        try {
            freehandSmoothing.Items.Clear();
            foreach (string key in new[] { "fh.off", "fh.low", "fh.medium", "fh.high" }) freehandSmoothing.Items.Add(Lang.T(key));
            freehandSmoothing.SelectedIndex = selected < 0 ? 2 : selected;
        } finally { refreshingFreehandLanguage = false; }
        freehandSmoothingLabel.Text = Lang.T("fh.smoothing"); freehandSmoothing.ToolTip = Lang.T("fh.smoothing.tip");
        System.Windows.Automation.AutomationProperties.SetName(freehandSmoothing, Lang.T("fh.smoothing"));
        System.Windows.Automation.AutomationProperties.SetName(canvas, Lang.T(freehandMode ? "fh.canvasName" : "vp.canvasName"));
        UpdateFreehandTools();
    }

    private void UpdateFreehandTools()
    {
        if (previewButton is { } preview) preview.IsEnabled = editingAllowed && !RecordingFreehand && !ProcessingFreehand;
        if (saveButton is { } save) save.IsEnabled = editingAllowed && !RecordingFreehand && !ProcessingFreehand && !freehandNeedsProcessing;
        if (sampleButton is { } sample) sample.IsEnabled = editingAllowed && !RecordingFreehand && !ProcessingFreehand;
        freehandSmoothingLabel.Visibility = freehandSmoothing.Visibility = shape.SelectedIndex == FreehandIndex ? Visibility.Visible : Visibility.Collapsed;
        freehandTool.IsEnabled = editingAllowed;
        freehandSmoothing.IsEnabled = editingAllowed && !RecordingFreehand;
        freehandTool.Background = freehandMode ? WorkbenchTheme.Brush("#EDF8F8") : Brushes.White;
        freehandTool.Foreground = freehandMode ? WorkbenchTheme.Teal : WorkbenchTheme.Ink;
        freehandUndo.IsEnabled = editingAllowed && shape.SelectedIndex == FreehandIndex && !RecordingFreehand && freehandStrokes.Count > 0;
        freehandClear.IsEnabled = editingAllowed && shape.SelectedIndex == FreehandIndex && freehandPoints.Count > 0;
        canvas.Cursor = freehandMode || boxMode ? Cursors.Cross : Cursors.Hand;
        System.Windows.Automation.AutomationProperties.SetName(canvas, Lang.T(freehandMode ? "fh.canvasName" : "vp.canvasName"));
    }

    private void InvalidateFreehand()
    {
        parametersDirty = true; operationError = null;
        PausePlayback();
        InvalidPlan(() => Lang.T("err.needPreview"));
        UpdateFreehandTools();
    }

    private bool BeginFreehandStroke(Point screen)
    {
        if (!editingAllowed || ProcessingFreehand || !freehandMode || shape.SelectedIndex != FreehandIndex || RecordingFreehand || !cameraReady || !Plot.Contains(screen)) return false;
        Point world = ToWorld(screen);
        if (freehandPoints.Count > 0 && ((ToScreen(freehandPoints[^1].Point) - screen).Length > 10 || (freehandPoints[^1].Point - world).Length > 3)) {
            Lang.Text(result, () => Lang.T("fh.resumeNear")); return false;
        }
        freehandPlot = Plot; autoFit = false;
        freehandStrokeStart = freehandPoints.Count;
        if (freehandPoints.Count == 0) freehandPoints.Add(new(world.X, world.Y));
        InvalidateFreehand(); RequestViewportRedraw(); return true;
    }

    private void AddFreehandPoint(Point screen, bool terminal = false)
    {
        if (!RecordingFreehand) return;
        if (!Plot.Contains(screen)) { CancelFreehandStroke(); Lang.Text(result, () => Lang.T("fh.cancelOutside")); return; }
        var world = ToWorld(screen); double distance = (world - freehandPoints[^1].Point).Length;
        if (distance < (terminal ? 1e-9 : FreehandTrajectory.CaptureSpacing)) return;
        int capturedCount = freehandRawStrokes.Sum(s => s.Length) - Math.Max(0, freehandRawStrokes.Count - 1)
            + freehandPoints.Count - freehandStrokeStart;
        if (freehandPoints.Count == FreehandTrajectory.MaxPoints || capturedCount >= FreehandTrajectory.MaxPoints) {
            CancelFreehandStroke(); Lang.Text(result, () => Lang.T("err.freehandPoints")); return;
        }
        freehandPoints.Add(new(world.X, world.Y)); RequestViewportRedraw();
    }

    private void CompleteFreehandStroke(Point screen)
    {
        AddFreehandPoint(screen, terminal: true);
        if (!RecordingFreehand) return;
        if (freehandPoints.Count <= Math.Max(1, freehandStrokeStart)) { CancelFreehandStroke(); return; }
        int first = Math.Max(0, freehandStrokeStart - 1);
        var stroke = freehandPoints.Skip(first).ToArray();
        freehandStrokes.Add(freehandStrokeStart); freehandLevels.Add(freehandSmoothing.SelectedIndex); freehandRawStrokes.Add(stroke);
        freehandNeedsProcessing = true;
        freehandStrokeStart = -1; freehandPlot = null;
        canvas.ReleaseMouseCapture(); QueueFreehandWork(); UpdateLamps(); RedrawViewport();
    }

    private void CancelFreehandStroke()
    {
        if (!RecordingFreehand) return;
        int start = freehandStrokeStart; freehandStrokeStart = -1;
        freehandPoints.RemoveRange(start, freehandPoints.Count - start); freehandPlot = null;
        canvas.ReleaseMouseCapture(); UpdateFreehandTools(); UpdateLamps(); RequestViewportRedraw();
    }

    internal bool CancelFreehandOnEscape()
    {
        if (!RecordingFreehand) return false;
        CancelFreehandStroke(); return true;
    }

    private void UndoFreehand()
    {
        if (!editingAllowed || RecordingFreehand || shape.SelectedIndex != FreehandIndex || freehandStrokes.Count == 0) return;
        int start = freehandStrokes[^1]; freehandStrokes.RemoveAt(freehandStrokes.Count - 1); freehandLevels.RemoveAt(freehandLevels.Count - 1); freehandRawStrokes.RemoveAt(freehandRawStrokes.Count - 1);
        freehandPoints.RemoveRange(start, freehandPoints.Count - start);
        if (freehandRawStrokes.Count == 0) freehandNeedsProcessing = freehandReprocessAll = false;
        InvalidateFreehand();
        if (freehandNeedsProcessing && freehandRawStrokes.Count > 0) QueueFreehandWork();
    }

    private void ClearFreehand()
    {
        if (!editingAllowed || shape.SelectedIndex != FreehandIndex) return;
        CancelFreehandStroke(); freehandPoints.Clear(); freehandStrokes.Clear(); freehandLevels.Clear(); freehandRawStrokes.Clear();
        freehandNeedsProcessing = freehandReprocessAll = false; InvalidateFreehand();
    }

    private void RestoreFreehand(FreehandPoint[]? points)
    {
        CancelFreehandWork(); freehandNeedsProcessing = freehandReprocessAll = false;
        CancelFreehandStroke(); freehandPoints.Clear(); freehandStrokes.Clear(); freehandLevels.Clear(); freehandRawStrokes.Clear();
        if (points is not null) freehandPoints.AddRange(points);
        if (freehandPoints.Count > 0) { freehandStrokes.Add(0); freehandLevels.Add(0); freehandRawStrokes.Add(freehandPoints.ToArray()); }
        UpdateFreehandTools();
    }

    private void LoadFreehandExample() => RestoreFreehand(FreehandTrajectory.Example);

    private void DrawFreehand()
    {
        if (shape.SelectedIndex != FreehandIndex || freehandPoints.Count == 0 || !cameraReady) return;
        var overlay = new Canvas { Clip = new RectangleGeometry(Plot), IsHitTestVisible = false };
        var draft = new Polyline { Stroke = Brushes.Gray, StrokeThickness = 1.5, StrokeDashArray = new DoubleCollection([4, 3]) };
        foreach (var p in freehandPoints) draft.Points.Add(ToScreen(p.Point)); overlay.Children.Add(draft);
        foreach (var (point, key) in new[] { (freehandPoints[0], "fh.start"), (freehandPoints[^1], "fh.end") }) {
            var screen = ToScreen(point.Point);
            var marker = new Ellipse { Width = 10, Height = 10, Fill = Brushes.White, Stroke = WorkbenchTheme.Teal, StrokeThickness = 2 };
            Canvas.SetLeft(marker, screen.X - 5); Canvas.SetTop(marker, screen.Y - 5); overlay.Children.Add(marker);
            var label = new TextBlock { Text = Lang.T(key), Foreground = WorkbenchTheme.Teal, FontSize = 11 };
            Canvas.SetLeft(label, screen.X + 7); Canvas.SetTop(label, screen.Y + 7); overlay.Children.Add(label);
        }
        canvas.Children.Add(overlay);
    }
}
