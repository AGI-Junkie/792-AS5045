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
    private int freehandStrokeStart = -1;
    private bool freehandMode, editingAllowed = true;
    private Rect? freehandPlot;
    private readonly Button freehandTool = new(), freehandUndo = new(), freehandClear = new();
    private Border xRow = null!, yRow = null!;
    private FrameworkElement transformSection = null!;
    private bool RecordingFreehand => freehandStrokeStart >= 0;

    private void BuildFreehandTools(Panel bar)
    {
        freehandTool.Click += (_, _) => {
            CancelFreehandStroke();
            if (shape.SelectedIndex != FreehandIndex) shape.SelectedIndex = FreehandIndex;
            else freehandMode = !freehandMode;
            boxMode = false; UpdateBoxTool(); UpdateFreehandTools(); RedrawViewport();
        };
        freehandUndo.Click += (_, _) => UndoFreehand();
        freehandClear.Click += (_, _) => ClearFreehand();
        foreach (var button in new[] { freehandTool, freehandUndo, freehandClear }) {
            button.Padding = new Thickness(8, 2, 8, 2); bar.Children.Add(button); editControls.Add(button);
        }
        RefreshFreehandLanguage();
    }

    private void RefreshFreehandLanguage()
    {
        foreach (var (button, name, tip) in new[] {
            (freehandTool, "fh.draw", "fh.draw.tip"), (freehandUndo, "fh.undo", "fh.undo.tip"), (freehandClear, "fh.clear", "fh.clear.tip") }) {
            button.Content = Lang.T(name); button.ToolTip = Lang.T(tip);
            System.Windows.Automation.AutomationProperties.SetName(button, Lang.T(name));
        }
        System.Windows.Automation.AutomationProperties.SetName(canvas, Lang.T(freehandMode ? "fh.canvasName" : "vp.canvasName"));
        UpdateFreehandTools();
    }

    private void UpdateFreehandTools()
    {
        if (previewButton is { } preview) preview.IsEnabled = editingAllowed && !RecordingFreehand;
        if (saveButton is { } save) save.IsEnabled = editingAllowed && !RecordingFreehand;
        if (sampleButton is { } sample) sample.IsEnabled = editingAllowed && !RecordingFreehand;
        freehandTool.IsEnabled = editingAllowed;
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
        if (!editingAllowed || !freehandMode || shape.SelectedIndex != FreehandIndex || RecordingFreehand || !cameraReady || !Plot.Contains(screen)) return false;
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
        if (freehandPoints.Count == FreehandTrajectory.MaxPoints) {
            CancelFreehandStroke(); Lang.Text(result, () => Lang.T("err.freehandPoints")); return;
        }
        freehandPoints.Add(new(world.X, world.Y)); RequestViewportRedraw();
    }

    private void CompleteFreehandStroke(Point screen)
    {
        AddFreehandPoint(screen, terminal: true);
        if (!RecordingFreehand) return;
        if (freehandPoints.Count <= Math.Max(1, freehandStrokeStart)) { CancelFreehandStroke(); return; }
        freehandStrokes.Add(freehandStrokeStart);
        freehandStrokeStart = -1; freehandPlot = null;
        canvas.ReleaseMouseCapture(); UpdateFreehandTools(); UpdateLamps(); RedrawViewport();
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
        int start = freehandStrokes[^1]; freehandStrokes.RemoveAt(freehandStrokes.Count - 1);
        freehandPoints.RemoveRange(start, freehandPoints.Count - start); InvalidateFreehand();
    }

    private void ClearFreehand()
    {
        if (!editingAllowed || shape.SelectedIndex != FreehandIndex) return;
        CancelFreehandStroke(); freehandPoints.Clear(); freehandStrokes.Clear(); InvalidateFreehand();
    }

    private void RestoreFreehand(FreehandPoint[]? points)
    {
        CancelFreehandStroke(); freehandPoints.Clear(); freehandStrokes.Clear();
        if (points is not null) freehandPoints.AddRange(points);
        if (freehandPoints.Count > 0) freehandStrokes.Add(0);
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
