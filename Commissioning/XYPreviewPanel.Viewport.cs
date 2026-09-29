using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Path = System.Windows.Shapes.Path;

namespace TwoLinkCommissioning;

public sealed partial class XYPreviewPanel
{
    private Point cameraCenter, dragFrom, dragCenter, dragTo;
    private double pixelsPerMm = 1;
    private bool cameraReady, autoFit = true, dragging, boxDrag;
    private readonly CheckBox liveLayer = new() { Content = "命令位置", IsChecked = true, Margin = new Thickness(8,8,4,4) };
    private readonly CheckBox unreachableLayer = new() { Content = "不可达区域", IsChecked = true, Margin = new Thickness(8,8,4,4) };
    private readonly Button boxTool = new() { Content = "框选", ToolTip = "点击后在画布上拖出矩形，放大选区；仅改变视图。" };
    private readonly TextBlock pointerInfo = new() { FontSize = 11, Foreground = WorkbenchTheme.Muted, IsHitTestVisible = false };
    private bool boxMode;
    private readonly DispatcherTimer viewportRedrawTimer = new(DispatcherPriority.Background) { Interval = TimeSpan.FromMilliseconds(33) };
    private Rect Plot => new(48, 16, Math.Max(1, canvas.ActualWidth - 68), Math.Max(1, canvas.ActualHeight - 64));
    private Point ToScreen(Point p) => new(Plot.Left + Plot.Width / 2 + (p.X-cameraCenter.X)*pixelsPerMm,
        Plot.Top + Plot.Height / 2 - (p.Y-cameraCenter.Y)*pixelsPerMm);
    private Point ToWorld(Point p) => new(cameraCenter.X+(p.X-Plot.Left-Plot.Width/2)/pixelsPerMm,
        cameraCenter.Y-(p.Y-Plot.Top-Plot.Height/2)/pixelsPerMm);

    private UIElement BuildViewportToolbar()
    {
        var bar = new WrapPanel { Margin = new Thickness(0,0,0,6) };
        void Action(string label, string hint, Action action) {
            var button = new Button { Content = label, ToolTip = hint, Padding = new Thickness(10,6,10,6) };
            button.Click += (_,_) => action(); bar.Children.Add(button);
        }
        Action("全臂", "显示机械臂与理论可达圆；可达圆不代表已验证的机械安全范围。", () => FitView(false));
        Action("适合轨迹", "完整显示当前轨迹。", () => FitView(true));
        Action("−", "缩小视图", () => ZoomAt(ToScreen(cameraCenter), 1/1.3));
        Action("＋", "放大视图；也可滚轮缩放。", () => ZoomAt(ToScreen(cameraCenter), 1.3));
        boxTool.Click += (_,_) => { boxMode = !boxMode; UpdateBoxTool(); }; bar.Children.Add(boxTool);
        unreachableLayer.ToolTip = "透明红色＝按当前关节范围与连杆长度算出的不可达区域。\n留空使用默认 q1 [−30°, +30°]、q2 [−60°, +60°]；填写后按你的范围实时更新。\n红区、规划与设备执行使用同一范围；设备确认同步后才允许运动。红色边界是栅格近似，鼠标位置使用精确逆解检查。";
        unreachableLayer.Checked += (_,_) => { overlayUnreachable = true; RedrawViewport(); };
        unreachableLayer.Unchecked += (_,_) => { overlayUnreachable = false; RedrawViewport(); };
        bar.Children.Add(unreachableLayer);
        liveLayer.Checked += (_,_) => RedrawViewport(); liveLayer.Unchecked += (_,_) => RedrawViewport();
        bar.Children.Add(liveLayer);
        return bar;
    }
    private void UpdateBoxTool()
    {
        boxTool.Content = boxMode ? "框选中…" : "框选";
        boxTool.Background = boxMode ? WorkbenchTheme.Brush("#EDF8F8") : Brushes.White;
        boxTool.Foreground = boxMode ? WorkbenchTheme.Teal : WorkbenchTheme.Ink;
        boxTool.BorderBrush = boxMode ? WorkbenchTheme.Teal : WorkbenchTheme.Line;
        canvas.Cursor = boxMode ? Cursors.Cross : Cursors.Hand;
    }
    private string PointerHint => overlayLimits is null || !overlayUnreachable ? "滚轮缩放 · 拖动平移 · 单位 mm"
        : "滚轮缩放 · 拖动平移 · 单位 mm · 红色＝按当前关节范围不可达";
    private string PointerReach(Point world)
    {
        if(overlayLimits is null || !overlayUnreachable) return "";
        try { return Reachability(Model).IsReachable(world) ? " · 可达" : " · 不可达（红色区域）"; }
        catch(ArgumentException) { return ""; }
    }
    // Preview ranges are user editable; firmware restrictions remain separate execution gates.
    private ReachabilityMask Reachability(PlanarArm model)
    {
        var limits = PreviewLimits();
        if(reachability is null || !reachability.Matches(model,limits)) {
            reachability = ReachabilityMask.Build(model, limits);
        }
        return reachability;
    }
    private bool SampleReachable(PlanarArm model, Point target)
    {
        try { return Reachability(model).IsReachable(target); } catch(ArgumentException) { return true; }
    }
    private void InitializeViewport()
    {
        viewportRedrawTimer.Tick += (_,_) => RedrawViewport();
        Unloaded += (_,_) => viewportRedrawTimer.Stop();
        canvas.Cursor = Cursors.Hand;
        System.Windows.Automation.AutomationProperties.SetName(canvas,"轨迹视图：滚轮缩放，拖动平移，框选按钮放大局部；不控制机械臂运动。");
        canvas.MouseWheel += (_,e) => { if(cameraReady && !dragging) ZoomAt(e.GetPosition(canvas),Math.Pow(1.2,e.Delta/120.0)); e.Handled=true; };
        canvas.MouseDown += (_,e) => {
            if(!cameraReady || !Plot.Contains(e.GetPosition(canvas)) || e.ChangedButton is not (MouseButton.Left or MouseButton.Middle)) return;
            dragFrom=dragTo=e.GetPosition(canvas); dragCenter=cameraCenter;
            boxDrag=boxMode && e.ChangedButton==MouseButton.Left; dragging=canvas.CaptureMouse(); e.Handled=true;
        };
        canvas.MouseMove += (_,e) => {
            var p=e.GetPosition(canvas);
            if(dragging) {
                dragTo=new Point(Math.Clamp(p.X,Plot.Left,Plot.Right),Math.Clamp(p.Y,Plot.Top,Plot.Bottom));
                if(!boxDrag) PanFrom(dragCenter,p-dragFrom);
                RequestViewportRedraw();
            }
            if(cameraReady) { var world=ToWorld(p); pointerInfo.Text=$"鼠标 X {world.X:F2}  Y {world.Y:F2} mm"+PointerReach(world); }
        };
        canvas.MouseUp += (_,e) => {
            if(!dragging) return;
            if(boxDrag) FitSelection(new Rect(dragFrom,dragTo));
            dragging=false; canvas.ReleaseMouseCapture(); boxMode=false; UpdateBoxTool(); RedrawViewport(); e.Handled=true;
        };
        canvas.LostMouseCapture += (_,_) => { dragging=false; boxMode=false; UpdateBoxTool(); RedrawViewport(); };
        canvas.MouseLeave += (_,_) => { if(!dragging) pointerInfo.Text=PointerHint; };
        canvas.SizeChanged += (_,_) => { if(autoFit) cameraReady=false; };
    }
    internal void FitView(bool detail)
    {
        detailView=detail; cameraReady=false; autoFit=true;
        boxMode=false; UpdateBoxTool(); RedrawViewport();
    }
    private void ZoomAt(Point anchor, double factor)
    {
        if(!cameraReady || !double.IsFinite(factor) || factor<=0) return;
        var before=ToWorld(anchor);
        pixelsPerMm=Math.Clamp(pixelsPerMm*factor,0.05,4000);
        cameraCenter+=before-ToWorld(anchor); autoFit=false; RequestViewportRedraw();
    }
    private void FitSelection(Rect selected)
    {
        if(selected.Width<8 || selected.Height<8) return;
        var center=ToWorld(new Point(selected.Left+selected.Width/2,selected.Top+selected.Height/2));
        pixelsPerMm=Math.Clamp(pixelsPerMm*Math.Min(Plot.Width/selected.Width,Plot.Height/selected.Height),0.05,4000);
        cameraCenter=center; autoFit=false;
    }
    private void PanFrom(Point center, Vector screenDelta)
    {
        cameraCenter=center+new Vector(-screenDelta.X/pixelsPerMm,screenDelta.Y/pixelsPerMm); autoFit=false;
    }
    private void RequestViewportRedraw()
    {
        if (!viewportRedrawTimer.IsEnabled) viewportRedrawTimer.Start();
    }
    private void RedrawViewport()
    {
        viewportRedrawTimer.Stop();
        try {
            if(overlayLimits is null) { canvas.Children.Clear(); return; }
            if(timing is not null) DrawSample();
            else if(pointTarget is {} pose) {
                var model=Model; var tip=model.Forward(pose).Tip;
                Draw(model,pose,tip.X,tip.Y,planningLimits is null || planningLimits.Value.Contains(pose), Reachability(model).IsReachable(tip));
            } else Draw(Model,default,0,0,true,false,showPreview:false);
        }
        catch(ArgumentException) { canvas.Children.Clear(); }
    }
    private static double GridStep(double requested)
    {
        double power=Math.Pow(10,Math.Floor(Math.Log10(requested))), n=requested/power;
        return (n<1.5?1:n<3.5?2:n<7.5?5:10)*power;
    }
    private void Draw(PlanarArm model, JointPose pose, double tx, double ty, bool inRange, bool reachable, bool showPreview = true)
    {
        canvas.Children.Clear();
        if(canvas.ActualWidth<100 || canvas.ActualHeight<100) return;
        bool detail=detailView && showPreview;
        if(!cameraReady) {
            Rect bounds=new(-model.OuterRadius,-model.OuterRadius,model.OuterRadius*2,model.OuterRadius*2);
            if(detail) {
                var points=trajectory?.Samples.Select(s=>s.Target).ToArray() ?? [new Point(tx,ty)];
                double left=points.Min(p=>p.X), right=points.Max(p=>p.X), bottom=points.Min(p=>p.Y), top=points.Max(p=>p.Y);
                bounds=new Rect(left,bottom,right-left,top-bottom);
            }
            cameraCenter=new Point(bounds.Left+bounds.Width/2,bounds.Top+bounds.Height/2);
            double padding=detail?1.5:1.12;
            pixelsPerMm=Math.Clamp(Math.Min(Plot.Width/Math.Max(12,bounds.Width*padding),Plot.Height/Math.Max(12,bounds.Height*padding)),0.05,4000);
            cameraReady=true;
        }
        var plot=new Canvas { Clip=new RectangleGeometry(Plot), IsHitTestVisible=false }; canvas.Children.Add(plot);
        static void Line(Canvas host, Point a, Point b, Brush color, double thickness=1, bool dashed=false) => host.Children.Add(new Line {
            X1=a.X,Y1=a.Y,X2=b.X,Y2=b.Y,Stroke=color,StrokeThickness=thickness,StrokeDashArray=dashed?new DoubleCollection([4,3]):null });
        static void Circle(Canvas host, Point p, double radius, Brush stroke, Brush? fill=null) {
            var circle=new Ellipse { Width=radius*2,Height=radius*2,Stroke=stroke,Fill=fill,StrokeThickness=1.5 };
            Canvas.SetLeft(circle,p.X-radius); Canvas.SetTop(circle,p.Y-radius); host.Children.Add(circle);
        }
        static void Arc(Canvas host, Point center, double radius, Brush stroke, double thickness=1.5) {
            var arc=new Path { Stroke=stroke, StrokeThickness=thickness, IsHitTestVisible=false };
            var figure=new PathFigure { StartPoint=new Point(center.X-radius,center.Y), IsClosed=true };
            figure.Segments.Add(new ArcSegment(new Point(center.X+radius,center.Y),new Size(radius,radius),0,radius*2>=900, SweepDirection.Clockwise,true));
            figure.Segments.Add(new ArcSegment(new Point(center.X-radius,center.Y),new Size(radius,radius),0,radius*2>=900, SweepDirection.Clockwise,true));
            var geometry=new PathGeometry(); geometry.Figures.Add(figure); arc.Data=geometry; host.Children.Add(arc);
        }
        static void Label(Canvas host,string text,Point p,Brush? color=null,double size=11) {
            var label=new TextBlock { Text=text,Foreground=color??WorkbenchTheme.Muted,FontSize=size,IsHitTestVisible=false };
            Canvas.SetLeft(label,p.X); Canvas.SetTop(label,p.Y); host.Children.Add(label);
        }
        var origin=ToScreen(new Point());
        if(overlayUnreachable && overlayLimits is not null) {
            var mask=Reachability(model);
            var topLeft=ToScreen(new Point(-mask.ViewRadius,mask.ViewRadius));
            double side=mask.ViewRadius*2*pixelsPerMm;
            var bounds=new Rect(topLeft,new Size(side,side));
            // Outside the bitmap's world square is unreachable too, including when panning away from the arm.
            plot.Children.Add(new Path { IsHitTestVisible=false, Fill=WorkbenchTheme.Brush("#38B42338"),
                Data=new CombinedGeometry(GeometryCombineMode.Exclude,new RectangleGeometry(Plot),new RectangleGeometry(bounds)) });
            var image=new Image { Source=mask.Bitmap, Width=side, Height=side, Stretch=Stretch.Fill, IsHitTestVisible=false };
            RenderOptions.SetBitmapScalingMode(image,BitmapScalingMode.NearestNeighbor);
            Canvas.SetLeft(image,topLeft.X); Canvas.SetTop(image,topLeft.Y); plot.Children.Add(image);
        }
        if(!detail) {
            Arc(plot,origin,model.OuterRadius*pixelsPerMm,WorkbenchTheme.Brush("#8FB42338"));
            if(model.InnerRadius*pixelsPerMm>1.5) Arc(plot,origin,model.InnerRadius*pixelsPerMm,WorkbenchTheme.Brush("#8FB42338"));
        }
        double step=GridStep(65/pixelsPerMm);
        var low=ToWorld(new Point(Plot.Left,Plot.Bottom)); var high=ToWorld(new Point(Plot.Right,Plot.Top));
        for(double gx=Math.Ceiling(low.X/step)*step;gx<=high.X;gx+=step) {
            double px=ToScreen(new Point(gx,0)).X;
            Line(plot,new Point(px,Plot.Top),new Point(px,Plot.Bottom),WorkbenchTheme.Line);
            Label(canvas,(Math.Abs(gx)<step/100?0:gx).ToString("0.###",CultureInfo.InvariantCulture),new Point(px-12,Plot.Bottom+4));
        }
        for(double gy=Math.Ceiling(low.Y/step)*step;gy<=high.Y;gy+=step) {
            double py=ToScreen(new Point(0,gy)).Y;
            Line(plot,new Point(Plot.Left,py),new Point(Plot.Right,py),WorkbenchTheme.Line);
            Label(canvas,(Math.Abs(gy)<step/100?0:gy).ToString("0.###",CultureInfo.InvariantCulture),new Point(1,py-7));
        }
        Line(plot,new Point(Plot.Left,origin.Y),new Point(Plot.Right,origin.Y),WorkbenchTheme.Brush("#A6B6CC"));
        Line(plot,new Point(origin.X,Plot.Top),new Point(origin.X,Plot.Bottom),WorkbenchTheme.Brush("#A6B6CC"));
        Label(canvas,"Y / mm",new Point(1,0)); Label(canvas,"X / mm",new Point(Plot.Right-42,Plot.Bottom+19));
        if(trajectory is {} path) {
            var poly=new Polyline { Stroke=WorkbenchTheme.Teal,StrokeThickness=2.4 };
            foreach(var s in path.Samples) poly.Points.Add(ToScreen(s.Target)); plot.Children.Add(poly);
            var start=ToScreen(path.Samples[0].Target); Circle(plot,start,5,WorkbenchTheme.Teal,Brushes.White);
            bool closed=(path.Samples[^1].Target-path.Samples[0].Target).Length<=0.01;
            Label(plot,closed?"起点 / 收笔点":"起点",start+new Vector(9,6),WorkbenchTheme.Teal);
            for(int segment=0; segment<path.Stops.Length-1; segment++) {
                int i=path.Shape==DrawingShape.Circle?Math.Max(1,path.Samples.Length/4):
                    Math.Max(path.Stops[segment]+1,(path.Stops[segment]+path.Stops[segment+1])/2);
                var end=ToScreen(path.Samples[i].Target); var delta=end-ToScreen(path.Samples[i-1].Target);
                if(delta.Length>0.001) { delta.Normalize(); var side=new Vector(-delta.Y,delta.X);
                    Line(plot,end,end-delta*10+side*4,WorkbenchTheme.Teal,2); Line(plot,end,end-delta*10-side*4,WorkbenchTheme.Teal,2); }
                if(path.Shape==DrawingShape.Rectangle && segment<3) {
                    var corner=ToScreen(path.Samples[path.Stops[segment+1]].Target);
                    Circle(plot,corner,3,WorkbenchTheme.Teal,Brushes.White);
                    var diagonal=ToScreen(path.Samples[path.Stops[2]].Target)-start;
                    if(Math.Abs(diagonal.X)>120 && Math.Abs(diagonal.Y)>55)
                        Label(plot,$"角 {segment+1} · 停顿",corner+new Vector(segment==2?-82:7,-20),WorkbenchTheme.Teal);
                }
            }
            if((path.Samples[^1].Target-path.Samples[0].Target).Length>0.01) {
                var finish=ToScreen(path.Samples[^1].Target); Circle(plot,finish,4,WorkbenchTheme.Teal,WorkbenchTheme.Teal); Label(plot,"终点",finish+new Vector(8,-22));
            }
        }
        var (elbow,tip)=model.Forward(pose);
        var color=inRange?WorkbenchTheme.Blue:WorkbenchTheme.Red;
        if(showPreview) {
            Line(plot,origin,ToScreen(elbow),color,detail?1.5:4,detail); Line(plot,ToScreen(elbow),ToScreen(tip),color,detail?1.5:4,detail);
            Circle(plot,origin,5,color,Brushes.White); Circle(plot,ToScreen(elbow),5,color,Brushes.White);
            Circle(plot,ToScreen(new Point(tx,ty)),5,reachable?WorkbenchTheme.Brush("#C66B05"):WorkbenchTheme.Red,reachable?WorkbenchTheme.Brush("#FFCC66"):WorkbenchTheme.Brush("#FFC9C9"));
        }
        if(liveLayer.IsChecked==true && startFeedback is {Referenced:true} live) {
            var p=ToScreen(model.Forward(live.Command).Tip);
            Line(plot,p-new Vector(8,0),p+new Vector(8,0),Brushes.MediumVioletRed,2);
            Line(plot,p-new Vector(0,8),p+new Vector(0,8),Brushes.MediumVioletRed,2);
        }
        Label(canvas,"─ 规划轨迹",new Point(Plot.Left,Plot.Bottom+29),WorkbenchTheme.Teal);
        Label(canvas,"● 预览点",new Point(Plot.Left+88,Plot.Bottom+29),WorkbenchTheme.Brush("#A35600"));
        Label(canvas,"＋ 命令估算 XY（非实测）",new Point(Plot.Left+164,Plot.Bottom+29),Brushes.MediumVioletRed);
        Label(canvas,"■ 不可达区域（按当前关节范围）",new Point(Plot.Left+332,Plot.Bottom+29),WorkbenchTheme.Brush("#8FB42338"));
        pointerInfo.Text=string.IsNullOrEmpty(pointerInfo.Text)?PointerHint:pointerInfo.Text;
        Canvas.SetLeft(pointerInfo,Plot.Left+8); Canvas.SetTop(pointerInfo,Plot.Top+6); canvas.Children.Add(pointerInfo);
        // The overview remains a context view; its pose is the offline preview, never measured XY.
        if(detail && canvas.ActualWidth>560 && canvas.ActualHeight>280) {
            var box=new Rect(Plot.Right-164,Plot.Top+4,160,116);
            var background=new Rectangle { Width=box.Width,Height=box.Height,Fill=Brushes.White,Stroke=WorkbenchTheme.Line,RadiusX=5,RadiusY=5,IsHitTestVisible=false };
            Canvas.SetLeft(background,box.Left); Canvas.SetTop(background,box.Top); canvas.Children.Add(background);
            var overview=new Canvas { Clip=new RectangleGeometry(box),IsHitTestVisible=false }; canvas.Children.Add(overview);
            double k=42/model.OuterRadius; var o=new Point(box.Left+box.Width/2,box.Top+66);
            Point Mini(Point p)=>o+new Vector(p.X*k,-p.Y*k);
            Circle(overview,o,model.OuterRadius*k,WorkbenchTheme.Line);
            Line(overview,o,Mini(elbow),WorkbenchTheme.Blue,2); Line(overview,Mini(elbow),Mini(tip),WorkbenchTheme.Blue,2);
            Circle(overview,o,3,WorkbenchTheme.Blue,Brushes.White);
            if(trajectory is {} miniPath) { var line=new Polyline {Stroke=WorkbenchTheme.Teal,StrokeThickness=2}; foreach(var s in miniPath.Samples) line.Points.Add(Mini(s.Target)); overview.Children.Add(line); }
            var topLeft=Mini(new Point(low.X,high.Y));
            var window=new Rectangle {Width=(high.X-low.X)*k,Height=(high.Y-low.Y)*k,Stroke=WorkbenchTheme.Teal,StrokeThickness=1,Fill=WorkbenchTheme.Brush("#18087F8C")};
            Canvas.SetLeft(window,topLeft.X); Canvas.SetTop(window,topLeft.Y); overview.Children.Add(window);
            Label(overview,"全臂概览 · 预览姿态",new Point(box.Left+8,box.Top+5));
        }
        double scaleLength=step*pixelsPerMm;
        var scaleStart=new Point(Plot.Left+12,Plot.Bottom-18);
        Line(canvas,scaleStart,scaleStart+new Vector(scaleLength,0),WorkbenchTheme.Ink,2);
        Label(canvas,$"{step:0.###} mm",scaleStart+new Vector(0,-18));
        if(dragging && boxDrag) {
            var r=new Rect(dragFrom,dragTo); var selection=new Rectangle {Width=r.Width,Height=r.Height,Stroke=WorkbenchTheme.Blue,Fill=WorkbenchTheme.Brush("#22245BE8"),IsHitTestVisible=false};
            Canvas.SetLeft(selection,r.Left); Canvas.SetTop(selection,r.Top); canvas.Children.Add(selection);
        }
    }
    private void ViewportSelfTest()
    {
        Measure(new Size(900,720)); Arrange(new Rect(0,0,900,720)); UpdateLayout(); RedrawViewport();
        int requests=0; PointMoveRequested+=CountPoint; TrajectoryRequested+=CountPath;
        void CountPoint(JointPose _) => requests++;
        void CountPath(ExecutionTrajectory _) => requests++;
        var anchor=new Point(Plot.Left+Plot.Width*0.3,Plot.Top+Plot.Height*0.6); var before=ToWorld(anchor);
        ZoomAt(anchor,2);
        if((ToWorld(anchor)-before).Length>1e-8) throw new InvalidOperationException("Zoom must keep cursor world point fixed");
        var c=cameraCenter; double scale=pixelsPerMm;
        SetStartFeedback(new TelemetrySample(0,1,1,true,2,0,0,0,0,0,0));
        if(cameraCenter!=c || pixelsPerMm!=scale) throw new InvalidOperationException("Telemetry changed viewport");
        var box=new Rect(Plot.Left+20,Plot.Top+20,Plot.Width/2,Plot.Height/2);
        var expected=ToWorld(new Point(box.Left+box.Width/2,box.Top+box.Height/2)); FitSelection(box);
        if((cameraCenter-expected).Length>1e-8 || pixelsPerMm<=scale) throw new InvalidOperationException("Box zoom failed");
        c=cameraCenter; scale=pixelsPerMm;
        FitSelection(new Rect(0,0,2,2));
        if(cameraCenter!=c || pixelsPerMm!=scale) throw new InvalidOperationException("Accidental click changed viewport");
        PanFrom(c,new Vector(37,-21));
        if((cameraCenter-(c+new Vector(-37/scale,-21/scale))).Length>1e-8) throw new InvalidOperationException("Pan axis conversion failed");
        ZoomAt(anchor,1e12); if(pixelsPerMm!=4000) throw new InvalidOperationException("Upper zoom bound failed");
        ZoomAt(anchor,1e-12); if(pixelsPerMm!=0.05) throw new InvalidOperationException("Lower zoom bound failed");
        foreach(bool detail in new[] { false, true }) {
            FitView(detail);
            var plot=canvas.Children.OfType<Canvas>().First();
            var links=plot.Children.OfType<Line>().Count(line=>line.Stroke==WorkbenchTheme.Blue || line.Stroke==WorkbenchTheme.Red);
            var strokes=plot.Children.OfType<Path>().Count(shape=>shape.Stroke is not null);
            var fills=plot.Children.OfType<Path>().Count(shape=>shape.Fill is not null);
            int images=plot.Children.OfType<Image>().Count();
            if(links!=2 || strokes!=(detail?0:2) || fills!=1 || images!=1)
                throw new InvalidOperationException($"Preview links/不可达区域层不符：detail={detail} links={links} strokes={strokes} fills={fills} images={images}");
        }
        if(requests!=0 || GridStep(4)!=5) throw new InvalidOperationException("Viewport separation failed");
        PointMoveRequested-=CountPoint; TrajectoryRequested-=CountPath; SetStartFeedback(null); FitView(false);
    }
    internal void CheckViewportLayout()
    {
        if(canvas.ActualWidth<400 || canvas.ActualHeight<260 || double.IsFinite(canvas.Height))
            throw new InvalidOperationException($"Viewport squeezed or fixed: {canvas.ActualWidth} x {canvas.ActualHeight}");
        var bottom=execute.TranslatePoint(new Point(0,execute.ActualHeight),this);
        if(bottom.Y>ActualHeight || bottom.X<0) throw new InvalidOperationException("Motion actions clipped");
    }
}
