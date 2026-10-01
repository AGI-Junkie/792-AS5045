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
    private readonly CheckBox liveLayer = new() { Content = "设备姿态", IsChecked = true, Margin = new Thickness(8,8,4,4),
        ToolTip = "运动期间，两根连杆跟随设备回报的命令步数。arm2 和末端位置为模型估算，非实测；离线播放使用规划姿态。" };
    private readonly CheckBox unreachableLayer = new() { Content = "不可达区域", IsChecked = true, Margin = new Thickness(8,8,4,4) };
    private readonly Button boxTool = new() { Content = "框选", ToolTip = "点击后在画布上拖出矩形，放大选区；仅改变视图。" };
    private readonly TextBlock pointerInfo = new() { FontSize = 11, Foreground = WorkbenchTheme.Muted, IsHitTestVisible = false, TextTrimming=TextTrimming.CharacterEllipsis };
    private bool boxMode;
    private readonly DispatcherTimer viewportRedrawTimer = new(DispatcherPriority.Background) { Interval = TimeSpan.FromMilliseconds(33) };
    private bool HasTrajectoryDetail => trajectory is not null && canvas.ActualWidth>560 && canvas.ActualHeight>280;
    private Rect Plot => new(48, 16, Math.Max(1, canvas.ActualWidth - 68 - (HasTrajectoryDetail?210:0)), Math.Max(1, canvas.ActualHeight - 82));
    private bool FollowCommand => commandView && liveLayer.IsChecked==true && startFeedback is {Referenced:true};
    private Point ToScreen(Point p) => new(Plot.Left + Plot.Width / 2 + (p.X-cameraCenter.X)*pixelsPerMm,
        Plot.Top + Plot.Height / 2 - (p.Y-cameraCenter.Y)*pixelsPerMm);
    private Point ToWorld(Point p) => new(cameraCenter.X+(p.X-Plot.Left-Plot.Width/2)/pixelsPerMm,
        cameraCenter.Y-(p.Y-Plot.Top-Plot.Height/2)/pixelsPerMm);

    private UIElement BuildViewportToolbar()
    {
        var bar = new WrapPanel { VerticalAlignment = VerticalAlignment.Center };
        void Action(string label, string hint, Action action) {
            var button = new Button { Content = label, ToolTip = hint, Padding = new Thickness(10,2,10,2) };
            button.Click += (_,_) => action(); bar.Children.Add(button);
        }
        Action("全臂", "显示机械臂与理论可达圆；可达圆不代表已验证的机械安全范围。", () => FitView(false));
        Action("适合运动", "显示基座、两根连杆和整条轨迹；按全部规划姿态缩放。", () => FitView(true));
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
        bool follow=FollowCommand;
        var armPose=follow?startFeedback!.Command:pose;
        var (elbow,tip)=model.Forward(armPose);
        bool detail=detailView && (showPreview || follow);
        if(detail && autoFit && cameraReady && follow &&
           new[] {new Point(),elbow,tip}.Any(p=>!Plot.Contains(ToScreen(p)))) cameraReady=false;
        if(!cameraReady) {
            Rect bounds=new(-model.OuterRadius,-model.OuterRadius,model.OuterRadius*2,model.OuterRadius*2);
            if(detail) {
                bounds=new Rect(new Point(),new Point(tx,ty));
                if(trajectory is {} motion) foreach(var sample in motion.Samples) {
                    var joints=model.Forward(sample.Pose); bounds.Union(joints.Elbow); bounds.Union(joints.Tip);
                }
                bounds.Union(elbow); bounds.Union(tip);
            }
            cameraCenter=new Point(bounds.Left+bounds.Width/2,bounds.Top+bounds.Height/2);
            double padding=detail?1.18:1.12;
            pixelsPerMm=Math.Clamp(Math.Min(Plot.Width/Math.Max(12,bounds.Width*padding),Plot.Height/Math.Max(12,bounds.Height*padding)),0.05,4000);
            cameraReady=true;
        }
        var plot=new Canvas { Clip=new RectangleGeometry(Plot), IsHitTestVisible=false }; canvas.Children.Add(plot);
        static void Line(Canvas host, Point a, Point b, Brush color, double thickness=1, bool dashed=false) => host.Children.Add(new Line {
            X1=a.X,Y1=a.Y,X2=b.X,Y2=b.Y,Stroke=color,StrokeThickness=thickness,
            StrokeStartLineCap=PenLineCap.Round,StrokeEndLineCap=PenLineCap.Round,StrokeDashArray=dashed?new DoubleCollection([4,3]):null });
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
            Line(plot,new Point(px,Plot.Top),new Point(px,Plot.Bottom),WorkbenchTheme.GridLine);
            Label(canvas,(Math.Abs(gx)<step/100?0:gx).ToString("0.###",CultureInfo.InvariantCulture),new Point(px-12,Plot.Bottom+4));
        }
        for(double gy=Math.Ceiling(low.Y/step)*step;gy<=high.Y;gy+=step) {
            double py=ToScreen(new Point(0,gy)).Y;
            Line(plot,new Point(Plot.Left,py),new Point(Plot.Right,py),WorkbenchTheme.GridLine);
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
        var color=inRange || follow?WorkbenchTheme.Blue:WorkbenchTheme.Red;
        var secondColor=inRange || follow?Brushes.SlateBlue:WorkbenchTheme.Red;
        if(showPreview || follow) {
            var joint=ToScreen(elbow); var end=ToScreen(tip);
            Line(plot,origin,joint,color,5); Line(plot,joint,end,secondColor,5);
            Circle(plot,origin,7,color,Brushes.White); Circle(plot,joint,6,secondColor,Brushes.White);
            Label(plot,$"arm1 · {armPose.Q1:F1}°",origin+(joint-origin)*0.5+new Vector(8,-22),color);
            Label(plot,$"arm2 · {armPose.Q2:F1}°",joint+(end-joint)*0.5+new Vector(8,-22),secondColor);
            Circle(plot,follow?end:ToScreen(new Point(tx,ty)),6,reachable || follow?WorkbenchTheme.Brush("#C66B05"):WorkbenchTheme.Red,
                reachable || follow?WorkbenchTheme.Brush("#FFCC66"):WorkbenchTheme.Brush("#FFC9C9"));
        }
        if(liveLayer.IsChecked==true && startFeedback is {Referenced:true} live) {
            var p=ToScreen(model.Forward(live.Command).Tip);
            Line(plot,p-new Vector(8,0),p+new Vector(8,0),Brushes.MediumVioletRed,2);
            Line(plot,p-new Vector(0,8),p+new Vector(0,8),Brushes.MediumVioletRed,2);
        }
        Label(canvas,"━━ arm1",new Point(Plot.Left,Plot.Bottom+25),WorkbenchTheme.Blue);
        Label(canvas,"━━ arm2",new Point(Plot.Left+78,Plot.Bottom+25),Brushes.SlateBlue);
        Label(canvas,"─ 规划轨迹",new Point(Plot.Left+156,Plot.Bottom+25),WorkbenchTheme.Teal);
        Label(canvas,"● 末端",new Point(Plot.Left+250,Plot.Bottom+25),WorkbenchTheme.Brush("#A35600"));
        Label(canvas,follow?"设备命令姿态 · 非实测":"离线规划姿态",new Point(Plot.Left,Plot.Bottom+43));
        Label(canvas,"＋ 命令估算 XY（非实测）",new Point(Plot.Left+180,Plot.Bottom+43),Brushes.MediumVioletRed);
        pointerInfo.Text=string.IsNullOrEmpty(pointerInfo.Text)?PointerHint:pointerInfo.Text;
        pointerInfo.MaxWidth=Math.Max(1,Plot.Width-16);
        Canvas.SetLeft(pointerInfo,Plot.Left+8); Canvas.SetTop(pointerInfo,Plot.Top+6); canvas.Children.Add(pointerInfo);
        // Keep small drawings legible while the main view shows the complete arm.
        if(trajectory is {} miniPath && HasTrajectoryDetail) {
            var box=new Rect(Plot.Right+14,Plot.Top+4,190,144);
            var background=new Rectangle { Width=box.Width,Height=box.Height,Fill=Brushes.White,Stroke=WorkbenchTheme.Line,RadiusX=5,RadiusY=5,IsHitTestVisible=false };
            Canvas.SetLeft(background,box.Left); Canvas.SetTop(background,box.Top); canvas.Children.Add(background);
            var overview=new Canvas { Clip=new RectangleGeometry(box),IsHitTestVisible=false }; canvas.Children.Add(overview);
            Rect bounds=new(miniPath.Samples[0].Target,new Size());
            foreach(var sample in miniPath.Samples) bounds.Union(sample.Target);
            double k=Math.Min((box.Width-24)/Math.Max(12,bounds.Width*1.3),(box.Height-40)/Math.Max(12,bounds.Height*1.3));
            var center=new Point(bounds.Left+bounds.Width/2,bounds.Top+bounds.Height/2);
            var o=new Point(box.Left+box.Width/2,box.Top+28+(box.Height-36)/2);
            Point Mini(Point p)=>o+new Vector((p.X-center.X)*k,-(p.Y-center.Y)*k);
            var line=new Polyline {Stroke=WorkbenchTheme.Teal,StrokeThickness=2.4};
            foreach(var sample in miniPath.Samples) line.Points.Add(Mini(sample.Target)); overview.Children.Add(line);
            Circle(overview,Mini(miniPath.Samples[0].Target),4,WorkbenchTheme.Teal,Brushes.White);
            Circle(overview,Mini(tip),5,WorkbenchTheme.Brush("#C66B05"),WorkbenchTheme.Brush("#FFCC66"));
            Label(overview,"轨迹局部 · "+shape.SelectedItem,new Point(box.Left+8,box.Top+5));
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
        bool initialDetail=detailView;
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
            var links=plot.Children.OfType<Line>().Count(line=>line.Stroke==WorkbenchTheme.Blue || line.Stroke==WorkbenchTheme.Red || line.Stroke==Brushes.SlateBlue);
            var strokes=plot.Children.OfType<Path>().Count(shape=>shape.Stroke is not null);
            var fills=plot.Children.OfType<Path>().Count(shape=>shape.Fill is not null);
            int images=plot.Children.OfType<Image>().Count();
            if(links!=2 || strokes!=(detail?0:2) || fills!=1 || images!=1)
                throw new InvalidOperationException($"Preview links/不可达区域层不符：detail={detail} links={links} strokes={strokes} fills={fills} images={images}");
        }
        if(requests!=0 || GridStep(4)!=5) throw new InvalidOperationException("Viewport separation failed");
        PointMoveRequested-=CountPoint; TrajectoryRequested-=CountPath; SetStartFeedback(null); FitView(initialDetail);
    }
    private static void ArmMotionPreviewSelfTest()
    {
        var panel = new XYPreviewPanel();
        panel.Measure(new Size(900,720)); panel.Arrange(new Rect(0,0,900,720)); panel.UpdateLayout();
        int requests=0; panel.PointMoveRequested+=_=>requests++; panel.TrajectoryRequested+=_=>requests++;
        (Point Elbow, Point Tip) CheckPose(JointPose pose)
        {
            var links=panel.canvas.Children.OfType<Canvas>().First().Children.OfType<Line>()
                .Where(line=>line.Stroke==WorkbenchTheme.Blue || line.Stroke==WorkbenchTheme.Red || line.Stroke==Brushes.SlateBlue).ToArray();
            var (elbow,tip)=panel.Model.Forward(pose);
            Point[] joints=[panel.ToScreen(new Point()),panel.ToScreen(elbow),panel.ToScreen(tip)];
            if(links.Length!=2 || joints.Any(p=>!panel.Plot.Contains(p)))
                throw new InvalidOperationException($"Both arm links must be visible: shape={panel.shape.SelectedIndex}, branch={panel.branch.SelectedIndex}, detail={panel.detailView}");
            if(panel.canvas.Children.OfType<Canvas>().Skip(1).Any(layer=>layer.Clip is {} clip && joints.Any(p=>clip.Bounds.Contains(p))))
                throw new InvalidOperationException("Trajectory detail must not cover arm joints");
            for(int axis=0;axis<2;axis++) {
                if((new Point(links[axis].X1,links[axis].Y1)-joints[axis]).Length>1e-7 ||
                   (new Point(links[axis].X2,links[axis].Y2)-joints[axis+1]).Length>1e-7 ||
                   links[axis].StrokeThickness<4 || links[axis].StrokeDashArray is { Count: >0 })
                    throw new InvalidOperationException("Arm links must follow the displayed joint pose with clear solid strokes");
            }
            return (elbow,tip);
        }
        for(int selected=0;selected<4;selected++) for(int branch=0;branch<2;branch++) {
            panel.updating=true;
            panel.shape.SelectedIndex=selected; panel.branch.SelectedIndex=branch;
            panel.x.Text="260"; panel.y.Text="0"; panel.a.Text=selected==1?"290":"25"; panel.b.Text="30";
            panel.min1.Text="-135";panel.max1.Text="135";panel.min2.Text="-155";panel.max2.Text="155";
            panel.updating=false; panel.ShapeFields(); panel.Refresh();
            if(panel.pointTarget is null || selected>0 && panel.timing is null)
                throw new InvalidOperationException("Arm preview check needs a valid shape");
            foreach(bool detail in new[] {false,true}) {
                panel.FitView(detail);
                var first=CheckPose(panel.pointTarget.Value);
                bool moved=false;
                if(panel.timing is {} plan) {
                    foreach(double fraction in new[] {0.25,0.5,0.75,1.0}) {
                        panel.cursor.Value=plan.Duration*fraction; panel.RedrawViewport();
                        var current=CheckPose(plan.At(panel.cursor.Value).Pose);
                        moved|=(current.Elbow-first.Elbow).Length>0.1 && (current.Tip-first.Tip).Length>0.1;
                    }
                    if(!moved) throw new InvalidOperationException("Both arm links must move during every shape preview");
                    panel.cursor.Value=0;
                }
                panel.SetDeviceStatus("模拟设备运动（非实机）",false,true,25,false);
                (Point Elbow,Point Tip)? firstCommand=null;
                TelemetrySample? lastSample=null;
                foreach(double fraction in new[] {0.25,0.75}) {
                    var command=panel.timing is {} motion?motion.At(motion.Duration*fraction).Pose:
                        new JointPose(panel.pointTarget.Value.Q1*fraction,panel.pointTarget.Value.Q2*fraction);
                    var sample=new TelemetrySample(0,1,1,true,2,ExecutionTrajectory.Counts(0,command),ExecutionTrajectory.Counts(1,command),0,0,0,0);
                    panel.SetStartFeedback(sample); panel.RedrawViewport(); var current=CheckPose(sample.Command);
                    if(firstCommand is {} from && ((current.Elbow-from.Elbow).Length<0.1 || (current.Tip-from.Tip).Length<0.1))
                        throw new InvalidOperationException("Both links must move with device feedback, including point moves");
                    firstCommand=current; lastSample=sample;
                }
                panel.SetDeviceStatus(null,false,false,0,false); panel.RedrawViewport(); CheckPose(lastSample!.Command);
                panel.liveLayer.IsChecked=false; panel.RedrawViewport(); CheckPose(panel.pointTarget.Value);
                panel.liveLayer.IsChecked=true; panel.RedrawViewport(); CheckPose(lastSample.Command);
                panel.SetStartFeedback(null); panel.RedrawViewport(); CheckPose(panel.pointTarget.Value);
            }
        }
        if(requests!=0) throw new InvalidOperationException("Preview sent a motion request");
    }
    internal void CheckViewportLayout()
    {
        if(canvas.ActualWidth<400 || canvas.ActualHeight<260 || double.IsFinite(canvas.Height))
            throw new InvalidOperationException($"Viewport squeezed or fixed: {canvas.ActualWidth} x {canvas.ActualHeight}");
        var bottom=run.TranslatePoint(new Point(0,run.ActualHeight),this);
        if(bottom.Y>ActualHeight || bottom.X<0) throw new InvalidOperationException("Motion actions clipped");
    }
}
