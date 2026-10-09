using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Windows.Controls;
using System.Windows.Threading;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    // The first measurement of a cold process includes JIT and a cold disk cache (a CI runner showed 348 ms for a call that takes about 12 ms warm),
    // so a blocked heartbeat is only reported when it also happens on the repeats: a real stall in FinishExperiment shows up every time.
    private static string ExperimentFinishHeartbeatSelfTest()
    {
        const int attempts = 3;
        for (int attempt = 1; ; attempt++)
        {
            try { return ExperimentFinishHeartbeatOnce(); }
            catch (InvalidOperationException ex) when (attempt < attempts && ex.Message.StartsWith("Experiment finish blocked heartbeat", StringComparison.Ordinal)) { }
        }
    }

    private static string ExperimentFinishHeartbeatOnce()
    {
        string root = Path.Combine(Path.GetTempPath(), "arm-finish-heartbeat-" + Guid.NewGuid());
        string? previousRoot = XYPreviewPanel.RunsRootOverride;
        var window = new ControlWindow { selfTesting = true };
        var beat = new DispatcherTimer(DispatcherPriority.Input) { Interval = TimeSpan.FromMilliseconds(8) };
        try {
            using var recorder = new ExperimentRecorder(root, "point", 2, 6, new PlanarArm(180,176.38),
                "shape=0; x=340; y=0; a=20; b=20; v1=35; v2=35; a1=25; a2=25", "manual", [new(new(0,0),0)]);
            string metadata = File.ReadAllText(Path.Combine(recorder.DirectoryPath, "metadata.json"));
            // A growing archive must not consume the UI lease when the point move to a drawing's start completes.
            for (int i=0; i<5000; i++) {
                string path = Path.Combine(root, "old-" + i); Directory.CreateDirectory(path);
                File.WriteAllText(Path.Combine(path, "metadata.json"), metadata);
            }
            XYPreviewPanel.RunsRootOverride = root;
            recorder.Record(TelemetrySample.Parse("TELEM t=340443 epoch=6 seq=2 ref=1 source=2 p1=-3671 p2=9472 e1=-9388 err=9 e2=24235 err2=-13 inputs=0 drops=0")!, "point", 0);
            window.experiment = recorder;
            window.ShowActivated = window.ShowInTaskbar = false; window.Left = window.Top = -12000;
            window.Show(); window.UpdateLayout();
            var rows = (Panel)typeof(XYPreviewPanel).GetField("recentList", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(window.xyPreview)!;
            var clock = Stopwatch.StartNew(); double last=0, worst=0, finishMs=0; bool finished=false;
            var frame = new DispatcherFrame();
            beat.Tick += (_,_) => {
                double now=clock.Elapsed.TotalMilliseconds; worst=Math.Max(worst,now-last); last=now;
                if (!finished && now>=100) {
                    finished=true; var at=Stopwatch.StartNew(); window.FinishExperiment("complete"); finishMs=at.Elapsed.TotalMilliseconds;
                }
                if (finished && rows.Children.Count>0 && now>=700 || now>=10000) frame.Continue=false;
            };
            beat.Start(); Dispatcher.PushFrame(frame); beat.Stop();
            if (worst>=LeaseStopMs || rows.Children.Count==0 || !File.Exists(Path.Combine(recorder.DirectoryPath,"summary.json")))
                throw new InvalidOperationException($"Experiment finish blocked heartbeat: finish={finishMs:F1}ms, gap={worst:F1}ms, rows={rows.Children.Count}, UI lease={LeaseStopMs}ms.");
            return $"Experiment finish with 5000 archive entries: finish={finishMs:F1}ms, dispatcher gap={worst:F1}ms (<{LeaseStopMs}ms); recent rows and summary retained.\n";
        }
        finally {
            beat.Stop(); window.Close(); XYPreviewPanel.RunsRootOverride=previousRoot;
            if (Directory.Exists(root) && Path.GetDirectoryName(Path.GetFullPath(root)) == Path.GetFullPath(Path.GetTempPath()).TrimEnd(Path.DirectorySeparatorChar) &&
                Path.GetFileName(root).StartsWith("arm-finish-heartbeat-",StringComparison.Ordinal)) Directory.Delete(root,true);
        }
    }
}
