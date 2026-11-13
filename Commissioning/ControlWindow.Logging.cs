using System.IO;
using System.Windows;
using System.Windows.Controls;

namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private readonly ComboBox logLevelBox = new() { Width = 140, Margin = new Thickness(4, 2, 4, 2) };
    private readonly TextBlock logLevelLabel = new() { VerticalAlignment = VerticalAlignment.Center };
    private readonly Queue<(LogLevel Level, string Entry)> recentLog = new();
    private LogLevel minimumLogLevel = LogLevel.Info;

    private void RefreshLogLanguage()
    {
        logLevelLabel.Text = Lang.T("log.level");
        logLevelBox.ToolTip = Lang.T("log.level.tip");
        System.Windows.Automation.AutomationProperties.SetName(logLevelBox, Lang.T("log.level"));
        foreach (ComboBoxItem item in logLevelBox.Items)
            item.Content = Lang.T("log." + item.Tag);
    }

    private void RefreshLog()
    {
        log.Text = string.Concat(recentLog.Where(e => e.Level >= minimumLogLevel).Select(e => e.Entry));
    }

    internal static void LoggingSelfTest()
    {
        static void Check(bool ok) { if (!ok) throw new InvalidOperationException("Log level self-test failed"); }
        Check(LogLevels.Classify("TELEM t=1 err=0") == LogLevel.Trace);
        Check(LogLevels.Classify("STATE fw=stage2-v11 fault=0 enc=1") == LogLevel.Debug);
        Check(LogLevels.Classify("STATE fw=stage2-v11 fault=1 enc=1") == LogLevel.Error);
        Check(LogLevels.Classify("LINK tx_dropped=0") == LogLevel.Debug);
        Check(LogLevels.Classify("HOST connected COM3") == LogLevel.Info);
        Check(LogLevels.Classify("PTP refused: stale reference") == LogLevel.Warning);
        Check(LogLevels.Classify("ARM1 SLIP WARN: 31 lsb") == LogLevel.Warning);
        Check(LogLevels.Classify("[12:34:56.789] FAULT_SNAPSHOT reason=deadman") == LogLevel.Error);
        var window = new ControlWindow();
        window.recording = new StreamWriter(new MemoryStream()) { AutoFlush = true };
        window.HandleLine("TELEM t=1"); // filtering never prevents the protocol handler from receiving text
        window.Append("HOST connected COM3");
        window.Append("HOST timeout", LogLevel.Error);
        Check(!window.log.Text.Contains("TELEM") && window.log.Text.Contains("connected"));
        window.logLevelBox.SelectedIndex = (int)LogLevel.Error;
        Check(!window.log.Text.Contains("connected") && window.log.Text.Contains("timeout"));
        window.HandleLine("STATE fw=stage2-v11 enabled=0 home=0 homed1=0 homed2=0 fault=0 enc=1 dir=1 enc2=1 dir2=1 arm2fb=1");
        Check(window.compatible && !window.log.Text.Contains("STATE fw="));
        window.logLevelBox.SelectedIndex = (int)LogLevel.Trace;
        Check(window.log.Text.Contains("TELEM"));
        var stream = (MemoryStream)window.recording.BaseStream;
        string recorded = System.Text.Encoding.UTF8.GetString(stream.ToArray());
        Check(recorded.Contains("] TELEM t=1") && !recorded.Contains("[Trace]"));
        window.recording.Dispose(); window.recording = null;
        window.Append("HOST exception", LogLevel.Error);
        Check(window.log.Text.Contains("exception"));
        for (int i = 0; i < 1001; i++) window.Append("HOST retained " + i);
        window.RefreshLog();
        Check(window.recentLog.Count == 1000 && !window.log.Text.Contains("HOST retained 0"));
        Check(window.log.Text.Contains("HOST retained 1000"));
        window.log.Text = new string('x', 400001) + "keep" + Environment.NewLine;   // a selected log still has a ceiling, and keeps the selection
        window.log.Select(window.log.Text.Length - 4 - Environment.NewLine.Length, 4);
        window.Append("HOST after the cap");
        Check(window.log.Text.Length < 60000 && window.log.SelectedText == "keep" && window.log.Text.EndsWith("HOST after the cap" + Environment.NewLine));
        // A slow UI lease line says what held the dispatcher.
        window.PublishHeartbeatLease();
        var frame = new System.Windows.Threading.DispatcherFrame();
        window.Dispatcher.BeginInvoke(System.Windows.Threading.DispatcherPriority.Background, () => { Thread.Sleep(150); frame.Continue = false; });
        System.Windows.Threading.Dispatcher.PushFrame(frame);
        window.PublishHeartbeatLease();
        Check(window.log.Text.Contains("HOST UI lease gap max=") && window.log.Text.Contains("(longest Background operation 1"));
        window.Close();
    }
}
