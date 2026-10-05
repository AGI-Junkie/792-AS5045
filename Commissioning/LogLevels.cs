namespace TwoLinkCommissioning;

internal enum LogLevel { Trace, Debug, Info, Warning, Error }

internal static class LogLevels
{
    private static readonly string[] Diagnostic = ["TELEM_DIAG", "TIMING ", "ENC_DIAG", "PTP_RX ", "PTP_STATE ",
            "MOTION_RATES ", "PTP_CHECK ", "CONTROL ", "STATE ", "LINK ", "TRAJ_STATE ", "STOP context:"];

    // Classify existing wire text without changing the protocol or raw recordings.
    internal static LogLevel Classify(string line)
    {
        if (line.Length >= 15 && line[0] == '[' && line[13] == ']') line = line[15..];
        if (line.StartsWith("FAULT_SNAPSHOT ") || line.StartsWith("LAST_STOP ") ||
            line.StartsWith("STOP:") || line.StartsWith("UART RX fault:") ||
            (line.StartsWith("STATE ") && line.Contains(" fault=1 ")))
            return LogLevel.Error;
        if (line.StartsWith("LIMIT:") || line.Contains("SLIP WARN:") ||
            line.Contains(" refused") || line.Contains(" RX dropped:") ||
            line.StartsWith("TRAJ RX framing discarded")) return LogLevel.Warning;
        if (line.StartsWith("TELEM ")) return LogLevel.Trace;
        return Diagnostic.Any(prefix => line.StartsWith(prefix, StringComparison.Ordinal)) ? LogLevel.Debug : LogLevel.Info;
    }
}
