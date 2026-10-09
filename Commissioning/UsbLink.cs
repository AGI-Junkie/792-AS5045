using Microsoft.Win32;
using System.IO;

namespace TwoLinkCommissioning;

// The firmware's own USB CDC port (J6) versus the KitProg UART bridge or any other adapter.  The firmware puts its
// answers on USB only while the host holds DTR, so the host has to know which port it is opening.  VID/PID are the
// ones in the firmware's device descriptor (UsbCdc_DEVICE0_DESCR: 0x04B4 / 0xF232).
internal static class UsbLink
{
    internal const string DeviceKey = "VID_04B4&PID_F232";

    // Windows records every USB serial port under Enum\USB\<VID&PID>\<instance>\Device Parameters\PortName.
    // ponytail: the entry outlives the device, so a COM number once used by the firmware and later reused by another
    // adapter reads as USB; Windows keeps a port number with its device, so this does not happen in practice.
    internal static bool IsFirmwareCdc(string portName)
    {
        try {
            using var device = Registry.LocalMachine.OpenSubKey(@"SYSTEM\CurrentControlSet\Enum\USB\" + DeviceKey);
            if (device is null) return false;
            foreach (string instance in device.GetSubKeyNames()) {
                using var parameters = device.OpenSubKey(instance + @"\Device Parameters");
                if (string.Equals(parameters?.GetValue("PortName") as string, portName, StringComparison.OrdinalIgnoreCase)) return true;
            }
        }
        catch (Exception ex) when (ex is System.Security.SecurityException or UnauthorizedAccessException or IOException) { }
        return false;
    }
}
