namespace TwoLinkCommissioning;

public sealed partial class ControlWindow
{
    private static string UsbLinkSelfTest()
    {
        byte[] packet = [0x81, 0xa1, 0x83, 0xa3, 0x85, 0xa5];
        var uart = new List<(int Offset, int Count)>();
        var usb = new List<(int Offset, int Count)>();
        WriteControlFrame((_, o, n) => uart.Add((o, n)), packet);
        WriteControlFrame((_, o, n) => usb.Add((o, n)), packet, single: true);
        if (uart.Count != packet.Length * 2 || uart.Any(w => w.Count != 1))
            throw new InvalidOperationException("UART frame must stay two paced copies per byte.");
        if (usb.Count != 1 || usb[0] != (0, packet.Length))
            throw new InvalidOperationException("USB CDC frame must be one write of one copy.");
        if (UsbLink.IsFirmwareCdc("COM_NO_SUCH_PORT"))
            throw new InvalidOperationException("An unknown port must not be taken for the USB CDC device.");
        return $"USB link: {packet.Length}-byte frame = {uart.Count} paced writes on the UART, 1 write on USB CDC; an unknown port is not the USB device.\n";
    }
}
