<#
  LEGACY ONLY: stage2-v7 requires the matching WPF host; this script cannot
  enable, home or jog v7. Raw STOP remains available.
  jog.ps1 - hold-to-move keyboard jog for the legacy AS5045_enc firmware
  (KEYBOARD_JOG=1 in DRV8825.cydsn\main.c).

    a / d          arm1 (42 motor, stepper2) left / right
    Left / Right   arm2 (28 motor, stepper1) left / right
    c              explicitly enable after input/encoder checks
    r              two-touch home: arm1, 125-degree straight, then arm2
    Space          latched stop (C required before moving again)
    i              print live switch/fault GPIOs without moving
    p              print both positions + receive statistics
    e              print one raw AS5045 frame (use while adjusting the magnet)
    h              firmware help
    q / Esc        stop and quit

  Hold a key to move, release to stop.  The script sends the key state every
  40 ms; the firmware stops everything if it hears nothing for 250 ms, so
  closing this window (or a crash) also stops the arm.

  Wire protocol: every byte is odd (bit 0 set) and is sent THREE times as
  separate writes about 1 ms apart - the firmware receives by polling a pin
  and re-syncs on the rising edge of the start bit, which only exists one
  bit after the falling edge when d0 = 1.  Copies must not be back to back.
    0x41 | (mask << 1)   heartbeat / key state
    '1' '3' 0x71 's' '?' commands (firmware de-duplicates the copies)
                         (physical R maps to 0x71: protocol bytes must be odd)

  Input is accepted only while the original terminal window has focus.
  Losing focus sends a latched stop. Prefer the WPF commissioning app when
  using Windows Terminal tabs (tabs can share a window handle).

  Usage:  powershell -ExecutionPolicy Bypass -File .\jog.ps1 [-Port COM8]
#>
param(
    [string]$Port = "",
    [int]$Baud = 115200
)

Add-Type -Namespace Jog -Name Native -MemberDefinition @'
[DllImport("user32.dll")] public static extern short GetAsyncKeyState(int vKey);
[DllImport("user32.dll")] public static extern System.IntPtr GetForegroundWindow();
'@

function Test-KeyDown([int]$vk)
{
    return (([int][Jog.Native]::GetAsyncKeyState($vk)) -band 0x8000) -ne 0
}

if (-not $Port)
{
    $dev = Get-CimInstance Win32_PnPEntity |
           Where-Object { $_.Name -match 'KitProg.*\((COM\d+)\)' } |
           Select-Object -First 1
    if ($dev -and ($dev.Name -match '\((COM\d+)\)')) { $Port = $Matches[1] }
    else { throw "KitProg USB-UART not found; pass -Port COMx" }
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud,
        ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$sp.Encoding    = [System.Text.Encoding]::ASCII
$sp.ReadTimeout = 10
$sp.Open()
Write-Host "stage1: $Port @ $Baud. C enable, R home, Space STOP. Hold a/d or Left/Right. Q/Esc quit."
$controlWindow = [Jog.Native]::GetForegroundWindow()
$wasFocused = $true

# Send one protocol byte as three separate, spaced writes.
function Send-Byte([byte]$b)
{
    $one = [byte[]]@($b)
    for ($k = 0; $k -lt 3; $k++)
    {
        $sp.Write($one, 0, 1)
        Start-Sleep -Milliseconds 1
    }
}

# virtual-key codes
$VK_A = 0x41; $VK_D = 0x44; $VK_LEFT = 0x25; $VK_RIGHT = 0x27
$VK_Q = 0x51; $VK_R = 0x52; $VK_ESC = 0x1B
$edgeKeys = @(
    @{ vk = 0x43; c = 0x63 },             # C -> explicit enable
    @{ vk = 0x50; c = [byte][char]'s' },   # P -> status
    @{ vk = 0x45; c = [byte][char]'e' },   # E -> one raw encoder frame (magnet adjustment)
    @{ vk = $VK_R; c = 0x71 },              # R -> home; ASCII r (0x72) is invalid here
    @{ vk = 0x49; c = [byte][char]'i' },    # I -> live GPIOs, no motor command
    @{ vk = 0x48; c = [byte][char]'?' }    # H -> help
)
$edgeState = @{}

try
{
    Send-Byte 0x79                       # connecting never resumes an old motion
    while ($true)
    {
        # swallow the characters the held keys type into this console
        # (KeyAvailable throws when stdin is redirected, e.g. run headless)
        if (-not [Console]::IsInputRedirected)
        {
            try { while ([Console]::KeyAvailable) { [void][Console]::ReadKey($true) } }
            catch { }
        }

        $focused = [Jog.Native]::GetForegroundWindow() -eq $controlWindow
        if (-not $focused)
        {
            if ($wasFocused) { Send-Byte 0x79 }
            $wasFocused = $false
            Send-Byte 0x41
            $rx = $sp.ReadExisting()
            if ($rx) { Write-Host -NoNewline $rx }
            Start-Sleep -Milliseconds 35
            continue
        }
        $wasFocused = $true
        if ((Test-KeyDown $VK_Q) -or (Test-KeyDown $VK_ESC)) { break }
        if (Test-KeyDown 0x20)
        {
            Send-Byte 0x79
            Send-Byte 0x41
            $rx = $sp.ReadExisting()
            if ($rx) { Write-Host -NoNewline $rx }
            Start-Sleep -Milliseconds 35
            continue
        }

        foreach ($k in $edgeKeys)
        {
            $down = Test-KeyDown $k.vk
            if ($down -and -not $edgeState[$k.vk]) { Send-Byte $k.c }
            $edgeState[$k.vk] = $down
        }

        $mask = 0
        if (Test-KeyDown $VK_A)     { $mask = $mask -bor 1 }
        if (Test-KeyDown $VK_D)     { $mask = $mask -bor 2 }
        if (Test-KeyDown $VK_LEFT)  { $mask = $mask -bor 4 }
        if (Test-KeyDown $VK_RIGHT) { $mask = $mask -bor 8 }
        Send-Byte ([byte](0x41 -bor ($mask -shl 1)))

        $rx = $sp.ReadExisting()
        if ($rx) { Write-Host -NoNewline $rx }

        Start-Sleep -Milliseconds 35
    }
}
finally
{
    try
    {
        Send-Byte 0x79                       # cancels home as well as jogging
        Start-Sleep -Milliseconds 150
        $rx = $sp.ReadExisting()
        if ($rx) { Write-Host -NoNewline $rx }
    }
    catch { }
    $sp.Close()
    Write-Host "`njog: stopped, port closed."
}
