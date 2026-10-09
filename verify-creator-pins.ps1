param(
    [string]$ProjectPath = (Join-Path $PSScriptRoot 'DRV8825.cydsn'),
    [switch]$NamedInstances,
    [switch]$HardwareUart
)

# Default mode accepts functional or legacy names; -NamedInstances requires functional names.
$ErrorActionPreference = 'Stop'
$expected = @'
Logical,DefaultInstance,Port,Bit,Mode,Drive,Initial
CSN,CSN,12,1,O,110,1
CLK,CLK,12,0,O,110,1
DO,Pin_1,1,3,I,001,0
DIR1,Pin_2,2,0,O,110,0
STEP1,Pin_3,2,1,O,110,0
SLEEP1,Pin_4,2,2,O,110,0
RST1,Pin_5,2,3,O,110,0
M1_2,Pin_6,2,4,O,110,0
M1_1,Pin_7,2,5,O,110,1
M1_0,Pin_8,2,6,O,110,1
EN1,Pin_9,2,7,O,110,1
DIR2,Pin_10,12,4,O,110,0
STEP2,Pin_11,12,3,O,110,1
SLEEP2,Pin_12,12,2,O,110,0
RST2,Pin_13,1,2,O,110,0
M2_2,Pin_14,1,4,O,110,0
M2_1,Pin_15,1,5,O,110,1
M2_0,Pin_16,1,6,O,110,1
EN2,Pin_17,1,7,O,110,1
LIMIT1,Pin_18,0,3,I,010,1
LIMIT2,Pin_19,0,4,I,010,1
ARM2_NPN,Pin_20,1,0,I,010,1
DBG_TX,Pin_21,12,7,O,110,1
DBG_RX,Pin_22,12,6,I,010,1
FAULT,Pin_23,12,5,I,010,1
'@ | ConvertFrom-Csv

$verilog = Get-Content -LiteralPath (Join-Path $ProjectPath 'codegentemp\DRV8825.v') -Raw
$fitter = Get-Content -LiteralPath (Join-Path $ProjectPath 'Generated_Source\PSoC5\cyfitter.h') -Raw
$main = Get-Content -LiteralPath (Join-Path $ProjectPath 'main.c') -Raw
$hw = @{}
foreach ($flag in @('HW_SSI', 'HW_STEP', 'HW_PINIRQ', 'HW_TXDMA')) {
    $define = [regex]::Match($main, '(?m)^#define\s+' + $flag + '\s+(?<value>[^\r\n]+)')
    $value = [regex]::Match($define.Groups['value'].Value, '^\(?([01])u?\)?(?:\s|$)')
    if ($define.Success -and -not $value.Success) { throw "Cannot read $flag default in main.c" }
    $hw[$flag] = $value.Success -and $value.Groups[1].Value -eq '1'
}
$hasUart = $HardwareUart -or $hw.HW_TXDMA -or $verilog -match '\bHostUart\s*\('
$blocks = [regex]::Matches($verilog, '(?s)\bcy_psoc3_pins_v1_10\s*#\((?<params>.*?)\)\s*(?<name>[A-Za-z][A-Za-z0-9_]*)\s*\((?<ports>.*?)\);')
$issues = [System.Collections.Generic.List[string]]::new()
$byName = @{}
foreach ($block in $blocks) {
    $name = $block.Groups['name'].Value
    if ($byName.ContainsKey($name)) { $issues.Add("Duplicate Verilog instance: $name") }
    $byName[$name] = $block
}
if ($blocks.Count -ne 25) { $issues.Add("Expected 25 Pins instances; found $($blocks.Count)") }
$defaultCount = @($byName.Keys | Where-Object { $_ -cmatch '^Pin_[0-9]+$' }).Count
if ($NamedInstances -and $defaultCount) { $issues.Add("Functional names incomplete: $defaultCount default Pin_* names remain") }

function Check-Value([string]$Label, [string]$Actual, [string]$Wanted) {
    if ($Actual -cne $Wanted) { $issues.Add("${Label}: expected [$Wanted], found [$Actual]") }
}

function Port-Net([string]$Ports, [string]$Terminal) {
    $value = [regex]::Match($Ports, '\.' + [regex]::Escape($Terminal) + '\(\s*(?<net>[^()]+)\)').Groups['net'].Value
    ($value.Trim([char[]]'{} ') -replace '\[0:0\]$', '')
}

function Component-Ports([string]$Name) {
    $match = [regex]::Match($verilog, '(?s)\b' + [regex]::Escape($Name) + '\s*\((?<ports>.*?)\);')
    if (-not $match.Success) { $issues.Add("Missing hardware instance: $Name") }
    $match.Groups['ports'].Value
}

function Check-Connection([string]$Label, [string]$Actual, [string]$Wanted) {
    if (-not $Wanted -or $Actual -cne $Wanted) { $issues.Add("${Label}: expected connection [$Wanted], found [$Actual]") }
}

$pinPorts = @{}
$report = foreach ($pin in $expected) {
    # DO is reserved by Creator; its functional instance is AS5045_DO.
    $functionalName = if ($pin.Logical -eq 'DO') { 'AS5045_DO' } else { $pin.Logical }
    $name = if ($NamedInstances -or $byName.ContainsKey($functionalName)) { $functionalName } else { $pin.DefaultInstance }
    if (-not $byName.ContainsKey($name)) {
        $issues.Add("Missing instance: $name ($($pin.Logical))")
        continue
    }
    $block = $byName[$name]
    $ports = $block.Groups['ports'].Value
    $pinPorts[$pin.Logical] = $ports
    $params = @{}
    foreach ($entry in [regex]::Matches($block.Groups['params'].Value, '\.(?<key>\w+)\((?<value>[^()]*)\)')) {
        $params[$entry.Groups['key'].Value] = $entry.Groups['value'].Value.Trim()
    }
    $wanted = @{
        width = '1'; pin_mode = '"' + $pin.Mode + '"'
        drive_mode = "3'b" + $pin.Drive; init_dr_st = "1'b" + $pin.Initial
        ibuf_enabled = "1'b1"; intr_mode = "2'b00"
        output_conn = "1'b0"; oe_conn = "1'b0"
    }
    $hardwareOutput = ($hasUart -and $pin.Logical -eq 'DBG_TX') -or
        ($hw.HW_SSI -and $pin.Logical -in @('CSN','CLK')) -or
        ($hw.HW_STEP -and $pin.Logical -in @('STEP1','STEP2'))
    if ($hardwareOutput) { $wanted.output_conn = "1'b1" }
    if ($hw.HW_PINIRQ -and $pin.Logical -eq 'FAULT') { $wanted.intr_mode = "2'b10" }
    if ($hw.HW_PINIRQ -and $pin.Logical -eq 'LIMIT2') { $wanted.intr_mode = "2'b01" }
    foreach ($key in $wanted.Keys) { Check-Value "$name.$key" $params[$key] $wanted[$key] }
    if ($pin.Mode -eq 'I') { Check-Value "$name.input_sync" $params['input_sync'] "1'b0" }
    if (-not $hardwareOutput -and (Port-Net $ports 'y') -cne "1'b0") {
        $issues.Add("$name has a hardware-driven y terminal")
    }
    # ponytail: input HW visibility is front-end-only; verify its checkbox in Creator.
    $hardwareInput = ($hasUart -and $pin.Logical -eq 'DBG_RX') -or ($hw.HW_SSI -and $pin.Logical -eq 'DO')
    $fb = Port-Net $ports 'fb'
    if (-not $hardwareInput -and (-not $fb -or [regex]::Matches($verilog, '\b' + [regex]::Escape($fb) + '\b').Count -ne 2)) {
        $issues.Add("$name input feedback net is missing or has hardware consumers")
    }
    $pc = [regex]::Match($fitter, '(?m)^#define\s+' + [regex]::Escape($name) + '__0__PC\s+(?<value>\S+)').Groups['value'].Value
    Check-Value "$name physical pin" $pc "CYREG_PRT$($pin.Port)_PC$($pin.Bit)"
    foreach ($field in @(@('PORT', $pin.Port), @('SHIFT', $pin.Bit))) {
        $actual = [regex]::Match($fitter, '(?m)^#define\s+' + [regex]::Escape($name) + '__0__' + $field[0] + '\s+(?<value>\S+)').Groups['value'].Value
        Check-Value "$name $($field[0])" $actual ($field[1] + 'u')
    }
    [pscustomobject]@{ Function = $pin.Logical; Instance = $name; Pin = "P$($pin.Port)[$($pin.Bit)]"; Mode = $pin.Mode; Drive = $pin.Drive; Initial = $pin.Initial }
}
if ($hw.HW_SSI) {
    $ssi = Component-Ports 'Ssi'
    foreach ($route in @(@('CSN','y','ss'), @('CLK','y','sclk'), @('DO','fb','miso'))) {
        Check-Connection "Ssi.$($route[2])" (Port-Net $ssi $route[2]) (Port-Net $pinPorts[$route[0]] $route[1])
    }
}
if ($hw.HW_STEP) {
    $arm1 = Component-Ports 'StepArm1'; $arm2 = Component-Ports 'StepArm2'
    Check-Connection 'StepArm2.pwm -> STEP1' (Port-Net $arm2 'pwm') (Port-Net $pinPorts.STEP1 'y')
    $step2 = Port-Net $pinPorts.STEP2 'y'; $pwm1 = Port-Net $arm1 'pwm'
    if (-not $step2 -or -not $pwm1 -or $verilog -notmatch ('\bassign\s+' + [regex]::Escape($step2) + '\s*=\s*~\s*' + [regex]::Escape($pwm1) + '\s*;')) {
        $issues.Add('STEP2 inversion: expected ~StepArm1.pwm')
    }
}
if ($hw.HW_PINIRQ) {
    foreach ($route in @(@('FAULT','FaultIsr'), @('LIMIT2','LimitIsr'))) {
        Check-Connection "$($route[1]).int_signal" (Port-Net (Component-Ports $route[1]) 'int_signal') (Port-Net $pinPorts[$route[0]] 'interrupt')
    }
}
if ($hasUart) {
    $uart = Component-Ports 'HostUart'
    foreach ($route in @(@('DBG_RX','fb','rx'), @('DBG_TX','y','tx'))) {
        Check-Connection "HostUart.$($route[2]) -> $($route[0])" (Port-Net $uart $route[2]) (Port-Net $pinPorts[$route[0]] $route[1])
    }
    $header = Get-Content -LiteralPath (Join-Path $ProjectPath 'Generated_Source\PSoC5\HostUart.h') -Raw
    $uartSettings = @{ RX_INTERRUPT_ENABLED = '1'; RX_BUFFER_SIZE = '256'; TX_INTERRUPT_ENABLED = '1'; TX_BUFFER_SIZE = '256' }
    $fifoTx = $hw.HW_TXDMA -or ($header -match '#define\s+HostUart_TX_INTERRUPT_ENABLED\s+\(0u\)' -and
                              $header -match '#define\s+HostUart_TX_BUFFER_SIZE\s+\(4u\)')
    if ($fifoTx) { $uartSettings.TX_INTERRUPT_ENABLED = '0'; $uartSettings.TX_BUFFER_SIZE = '4' }
    foreach ($setting in $uartSettings.Keys) {
        $value = $uartSettings[$setting]
        if ($header -notmatch ('#define\s+HostUart_' + $setting + '\s+\(' + $value + 'u\)')) { $issues.Add("UART configuration mismatch: $setting") }
    }
    if ($hw.HW_TXDMA) {
        $mask = [regex]::Match($header, '(?m)^#define\s+HostUart_INIT_TX_INTERRUPTS_MASK(?:[^\r\n]*\\\r?\n)*[^\r\n]*').Value
        foreach ($bit in @('COMPLETE','FIFO_EMPTY','FIFO_FULL','FIFO_NOT_FULL')) {
            $value = if ($bit -eq 'FIFO_NOT_FULL') { '1' } else { '0' }
            if ($mask -notmatch ('\(' + $value + '\s*<<\s*HostUart_TX_STS_' + $bit + '_SHIFT\)')) {
                $issues.Add("UART DMA request mask mismatch: $bit")
            }
        }
        $sync = Component-Ports 'TxReqSync'; $dma = Component-Ports 'TxDma'
        Check-Connection 'TxReqSync.s_in' (Port-Net $sync 's_in') (Port-Net $uart 'tx_interrupt')
        Check-Connection 'TxDma.drq' (Port-Net $dma 'drq') (Port-Net $sync 's_out')
        Check-Connection 'TxReqSync.clock' (Port-Net $sync 'clock') (Port-Net (Component-Ports 'ClkDma') 'clock_out')
        $dmaParams = [regex]::Match($verilog, '(?s)\bcy_dma_v1_0\s*#\((?<params>.*?)\)\s*TxDma\s*\(').Groups['params'].Value
        if ($dmaParams -notmatch '\.drq_type\(2''b01\)') { $issues.Add('TxDma.drq_type must be level (01)') }
    }
    if ($verilog -notmatch 'BaudRate=115200,') { $issues.Add('Expected UART baud 115200') }
}
# main.c addresses pins by register; every PIN_<name> there must be the port/bit the table (and so TopDesign) says.
$defs = @{}
foreach ($m in [regex]::Matches($main, '(?m)^#define\s+PIN_(?<n>\w+)\s+CYREG_PRT(?<p>\d+)_PC(?<b>\d+)')) {
    $defs[$m.Groups['n'].Value] = "CYREG_PRT$($m.Groups['p'].Value)_PC$($m.Groups['b'].Value)"
}
foreach ($pin in $expected) {
    $want = "CYREG_PRT$($pin.Port)_PC$($pin.Bit)"
    if (-not $defs.ContainsKey($pin.Logical)) { $issues.Add("main.c has no PIN_$($pin.Logical)") }
    elseif ($defs[$pin.Logical] -cne $want) { $issues.Add("main.c PIN_$($pin.Logical) is $($defs[$pin.Logical]) but TopDesign has $want") }
}
$rawUses = [regex]::Matches(($main -replace '(?m)^#define\s+PIN_\w+\s+CYREG_PRT\d+_PC\d+', ''), 'CYREG_PRT\d+_PC\d+').Count
if ($rawUses) { $issues.Add("main.c uses $rawUses raw CYREG_PRT pin name(s) outside its PIN_* table") }
$report | Format-Table -AutoSize
if ($issues.Count) { throw ($issues -join [Environment]::NewLine) }
Write-Output 'PASS: 25 physical pins, electrical settings, interrupts and hardware routes verified; main.c PIN_* names match.'
if (-not $defaultCount) {
    Write-Output 'PASS: all 25 functional instance names verified (DO = AS5045_DO).'
} else {
    Write-Output "PASS: $defaultCount legacy Pin_* names verified; use -NamedInstances to require functional names."
}
if ($hasUart) {
    $txMode = if ($hw.HW_TXDMA) { 'TX FIFO 4 bytes with DMA' } elseif ($fifoTx) { 'TX FIFO 4 bytes with CPU' } else { 'TX interrupt buffer 256 bytes' }
    Write-Output "PASS: HostUart 115200, RX P12[6] interrupt buffer 256 bytes, TX P12[7] $txMode."
}
Write-Output 'Input HW checkbox visibility requires Creator inspection; it is absent from generated Verilog.'
