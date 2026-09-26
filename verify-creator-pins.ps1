param(
    [string]$ProjectPath = (Join-Path $PSScriptRoot 'DRV8825.cydsn'),
    [switch]$NamedInstances,
    [switch]$HardwareUart
)

# Default mode checks the before-rename CSN, CLK, Pin_1..Pin_23 layout.
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

$report = foreach ($pin in $expected) {
    $name = if ($NamedInstances) { $pin.Logical } else { $pin.DefaultInstance }
    # DO is reserved by Creator; its functional instance is AS5045_DO.
    if ($NamedInstances -and $pin.Logical -eq 'DO') { $name = 'AS5045_DO' }
    if (-not $byName.ContainsKey($name)) {
        $issues.Add("Missing instance: $name ($($pin.Logical))")
        continue
    }
    $block = $byName[$name]
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
    if ($HardwareUart -and $name -eq 'DBG_TX') { $wanted.output_conn = "1'b1" }
    foreach ($key in $wanted.Keys) { Check-Value "$name.$key" $params[$key] $wanted[$key] }
    if ($pin.Mode -eq 'I') { Check-Value "$name.input_sync" $params['input_sync'] "1'b0" }
    if (-not ($HardwareUart -and $name -eq 'DBG_TX') -and $block.Groups['ports'].Value -notmatch '\.y\(\{1''b0\}\)') {
        $issues.Add("$name has a hardware-driven y terminal")
    }
    # ponytail: input HW visibility is front-end-only; verify its checkbox in Creator.
    $fb = [regex]::Match($block.Groups['ports'].Value, '\.fb\(\{(?<net>[A-Za-z][A-Za-z0-9_]*)\[0:0\]\}\)')
    if (-not ($HardwareUart -and $name -eq 'DBG_RX') -and (-not $fb.Success -or [regex]::Matches($verilog, '\b' + [regex]::Escape($fb.Groups['net'].Value) + '\b').Count -ne 2)) {
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
if ($HardwareUart) {
    $uart = [regex]::Match($verilog, '(?s)UART_v2_50_\d+\s+HostUart\s*\((?<ports>.*?)\);').Groups['ports'].Value
    foreach ($route in @(@('DBG_RX','fb','rx'), @('DBG_TX','y','tx'))) {
        $ports = $byName[$route[0]].Groups['ports'].Value
        $net = [regex]::Match($ports, '\.' + $route[1] + '\(\{(?<net>\w+)\}\)').Groups['net'].Value
        if (-not $net -or $uart -notmatch ('\.' + $route[2] + '\(' + [regex]::Escape($net) + '\)')) {
            $issues.Add("HostUart.$($route[2]) does not connect to $($route[0])")
        }
    }
    $header = Get-Content -LiteralPath (Join-Path $ProjectPath 'Generated_Source\PSoC5\HostUart.h') -Raw
    foreach ($setting in @('RX_INTERRUPT_ENABLED','TX_INTERRUPT_ENABLED','RX_BUFFER_SIZE','TX_BUFFER_SIZE')) {
        $value = if ($setting.EndsWith('SIZE')) { '256' } else { '1' }
        if ($header -notmatch ('#define\s+HostUart_' + $setting + '\s+\(' + $value + 'u\)')) { $issues.Add("UART configuration mismatch: $setting") }
    }
    if ($verilog -notmatch 'BaudRate=115200,') { $issues.Add('Expected UART baud 115200') }
}
$report | Format-Table -AutoSize
if ($issues.Count) { throw ($issues -join [Environment]::NewLine) }
Write-Output 'PASS: 25 physical pins, electrical settings, interrupts and hardware routes verified.'
if ($NamedInstances) {
    Write-Output 'PASS: all 25 functional instance names verified (DO = AS5045_DO).'
} else {
    Write-Output "BEFORE-RENAME: $defaultCount default Pin_* names verified; functional names remain pending."
}
Write-Output 'Input HW checkbox visibility requires Creator inspection; it is absent from generated Verilog.'
