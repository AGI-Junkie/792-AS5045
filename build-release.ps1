param(
    [string]$OutputDirectory = 'deliverables\stage2-workbench-v6-uart',
    [string]$Creator = 'D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\cyprjmgr.exe'
)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath($PSScriptRoot)
$dest = [IO.Path]::GetFullPath((Join-Path $repo $OutputDirectory))
if (-not $dest.StartsWith($repo + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) { throw 'Output directory must be inside this project.' }
if (-not (Test-Path -LiteralPath $Creator)) { throw "PSoC Creator command not found: $Creator" }
$running = Get-Process -Name TwoLinkCommissioning -ErrorAction SilentlyContinue | Where-Object { $_.Path -and $_.Path.StartsWith($dest,[StringComparison]::OrdinalIgnoreCase) }
if ($running) { throw 'Close the host from this output directory before rebuilding it.' }
New-Item -ItemType Directory -Force -Path $dest | Out-Null
Push-Location -LiteralPath $repo
try {
    Write-Host 'Building firmware (no programming)...'
    & $Creator -wrk (Join-Path $repo 'DRV8825.cywrk') -build *> (Join-Path $dest 'firmware-build.log')
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed; see $dest\firmware-build.log" }
    & (Join-Path $repo 'verify-creator-pins.ps1') -NamedInstances -HardwareUart *> (Join-Path $dest 'pin-checks.log')
    Write-Host 'Publishing host...'
    dotnet publish Commissioning/Commissioning.csproj -c Release -r win-x64 --self-contained false -o (Join-Path $dest 'host') --nologo *> (Join-Path $dest 'host-build.log')
    if ($LASTEXITCODE -ne 0) { throw "Host build failed; see $dest\host-build.log" }
    Write-Host 'Testing UI, protocol, planner, recorder; rendering offline screenshots...'
    $snapshot = Join-Path $dest 'preview.png'
    $process = Start-Process -FilePath (Join-Path $dest 'host\TwoLinkCommissioning.exe') -ArgumentList @('--self-test','--snapshot',('"' + $snapshot + '"')) -WindowStyle Hidden -Wait -PassThru
    "Host self-test exit=$($process.ExitCode); no serial opened." | Set-Content (Join-Path $dest 'host-tests.log')
    if ($process.ExitCode -ne 0) { throw 'Host self-test failed; see commissioning-self-test-error.txt.' }
    Write-Host 'Testing actual firmware C with mocked GPIO and host-generated paths...'
    python tests/test_stage1.py "--host-plan=$dest\preview.execution.bin" *> (Join-Path $dest 'firmware-tests.log')
    if ($LASTEXITCODE -ne 0) { throw "Firmware checks failed; see $dest\firmware-tests.log" }
    foreach ($shape in @('line','rectangle')) {
        python tests/test_stage1.py traj_host_plan traj_manual_plan "--host-plan=$dest\$shape.execution.bin" *> (Join-Path $dest "$shape-tests.log")
        if ($LASTEXITCODE -ne 0) { throw "$shape execution checks failed." }
    }
    Copy-Item -LiteralPath 'DRV8825.cydsn\CortexM3\ARM_GCC_541\Debug\DRV8825.hex' -Destination (Join-Path $dest 'firmware.hex')
    Copy-Item -LiteralPath 'STAGE2_HARDWARE_UART.md' -Destination (Join-Path $dest 'README.md')
    $report = @(
        "Two-Link Studio v6 hardware UART build - $(Get-Date -Format o)",
        'Creator 4.4 ARM GCC 5.4 Debug firmware; .NET 10 Windows WPF Release host.',
        'PTP_COMMISSIONED=1; provisional ranges q1 +/-30, q2 +/-60 degrees.',
        'HostUart: UDB UART 115200 8N1, RX P12[6], TX P12[7], 256-byte RX/TX interrupt buffers.',
        'v6 host uses one 36-byte encoded frame per write; older firmware retains paced triplication.',
        'CRC, sequence/epoch ACK, retry, explicit enable and 250 ms firmware deadman retained.',
        'Host self-test passed: guards, parameter roundtrip, recording, numerical planning, UI snapshots.',
        (Get-Content (Join-Path $dest 'firmware-tests.log') -Tail 1),
        ('Line: ' + (Get-Content (Join-Path $dest 'line-tests.log') -Tail 1)),
        ('Rectangle: ' + (Get-Content (Join-Path $dest 'rectangle-tests.log') -Tail 1)),
        'No serial connection, programming, or real motor movement performed.',
        'Screenshots contain explicitly marked simulated data, not hardware evidence.',
        'Arm2 and pen XY are not measured; real-machine acceptance and calibration remain with operator.'
    )
    $report | Set-Content -Encoding utf8 (Join-Path $dest 'BUILD_REPORT.txt')
    $sources = @('DRV8825.cydsn\main.c','DRV8825.cydsn\DRV8825.cyprj','DRV8825.cydsn\DRV8825.cydwr','DRV8825.cydsn\TopDesign\TopDesign.cysch','Commissioning\Commissioning.csproj','tests\test_stage1.py','build-release.ps1','verify-creator-pins.ps1','STAGE2_HARDWARE_UART.md')
    $sources += Get-ChildItem -LiteralPath Commissioning -Filter '*.cs' | ForEach-Object { 'Commissioning\' + $_.Name }
    $sources | ForEach-Object { '{0}  {1}' -f (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash,$_ } | Set-Content -Encoding ascii (Join-Path $dest 'SOURCE_SHA256SUMS.txt')
    Get-ChildItem -LiteralPath $dest -File -Recurse | Where-Object Name -ne 'SHA256SUMS.txt' | Sort-Object FullName | ForEach-Object {
        '{0}  {1}' -f (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash,$_.FullName.Substring($dest.Length+1)
    } | Set-Content -Encoding ascii (Join-Path $dest 'SHA256SUMS.txt')
    Write-Host "Build and offline checks passed: $dest"
} finally { Pop-Location }
