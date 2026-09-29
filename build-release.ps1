param(
    [string]$OutputDirectory = '',
    [string]$Creator = 'D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\cyprjmgr.exe'
)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath($PSScriptRoot)
$release = (Get-Content -LiteralPath (Join-Path $repo 'RELEASE_VERSION') -Raw).Trim()
if ($release -notmatch '^stage2-workbench-v[0-9]+-[a-z0-9-]+$') { throw 'Invalid RELEASE_VERSION.' }
if (-not $OutputDirectory) { $OutputDirectory = Join-Path 'deliverables' $release }
$dest = [IO.Path]::GetFullPath((Join-Path $repo $OutputDirectory))
if (-not $dest.StartsWith($repo + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) { throw 'Output directory must be inside this project.' }
if (-not (Test-Path -LiteralPath $Creator)) { throw "PSoC Creator command not found: $Creator" }
$running = Get-Process -Name TwoLinkCommissioning -ErrorAction SilentlyContinue | Where-Object { $_.Path -and $_.Path.StartsWith($dest,[StringComparison]::OrdinalIgnoreCase) }
if ($running) { throw 'Close the host from this output directory before rebuilding it.' }
New-Item -ItemType Directory -Force -Path $dest | Out-Null
Push-Location -LiteralPath $repo
try {
    $sources = @('DRV8825.cydsn\main.c','DRV8825.cydsn\DRV8825.cyprj','DRV8825.cydsn\DRV8825.cydwr','DRV8825.cydsn\TopDesign\TopDesign.cysch','Commissioning\Commissioning.csproj','tests\test_stage1.py','build-release.ps1','verify-creator-pins.ps1','RELEASE_VERSION','WORKBENCH_HARDENING.md','RUN_REVIEW_20260929.md')
    $sources += Get-ChildItem -LiteralPath Commissioning -Filter '*.cs' | ForEach-Object { 'Commissioning\' + $_.Name }
    $sourceHashes = @{}
    foreach ($source in $sources) { $sourceHashes[$source] = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash }
    foreach ($reportName in @('BUILD_REPORT.txt','SHA256SUMS.txt','SOURCE_SHA256SUMS.txt')) {
        $reportPath=Join-Path $dest $reportName
        if (Test-Path -LiteralPath $reportPath) { Remove-Item -LiteralPath $reportPath }
    }
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
    python tests/test_stage1.py "--host-plan=$dest\preview.execution.bin" "--host-control=$dest\preview.control.bin" *> (Join-Path $dest 'firmware-tests.log')
    if ($LASTEXITCODE -ne 0) { throw "Firmware checks failed; see $dest\firmware-tests.log" }
    foreach ($shape in @('line','rectangle')) {
        python tests/test_stage1.py traj_host_plan traj_manual_plan "--host-plan=$dest\$shape.execution.bin" *> (Join-Path $dest "$shape-tests.log")
        if ($LASTEXITCODE -ne 0) { throw "$shape execution checks failed." }
    }
    Copy-Item -LiteralPath 'DRV8825.cydsn\CortexM3\ARM_GCC_541\Debug\DRV8825.hex' -Destination (Join-Path $dest 'firmware.hex')
    Copy-Item -LiteralPath 'WORKBENCH_HARDENING.md' -Destination (Join-Path $dest 'README.md')
    Copy-Item -LiteralPath 'RUN_REVIEW_20260929.md' -Destination (Join-Path $dest 'RUN_REVIEW_20260929.md')
    $report = @(
        "Two-Link Studio $release build - $(Get-Date -Format o)",
        'Creator 4.4 ARM GCC 5.4 Debug firmware; .NET 10 Windows WPF Release host.',
        'PTP_COMMISSIONED=1; boot defaults q1 +/-30, q2 +/-60 degrees. Host synchronizes user limits while disabled; motion is gated until both axes match. Range fields use 0.01 degree precision, within +/-180 and containing zero.',
        'Global motion settings require stage2-v7-global1: per-axis rates/acceleration acknowledged while disabled. Preview/CSV/plot share timing; jog/home/PTP obey the configured profile and existing mode caps.',
        'HostUart: UDB UART 115200 8N1, RX P12[6], TX P12[7], 256-byte RX/TX interrupt buffers.',
        'v7 host writes two copies of each byte with >=0.5 ms spacing; firmware deduplicates them into one frame. Data and command frames are 36 wire bytes; the compact heartbeat (D5..D7, advertised by short_hb=1) is 6 wire bytes. Frames are written by one dedicated thread; a heartbeat follows each data frame and idle spacing is >=20 ms after a completed heartbeat.',
        'CRC + control sequence/epoch protect enable, home and input leases; raw STOP remains immediate.',
        'Hardware monotonic deadlines, 250 ms deadman, 256-384 ms foreground-fed watchdog; no pulse catch-up.',
        'Encoder gap/jump/overflow latches position invalid until explicit enable establishes a new origin.',
        'Host self-test passed: guards, parameter roundtrip, recording, numerical planning, UI snapshots.',
        (Get-Content (Join-Path $dest 'heartbeat-tests.txt') | Where-Object { $_ -like 'v7 real upload:*' }),
        (Get-Content (Join-Path $dest 'firmware-tests.log') -Tail 1),
        ('Line: ' + (Get-Content (Join-Path $dest 'line-tests.log') -Tail 1)),
        ('Rectangle: ' + (Get-Content (Join-Path $dest 'rectangle-tests.log') -Tail 1)),
        'No serial connection, programming, or real motor movement performed.',
        'Screenshots contain explicitly marked simulated data, not hardware evidence.',
        'Arm2 and pen XY are not measured; real-machine acceptance and calibration remain with operator.'
    )
    foreach ($source in $sources) {
        if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -ne $sourceHashes[$source]) {
            throw "Source changed during build: $source. Rebuild before using this package."
        }
    }
    $report | Set-Content -Encoding utf8 (Join-Path $dest 'BUILD_REPORT.txt')
    $sources | ForEach-Object { '{0}  {1}' -f (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash,$_ } | Set-Content -Encoding ascii (Join-Path $dest 'SOURCE_SHA256SUMS.txt')
    Get-ChildItem -LiteralPath $dest -File -Recurse | Where-Object Name -ne 'SHA256SUMS.txt' | Sort-Object FullName | ForEach-Object {
        '{0}  {1}' -f (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash,$_.FullName.Substring($dest.Length+1)
    } | Set-Content -Encoding ascii (Join-Path $dest 'SHA256SUMS.txt')
    Write-Host "Build and offline checks passed: $dest"
} finally { Pop-Location }
