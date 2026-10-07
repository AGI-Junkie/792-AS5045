param(
    [string]$OutputDirectory = '',
    [string]$Creator = 'D:\Psoc Creator\PSoC Creator\4.4\PSoC Creator\bin\cyprjmgr.exe',
    # Build only into out\<release>: leave out\latest (and a host running from it) alone.
    [switch]$NoLatestCopy
)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath($PSScriptRoot)
$release = (Get-Content -LiteralPath (Join-Path $repo 'RELEASE_VERSION') -Raw).Trim()
if ($release -notmatch '^stage2-workbench-v[0-9]+-[a-z0-9-]+$') { throw 'Invalid RELEASE_VERSION.' }
if (-not $OutputDirectory) { $OutputDirectory = Join-Path 'out' $release }
$dest = [IO.Path]::GetFullPath((Join-Path $repo $OutputDirectory))
if (-not $dest.StartsWith($repo + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) { throw 'Output directory must be inside this project.' }
if (-not (Test-Path -LiteralPath $Creator)) { throw "PSoC Creator command not found: $Creator" }
# Every passing build is also copied to out\latest, the one fixed place to run the host from.
$latest = Join-Path $repo 'out\latest'
$running = Get-Process -Name TwoLinkCommissioning -ErrorAction SilentlyContinue | Where-Object { $_.Path -and ($_.Path.StartsWith($dest,[StringComparison]::OrdinalIgnoreCase) -or (-not $NoLatestCopy -and $_.Path.StartsWith($latest + '',[StringComparison]::OrdinalIgnoreCase))) }
if ($running) { throw 'Close the host from this output directory before rebuilding it.' }
New-Item -ItemType Directory -Force -Path $dest | Out-Null
# Creator rewrites the tracked project file (DRV8825.cyprj: a regenerated item list) on the first build of a fresh checkout.
# That is its bookkeeping, not a change to this project, and it would show up as a ~1400-line diff and trip the
# "source changed during build" check below.  Keep the bytes from before the build and put them back afterwards.
$cyprj = Join-Path $repo 'DRV8825.cydsn\DRV8825.cyprj'
$cyprjBefore = [IO.File]::ReadAllBytes($cyprj)
function Restore-CreatorProject {
    $now = [IO.File]::ReadAllBytes($cyprj)
    if (-not [System.Linq.Enumerable]::SequenceEqual([byte[]]$now, [byte[]]$cyprjBefore)) {
        [IO.File]::WriteAllBytes($cyprj, $cyprjBefore)
        Write-Host 'Restored DRV8825.cyprj to its pre-build content (Creator had regenerated it).'
    }
}
Push-Location -LiteralPath $repo
try {
    $sources = @('DRV8825.cydsn\main.c','DRV8825.cydsn\traj.h','DRV8825.cydsn\hw_ssi.h','DRV8825.cydsn\DRV8825.cyprj','DRV8825.cydsn\DRV8825.cydwr','DRV8825.cydsn\TopDesign\TopDesign.cysch','Commissioning\Commissioning.csproj','tests\test_stage1.py','build-release.ps1','verify-creator-pins.ps1','RELEASE_VERSION','README.md','CHANGELOG.md','docs\PROTOCOL.md','docs\ARM2_FEEDBACK.md','docs\STOP_DIAG.md','docs\STATE_MODEL.md','docs\UART_DIAGNOSIS.md','docs\history\RUN_REVIEW_20260929.md','tools\check-log.py')
    $sources += Get-ChildItem -LiteralPath Commissioning -Filter '*.cs' | ForEach-Object { 'Commissioning\' + $_.Name }
    $sourceHashes = @{}
    foreach ($source in $sources) { $sourceHashes[$source] = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash }
    # Numbers are read from main.c itself (macro value, trailing 'u' dropped) for the contract check and the report.
    $mainText = Get-Content -LiteralPath 'DRV8825.cydsn\main.c' -Raw
    function Def([string]$name) {
        ([regex]::Match($mainText, '(?m)^#define\s+' + [regex]::Escape($name) + '\s+\(?"?(-?[A-Za-z0-9_.-]+)"?\)?').Groups[1].Value) -replace 'u$', ''
    }
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
    Write-Host 'Checking the numbers shared with the firmware...'
    $contractFile = Join-Path $dest 'contract.txt'
    $process = Start-Process -FilePath (Join-Path $dest 'host\TwoLinkCommissioning.exe') -ArgumentList @('--print-contract', ('"' + $contractFile + '"')) -WindowStyle Hidden -Wait -PassThru
    if ($process.ExitCode -ne 0) { throw 'Host contract dump failed.' }
    # DeviceContract.cs name -> the main.c macros that must hold the same value.
    $shared = [ordered]@{
        FullStepsPerRev = 'MOTOR_FULLSTEPS_REV', 'ARM2_MOTOR_FULLSTEPS_REV'; Microsteps = 'USTEP_DIV'; Arm1GearRatio = 'GEAR_RATIO'
        Arm2GearRatioX100 = 'ARM2_GEAR_RATIO_X100'; EncoderCounts = 'ENCODER_CPR'; RateMin = 'CFG_RATE_MIN'
        Arm1RateMax = 'CFG_RATE_MAX_AXIS1'; Arm2RateMax = 'CFG_RATE_MAX_AXIS2'; AccelMin = 'CFG_ACCEL_MIN'
        Arm1AccelMax = 'CFG_ACCEL_MAX_AXIS1'; Arm2AccelMax = 'CFG_ACCEL_MAX_AXIS2'
        PlanRateMax = 'PTP_RATE_MAX', 'TRAJ_RATE_MAX'; PlanAccelMax = 'PTP_ACCEL', 'TRAJ_ACCEL_MAX'; HomeSlowRate = 'HOME_SLOW_RATE_USTEP_S'
        TrajCapacity = 'TRAJ_CAPACITY'; TrajNodeMinMs = 'TRAJ_NODE_MIN_MS'; TrajNodeMaxMs = 'TRAJ_NODE_MAX_MS'
        TrajTotalMaxMs = 'TRAJ_TOTAL_MAX_MS'; JointLimitCdMax = 'CFG_JOINT_CD_MAX'; ElbowDefaultCd = 'PTP_ELBOW_DEFAULT_CD'; TelemRateMin = 'TELEM_HZ_MIN'; TelemRateMax = 'TELEM_HZ_MAX'; DeadmanMs = 'JOG_DEADMAN_MS'; ProtocolVersion = 'PROTO_VERSION'
    }
    $hostValues = @{}
    foreach ($line in Get-Content -LiteralPath $contractFile) { $name, $value = $line -split '=', 2; $hostValues[$name] = [int]$value }
    $mismatch = @()
    foreach ($name in $shared.Keys) {
        foreach ($macro in $shared[$name]) {
            if ([int](Def $macro) -ne $hostValues[$name]) { $mismatch += "$name=$($hostValues[$name]) but main.c $macro=$(Def $macro)" }
        }
    }
    if ($hostValues['Arm1StepsPerRev'] -ne [int](Def 'MOTOR_FULLSTEPS_REV') * [int](Def 'USTEP_DIV') * [int](Def 'GEAR_RATIO')) { $mismatch += 'Arm1StepsPerRev differs from main.c' }
    if ($hostValues['Arm2StepsPerRev'] -ne [int](Def 'ARM2_MOTOR_FULLSTEPS_REV') * [int](Def 'USTEP_DIV') * [int](Def 'ARM2_GEAR_RATIO_X100') / 100) { $mismatch += 'Arm2StepsPerRev differs from main.c' }
    $unchecked = @($hostValues.Keys | Where-Object { -not $shared.Contains($_) -and $_ -notin @('Arm1StepsPerRev', 'Arm2StepsPerRev') })
    if ($unchecked) { $mismatch += "not compared with main.c: $($unchecked -join ', ')" }
    if ($mismatch) { throw ("Host and firmware disagree on shared numbers:" + [Environment]::NewLine + ($mismatch -join [Environment]::NewLine)) }
    $contractLine = "Host/firmware contract: $($hostValues.Count) shared numbers (DeviceContract.cs) agree with main.c."
    Write-Host $contractLine
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
    Copy-Item -LiteralPath 'README.md' -Destination (Join-Path $dest 'README.md')
    Copy-Item -LiteralPath 'CHANGELOG.md' -Destination (Join-Path $dest 'CHANGELOG.md')
    if (Test-Path -LiteralPath (Join-Path $dest 'docs')) { Remove-Item -LiteralPath (Join-Path $dest 'docs') -Recurse -Force }
    Copy-Item -LiteralPath 'docs' -Destination (Join-Path $dest 'docs') -Recurse
    New-Item -ItemType Directory -Force -Path (Join-Path $dest 'tools') | Out-Null
    Copy-Item -LiteralPath 'tools\check-log.py' -Destination (Join-Path $dest 'tools')
    # Facts only: everything below is read from the sources or from this run's own logs, so it cannot go stale.
    function Deg([string]$name) { [math]::Round([int](Def $name) / 100.0, 2) }
    $describe = 'unknown'; $branch = 'unknown'; $hwTags = 'none'
    try {
        $describe = (git describe --tags --always --dirty | Out-String).Trim()
        $branch = (git rev-parse --abbrev-ref HEAD | Out-String).Trim()
        $tags = @(git tag --points-at HEAD | Where-Object { $_ -like 'hw-tested/*' })
        if ($tags.Count) { $hwTags = $tags -join ', ' }
    } catch { }
    # Creator only prints its size lines when it actually links, so read the ELF itself.
    $sizes = (& (Join-Path (Split-Path $Creator) 'cyelftool.exe') -S 'DRV8825.cydsn\CortexM3\ARM_GCC_541\Debug\DRV8825.elf' | Out-String)
    $flashBytes = [regex]::Match($sizes, 'code:(\d+)').Groups[1].Value; $sramBytes = [regex]::Match($sizes, 'sram:(\d+)').Groups[1].Value
    $hexHash = (Get-FileHash -LiteralPath (Join-Path $dest 'firmware.hex') -Algorithm SHA256).Hash.Substring(0, 16).ToLower()
    $report = @(
        "Two-Link Studio $release build - $(Get-Date -Format o)",
        "Source: git $describe on branch $branch (hw-tested tag on this commit: $hwTags)",
        "Firmware: FW_BUILD_ID=$(Def 'FW_BUILD_ID'), PROTO_VERSION=$(Def 'PROTO_VERSION'), firmware.hex sha256=$hexHash..., code $flashBytes of 262144 bytes, SRAM $sramBytes of 65536 bytes (cyelftool -S)",
        'Toolchain: Creator 4.4 ARM GCC 5.4 Debug firmware; .NET 10 Windows WPF Release host.',
        "Defaults in main.c: PTP_COMMISSIONED=$(Def 'PTP_COMMISSIONED'); boot joint ranges q1 $(Deg 'PTP_MIN1_CD')..$(Deg 'PTP_MAX1_CD'), q2 $(Deg 'PTP_MIN2_CD')..$(Deg 'PTP_MAX2_CD') degrees (software bounds, not measured mechanical limits); deadman $(Def 'JOG_DEADMAN_MS') ms.",
        'HostUart: UDB UART 115200 8N1, RX P12[6], TX P12[7], 256-byte RX/TX interrupt buffers (checked by verify-creator-pins.ps1).',
        'Protocol, frame formats and state model: docs/PROTOCOL.md and docs/STATE_MODEL.md (in this folder).',
        (Get-Content (Join-Path $dest 'pin-checks.log') | Where-Object { $_ -like 'PASS:*' }),
        $contractLine,
        'Host self-test passed: guards, parameter roundtrip, recording, numerical planning, UI snapshots.',
        (Get-Content (Join-Path $dest 'heartbeat-tests.txt') | Where-Object { $_ -match 'UI lease max gap' }),
        (Get-Content (Join-Path $dest 'firmware-tests.log') | Where-Object { $_ -like 'fuzz coverage*' -or $_ -like '*actual-C simulation cases passed*' }),
        ('Line: ' + (Get-Content (Join-Path $dest 'line-tests.log') -Tail 1)),
        ('Rectangle: ' + (Get-Content (Join-Path $dest 'rectangle-tests.log') -Tail 1)),
        'No serial connection, programming, or real motor movement performed by this build.',
        'Screenshots contain explicitly marked simulated data, not hardware evidence.',
        "Arm2 encoder feedback enabled: $(Def 'ARM2_ENCODER_FEEDBACK') (0 = arm2 commanded steps without feedback; arm1 encoder still required). Pen XY is not measured. Tracking servo, real-machine acceptance and calibration remain."
    )
    $report = @($report | ForEach-Object { $_ } | Where-Object { $null -ne $_ })
    Restore-CreatorProject
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
    if ($NoLatestCopy) {
        Write-Host "Not copied to $latest (-NoLatestCopy)"
    } elseif ($dest -ne $latest) {
        if (Test-Path -LiteralPath $latest) { Remove-Item -LiteralPath $latest -Recurse -Force }
        Copy-Item -LiteralPath $dest -Destination $latest -Recurse
        Write-Host "Copied to $latest (run host\TwoLinkCommissioning.exe from there)"
    }
} finally {
    Restore-CreatorProject
    Pop-Location
}
