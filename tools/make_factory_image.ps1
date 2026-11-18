# make_factory_image.ps1 — build a release firmware image set.
#
# Default: the vendor functional-test firmware, NARBIS_TEST_MODE=1 ->
#   vendor\narbis-earclip-functest.bin (merged single-file image:
#   bootloader + partition table + otadata + app, flashable at 0x0) and
#   vendor\ota\narbis_earclip_v2_functest.bin (app image for BLE OTA).
#
# -Bench additionally sets NARBIS_BENCH_BUILD=1 (bare-XIAO build: sensors
# optional, onboard LED 2 Hz) -> vendor\narbis-earclip-bench.bin.
#
# -Production builds board.h AS COMMITTED (NARBIS_TEST_MODE 0, the
# "Narbis Edge Clip" product since v0.4.0) ->
#   vendor\narbis-earclip.bin + vendor\ota\narbis_earclip_v2.bin.
# Tag the source v<x.y.z> first so the device reports a clean version
# (tools\publish_ota_release.ps1 -Production refuses anything else).
#
# Restores board.h afterwards regardless of outcome.
param([switch]$Bench, [switch]$Production)
if ($Bench -and $Production) { throw '-Bench and -Production are exclusive' }
$repo = Split-Path $PSScriptRoot -Parent
$board = Join-Path $repo 'firmware\main\board.h'
$vendor = Join-Path $repo 'vendor'

# Activate BEFORE going strict: export.ps1 writes progress to stderr,
# which ErrorActionPreference=Stop would escalate into a fatal error.
. C:\Espressif\frameworks\esp-idf-v5.5.1\export.ps1 | Out-Null
$ErrorActionPreference = 'Stop'

# The factory image builds in its OWN build dir: (a) the developer's
# firmware/build stays bound to whatever terminal/python configured it
# (idf.py refuses env mismatches), and (b) the NARBIS_TEST_MODE flip
# would otherwise force a near-full rebuild of the dev dir every run.
# Production gets a third dir so a TEST build never leaves objects behind
# that a production build could pick up.
$bdir = Join-Path $repo $(if ($Production) { 'firmware\build_prod' } else { 'firmware\build_factory' })
function Invoke-Idf { python "$env:IDF_PATH\tools\idf.py" -B $bdir @args; if ($LASTEXITCODE -ne 0) { throw "idf.py $($args -join ' ') failed" } }

$orig = Get-Content $board -Raw
if ($orig -notmatch '#define NARBIS_TEST_MODE 0') {
    throw 'board.h is not in the expected production state (NARBIS_TEST_MODE 0)'
}
if ($orig -notmatch '#define NARBIS_GATED_PROFILE 0') {
    throw 'board.h is not in the expected state (NARBIS_GATED_PROFILE 0)'
}
# Stamp BEFORE flipping board.h: the flip itself would make describe say
# "-dirty" for every build. The TEST_MODE=1 delta is recorded separately
# in the version line; the describe identifies the SOURCE tree.
# vendor\ is THIS script's own output — a stale bin/txt from the previous
# build must not mark the SOURCE dirty (it did: every commit-then-rebuild
# cycle stamped dirty until vendor\ was also committed).
$ver = git -C $repo describe --tags --always --match 'v[0-9]*'   # as CMakeLists.txt
$dirty = git -C $repo status --porcelain -- ':!vendor'
if ($dirty) { $ver = "$ver-dirty" }
$outName = if ($Bench) { 'narbis-earclip-bench' } elseif ($Production) { 'narbis-earclip' } else { 'narbis-earclip-functest' }
try {
    if (-not $Production) {
        $mod = $orig -replace '#define NARBIS_TEST_MODE 0', '#define NARBIS_TEST_MODE 1'
        if ($Bench) {
            $mod = $mod -replace '#define NARBIS_BENCH_BUILD 0', '#define NARBIS_BENCH_BUILD 1'
        }
        Set-Content $board $mod -NoNewline
    }
    Set-Location (Join-Path $repo 'firmware')
    Invoke-Idf reconfigure   # re-runs git describe: PROJECT_VER is cached
                             # at configure time and would go stale
    Invoke-Idf build
    Invoke-Idf merge-bin

    New-Item -ItemType Directory -Force $vendor | Out-Null
    Copy-Item (Join-Path $bdir 'merged-binary.bin') (Join-Path $vendor ($outName + '.bin')) -Force
    $flags = if ($Bench) { 'NARBIS_TEST_MODE=1 BENCH=1' } elseif ($Production) { 'NARBIS_TEST_MODE=0 (production)' } else { 'NARBIS_TEST_MODE=1' }
    Set-Content (Join-Path $vendor ($outName + '.version.txt')) `
        "$($outName).bin  |  $ver  |  built $(Get-Date -Format s)  |  $flags"
    # The APP image of the same build, for BLE OTA (web updater / phone
    # apps / testapp / narbis-ota). Not committed (vendor\ota\ is
    # gitignored): it ships as a GitHub release asset,
    # tools\publish_ota_release.ps1.
    $otaDir = Join-Path $vendor 'ota'
    New-Item -ItemType Directory -Force $otaDir | Out-Null
    $otaName = if ($Bench) { 'narbis_earclip_v2_bench.bin' } elseif ($Production) { 'narbis_earclip_v2.bin' } else { 'narbis_earclip_v2_functest.bin' }
    Copy-Item (Join-Path $bdir 'narbis_earclip_v2.bin') (Join-Path $otaDir $otaName) -Force
    Set-Content (Join-Path $otaDir ($otaName + '.version.txt')) "$otaName  |  $ver  |  $flags"
    Write-Host "OK -> vendor\$($outName).bin + vendor\ota\$otaName ($ver)"
}
finally {
    Set-Content $board $orig -NoNewline
    Write-Host 'board.h restored to production (NARBIS_TEST_MODE 0)'
}
