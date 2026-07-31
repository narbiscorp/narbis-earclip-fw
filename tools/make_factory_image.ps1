# make_factory_image.ps1 — build the vendor functional-test firmware:
# NARBIS_TEST_MODE=1, merged single-file image (bootloader + partition
# table + otadata + app) flashable at offset 0x0.
#
# Restores board.h to NARBIS_TEST_MODE 0 afterwards regardless of outcome.
# Output: vendor\narbis-earclip-functest.bin (+ a version .txt beside it).
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
$bdir = Join-Path $repo 'firmware\build_factory'
function Invoke-Idf { python "$env:IDF_PATH\tools\idf.py" -B $bdir @args; if ($LASTEXITCODE -ne 0) { throw "idf.py $($args -join ' ') failed" } }

$orig = Get-Content $board -Raw
if ($orig -notmatch '#define NARBIS_TEST_MODE 0') {
    throw 'board.h is not in the expected production state (NARBIS_TEST_MODE 0)'
}
# Stamp BEFORE flipping board.h: the flip itself would make describe say
# "-dirty" for every build. The TEST_MODE=1 delta is recorded separately
# in the version line; the describe identifies the SOURCE tree.
$ver = git -C $repo describe --tags --dirty --always
try {
    Set-Content $board ($orig -replace '#define NARBIS_TEST_MODE 0',
                                      '#define NARBIS_TEST_MODE 1') -NoNewline
    Set-Location (Join-Path $repo 'firmware')
    Invoke-Idf build
    Invoke-Idf merge-bin

    New-Item -ItemType Directory -Force $vendor | Out-Null
    Copy-Item (Join-Path $bdir 'merged-binary.bin') (Join-Path $vendor 'narbis-earclip-functest.bin') -Force
    Set-Content (Join-Path $vendor 'narbis-earclip-functest.version.txt') `
        "narbis-earclip-functest.bin  |  $ver  |  built $(Get-Date -Format s)  |  NARBIS_TEST_MODE=1"
    Write-Host "OK -> vendor\narbis-earclip-functest.bin ($ver)"
}
finally {
    Set-Content $board $orig -NoNewline
    Write-Host 'board.h restored to production (NARBIS_TEST_MODE 0)'
}
