# publish_ota_release.ps1 - publish the build that make_factory_image.ps1
# last made as a GitHub release of narbiscorp/narbis-earclip-fw, where the
# web OTA updater (https://narbiscorp.github.io/edge-OTA/, repo
# narbiscorp/edge-OTA) and the Edge HRV phone apps find it.
#
# Every client keys on the asset NAME:
#  - narbis_earclip_v2_functest.bin: "Narbis Edge Earclip TEST" boards
#    (web updater only). Default mode: tag functest-<version>, a
#    PRE-release, never the repo's Latest badge.
#  - narbis_earclip_v2.bin: production "Narbis Edge Clip" boards (v0.4.0+)
#    in the web updater AND the phone apps, which skip pre-releases. Mode
#    -Production: tag = the version tag itself (v<x.y.z>, which must
#    already exist at the source commit, so the device reports exactly
#    that), a FULL release that becomes Latest.
# The merged USB image of the same build rides along for the vendor.
#
# The source commit must already be on origin/master. git describe only
# counts v<digit> tags (CMakeLists.txt), so functest-* tags never become
# the base of a version string.
#
# Publishing makes the image installable by anyone with the updater or the
# app and a board in range: OTA-test it on hardware first. -DryRun only
# prints.
param([string]$Notes = '', [switch]$DryRun, [switch]$Production)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$gh = 'narbiscorp/narbis-earclip-fw'
if ($Production) {
    $appName = 'narbis_earclip_v2.bin'
    $usbName = 'narbis-earclip'
} else {
    $appName = 'narbis_earclip_v2_functest.bin'
    $usbName = 'narbis-earclip-functest'
}
$app = Join-Path $repo "vendor\ota\$appName"
$usb = Join-Path $repo "vendor\$usbName.bin"
$usbTxt = Join-Path $repo "vendor\$usbName.version.txt"

foreach ($f in @($app, "$app.version.txt", $usb, $usbTxt)) {
    if (-not (Test-Path $f)) { throw "missing $f - run tools\make_factory_image.ps1$(if ($Production) { ' -Production' }) first" }
}
# "<name>  |  <version>  |  ..." - both files must come from the same build.
$ver = ((Get-Content "$app.version.txt" -Raw) -split '\|')[1].Trim()
$usbVer = ((Get-Content $usbTxt -Raw) -split '\|')[1].Trim()
if ($ver -ne $usbVer) { throw "app image is $ver but the USB image is $usbVer - rebuild" }
if ($ver -match '-dirty$') { throw "$ver was built from uncommitted source - commit, rebuild" }
if ($Production) {
    # A production release is cut from a version tag, nothing in between.
    if ($ver -notmatch '^v\d+\.\d+\.\d+$') { throw "production needs a clean v<x.y.z> build, got '$ver' - tag the commit, rebuild" }
    $sha = git -C $repo rev-parse "$ver^{commit}"
} else {
    if ($ver -notmatch '-g([0-9a-f]{7,})$') { throw "cannot find the commit in '$ver'" }
    $sha = git -C $repo rev-parse $Matches[1]
}
git -C $repo fetch -q origin master
git -C $repo merge-base --is-ancestor $sha origin/master
if ($LASTEXITCODE -ne 0) { throw "$sha is not on origin/master - push the source first" }

# The image itself: ESP app magic, V2.1 project name, version as stamped
# (+ "-dirty" from the build-time TEST flip, see make_factory_image.ps1).
[byte[]]$hdr = [System.IO.File]::ReadAllBytes($app)[0..0x6F]
$proj = [System.Text.Encoding]::ASCII.GetString($hdr, 0x50, 32).Split([char]0)[0]
$appVer = [System.Text.Encoding]::ASCII.GetString($hdr, 0x30, 32).Split([char]0)[0]
if ($hdr[0] -ne 0xE9 -or $proj -ne 'narbis_earclip_v2') { throw "not a V2.1 app image (project '$proj')" }
if (-not $appVer.StartsWith($ver)) { throw "image says $appVer, version file says $ver" }
if ($Production -and $appVer -ne $ver) { throw "production image says $appVer, expected exactly $ver" }

if ($Production) {
    $tag = $ver
    $title = "Narbis Edge Clip firmware $ver"
    $body = @"
$Notes

- ``narbis_earclip_v2.bin``: app image for BLE OTA. The Edge HRV apps (Android, iOS) and the web updater (https://narbiscorp.github.io/edge-OTA/) offer it to "Narbis Edge Clip" boards and to older "Narbis Edge Earclip[ TEST]" V2.1 boards. Keeps knobs and bonds.
- ``narbis-earclip.bin``: merged USB image, ``esptool write_flash 0x0``. Wipes NVS (knobs, bonds).

Built from $gh $($sha.Substring(0, 7)), NARBIS_TEST_MODE=0 (production). Device reports $appVer.
"@.Trim()
} else {
    $tag = "functest-$ver"
    $title = "Functional-test firmware $ver"
    $body = @"
$Notes

- ``narbis_earclip_v2_functest.bin``: app image for BLE OTA. The web updater (https://narbiscorp.github.io/edge-OTA/) lists it for boards named "Narbis Edge Earclip TEST". Keeps knobs and bonds.
- ``narbis-earclip-functest.bin``: merged USB image, ``esptool write_flash 0x0``. Wipes NVS (knobs, bonds).

Built from $gh $($sha.Substring(0, 7)), NARBIS_TEST_MODE=1. Device reports $appVer.
"@.Trim()
}

Write-Host "release $tag at $sha ($(if ($Production) { 'FULL release, Latest' } else { 'pre-release' })) on $gh"
Write-Host "  $app  ($((Get-Item $app).Length) B)"
Write-Host "  $usb  ($((Get-Item $usb).Length) B)"
if ($DryRun) { Write-Host '--- notes ---'; Write-Host $body; return }
# Notes go through a file: Windows PowerShell 5.1 does not escape the
# double quotes inside a native argument, so --notes "$body" split at the
# first quote and gh read the pieces as asset paths.
$notesFile = [System.IO.Path]::GetTempFileName()
try {
    [System.IO.File]::WriteAllText($notesFile, $body, (New-Object System.Text.UTF8Encoding($false)))
    if ($Production) {
        gh release create $tag $app $usb --repo $gh --verify-tag --latest `
            --title $title --notes-file $notesFile
    } else {
        gh release create $tag $app $usb --repo $gh --target $sha --prerelease `
            --title $title --notes-file $notesFile
    }
    if ($LASTEXITCODE -ne 0) { throw 'gh release create failed' }
}
finally {
    Remove-Item $notesFile -ErrorAction SilentlyContinue
}
