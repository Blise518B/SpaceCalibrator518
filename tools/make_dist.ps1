# Assembles dist\ from the RelWithDebInfo build so the fork runs like the Steam build:
#   overlay: dist\SpaceCalibrator.exe, openvr_api.dll, manifest.vrmanifest, icons, assets\
#   driver:  dist\driver.vrdrivermanifest, resources\, bin\win64\driver_01spacecalibrator.dll,
#            driver-interface.txt (what the installed driver speaks, read by tools\install.ps1)
# The overlay registers its own folder as an external SteamVR driver (see src\overlay\vr_core.cpp).
# Prefer tools\install.ps1, which picks the part that can be replaced right now.
#   -Part overlay   only the overlay files (safe while SteamVR runs, once the overlay is closed)
#   -Part driver    only the driver files (SteamVR must be closed: vrserver locks the DLL)
#   -Part all       both (default)
#   -Dest <dir>     assemble somewhere else than dist\ (tools\make_release_zip.ps1 uses this)
#   -NoPdb          leave out the .pdb files
param([ValidateSet('all', 'overlay', 'driver')][string]$Part = 'all', [string]$Dest = '', [switch]$NoPdb)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$art = Join-Path $root 'bin\artifacts\RelWithDebInfo'
$drv = Join-Path $root 'bin\driver_01spacecalibrator\bin\win64'
$dist = if ($Dest) { $Dest } else { Join-Path $root 'dist' }
. (Join-Path $root 'tools\install_common.ps1')

if ($Part -ne 'driver' -and -not (Test-Path (Join-Path $art 'SpaceCalibrator.exe'))) { throw "build first: $art\SpaceCalibrator.exe missing" }
if ($Part -ne 'overlay' -and -not (Test-Path (Join-Path $drv 'driver_01spacecalibrator.dll'))) { throw "build first: $drv\driver_01spacecalibrator.dll missing" }
New-Item -ItemType Directory -Force -Path $dist | Out-Null

if ($Part -ne 'driver') {
    foreach ($f in 'SpaceCalibrator.exe','SpaceCalibrator.pdb','openvr_api.dll','manifest.vrmanifest','icon.png','taskbar_icon.png','LICENSE','README') {
        if ($NoPdb -and $f -like '*.pdb') { continue }
        $src = Join-Path $art $f
        if (Test-Path $src) { Copy-Item $src $dist -Force }
    }
    if (Test-Path (Join-Path $art 'assets')) {
        if (Test-Path (Join-Path $dist 'assets')) { Remove-Item (Join-Path $dist 'assets') -Recurse -Force }
        Copy-Item (Join-Path $art 'assets') (Join-Path $dist 'assets') -Recurse -Force
    }
}
if ($Part -ne 'overlay') {
    New-Item -ItemType Directory -Force -Path (Join-Path $dist 'bin\win64'), (Join-Path $dist 'resources\settings') | Out-Null
    Copy-Item (Join-Path $drv 'driver_01spacecalibrator.dll') (Join-Path $dist 'bin\win64') -Force
    $pdb = Join-Path $drv 'driver_01spacecalibrator.pdb'
    if ((Test-Path $pdb) -and -not $NoPdb) { Copy-Item $pdb (Join-Path $dist 'bin\win64') -Force }
    Copy-Item (Join-Path $root 'driver_01spacecalibrator\driver.vrdrivermanifest') $dist -Force
    Copy-Item (Join-Path $root 'driver_01spacecalibrator\resources\driver.vrresources') (Join-Path $dist 'resources') -Force
    Copy-Item (Join-Path $root 'driver_01spacecalibrator\resources\settings\default.vrsettings') (Join-Path $dist 'resources\settings') -Force
    Set-Content -Path (Join-Path $dist 'driver-interface.txt') -Value (Get-SourceDriverInterface $root) -Encoding ascii
}
Write-Host "dist ($Part) assembled at $dist"