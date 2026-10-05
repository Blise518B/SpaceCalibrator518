# Builds the downloads for GitHub releases: release\SpaceCalibrator518-Setup.exe and
# release\SpaceCalibrator518-Windows.zip (portable).
# The zip holds one SpaceCalibrator518\ folder: overlay and driver as make_dist.ps1 lays them out
# (without .pdb; README there is upstream's NOTICE), use-driver.ps1 with two .bat wrappers,
# steamvr-app.ps1 and INSTALL.txt. The installer (install\SpaceCalibrator518.iss, Inno Setup 6:
# winget install JRSoftware.InnoSetup) installs the same folder. dist\ is not touched.
#   powershell -ExecutionPolicy Bypass -File tools\make_release_zip.ps1 [-Version 1.0.2]
# -Version: the installer's version, by default the newest tag (git describe).
# Build from a neutral path such as C:\src\SpaceCalibrator518: the exe keeps source paths (log
# lines, debug info), so a checkout under your user folder would ship your user name.
# Needs Visual Studio 2022 Build Tools (C++ workload), CMake 3.24+ and the submodules.
# Upload both under exactly these names: the README links releases/latest/download/<name>
param([string]$Version = '')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root
if ($env:USERNAME -and $root -like "*\$env:USERNAME\*") {
    Write-Warning "building under $root : this path, with your user name, ends up in the exe"
}

$out = Join-Path $root 'release'
$stage = Join-Path $out 'SpaceCalibrator518'
$zip = Join-Path $out 'SpaceCalibrator518-Windows.zip'
$setup = Join-Path $out 'SpaceCalibrator518-Setup.exe'
if (-not $Version) { $Version = ((git describe --tags --abbrev=0 2>$null) -replace '^v', '') }
if ($Version -notmatch '^\d+(\.\d+){1,3}$') { throw "version '$Version': pass -Version 1.2.3" }
$iscc = @("$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe", "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe", "$env:ProgramFiles\Inno Setup 6\ISCC.exe") |
    Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $iscc) { throw 'Inno Setup 6 not found: winget install JRSoftware.InnoSetup' }
New-Item -ItemType Directory -Force -Path (Join-Path $root 'logs'), $out | Out-Null

# cmd does the redirection: PowerShell 5.1 would turn cmake's stderr lines into errors
Write-Host '[1/4] configure'
cmd /c 'cmake -G "Visual Studio 17 2022" -A x64 -B bin -S . > logs\release-configure.log 2>&1'
if ($LASTEXITCODE) { Get-Content logs\release-configure.log -Tail 30; throw 'configure failed, see logs\release-configure.log' }
Write-Host '[2/4] build (RelWithDebInfo)'
cmd /c 'cmake --build bin --config RelWithDebInfo --parallel > logs\release-build.log 2>&1'
if ($LASTEXITCODE) { Select-String -Path logs\release-build.log -Pattern ' error ' | Select-Object -First 30; throw 'build failed, see logs\release-build.log' }

Write-Host '[3/4] zip'
if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
if (Test-Path $zip) { Remove-Item $zip -Force }
if (Test-Path $setup) { Remove-Item $setup -Force }
& (Join-Path $root 'tools\make_dist.ps1') -Part all -Dest $stage -NoPdb
Copy-Item (Join-Path $root 'tools\use-driver.ps1'), (Join-Path $root 'tools\steamvr-app.ps1') $stage

$bat = "@echo off`r`npowershell -NoProfile -ExecutionPolicy Bypass -File `"%~dp0use-driver.ps1`" {0}`r`npause`r`n"
[IO.File]::WriteAllText((Join-Path $stage 'use-fork-driver.bat'), ($bat -f 'fork'), [Text.Encoding]::ASCII)
[IO.File]::WriteAllText((Join-Path $stage 'use-steam-driver.bat'), ($bat -f 'steam'), [Text.Encoding]::ASCII)

$readme = @'
SpaceCalibrator518
Glitch-robust Space Calibrator for SteamVR lighthouse trackers with an inside-out headset.
https://github.com/Blise518B/SpaceCalibrator518

1. Put this folder where it can stay. SteamVR loads the driver from here; after moving the
   folder, run step 2 again.
2. Close SteamVR and double-click use-fork-driver.bat. It tells SteamVR to load the driver from
   this folder. The Steam version of Space Calibrator uses the same driver name, so its driver
   is switched off at the same time.
3. Start SteamVR, then start SpaceCalibrator.exe once. From then on it starts with SteamVR.

Back to the Steam version: close SteamVR and double-click use-steam-driver.bat.
Settings: the Guard tab, or %APPDATA%\space-calibrator\guard.json
'@
[IO.File]::WriteAllText((Join-Path $stage 'INSTALL.txt'), ($readme -replace "`r?`n", "`r`n"), [Text.Encoding]::ASCII)

# entry by entry: ZipFile.CreateFromDirectory on Windows PowerShell 5.1 writes '\' into entry names
Add-Type -AssemblyName System.IO.Compression, System.IO.Compression.FileSystem
$archive = [IO.Compression.ZipFile]::Open($zip, [IO.Compression.ZipArchiveMode]::Create)
try {
    foreach ($f in Get-ChildItem $stage -Recurse -File) {
        $name = 'SpaceCalibrator518/' + $f.FullName.Substring($stage.Length + 1).Replace('\', '/')
        [void][IO.Compression.ZipFileExtensions]::CreateEntryFromFile($archive, $f.FullName, $name, [IO.Compression.CompressionLevel]::Optimal)
    }
} finally { $archive.Dispose() }
Write-Host ("release zip: {0} ({1:N1} MB)" -f $zip, ((Get-Item $zip).Length / 1MB))

Write-Host "[4/4] installer $Version"
& $iscc /Q "/DAppVersion=$Version" "/DStageDir=$stage" "/DOutDir=$out" (Join-Path $root 'install\SpaceCalibrator518.iss')
if ($LASTEXITCODE) { throw "Inno Setup failed ($LASTEXITCODE)" }
Write-Host ("installer: {0} ({1:N1} MB)" -f $setup, ((Get-Item $setup).Length / 1MB))
