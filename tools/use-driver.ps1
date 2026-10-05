# Switches which Space Calibrator driver SteamVR loads: this fork or the Steam version.
# Both builds are named 01spacecalibrator, so only one may sit in external_drivers; the add-on
# toggle in SteamVR would switch off both. Run with SteamVR closed; it takes effect on its next start.
#   powershell -ExecutionPolicy Bypass -File tools\use-driver.ps1 fork
#   powershell -ExecutionPolicy Bypass -File tools\use-driver.ps1 steam
#   powershell -ExecutionPolicy Bypass -File tools\use-driver.ps1 none   (no Space Calibrator driver at all)
# In a source checkout the fork is dist\; in the release zip and the installed copy this script sits
# next to the driver and the fork is its own folder. The installer runs `fork` after installing and
# `steam` (or `none` without the Steam version) when uninstalling.
param([Parameter(Mandatory = $true)][ValidateSet('fork', 'steam', 'none')][string]$Which)
$ErrorActionPreference = 'Stop'

if (Get-Process vrserver -ErrorAction SilentlyContinue) {
    throw 'SteamVR is running. Close it first: the driver list is only read when SteamVR starts.'
}

function Test-Spacecal([string]$dir) { Test-Path (Join-Path $dir 'bin\win64\driver_01spacecalibrator.dll') }
function Norm([string]$dir) { [IO.Path]::GetFullPath($dir).TrimEnd('\').ToLowerInvariant() }

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$forkDir = if (Test-Spacecal $here) { $here } else { Join-Path (Split-Path -Parent $here) 'dist' }
$pathsFile = Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath'
$cfg = Get-Content $pathsFile -Raw | ConvertFrom-Json
$vrpathreg = Join-Path $cfg.runtime[0] 'bin\win64\vrpathreg.exe'

function Find-SteamInstall {
    $steam = (Get-ItemProperty 'HKCU:\Software\Valve\Steam' -Name SteamPath).SteamPath -replace '/', '\'
    $libs = @($steam)
    $vdf = Join-Path $steam 'steamapps\libraryfolders.vdf'
    if (Test-Path $vdf) {
        $libs += Select-String -Path $vdf -Pattern '"path"\s+"([^"]+)"' |
            ForEach-Object { $_.Matches[0].Groups[1].Value -replace '\\\\', '\' }
    }
    foreach ($lib in $libs) {
        $dir = Join-Path $lib 'steamapps\common\01spacecalibrator'
        if (Test-Spacecal $dir) { return $dir }
    }
    throw 'Steam version of Space Calibrator not found in any Steam library.'
}

$target = switch ($Which) { 'fork' { $forkDir } 'steam' { Find-SteamInstall } default { $null } }
if ($target -and -not (Test-Spacecal $target)) { throw "No driver_01spacecalibrator.dll under $target (build first?)" }

Copy-Item $pathsFile "$pathsFile.bak-$(Get-Date -Format yyyyMMdd-HHmmss)"
$have = $false
foreach ($d in @($cfg.external_drivers | Where-Object { $_ })) {
    if ($target -and (Norm $d) -eq (Norm $target)) { $have = $true; continue }
    if (Test-Spacecal $d) { & $vrpathreg removedriver $d; Write-Host "removed $d" }
}
if ($target -and -not $have) { & $vrpathreg adddriver $target; Write-Host "added   $target" }

$now = @((Get-Content $pathsFile -Raw | ConvertFrom-Json).external_drivers | Where-Object { $_ -and (Test-Spacecal $_) })
Write-Host "Space Calibrator driver now: $(if ($now.Count) { $now -join ', ' } else { 'none' })"
Write-Host 'Start SteamVR to load it.'