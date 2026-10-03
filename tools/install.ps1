# Puts a fresh build (bin\) into dist\ with the least disruption. build.bat runs it last.
#
#   SteamVR closed                               -> everything, now
#   SteamVR running, driver interface unchanged  -> the overlay now, live: the running overlay is
#                                                   closed (it flushes its recording), replaced and
#                                                   started again; the recording continues without a
#                                                   gap. A changed driver DLL follows when SteamVR closes.
#   SteamVR running, driver interface changed    -> nothing now (the new overlay could not talk to the
#                                                   running driver); everything as soon as SteamVR closes
#
# The driver interface is the IPC protocol plus the pose ring layout (install_common.ps1). Log:
# logs\install.log
#   powershell -ExecutionPolicy Bypass -File tools\install.ps1 [-NoRestart] [-KeepAll]
# -NoRestart  leave the overlay closed after a live swap
# -KeepAll    also switch recorder.keep_all on in guard.json
param([switch]$NoRestart, [switch]$KeepAll)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
. (Join-Path $root 'tools\install_common.ps1')
$makeDist = Join-Path $root 'tools\make_dist.ps1'
$waiter = Join-Path $root 'tools\install-when-closed.ps1'

function Start-Waiter([string]$part) {
    # one waiter at a time: a newer build replaces a waiting one
    Get-CimInstance Win32_Process -Filter "Name = 'powershell.exe'" | Where-Object { $_.CommandLine -match 'install-when-closed\.ps1' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    $argList = @('-NoProfile', '-WindowStyle', 'Hidden', '-ExecutionPolicy', 'Bypass', '-File', "`"$waiter`"", '-Part', $part)
    if ($KeepAll) { $argList += '-KeepAll' }
    Start-Process -FilePath powershell.exe -ArgumentList $argList -WindowStyle Hidden
}

function Get-FileHashOrEmpty([string]$path) {
    if (Test-Path $path) { return (Get-FileHash $path -Algorithm SHA256).Hash }
    return ''
}

$steamvr = [bool](Get-Process vrserver -ErrorAction SilentlyContinue)
$built = Get-SourceDriverInterface $root
$installed = Get-InstalledDriverInterface $root
$driverChanged = (Get-FileHashOrEmpty (Join-Path $root 'bin\driver_01spacecalibrator\bin\win64\driver_01spacecalibrator.dll')) -ne (Get-FileHashOrEmpty (Join-Path $root 'dist\bin\win64\driver_01spacecalibrator.dll'))

if (-not $steamvr) {
    $wasRunning = Close-DistOverlay $root
    & $makeDist -Part all
    if ($KeepAll -and (Set-KeepAll)) { Write-InstallLog $root 'guard.json: recorder.keep_all = true' }
    Write-InstallLog $root "installed everything (driver interface $built)"
    Write-InstallLog $root (Register-SteamVrApp $root)
    if ($wasRunning -and -not $NoRestart) { Start-DistOverlay $root }
    exit 0
}

if ($installed -ne $built) {
    Write-InstallLog $root "driver interface changed ('$installed' -> '$built'): nothing is swapped while SteamVR runs; everything goes in as soon as SteamVR closes"
    Start-Waiter 'all'
    exit 0
}

$wasRunning = Close-DistOverlay $root
& $makeDist -Part overlay
if ($KeepAll -and (Set-KeepAll)) { Write-InstallLog $root 'guard.json: recorder.keep_all = true' }
if (-not $NoRestart) { Start-DistOverlay $root }
Write-InstallLog $root ("overlay swapped live" + $(if ($wasRunning) { ' (was running, restarted)' } else { '' }))
if ($driverChanged) {
    Write-InstallLog $root 'the driver DLL changed too (same interface): it goes in when SteamVR closes'
    Start-Waiter 'driver'
}