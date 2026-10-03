# Waits until SteamVR has closed, then installs the part of the build that needed it
# (install.ps1 starts this hidden; one at a time). The driver DLL is locked while vrserver runs.
#   -Part all     driver and overlay (the driver interface changed)
#   -Part driver  only the driver (the overlay was already swapped live)
# -KeepAll also switches recorder.keep_all on in guard.json. Log: logs\install.log
param([ValidateSet('all', 'driver')][string]$Part = 'all', [switch]$KeepAll, [int]$TimeoutHours = 48)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
. (Join-Path $root 'tools\install_common.ps1')

Write-InstallLog $root "waiting for SteamVR to close (part $Part)"
$deadline = (Get-Date).AddHours($TimeoutHours)
while ($true) {
    while (Get-Process vrserver -ErrorAction SilentlyContinue) {
        if ((Get-Date) -gt $deadline) { Write-InstallLog $root 'gave up waiting for SteamVR to close'; exit 1 }
        Start-Sleep -Seconds 1
    }
    Start-Sleep -Seconds 2 # let vrserver release the driver DLL
    if (Get-Process vrserver -ErrorAction SilentlyContinue) { continue } # restarted right away: wait again
    try {
        if ($Part -eq 'all') { [void](Close-DistOverlay $root) }
        & (Join-Path $root 'tools\make_dist.ps1') -Part $Part
        break
    } catch {
        Write-InstallLog $root "install attempt failed ($_), retrying when SteamVR is closed again"
        Start-Sleep -Seconds 5
    }
}
if ($KeepAll -and $Part -eq 'all' -and (Set-KeepAll)) { Write-InstallLog $root 'guard.json: recorder.keep_all = true' }
Write-InstallLog $root "installed ($Part, driver interface $(Get-InstalledDriverInterface $root)); SteamVR starts the overlay with itself"
Write-InstallLog $root (Register-SteamVrApp $root)