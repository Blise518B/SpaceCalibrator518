# Registers this folder's overlay with SteamVR so it starts with SteamVR, or removes that
# registration. The overlay does the same through IVRApplications whenever it runs; this works
# while SteamVR is closed (SteamVR holds both files and writes them back while it runs).
# The installer runs `register` after installing and `unregister` when uninstalling;
# tools\install_common.ps1 (Register-SteamVrApp) does the same for a source checkout's dist\.
#   powershell -ExecutionPolicy Bypass -File steamvr-app.ps1 register|unregister [-Dir <folder>]
# register: adds <folder>\manifest.vrmanifest to Steam\config\appconfig.json and drops other
#           registrations of the same app key (SteamVR keeps only the first manifest it reads for
#           a key, so an older copy, say an unpacked zip, could start instead); turns autolaunch on
#           only when SteamVR has no setting for the app yet.
# unregister: removes <folder>\manifest.vrmanifest from appconfig.json; SteamVR's per-app setting stays.
param(
    [Parameter(Mandatory = $true)][ValidateSet('register', 'unregister')][string]$Action,
    [string]$Dir = ''
)
$ErrorActionPreference = 'Stop'

if (Get-Process vrserver, vrmonitor -ErrorAction SilentlyContinue) {
    throw 'SteamVR is running. Close it first: it writes its own copy of these files back.'
}
$here = if ($Dir) { $Dir } else { Split-Path -Parent $MyInvocation.MyCommand.Path }
$manifestFile = [IO.Path]::GetFullPath((Join-Path $here 'manifest.vrmanifest'))

$steam = (Get-ItemProperty 'HKCU:\Software\Valve\Steam' -ErrorAction SilentlyContinue).SteamPath
if (-not $steam) {
    Write-Host 'Steam not found: the overlay registers itself when it runs with SteamVR'
    exit 0
}
$cfgDir = Join-Path ($steam -replace '/', '\') 'config'
$appconfig = Join-Path $cfgDir 'appconfig.json'
$utf8 = New-Object Text.UTF8Encoding($false)

function Norm([string]$p) { [IO.Path]::GetFullPath($p).ToLowerInvariant() }
function Get-AppKey([string]$file) {
    try { return (Get-Content $file -Raw | ConvertFrom-Json).applications[0].app_key } catch { return $null }
}

$cfg = if (Test-Path $appconfig) { Get-Content $appconfig -Raw | ConvertFrom-Json } else { [pscustomobject]@{} }
$paths = @()
if ($cfg.PSObject.Properties.Name -contains 'manifest_paths') { $paths = @($cfg.manifest_paths | Where-Object { $_ }) }
$mine = Norm $manifestFile
$key = if (Test-Path $manifestFile) { Get-AppKey $manifestFile } else { $null }

if ($Action -eq 'register') {
    if (-not $key) { throw "no usable manifest.vrmanifest in $here" }
    $keep = @()
    foreach ($p in $paths) {
        if ((Norm $p) -eq $mine) { continue }
        if ((Test-Path $p) -and ((Get-AppKey $p) -eq $key)) { Write-Host "dropped $p (same app key)"; continue }
        $keep += $p
    }
    $new = @($keep) + $manifestFile
} else {
    $new = @($paths | Where-Object { (Norm $_) -ne $mine })
}

if (($new -join '|') -ne ($paths -join '|')) {
    if (Test-Path $appconfig) { Copy-Item $appconfig "$appconfig.bak-sc518" -Force }
    $cfg | Add-Member -NotePropertyName manifest_paths -NotePropertyValue ([object[]]$new) -Force
    New-Item -ItemType Directory -Force -Path $cfgDir | Out-Null
    [IO.File]::WriteAllText($appconfig, ($cfg | ConvertTo-Json -Depth 8), $utf8)
}
Write-Host "$Action $manifestFile"

if ($Action -eq 'register') {
    $vrappDir = Join-Path $cfgDir 'vrappconfig'
    $vrapp = Join-Path $vrappDir "$key.vrappconfig"
    if (-not (Test-Path $vrapp)) {
        New-Item -ItemType Directory -Force -Path $vrappDir | Out-Null
        [IO.File]::WriteAllText($vrapp, "{`n   `"autolaunch`" : true,`n   `"last_launch_time`" : `"0`"`n}`n", $utf8)
        Write-Host 'starts with SteamVR'
    } else {
        $auto = (Get-Content $vrapp -Raw | ConvertFrom-Json).autolaunch
        Write-Host $(if ($auto) { 'starts with SteamVR' } else { 'autostart is off in SteamVR (setting kept)' })
    }
}
