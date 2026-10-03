# Shared by make_dist.ps1, install.ps1 and install-when-closed.ps1.

# Driver interface: what the overlay needs from the running driver. An overlay can be swapped
# live only when this is unchanged; everything else in the overlay (trust layer, solver, recorder,
# UI) is free to change while SteamVR runs.
#   ipc  = IPC_PROTOCOL_CURRENT in src\common\protocol.h (commands, shared transform/pose blocks)
#   ring = k_RING_LAYOUT_VERSION in src\common\pose_ring.h (pose ring header, slots, Record)
function Get-SourceDriverInterface([string]$root) {
    $protocol = Get-Content (Join-Path $root 'src\common\protocol.h') -Raw
    $ring = Get-Content (Join-Path $root 'src\common\pose_ring.h') -Raw
    $ipc = [regex]::Match($protocol, 'IPC_PROTOCOL_CURRENT\s*=\s*IPC_PROTOCOL_VER_(\d+)').Groups[1].Value
    $layout = [regex]::Match($ring, 'k_RING_LAYOUT_VERSION\s*=\s*(\d+)').Groups[1].Value
    if (-not $ipc -or -not $layout) { throw 'could not read the driver interface from the sources' }
    return "ipc=$ipc;ring=$layout"
}

function Get-InstalledDriverInterface([string]$root) {
    $f = Join-Path $root 'dist\driver-interface.txt'
    if (-not (Test-Path $f)) { return '' }
    return (Get-Content $f -Raw).Trim()
}

function Get-DistOverlay([string]$root) {
    $dist = Join-Path $root 'dist'
    Get-Process SpaceCalibrator -ErrorAction SilentlyContinue | Where-Object { $_.Path -and $_.Path.StartsWith($dist, [StringComparison]::OrdinalIgnoreCase) }
}

# WM_CLOSE to every window of the overlay: GLFW turns it into glfwWindowShouldClose, the overlay
# leaves its loop and the recorder flushes to disk. Killed only when it is still there after 15 s.
function Close-DistOverlay([string]$root) {
    $procs = @(Get-DistOverlay $root)
    if ($procs.Count -eq 0) { return $false }
    if (-not ('SC518.Win' -as [type])) {
        Add-Type -Namespace SC518 -Name Win -MemberDefinition @"
public delegate bool EnumProc(System.IntPtr hwnd, System.IntPtr lParam);
[System.Runtime.InteropServices.DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, System.IntPtr lParam);
[System.Runtime.InteropServices.DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(System.IntPtr hwnd, out uint pid);
[System.Runtime.InteropServices.DllImport("user32.dll")] public static extern bool PostMessage(System.IntPtr hwnd, uint msg, System.IntPtr w, System.IntPtr l);
public static int CloseWindowsOf(uint target) {
    int n = 0;
    EnumWindows((h, l) => { uint pid; GetWindowThreadProcessId(h, out pid); if (pid == target) { PostMessage(h, 0x0010, System.IntPtr.Zero, System.IntPtr.Zero); n++; } return true; }, System.IntPtr.Zero);
    return n;
}
"@
    }
    foreach ($p in $procs) { [void][SC518.Win]::CloseWindowsOf([uint32]$p.Id) }
    $deadline = (Get-Date).AddSeconds(15)
    while ((Get-DistOverlay $root) -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 250 }
    foreach ($p in @(Get-DistOverlay $root)) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Milliseconds 500
    return $true
}

function Start-DistOverlay([string]$root) {
    $dist = Join-Path $root 'dist'
    Start-Process -FilePath (Join-Path $dist 'SpaceCalibrator.exe') -WorkingDirectory $dist
}

# recorder.keep_all = true in guard.json (call while no overlay runs, or it writes its copy back)
function Set-KeepAll {
    $guard = Join-Path $env:APPDATA 'space-calibrator\guard.json'
    if (-not (Test-Path $guard)) { return $false }
    $cfg = Get-Content $guard -Raw | ConvertFrom-Json
    if ($cfg.recorder.PSObject.Properties.Name -contains 'keep_all') { $cfg.recorder.keep_all = $true } else { $cfg.recorder | Add-Member -NotePropertyName keep_all -NotePropertyValue $true }
    if (-not ($cfg.recorder.PSObject.Properties.Name -contains 'archive_max_gb')) { $cfg.recorder | Add-Member -NotePropertyName archive_max_gb -NotePropertyValue 0 }
    [IO.File]::WriteAllText($guard, ($cfg | ConvertTo-Json -Depth 6), (New-Object Text.UTF8Encoding($false)))
    return $true
}

function Write-InstallLog([string]$root, [string]$message) {
    $log = Join-Path $root 'logs\install.log'
    New-Item -ItemType Directory -Force -Path (Split-Path $log) | Out-Null
    $line = "[{0:yyyy-MM-dd HH:mm:ss}] {1}" -f (Get-Date), $message
    Add-Content -Path $log -Value $line
    Write-Host $line
}

# SteamVR starts the overlay with itself: dist\manifest.vrmanifest (own app key) in appconfig.json
# and, when SteamVR has no setting for the app yet, autolaunch on in its vrappconfig. Only while
# SteamVR is closed (it holds both files and writes them back); a running overlay registers itself
# through IVRApplications (src\overlay\vr_core.cpp). An existing autolaunch choice is kept.
function Register-SteamVrApp([string]$root) {
    if (Get-Process vrserver, vrmonitor -ErrorAction SilentlyContinue) { return 'SteamVR is running, the overlay registers itself' }
    $steam = (Get-ItemProperty 'HKCU:\Software\Valve\Steam' -ErrorAction SilentlyContinue).SteamPath
    if (-not $steam) { return 'Steam not found, the overlay registers itself' }
    $cfgDir = Join-Path ($steam -replace '/', '\') 'config'
    $manifestFile = Join-Path $root 'dist\manifest.vrmanifest'
    if (-not (Test-Path $manifestFile)) { return 'no manifest in dist' }
    $key = (Get-Content $manifestFile -Raw | ConvertFrom-Json).applications[0].app_key
    $utf8 = New-Object Text.UTF8Encoding($false)
    $done = @()

    $appconfig = Join-Path $cfgDir 'appconfig.json'
    $cfg = if (Test-Path $appconfig) { Get-Content $appconfig -Raw | ConvertFrom-Json } else { [pscustomobject]@{} }
    $paths = @()
    if ($cfg.PSObject.Properties.Name -contains 'manifest_paths') { $paths = @($cfg.manifest_paths) }
    if (-not ($paths | Where-Object { $_ -ieq $manifestFile })) {
        $paths += $manifestFile
        $cfg | Add-Member -NotePropertyName manifest_paths -NotePropertyValue $paths -Force
        [IO.File]::WriteAllText($appconfig, ($cfg | ConvertTo-Json -Depth 8), $utf8)
        $done += 'manifest registered'
    }

    $vrappDir = Join-Path $cfgDir 'vrappconfig'
    $vrapp = Join-Path $vrappDir "$key.vrappconfig"
    if (-not (Test-Path $vrapp)) {
        New-Item -ItemType Directory -Force -Path $vrappDir | Out-Null
        [IO.File]::WriteAllText($vrapp, "{`n   `"autolaunch`" : true,`n   `"last_launch_time`" : `"0`"`n}`n", $utf8)
        $done += 'starts with SteamVR'
    } else {
        $auto = (Get-Content $vrapp -Raw | ConvertFrom-Json).autolaunch
        $done += $(if ($auto) { 'starts with SteamVR' } else { 'autostart off (SteamVR setting kept)' })
    }
    return "SteamVR app $key" + $(if ($done.Count) { ': ' + ($done -join ', ') } else { '' })
}
