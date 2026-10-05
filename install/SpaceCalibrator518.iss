; SpaceCalibrator518 installer (Inno Setup 6). tools\make_release_zip.ps1 builds it next to the zip:
;   ISCC.exe /DAppVersion=1.0.2 /DStageDir=<release\SpaceCalibrator518> /DOutDir=<release> install\SpaceCalibrator518.iss
; Per user, no admin rights: %LOCALAPPDATA%\Programs\SpaceCalibrator518. With SteamVR closed it runs
;   use-driver.ps1 fork        SteamVR loads the driver from here instead of the Steam version's
;                              (both are named 01spacecalibrator, only one can be active)
;   steamvr-app.ps1 register   the overlay starts with SteamVR
; and on uninstall steamvr-app.ps1 unregister, then use-driver.ps1 steam (none without the Steam
; version). Output of both in setup-steamvr.log. Settings and recordings in %APPDATA%\space-calibrator stay.
; (install\installer.nsi is upstream's old installer for 1.5, not used by this fork.)

#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif
#ifndef StageDir
  #define StageDir "..\release\SpaceCalibrator518"
#endif
#ifndef OutDir
  #define OutDir "..\release"
#endif

[Setup]
AppId={{5C8E2F4A-518B-4E7D-9A51-1D0C5A518B18}
AppName=SpaceCalibrator518
AppVersion={#AppVersion}
AppVerName=SpaceCalibrator518 {#AppVersion}
AppPublisher=Blise518B
AppPublisherURL=https://github.com/Blise518B/SpaceCalibrator518
AppSupportURL=https://github.com/Blise518B/SpaceCalibrator518/issues
AppUpdatesURL=https://github.com/Blise518B/SpaceCalibrator518/releases
DefaultDirName={autopf}\SpaceCalibrator518
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
LicenseFile={#StageDir}\LICENSE
OutputDir={#OutDir}
OutputBaseFilename=SpaceCalibrator518-Setup
SetupIconFile=..\src\overlay\SpaceCalibrator.ico
UninstallDisplayIcon={app}\SpaceCalibrator.exe
UninstallDisplayName=SpaceCalibrator518
VersionInfoVersion={#AppVersion}
VersionInfoProductName=SpaceCalibrator518
VersionInfoDescription=SpaceCalibrator518 Setup
WizardStyle=modern
Compression=lzma2/max
SolidCompression=yes
; SteamVR is checked below; nothing else holds these files
CloseApplications=no
SetupLogging=yes

[Languages]
Name: "en"; MessagesFile: "compiler:Default.isl"

[Messages]
FinishedLabel=Setup has installed SpaceCalibrator518. SteamVR now loads its driver, and the overlay starts with SteamVR.

[InstallDelete]
; assets of an older version
Type: filesandordirs; Name: "{app}\assets"

[Files]
Source: "{#StageDir}\*"; DestDir: "{app}"; Excludes: "INSTALL.txt"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\SpaceCalibrator518"; Filename: "{app}\SpaceCalibrator.exe"; WorkingDir: "{app}"

[Run]
Filename: "steam://rungameid/250820"; Description: "Start SteamVR"; Flags: shellexec postinstall nowait skipifsilent

[UninstallDelete]
Type: files; Name: "{app}\setup-steamvr.log"

[Code]
const
  RunningMessage = 'SteamVR or Space Calibrator is running. Close SteamVR, then click Retry.';

function ProcessRunning(const Name: String): Boolean;
var
  Locator, Service, Found: Variant;
begin
  Result := False;
  try
    Locator := CreateOleObject('WbemScripting.SWbemLocator');
    Service := Locator.ConnectServer('.', 'root\CIMV2');
    Found := Service.ExecQuery('SELECT ProcessId FROM Win32_Process WHERE Name = ''' + Name + '''');
    Result := Found.Count > 0;
  except
    Log('process check failed: ' + GetExceptionMessage);
  end;
end;

function SteamVRRunning: Boolean;
begin
  Result := ProcessRunning('vrserver.exe') or ProcessRunning('vrmonitor.exe') or ProcessRunning('SpaceCalibrator.exe');
end;

// powershell with a script from the install folder, output appended to setup-steamvr.log
function RunScript(const Script, Args: String): Integer;
var
  Line: String;
  Code: Integer;
begin
  Line := '/C powershell.exe -NoProfile -ExecutionPolicy Bypass -File "' + ExpandConstant('{app}\') + Script + '" ' + Args +
    ' >> "' + ExpandConstant('{app}\setup-steamvr.log') + '" 2>&1';
  if not Exec(ExpandConstant('{cmd}'), Line, ExpandConstant('{app}'), SW_HIDE, ewWaitUntilTerminated, Code) then
    Code := -1;
  Log(Script + ' ' + Args + ': exit code ' + IntToStr(Code));
  Result := Code;
end;

function InitializeSetup: Boolean;
begin
  Result := True;
  if not FileExists(ExpandConstant('{localappdata}\openvr\openvrpaths.vrpath')) then begin
    if not WizardSilent then
      MsgBox('SteamVR has not been set up on this computer yet. Start SteamVR once, close it, then run this setup again.', mbError, MB_OK);
    Log('no openvrpaths.vrpath: SteamVR never ran');
    Result := False;
  end;
end;

function PrepareToInstall(var NeedsRestart: Boolean): String;
begin
  Result := '';
  while SteamVRRunning do begin
    if WizardSilent then begin
      Result := 'SteamVR or Space Calibrator is running. Close SteamVR and run the setup again.';
      exit;
    end;
    if MsgBox(RunningMessage, mbError, MB_RETRYCANCEL) = IDCANCEL then begin
      Result := 'SteamVR or Space Calibrator is still running. Close SteamVR and run the setup again.';
      exit;
    end;
  end;
end;

procedure CurStepChanged(CurStep: TSetupStep);
begin
  if CurStep <> ssPostInstall then
    exit;
  if RunScript('use-driver.ps1', 'fork') <> 0 then
    MsgBox('SteamVR could not be told to load this driver. With SteamVR closed, double-click use-fork-driver.bat in ' +
      ExpandConstant('{app}') + ' to try again. Details: setup-steamvr.log in the same folder.', mbError, MB_OK);
  if RunScript('steamvr-app.ps1', 'register') <> 0 then
    MsgBox('The overlay could not be set to start with SteamVR. Start SpaceCalibrator.exe once while SteamVR runs; it registers itself.' +
      ' Details: setup-steamvr.log in ' + ExpandConstant('{app}') + '.', mbInformation, MB_OK);
end;

function InitializeUninstall: Boolean;
begin
  Result := True;
  while SteamVRRunning do begin
    if UninstallSilent then begin
      Log('SteamVR or Space Calibrator is running: not uninstalled');
      Result := False;
      exit;
    end;
    if MsgBox(RunningMessage, mbError, MB_RETRYCANCEL) = IDCANCEL then begin
      Result := False;
      exit;
    end;
  end;
end;

procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);
begin
  if CurUninstallStep <> usUninstall then
    exit;
  // before the files go: the scripts run from the install folder
  RunScript('steamvr-app.ps1', 'unregister');
  if RunScript('use-driver.ps1', 'steam') <> 0 then
    RunScript('use-driver.ps1', 'none');
end;
