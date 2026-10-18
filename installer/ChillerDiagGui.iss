; Inno Setup script for Chiller Diagnostics GUI.
; Build with build.bat (runs PyInstaller first), or: ISCC installer\ChillerDiagGui.iss
; The version is read from the built exe, which gets it from version_info.txt.

#define AppName "Chiller Diagnostics GUI"
#define AppExeName "ChillerDiagGui.exe"
#define AppPublisher "Derek Phung"
#ifndef SourceDir
  #define SourceDir "..\dist\ChillerDiagGui"
#endif
#define AppVersion GetVersionNumbersString(SourceDir + "\" + AppExeName)
#if AppVersion == ""
  #error dist\ChillerDiagGui\ChillerDiagGui.exe not found - build the app with PyInstaller first
#endif

[Setup]
; AppId identifies the app for upgrades/uninstall - never change it
AppId={{DEC7FF61-182E-45FE-B486-25407B8C26DC}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher={#AppPublisher}
AppPublisherURL=https://github.com/Fainse/ChillerDiagGui
VersionInfoVersion={#AppVersion}
; Per-user install (no admin prompt) into %LOCALAPPDATA%\Programs.
; IT can install for all users with: ChillerDiagGui-Setup-x.exe /ALLUSERS
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=commandline
DefaultDirName={autopf}\ChillerDiagGui
DisableProgramGroupPage=yes
MinVersion=6.1sp1
OutputDir=..\dist
OutputBaseFilename=ChillerDiagGui-Setup-{#AppVersion}
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
UninstallDisplayIcon={app}\{#AppExeName}
UninstallDisplayName={#AppName}
CloseApplications=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[InstallDelete]
; Remove the previous version's libraries so stale files don't linger after an upgrade
Type: filesandordirs; Name: "{app}\deps"

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\{#AppName}"; Filename: "{app}\{#AppExeName}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExeName}"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
