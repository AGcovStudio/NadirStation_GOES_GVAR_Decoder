; Inno Setup 6 script - NadirStation GOES GVAR Decoder
; Build with scripts\build_release.bat (it creates the "dist" folder first), or open this file in Inno Setup.
#define MyAppName "NadirStation GOES GVAR Decoder"
#define MyAppVersion "1.0.0"
#define MyAppPublisher "AGcov（BI2QYN）"
#define MyAppExe "NadirStation_GOES_GVAR_Decoder.exe"
#define DistDir "..\dist"

[Setup]
; The AppId identifies this program for upgrades and uninstall. Do not change it between versions.
AppId={{5E1D7A42-93B8-4C6F-A0D4-7B2E8F13C95A}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
VersionInfoVersion={#MyAppVersion}
VersionInfoCompany=AGcov (BI2QYN)
VersionInfoDescription={#MyAppName} Setup
DefaultDirName={autopf}\NadirStation_GOES_GVAR_Decoder
DefaultGroupName=NadirStation GOES GVAR Decoder
DisableProgramGroupPage=yes
LicenseFile=EULA.txt
OutputDir=output
OutputBaseFilename=NadirStation_GOES_GVAR_Decoder_Setup_{#MyAppVersion}
SetupIconFile=..\resources\icon\app.ico
UninstallDisplayIcon={app}\{#MyAppExe}
Compression=lzma2/ultra64
SolidCompression=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
WizardStyle=modern
ShowLanguageDialog=auto
MinVersion=10.0

[Languages]
#if FileExists(CompilerPath + "Languages\ChineseSimplified.isl")
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"
#elif FileExists(CompilerPath + "Languages\Unofficial\ChineseSimplified.isl")
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\Unofficial\ChineseSimplified.isl"
#elif FileExists(SourcePath + "ChineseSimplified.isl")
Name: "chinesesimplified"; MessagesFile: "ChineseSimplified.isl"
#endif
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "{#DistDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "EULA.txt"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{autoprograms}\NadirStation GOES GVAR Decoder"; Filename: "{app}\{#MyAppExe}"
Name: "{autodesktop}\NadirStation GOES GVAR Decoder"; Filename: "{app}\{#MyAppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExe}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent
