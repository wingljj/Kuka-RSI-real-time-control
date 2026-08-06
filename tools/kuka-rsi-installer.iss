; KUKA RSI 实时位姿跟踪 - 离线安装包
; 编译: "D:\Software\innosetup\content\ISCC.exe" tools\kuka-rsi-installer.iss
;
; 离线：dist/ 已包含全部 Qt 运行库与插件，目标机无需安装 Qt。
; 安装范围：当前用户（PrivilegesRequired=lowest，无管理员弹窗）。

#define MyAppName "KUKA RSI 实时位姿跟踪"
#define MyAppVersion "1.1.3"
#define MyAppExeName "rsi_host.exe"
#define MyAppId "{{F3B1A6E8-2D4C-4E9A-9B7C-1A2B3C4D5E6F}}"

[Setup]
AppId={#MyAppId}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher=wingljj
AppComments=KUKA RSI POSCORR 实时位姿跟踪上位机（离线版）
DefaultDirName={autopf}\KukaRsiHost
DefaultGroupName=KUKA RSI 实时位姿跟踪
DisableProgramGroupPage=yes
OutputDir=..\release
OutputBaseFilename=Kuka-RSI-实时位姿跟踪-{#MyAppVersion}-离线安装包
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=lowest
ArchitecturesInstallIn64BitMode=x64compatible
SetupLogging=yes
UninstallDisplayIcon={app}\{#MyAppExeName}
UninstallDisplayName={#MyAppName}

[Languages]
Name: "chinesesimplified"; MessagesFile: "compiler:Default.isl,compiler:Languages\ChineseSimplified.isl"

[Tasks]
Name: "desktopicon"; Description: "创建桌面快捷方式"; GroupDescription: "附加任务:"

[Files]
; recursesubdirs 把 dist/ 下的 Qt 插件目录（platforms/styles/imageformats）一并装入
Source: "..\dist\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "*.log"

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{group}\启动模拟器（无需机器人）"; Filename: "{app}\启动模拟器.bat"
Name: "{group}\网络诊断"; Filename: "{app}\网络诊断.bat"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "启动 {#MyAppName}"; Flags: nowait postinstall skipifsilent

[Code]
// 安装目录若已存在旧版，先确认（防止覆盖正在运行的程序）
function InitializeSetup(): Boolean;
begin
  Result := True;
end;
