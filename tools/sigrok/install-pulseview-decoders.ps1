param(
    [string]$BuiltinDecoderDir = "C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders",
    [string]$InstallDecoderDir = "C:\ProgramData\libsigrokdecode\decoders",
    [string]$PulseViewExe = "C:\Program Files\sigrok\PulseView\pulseview.exe",
    [switch]$SkipShortcuts
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$customDecoderDir = Join-Path $scriptDir "decoders"

function Get-CustomDecoderDirs {
    param([string]$Root)

    Get-ChildItem -Path $Root -Directory | Where-Object {
        (Test-Path (Join-Path $_.FullName "pd.py")) -and
        (Test-Path (Join-Path $_.FullName "__init__.py"))
    }
}

function Copy-DecoderDir {
    param(
        [string]$Source,
        [string]$Destination
    )

    if (Test-Path $Destination) {
        Remove-Item -LiteralPath $Destination -Recurse -Force
    }
    Copy-Item -Path $Source -Destination $Destination -Recurse -Force
    $pycache = Join-Path $Destination "__pycache__"
    if (Test-Path $pycache) {
        Remove-Item -LiteralPath $pycache -Recurse -Force
    }
}

if (-not (Test-Path $BuiltinDecoderDir)) {
    throw "PulseView built-in decoder directory not found: $BuiltinDecoderDir"
}

$resolvedInstallParent = Split-Path -Parent $InstallDecoderDir
if (-not (Test-Path $resolvedInstallParent)) {
    New-Item -ItemType Directory -Force -Path $resolvedInstallParent | Out-Null
}
if (Test-Path $InstallDecoderDir) {
    $resolvedInstall = (Resolve-Path $InstallDecoderDir).Path
    if ($resolvedInstall -notlike "*\libsigrokdecode\decoders") {
        throw "Refusing to remove unexpected decoder directory: $resolvedInstall"
    }
    Remove-Item -LiteralPath $resolvedInstall -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $InstallDecoderDir | Out-Null

Copy-Item -Path (Join-Path $BuiltinDecoderDir "*") -Destination $InstallDecoderDir -Recurse -Force

foreach ($decoder in Get-CustomDecoderDirs -Root $customDecoderDir) {
    Copy-DecoderDir -Source $decoder.FullName -Destination (Join-Path $InstallDecoderDir $decoder.Name)
}

[Environment]::SetEnvironmentVariable("SIGROKDECODE_DIR", $InstallDecoderDir, "User")
$env:SIGROKDECODE_DIR = $InstallDecoderDir

if (-not $SkipShortcuts) {
    $launcher = Join-Path $scriptDir "start-pulseview.ps1"
    $powershell = Join-Path $env:WINDIR "System32\WindowsPowerShell\v1.0\powershell.exe"
    $shortcutTargets = @(
        (Join-Path ([Environment]::GetFolderPath("Desktop")) "PulseView Workbench Decoders.lnk"),
        (Join-Path ([Environment]::GetFolderPath("Desktop")) "PulseView BMS Decoders.lnk"),
        (Join-Path ([Environment]::GetFolderPath("Desktop")) "PulseView Pylon.lnk"),
        (Join-Path $env:APPDATA "Microsoft\Windows\Start Menu\Programs\PulseView Workbench Decoders.lnk"),
        (Join-Path $env:APPDATA "Microsoft\Windows\Start Menu\Programs\PulseView BMS Decoders.lnk"),
        (Join-Path $env:APPDATA "Microsoft\Windows\Start Menu\Programs\PulseView Pylon.lnk")
    )

    $ws = New-Object -ComObject WScript.Shell
    foreach ($shortcutPath in $shortcutTargets) {
        $shortcut = $ws.CreateShortcut($shortcutPath)
        $shortcut.TargetPath = $powershell
        $shortcut.Arguments = "-NoProfile -ExecutionPolicy Bypass -File `"$launcher`""
        $shortcut.WorkingDirectory = (Resolve-Path (Join-Path $scriptDir "..\..")).Path
        $shortcut.Description = "PulseView with workbench BMS protocol decoders from this firmware repository"
        if (Test-Path $PulseViewExe) {
            $shortcut.IconLocation = "$PulseViewExe,0"
        }
        $shortcut.Save()
    }
}

Write-Host "Installed PulseView decoders to $InstallDecoderDir"
Write-Host "User SIGROKDECODE_DIR=$InstallDecoderDir"
Write-Host "Restart PulseView before checking the decoder selector."
