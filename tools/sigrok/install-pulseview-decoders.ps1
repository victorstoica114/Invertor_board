param(
    [string]$BuiltinDecoderDir = "C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders",
    [string]$InstallDecoderDir = "C:\ProgramData\libsigrokdecode\decoders",
    [string]$PulseViewExe = "C:\Program Files\sigrok\PulseView\pulseview.exe",
    [switch]$SkipShortcuts
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$customDecoderDir = Join-Path $scriptDir "decoders"

if (-not (Test-Path $BuiltinDecoderDir)) {
    throw "PulseView built-in decoder directory not found: $BuiltinDecoderDir"
}

New-Item -ItemType Directory -Force -Path $InstallDecoderDir | Out-Null

Copy-Item -Path (Join-Path $BuiltinDecoderDir "*") -Destination $InstallDecoderDir -Recurse -Force

foreach ($name in @("pylon_rs485", "pylon_can")) {
    $src = Join-Path $customDecoderDir $name
    $dst = Join-Path $InstallDecoderDir $name
    if (-not (Test-Path $src)) {
        throw "Custom decoder not found: $src"
    }
    New-Item -ItemType Directory -Force -Path $dst | Out-Null
    Copy-Item -Path (Join-Path $src "*") -Destination $dst -Recurse -Force
}

[Environment]::SetEnvironmentVariable("SIGROKDECODE_DIR", $InstallDecoderDir, "User")
$env:SIGROKDECODE_DIR = $InstallDecoderDir

if (-not $SkipShortcuts) {
    $launcher = Join-Path $scriptDir "start-pulseview.ps1"
    $powershell = Join-Path $env:WINDIR "System32\WindowsPowerShell\v1.0\powershell.exe"
    $shortcutTargets = @(
        (Join-Path ([Environment]::GetFolderPath("Desktop")) "PulseView Pylon.lnk"),
        (Join-Path $env:APPDATA "Microsoft\Windows\Start Menu\Programs\PulseView Pylon.lnk")
    )

    $ws = New-Object -ComObject WScript.Shell
    foreach ($shortcutPath in $shortcutTargets) {
        $shortcut = $ws.CreateShortcut($shortcutPath)
        $shortcut.TargetPath = $powershell
        $shortcut.Arguments = "-NoProfile -ExecutionPolicy Bypass -File `"$launcher`""
        $shortcut.WorkingDirectory = (Resolve-Path (Join-Path $scriptDir "..\..")).Path
        $shortcut.Description = "PulseView with built-in, Pylon CAN, and Pylon RS485 decoders"
        if (Test-Path $PulseViewExe) {
            $shortcut.IconLocation = "$PulseViewExe,0"
        }
        $shortcut.Save()
    }
}

Write-Host "Installed PulseView decoders to $InstallDecoderDir"
Write-Host "User SIGROKDECODE_DIR=$InstallDecoderDir"
Write-Host "Restart PulseView before checking the decoder selector."
