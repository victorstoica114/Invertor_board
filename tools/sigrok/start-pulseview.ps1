param(
    [string]$PulseViewExe = "C:\Program Files\sigrok\PulseView\pulseview.exe",
    [string]$BuiltinDecoderDir = "C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders",
    [string]$SettingsFile = "",
    [string]$Driver = "",
    [switch]$Clean,
    [switch]$DontScan
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..\..")
$buildDir = Join-Path $repoRoot "build"
$bundleDir = Join-Path $buildDir "pulseview-decoders"
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
    Get-ChildItem -LiteralPath $Destination -Recurse -Force -Directory |
        Where-Object { $_.Name -eq "__pycache__" } |
        Remove-Item -Recurse -Force
    Get-ChildItem -LiteralPath $Destination -Recurse -Force -File |
        Where-Object { $_.Extension -in @(".pyc", ".pyo") } |
        Remove-Item -Force
}

if (-not (Test-Path $PulseViewExe)) {
    throw "PulseView executable not found: $PulseViewExe"
}

if (-not (Test-Path $BuiltinDecoderDir)) {
    throw "PulseView built-in decoder directory not found: $BuiltinDecoderDir"
}

if (-not (Test-Path $buildDir)) {
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

if (Test-Path $bundleDir) {
    $resolvedBundle = (Resolve-Path $bundleDir).Path
    $resolvedBuild = (Resolve-Path $buildDir).Path
    if (-not $resolvedBundle.StartsWith($resolvedBuild, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove unexpected path: $resolvedBundle"
    }
    Remove-Item -LiteralPath $resolvedBundle -Recurse -Force
}

New-Item -ItemType Directory -Path $bundleDir | Out-Null
Copy-Item -Path (Join-Path $BuiltinDecoderDir "*") -Destination $bundleDir -Recurse -Force
foreach ($decoder in Get-CustomDecoderDirs -Root $customDecoderDir) {
    Copy-DecoderDir -Source $decoder.FullName -Destination (Join-Path $bundleDir $decoder.Name)
}

$env:SIGROKDECODE_DIR = $bundleDir

$arguments = @()
if ($Clean) {
    $arguments += "--clean"
}
if ($Driver) {
    $arguments += "--driver"
    $arguments += $Driver
}
if ($DontScan) {
    $arguments += "--dont-scan"
}
if ($SettingsFile) {
    $resolvedSettings = (Resolve-Path $SettingsFile).Path
    $arguments += "--settings"
    $arguments += $resolvedSettings
}

if ($arguments.Count -gt 0) {
    Start-Process -FilePath $PulseViewExe -ArgumentList $arguments -WorkingDirectory (Split-Path -Parent $PulseViewExe)
} else {
    Start-Process -FilePath $PulseViewExe -WorkingDirectory (Split-Path -Parent $PulseViewExe)
}

Write-Host "PulseView started with SIGROKDECODE_DIR=$bundleDir"
if ($SettingsFile) {
    Write-Host "PulseView settings file=$resolvedSettings"
}
