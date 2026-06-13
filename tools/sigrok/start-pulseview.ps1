param(
    [string]$PulseViewExe = "C:\Program Files\sigrok\PulseView\pulseview.exe",
    [string]$BuiltinDecoderDir = "C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders"
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..\..")
$buildDir = Join-Path $repoRoot "build"
$bundleDir = Join-Path $buildDir "pulseview-decoders"
$customDecoderDir = Join-Path $scriptDir "decoders"

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
Copy-Item -Path (Join-Path $customDecoderDir "pylon_rs485") -Destination (Join-Path $bundleDir "pylon_rs485") -Recurse -Force
Copy-Item -Path (Join-Path $customDecoderDir "pylon_can") -Destination (Join-Path $bundleDir "pylon_can") -Recurse -Force

$env:SIGROKDECODE_DIR = $bundleDir
Start-Process -FilePath $PulseViewExe -WorkingDirectory (Split-Path -Parent $PulseViewExe)

Write-Host "PulseView started with SIGROKDECODE_DIR=$bundleDir"
