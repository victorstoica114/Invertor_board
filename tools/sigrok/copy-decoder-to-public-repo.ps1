param(
    [Parameter(Mandatory = $true)]
    [string]$DecoderName,

    [string]$PublicRepoDir = "C:\Users\Admin\Documents\sigrok-pylon-bms-decoders"
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourceRoot = Join-Path $scriptDir "decoders"
$source = Join-Path $sourceRoot $DecoderName
$destinationRoot = Join-Path $PublicRepoDir "decoders"
$destination = Join-Path $destinationRoot $DecoderName
$skipDirs = @("__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache")
$skipExtensions = @(".pyc", ".pyo")

function Test-GeneratedArtifact {
    param([System.IO.FileSystemInfo]$Item)

    if ($Item.PSIsContainer -and $skipDirs -contains $Item.Name) {
        return $true
    }
    if (-not $Item.PSIsContainer -and $skipExtensions -contains $Item.Extension) {
        return $true
    }
    return $false
}

if (-not (Test-Path -LiteralPath $source)) {
    throw "Decoder source not found: $source"
}
if (-not (Test-Path -LiteralPath (Join-Path $source "pd.py")) -or
    -not (Test-Path -LiteralPath (Join-Path $source "__init__.py"))) {
    throw "Not a PulseView decoder directory: $source"
}
if (-not (Test-Path -LiteralPath $PublicRepoDir)) {
    throw "Public decoder repo not found: $PublicRepoDir"
}

New-Item -ItemType Directory -Force -Path $destinationRoot | Out-Null

if (Test-Path -LiteralPath $destination) {
    $resolvedDestination = (Resolve-Path -LiteralPath $destination).Path
    $resolvedRoot = (Resolve-Path -LiteralPath $destinationRoot).Path
    if (-not $resolvedDestination.StartsWith($resolvedRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove unexpected path: $resolvedDestination"
    }
    Remove-Item -LiteralPath $resolvedDestination -Recurse -Force
}

New-Item -ItemType Directory -Force -Path $destination | Out-Null
$resolvedSource = (Resolve-Path -LiteralPath $source).Path

Get-ChildItem -LiteralPath $source -Recurse -Force | ForEach-Object {
    if (Test-GeneratedArtifact $_) {
        return
    }

    $relative = $_.FullName.Substring($resolvedSource.Length) -replace "^[\\/]+", ""
    foreach ($segment in ($relative -split "[\\/]")) {
        if ($skipDirs -contains $segment) {
            return
        }
    }

    $target = Join-Path $destination $relative
    if ($_.PSIsContainer) {
        New-Item -ItemType Directory -Force -Path $target | Out-Null
        return
    }

    $targetParent = Split-Path -Parent $target
    New-Item -ItemType Directory -Force -Path $targetParent | Out-Null
    Copy-Item -LiteralPath $_.FullName -Destination $target -Force
}

Write-Host "Copied decoder $DecoderName to $destination"
