param(
    [Parameter(Mandatory = $true)]
    [string]$InputFile,

    [ValidateSet("jkbms_modbus", "pylon_rs485", "growatt_rs485")]
    [string]$Protocol = "jkbms_modbus",

    [string]$Rx = "CH1",
    [int]$Baud = 115200,
    [ValidateSet("yes", "no")]
    [string]$InvertRx = "no",
    [int]$SamplePoint = 50,
    [int]$MaxLines = 200,
    [switch]$AllDecoderLines,
    [string]$SigrokCli = "$env:USERPROFILE\scoop\shims\sigrok-cli.exe",
    [string]$PulseViewDecoderDir = "C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders"
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..\..")
$customDecoderDir = Join-Path $scriptDir "decoders"
$safeProtocol = $Protocol -replace '[^A-Za-z0-9_.-]', '_'
$safeRx = $Rx -replace '[^A-Za-z0-9_.-]', '_'
$bundleDir = Join-Path $repoRoot "build\sigrok-cli-decoders-$safeProtocol-$safeRx-$PID"

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

if (-not (Test-Path $InputFile)) {
    throw "Input .sr file not found: $InputFile"
}
if (-not (Test-Path $SigrokCli)) {
    throw "sigrok-cli not found: $SigrokCli"
}
if (-not (Test-Path $PulseViewDecoderDir)) {
    throw "PulseView built-in decoder directory not found: $PulseViewDecoderDir"
}

if (Test-Path $bundleDir) {
    $resolvedBundle = (Resolve-Path $bundleDir).Path
    $resolvedBuild = (Resolve-Path (Join-Path $repoRoot "build")).Path
    if (-not $resolvedBundle.StartsWith($resolvedBuild, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove unexpected path: $resolvedBundle"
    }
    Remove-Item -LiteralPath $resolvedBundle -Recurse -Force
}

New-Item -ItemType Directory -Path $bundleDir | Out-Null
Copy-Item -Path (Join-Path $PulseViewDecoderDir "*") -Destination $bundleDir -Recurse -Force
Get-ChildItem -Path $customDecoderDir -Directory | ForEach-Object {
    if ((Test-Path (Join-Path $_.FullName "pd.py")) -and (Test-Path (Join-Path $_.FullName "__init__.py"))) {
        Copy-DecoderDir -Source $_.FullName -Destination (Join-Path $bundleDir $_.Name)
    }
}

$env:SIGROKDECODE_DIR = $bundleDir

$uart = "uart:rx=$Rx" +
    ":baudrate=$Baud" +
    ":data_bits=8" +
    ":parity=none" +
    ":stop_bits=1" +
    ":bit_order=lsb-first" +
    ":format=hex" +
    ":invert_rx=$InvertRx" +
    ":sample_point=$SamplePoint"
$stack = "$uart,$Protocol"

$keyPattern = switch ($Protocol) {
    "jkbms_modbus" { "JKBMS Modbus|crc=|CRC|response|request|read holding|decoded|warning|candidate|cell|SOC|SOH|pack|temp" }
    "pylon_rs485" { "Pylon|chk=|CHK|Response|Request|Decoded|warning|cell|SOC|SOH|status|payload" }
    "growatt_rs485" { "Growatt|crc=|CRC|response|request|decoded|warning|cell|SOC|SOH|pack|temp|limit" }
}

$decoderPattern = "$Protocol-\d+:"
$seen = 0
$printed = 0
$crcOk = 0
$crcBad = 0
$warnings = 0
$frames = 0

Write-Host "Input: $InputFile"
Write-Host "Decoder bundle: $bundleDir"
Write-Host "Stack: $stack"
Write-Host ""

& $SigrokCli -i $InputFile -P $stack --protocol-decoder-samplenum 2>&1 | ForEach-Object {
    $line = $_.ToString()
    if ($line -notmatch $decoderPattern) {
        return
    }

    $seen += 1
    if ($line -match "crc=OK|chk=OK|CRC 0x[0-9A-Fa-f]+ OK|CHK 0x[0-9A-Fa-f]+ OK") {
        $crcOk += 1
    }
    if ($line -match "crc=BAD|chk=BAD|CRC BAD|CRC 0x[0-9A-Fa-f]+ BAD|CHK 0x[0-9A-Fa-f]+ BAD") {
        $crcBad += 1
    }
    if ($line -match "warning|BAD|Incomplete|Frame error|tentative") {
        $warnings += 1
    }
    if ($line -match "req|rsp|request|response|frame") {
        $frames += 1
    }

    $shouldPrint = $AllDecoderLines -or ($line -match $keyPattern)
    if ($shouldPrint -and $printed -lt $MaxLines) {
        Write-Host $line
        $printed += 1
    }
}

Write-Host ""
Write-Host "Summary:"
Write-Host "  decoder lines : $seen"
Write-Host "  frame lines   : $frames"
Write-Host "  CRC OK lines  : $crcOk"
Write-Host "  CRC BAD lines : $crcBad"
Write-Host "  warning lines : $warnings"
if (($MaxLines -gt 0) -and ($printed -ge $MaxLines)) {
    Write-Host "  output clipped at $MaxLines lines; rerun with -MaxLines or -AllDecoderLines if needed"
}
