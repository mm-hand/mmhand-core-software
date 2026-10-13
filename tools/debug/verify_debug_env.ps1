[CmdletBinding()]
param(
    [ValidateSet("p4", "s3")]
    [string]$Target = "p4"
)

. (Join-Path $PSScriptRoot "debug_env.ps1") -Target $Target
Assert-DebugEnvironment

Write-Host "=== MMHand ESP32 debug environment (target: $Target) ===" -ForegroundColor Cyan
Write-Host "Repo root : $RepoRoot"
Write-Host "Sketch    : $SketchPath"
Write-Host "Build     : $BuildPath"
Write-Host "FQBN      : $Fqbn"
Write-Host "Board cfg : $OpenOcdBoardCfg"
Write-Host ""

& $ArduinoCli version
if ($LASTEXITCODE -ne 0) {
    throw "Unable to run Arduino CLI"
}

$gdbVersion = & $GdbExe --version | Select-Object -First 1
$openOcdVersion = & cmd.exe /d /c "`"$OpenOcdExe`" --version 2>&1" |
    Select-Object -First 1
Write-Host $gdbVersion
Write-Host $openOcdVersion

if (Test-Path -LiteralPath $ElfPath) {
    Write-Host "Debug ELF is ready: $ElfPath" -ForegroundColor Green
    $buildHashFile = Join-Path $BuildPath "$ElfName.sha256.txt"
    if (Test-Path -LiteralPath $buildHashFile) {
        Write-Host "Build SHA256: $(Get-Content -LiteralPath $buildHashFile)"
    }
    $flashedMarker = Join-Path $BuildPath "$ElfName.flashed.sha256"
    if (Test-Path -LiteralPath $flashedMarker) {
        Write-Host "Flashed SHA256: $(Get-Content -LiteralPath $flashedMarker)" -ForegroundColor Green
    } else {
        Write-Warning "No flashed-firmware marker yet; upload_debug.ps1 has not run for this build."
    }
} else {
    Write-Host "Debug ELF has not been built yet." -ForegroundColor Yellow
}

$usbDevices = & pnputil.exe /enum-devices /connected /class USB 2>$null | Out-String
if ($usbDevices -match "VID_303A|Espressif|USB JTAG") {
    Write-Host "An Espressif USB/JTAG device is visible." -ForegroundColor Green
} else {
    Write-Warning "No Espressif USB/JTAG device is visible. Connect the $SketchName Type-C debug (USB-JTAG) port before starting OpenOCD."
}

Write-Host "Environment verification completed." -ForegroundColor Green
