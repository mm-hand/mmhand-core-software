[CmdletBinding()]
param()

. (Join-Path $PSScriptRoot "p4_debug_env.ps1")
Assert-P4DebugEnvironment

Write-Host "=== MMHand ESP32-P4 debug environment ===" -ForegroundColor Cyan
Write-Host "Workspace : $P4DebugWorkspace"
Write-Host "Sketch    : $P4SketchPath"
Write-Host "Build     : $P4BuildPath"
Write-Host "FQBN      : $P4Fqbn"
Write-Host ""

& $ArduinoCli version
if ($LASTEXITCODE -ne 0) {
    throw "Unable to run Arduino CLI"
}

$gdbVersion = & $RiscvGdbExe --version | Select-Object -First 1
$openOcdVersion = & cmd.exe /d /c "`"$OpenOcdExe`" --version 2>&1" |
    Select-Object -First 1
Write-Host $gdbVersion
Write-Host $openOcdVersion

$workspaceItem = Get-Item -LiteralPath $P4DebugWorkspace -Force
if ($workspaceItem.LinkType -ne "Junction") {
    Write-Warning "$P4DebugWorkspace is not a junction; verify that sources are not duplicated."
} else {
    Write-Host "Junction target: $($workspaceItem.Target)" -ForegroundColor Green
}

if (Test-Path -LiteralPath $P4ElfPath) {
    Write-Host "Debug ELF is ready: $P4ElfPath" -ForegroundColor Green
} else {
    Write-Host "Debug ELF has not been built yet." -ForegroundColor Yellow
}

$usbDevices = & pnputil.exe /enum-devices /connected /class USB 2>$null | Out-String
if ($usbDevices -match "VID_303A|Espressif|USB JTAG") {
    Write-Host "An Espressif USB/JTAG device is visible." -ForegroundColor Green
} else {
    Write-Warning "No Espressif USB/JTAG device is visible. Connect the P4 Type-C debug port before starting OpenOCD."
}

Write-Host "Environment verification completed." -ForegroundColor Green
