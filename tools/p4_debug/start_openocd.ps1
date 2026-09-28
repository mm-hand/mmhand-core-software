[CmdletBinding()]
param()

. (Join-Path $PSScriptRoot "p4_debug_env.ps1")
Assert-P4DebugEnvironment

$env:OPENOCD_SCRIPTS = $OpenOcdScripts

Write-Host "Starting ESP32-P4 built-in USB-JTAG OpenOCD..." -ForegroundColor Cyan
Write-Host "GDB server: localhost:3333"
Write-Host "Press Ctrl+C in this terminal to stop OpenOCD."

& $OpenOcdExe `
    -s $OpenOcdScripts `
    -f "board/esp32p4-builtin.cfg"

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD exited with code $LASTEXITCODE"
}
