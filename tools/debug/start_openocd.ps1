[CmdletBinding()]
param(
    [ValidateSet("p4", "s3")]
    [string]$Target = "p4"
)

. (Join-Path $PSScriptRoot "debug_env.ps1") -Target $Target
Assert-DebugEnvironment

$env:OPENOCD_SCRIPTS = $OpenOcdScripts

Write-Host "Starting OpenOCD for $SketchName built-in USB-JTAG..." -ForegroundColor Cyan
Write-Host "GDB server: localhost:3333"
Write-Host "Keep this terminal open while debugging; press Ctrl+C to stop OpenOCD."

# gdb_memory_map disable: OpenOCD otherwise probes the flash map on every GDB
# connect and rejects the connection outright when that probe fails.
& $OpenOcdExe `
    -s $OpenOcdScripts `
    -f $OpenOcdBoardCfg `
    -c "gdb_memory_map disable"

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD exited with code $LASTEXITCODE"
}
