[CmdletBinding()]
param(
    [ValidateSet("p4", "s3")]
    [string]$Target = "p4"
)

. (Join-Path $PSScriptRoot "debug_env.ps1") -Target $Target
Assert-DebugEnvironment

$env:OPENOCD_SCRIPTS = $OpenOcdScripts

# Pin the physical USB-Serial-JTAG adapter recorded by upload_debug.ps1; every
# ESP32 board shares one VID/PID and OpenOCD would otherwise attach whichever
# board enumerates first.
$adapterArgs = @()
$jtagSerialFile = Join-Path $BuildPath "$ElfName.jtag_serial"
if (Test-Path $jtagSerialFile) {
    $jtagSerial = (Get-Content -LiteralPath $jtagSerialFile -Raw).Trim()
    if ($jtagSerial) {
        $adapterArgs = @("-c", "adapter serial $jtagSerial")
        Write-Host "Pinning USB-Serial-JTAG adapter: $jtagSerial" -ForegroundColor Cyan
    }
}

Write-Host "Starting OpenOCD for $SketchName built-in USB-JTAG..." -ForegroundColor Cyan
Write-Host "GDB server: localhost:3333"
Write-Host "Keep this terminal open while debugging; press Ctrl+C to stop OpenOCD."

# gdb_memory_map disable: OpenOCD otherwise probes the flash map on every GDB
# connect and rejects the connection outright when that probe fails.
& $OpenOcdExe `
    -s $OpenOcdScripts `
    -f $OpenOcdBoardCfg `
    @adapterArgs `
    -c "gdb_memory_map disable"

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD exited with code $LASTEXITCODE"
}
