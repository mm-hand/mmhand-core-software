[CmdletBinding()]
param(
    [ValidateSet("p4", "s3")]
    [string]$Target = "p4",
    [Parameter(Mandatory = $true)]
    [ValidatePattern("^COM[0-9]+$")]
    [string]$Port,
    [ValidateSet(115200, 460800, 921600)]
    [int]$UploadBaud = 115200
)

. (Join-Path $PSScriptRoot "debug_env.ps1") -Target $Target
Assert-DebugEnvironment
Assert-PathExists -Path $ElfPath -Description "Debug ELF"
Assert-PathExists -Path $BinPath -Description "Debug application binary"

Write-Host "Uploading the matching $SketchName debug firmware to $Port..." -ForegroundColor Cyan
Write-Host "EraseFlash remains disabled; calibration and multi-turn recovery are restored from desktop JSON."

& $ArduinoCli upload `
    --port $Port `
    --fqbn $Fqbn `
    --upload-property "upload.speed=$UploadBaud" `
    --verify `
    --input-dir $BuildPath `
    $SketchPath

if ($LASTEXITCODE -ne 0) {
    throw "Arduino debug upload failed with exit code $LASTEXITCODE"
}

# Record which ELF went onto the chip so gdb_batch.py can warn when the ELF
# it would load no longer matches the flashed firmware.
$elfHash = (Get-FileHash -LiteralPath $ElfPath -Algorithm SHA256).Hash
$flashedMarker = Join-Path $BuildPath "$ElfName.flashed.sha256"
"$elfHash  $ElfName" | Set-Content -LiteralPath $flashedMarker -Encoding ascii

# Record the physical USB-Serial-JTAG adapter so OpenOCD can pin it via
# `adapter serial`; every ESP32 board shares one VID/PID and OpenOCD would
# otherwise attach whichever board enumerates first.
$jtagSerial = & python -c "from serial.tools import list_ports; import sys; m = [p.serial_number for p in list_ports.comports() if p.device == sys.argv[1] and p.vid == 0x303A and p.pid == 0x1001]; print(m[0] if m and m[0] else '')" $Port
if ($jtagSerial) {
    Set-Content -LiteralPath (Join-Path $BuildPath "$ElfName.jtag_serial") -Value $jtagSerial -Encoding ascii
    Write-Host "JTAG adapter serial: $jtagSerial"
}
else {
    Write-Host "Upload port is not USB-Serial-JTAG; preserving any existing JTAG adapter serial."
}

Write-Host "Arduino debug firmware upload completed." -ForegroundColor Green
Write-Host "The ELF used by GDB is: $ElfPath"
