[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern("^COM[0-9]+$")]
    [string]$Port
)

. (Join-Path $PSScriptRoot "p4_debug_env.ps1")
Assert-P4DebugEnvironment
Assert-PathExists -Path $P4ElfPath -Description "Debug ELF"
Assert-PathExists -Path $P4BinPath -Description "Debug application binary"

Write-Host "Uploading the matching Arduino debug firmware to $Port..." -ForegroundColor Cyan
Write-Host "EraseFlash remains disabled; calibration and multi-turn recovery are restored from desktop JSON."

& $ArduinoCli upload `
    --port $Port `
    --fqbn $P4Fqbn `
    --input-dir $P4BuildPath `
    $P4SketchPath

if ($LASTEXITCODE -ne 0) {
    throw "Arduino debug upload failed with exit code $LASTEXITCODE"
}

Write-Host "Arduino debug firmware upload completed." -ForegroundColor Green
Write-Host "The ELF used by GDB is: $P4ElfPath"
