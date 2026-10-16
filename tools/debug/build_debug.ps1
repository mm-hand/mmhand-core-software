[CmdletBinding()]
param(
    [ValidateSet("p4", "s3")]
    [string]$Target = "p4",
    [switch]$ReuseCache,
    [ValidateRange(1, 32)]
    [int]$Jobs = 4
)

. (Join-Path $PSScriptRoot "debug_env.ps1") -Target $Target
Assert-DebugEnvironment

if ($Target -eq "p4") {
    & python (Join-Path $PSScriptRoot "patch_p4_usb_fifo.py") --tools-root $Esp32Tools
    if ($LASTEXITCODE -ne 0) { throw "P4 USB FIFO prerequisite failed" }
    $coreInventory = (& $ArduinoCli core list --format json | ConvertFrom-Json)
    if ($LASTEXITCODE -ne 0) { throw "Cannot identify the installed ESP32 core" }
    $coreVersion = ($coreInventory.platforms | Where-Object { $_.id -eq 'esp32:esp32' }).installed_version
    if (-not $coreVersion) { throw "ESP32 Arduino core is not installed" }
    $hwcdcSource = Join-Path (Split-Path $Esp32Tools) "hardware\esp32\$coreVersion\cores\esp32\HWCDC.cpp"
    & python (Join-Path $PSScriptRoot "patch_p4_hwcdc.py") --source $hwcdcSource
    if ($LASTEXITCODE -ne 0) { throw "P4 HWCDC empty-IN prerequisite failed" }
}

New-Item -ItemType Directory -Path $BuildPath -Force | Out-Null

$compileArgs = @(
    "compile",
    "--fqbn", $Fqbn,
    "--build-path", $BuildPath,
    "--optimize-for-debug",
    "--warnings", "default",
    "--jobs", "$Jobs"
)
if (-not $ReuseCache) {
    $compileArgs += "--clean"
}
$compileArgs += $SketchPath

Write-Host "Building $SketchName Arduino debug firmware with -Og -g3..." -ForegroundColor Cyan
Write-Host "Sketch: $SketchPath"
Write-Host "Build : $BuildPath"

& $ArduinoCli @compileArgs
if ($LASTEXITCODE -ne 0) {
    throw "Arduino debug build failed with exit code $LASTEXITCODE"
}

Assert-PathExists -Path $ElfPath -Description "Debug ELF"
Assert-PathExists -Path $BinPath -Description "Debug application binary"

$elfHash = (Get-FileHash -LiteralPath $ElfPath -Algorithm SHA256).Hash
$hashFile = Join-Path $BuildPath "$ElfName.sha256.txt"
"$elfHash  $ElfName" | Set-Content -LiteralPath $hashFile -Encoding ascii

Write-Host "Arduino debug build completed." -ForegroundColor Green
Write-Host "ELF   : $ElfPath"
Write-Host "SHA256: $elfHash"
Write-Host "Next  : run upload_debug.ps1 -Target $Target -Port COMx"
