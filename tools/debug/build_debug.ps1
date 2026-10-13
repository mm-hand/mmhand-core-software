[CmdletBinding()]
param(
    [ValidateSet("p4", "s3")]
    [string]$Target = "p4",
    [switch]$ReuseCache
)

. (Join-Path $PSScriptRoot "debug_env.ps1") -Target $Target
Assert-DebugEnvironment

New-Item -ItemType Directory -Path $BuildPath -Force | Out-Null

$compileArgs = @(
    "compile",
    "--fqbn", $Fqbn,
    "--build-path", $BuildPath,
    "--optimize-for-debug",
    "--warnings", "default",
    "--jobs", "0"
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
