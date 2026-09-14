[CmdletBinding()]
param(
    [switch]$ReuseCache
)

. (Join-Path $PSScriptRoot "p4_debug_env.ps1")
Assert-P4DebugEnvironment

New-Item -ItemType Directory -Path $P4BuildPath -Force | Out-Null

$compileArgs = @(
    "compile",
    "--fqbn", $P4Fqbn,
    "--build-path", $P4BuildPath,
    "--optimize-for-debug",
    "--warnings", "default",
    "--jobs", "0"
)
if (-not $ReuseCache) {
    $compileArgs += "--clean"
}
$compileArgs += $P4SketchPath

Write-Host "Building Arduino debug firmware with -Og -g3..." -ForegroundColor Cyan
Write-Host "Sketch: $P4SketchPath"
Write-Host "Build : $P4BuildPath"

& $ArduinoCli @compileArgs
if ($LASTEXITCODE -ne 0) {
    throw "Arduino debug build failed with exit code $LASTEXITCODE"
}

Assert-PathExists -Path $P4ElfPath -Description "Debug ELF"
Assert-PathExists -Path $P4BinPath -Description "Debug application binary"

$elfHash = (Get-FileHash -LiteralPath $P4ElfPath -Algorithm SHA256).Hash
$hashFile = Join-Path $P4BuildPath "ServoBoardMain.ino.elf.sha256.txt"
"$elfHash  ServoBoardMain.ino.elf" | Set-Content -LiteralPath $hashFile -Encoding ascii

Write-Host "Arduino debug build completed." -ForegroundColor Green
Write-Host "ELF   : $P4ElfPath"
Write-Host "SHA256: $elfHash"
Write-Host "Next  : run upload_debug.ps1 -Port COMx"
