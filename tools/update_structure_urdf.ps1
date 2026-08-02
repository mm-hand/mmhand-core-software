[CmdletBinding()]
param(
    [string]$StructureRepo = "D:\mmhand structure",

    [switch]$NoPull,

    [switch]$SkipP4Build
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$SourceRoot = Join-Path $StructureRepo "mmhand_description"
$AdapterRoot = Join-Path $StructureRepo "desktop_urdf_adapter"
$Builder = Join-Path $ProjectRoot "tools\sync_structure_urdf_adapter.py"
$Validator = Join-Path $ProjectRoot "tools\validate_urdf.py"
$ModelGenerator = Join-Path $ProjectRoot "tools\generate_robot_model.py"
$ArchitectureCheck = Join-Path $ProjectRoot "tools\p4_architecture\check_p4_architecture.ps1"
$P4Build = Join-Path $ProjectRoot "tools\p4_debug\build_debug.ps1"

if (-not (Test-Path -LiteralPath (Join-Path $StructureRepo ".git") -PathType Container)) {
    throw "Structure repository is missing: $StructureRepo"
}

if (-not $NoPull) {
    Write-Output "Updating structure repository..."
    & git -C $StructureRepo pull --ff-only
    if ($LASTEXITCODE -ne 0) {
        throw "git pull failed; adapter was not regenerated."
    }
    & git -C $StructureRepo lfs pull
    if ($LASTEXITCODE -ne 0) {
        throw "git lfs pull failed; adapter was not regenerated."
    }
}

Write-Output "Regenerating desktop URDF adapter..."
& python $Builder --source-root $SourceRoot --adapter-root $AdapterRoot
if ($LASTEXITCODE -ne 0) {
    throw "URDF adapter generation failed."
}

& python $Validator --urdf-root $AdapterRoot
if ($LASTEXITCODE -ne 0) {
    throw "URDF validation failed. The desktop Junction remains pointed at the previous adapter path."
}

Write-Output "Generating Desktop/P4/Wuji model contracts and product fingerprint..."
& python $ModelGenerator
if ($LASTEXITCODE -ne 0) {
    throw "Cross-platform robot-model generation failed."
}

# 生成后立即反向检查，保证脚本没有留下只更新一半的模型产物。
& python $ModelGenerator --check
if ($LASTEXITCODE -ne 0) {
    throw "Generated robot-model files are inconsistent."
}

& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $ArchitectureCheck
if ($LASTEXITCODE -ne 0) {
    throw "P4 architecture gate failed after URDF update."
}

& python -m pytest tests/desktop/core/test_angle_mapping.py tests/tools/test_generate_robot_model.py -q
if ($LASTEXITCODE -ne 0) {
    throw "Robot-model regression tests failed."
}

if (-not $SkipP4Build) {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $P4Build -ReuseCache
    if ($LASTEXITCODE -ne 0) {
        throw "P4 firmware build failed after URDF update."
    }
}

Write-Output "Structure URDF and every generated contract are ready. Restart the desktop application to reload caches."
