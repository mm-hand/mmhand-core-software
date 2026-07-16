[CmdletBinding()]
param(
    [string]$StructureRepo = "D:\mmhand structure",

    [switch]$NoPull
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$SourceRoot = Join-Path $StructureRepo "mmhand_description"
$AdapterRoot = Join-Path $StructureRepo "desktop_urdf_adapter"
$Builder = Join-Path $ProjectRoot "tools\sync_structure_urdf_adapter.py"
$Validator = Join-Path $ProjectRoot "tools\validate_urdf.py"

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

Write-Output "Structure URDF is ready. Restart the desktop application to reload URDF and tactile mesh caches."
