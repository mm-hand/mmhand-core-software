[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$UrdfRoot,

    [switch]$Apply
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Target = Join-Path $ProjectRoot "desktop\resources\hku_hand_v2_urdf"
$Backup = Join-Path $ProjectRoot "desktop\resources\hku_hand_v2_urdf_builtin"
$Validator = Join-Path $ProjectRoot "tools\validate_urdf.py"
$Source = (Resolve-Path -LiteralPath $UrdfRoot).Path
$SourceUrdf = Join-Path $Source "urdf\hand.urdf"

if (-not (Test-Path -LiteralPath $SourceUrdf -PathType Leaf)) {
    throw "UrdfRoot must contain urdf\hand.urdf: $Source"
}

Write-Output "Validating external URDF root: $Source"
& python $Validator --urdf-root $Source
if ($LASTEXITCODE -ne 0) {
    throw "External URDF validation failed; current desktop resources were not changed."
}

if (-not $Apply) {
    Write-Output "Validation passed. Dry run only. Re-run with -Apply to create the Junction:"
    Write-Output "  $Target -> $Source"
    exit 0
}

if (Test-Path -LiteralPath $Target) {
    $item = Get-Item -LiteralPath $Target -Force
    if ($item.LinkType -in @("Junction", "SymbolicLink")) {
        cmd.exe /c "rmdir `"$Target`""
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to remove existing URDF link: $Target"
        }
    } else {
        if (Test-Path -LiteralPath $Backup) {
            throw "Builtin backup already exists: $Backup. Refusing to overwrite it."
        }
        Move-Item -LiteralPath $Target -Destination $Backup
        Write-Output "Moved builtin URDF resources to: $Backup"
    }
}

New-Item -ItemType Junction -Path $Target -Target $Source | Out-Null
Write-Output "URDF Junction created: $Target -> $Source"
Write-Output "Restart the desktop application so the URDF and tactile mesh caches reload."
