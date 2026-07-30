<#
.SYNOPSIS
Checks P4 architecture boundaries, source headers, and generated files.

.DESCRIPTION
This script performs fast static checks only. A complete verification still
requires tools/p4_debug/build_debug.ps1.
#>
[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceRoot = Join-Path $repoRoot "ESP32-P4\ServoBoardMain\src"
$failures = New-Object "System.Collections.Generic.List[string]"

function Add-Failure([string]$message) {
    $script:failures.Add($message)
}

# Every P4 source file must declare its role at the top.
$sourceFiles = Get-ChildItem -LiteralPath $sourceRoot -Recurse -File |
    Where-Object { $_.Extension -in @(".c", ".cpp", ".h") }
foreach ($file in $sourceFiles) {
    $head = (Get-Content -LiteralPath $file.FullName -Encoding UTF8 -TotalCount 20) -join "`n"
    if ($head -notmatch "@file" -or $head -notmatch "@brief") {
        Add-Failure "Missing @file/@brief header: $($file.FullName)"
    }
}

# Feetech SDK access is restricted to the sole driver wrapper.
$sdkArgs = @(
    "-n",
    "SMS_STS|SyncWritePosEx|WritePosEx|EnableTorque|FeedBack\(|ReadPos\(",
    $sourceRoot,
    "-g", "*.cpp",
    "-g", "*.c",
    "-g", "*.h"
)
$sdkMatches = & rg @sdkArgs
if ($LASTEXITCODE -notin @(0, 1)) {
    Add-Failure "Failed to scan Feetech SDK access."
} elseif ($sdkMatches) {
    foreach ($line in $sdkMatches) {
        if ($line -notmatch "hardware[\\/]servo[\\/]FeetechServoDriver\.(h|cpp)") {
            Add-Failure "Feetech SDK access outside driver: $line"
        }
    }
}

# Only the gateway may read or write the servo target queue.
$queueArgs = @(
    "-n",
    "xQueue(Overwrite|Send|Receive|Reset|Peek)\([^\n]*servoTargetQueue",
    $sourceRoot,
    "-g", "*.cpp",
    "-g", "*.c",
    "-g", "*.h"
)
$queueMatches = & rg @queueArgs
if ($LASTEXITCODE -notin @(0, 1)) {
    Add-Failure "Failed to scan servoTargetQueue access."
} elseif ($queueMatches) {
    foreach ($line in $queueMatches) {
        if ($line -notmatch "actuator[\\/]ActuatorCommandGateway\.cpp") {
            Add-Failure "servoTargetQueue access outside gateway: $line"
        }
    }
}

# The generated URDF table must match the desktop mapping.
& python (Join-Path $repoRoot "tools\generate_p4_urdf_limits.py") --check
if ($LASTEXITCODE -ne 0) {
    Add-Failure "UrdfJointLimits.generated.h is stale."
}

if ($failures.Count -gt 0) {
    Write-Host "P4 architecture check FAILED:" -ForegroundColor Red
    $failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}

Write-Host "P4 architecture check passed." -ForegroundColor Green
Write-Host "Checked $($sourceFiles.Count) C/C++ source files."
