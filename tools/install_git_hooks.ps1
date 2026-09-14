<#
.SYNOPSIS
Enable repository-managed Git hooks for this checkout.

.DESCRIPTION
Only updates the local repository core.hooksPath setting.
#>
[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

Push-Location $repoRoot
try {
    git config --local core.hooksPath .githooks
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to set the local core.hooksPath."
    }
    Write-Host "Enabled .githooks/pre-commit." -ForegroundColor Green
    Write-Host "Commits now check P4 architecture and generated contracts."
} finally {
    Pop-Location
}
