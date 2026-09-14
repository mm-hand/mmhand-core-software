[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Destination,

    [double]$MaxSizeGB = 3.0,

    [string]$RepositoryUrl = "https://github.com/mm-hand/structure.git"
)

$ErrorActionPreference = "Stop"

if (Test-Path -LiteralPath $Destination) {
    $destinationItem = Get-Item -LiteralPath $Destination -Force
    if (-not $destinationItem.PSIsContainer) {
        throw "Destination exists but is not a directory: $Destination"
    }
    if (@(Get-ChildItem -LiteralPath $Destination -Force).Count -ne 0) {
        throw "Destination must be empty: $Destination"
    }
} else {
    $parent = Split-Path -Parent $Destination
    if (-not (Test-Path -LiteralPath $parent -PathType Container)) {
        throw "Destination parent does not exist: $parent"
    }
}

Write-Output "Cloning metadata only (no checkout yet): $RepositoryUrl"
& git clone --depth 1 --filter=blob:none --no-checkout $RepositoryUrl $Destination
if ($LASTEXITCODE -ne 0) {
    throw "Metadata clone failed."
}

$limitBytes = [int64]($MaxSizeGB * 1GB)
$knownBytes = [int64]0
$unknownEntries = 0
$treeLines = & git -C $Destination ls-tree -r -l HEAD
if ($LASTEXITCODE -ne 0) {
    throw "Cannot read repository tree after clone."
}

foreach ($line in $treeLines) {
    if ($line -match '^\d+\s+\S+\s+\S+\s+(\d+|-)\t') {
        if ($Matches[1] -eq "-") {
            $unknownEntries++
        } else {
            $knownBytes += [int64]$Matches[1]
        }
    }
}

$gitattributes = & git -C $Destination show HEAD:.gitattributes 2>$null
$usesLfs = ($LASTEXITCODE -eq 0) -and (($gitattributes -join "`n") -match 'filter=lfs')
$knownSizeGB = [math]::Round($knownBytes / 1GB, 3)
Write-Output "Known Git blob size: $knownSizeGB GB (limit: $MaxSizeGB GB)"

if ($knownBytes -gt $limitBytes -or $unknownEntries -gt 0 -or $usesLfs) {
    Write-Output "Stopped before checkout: the complete working tree cannot be proven to stay within the size limit."
    if ($usesLfs) {
        Write-Output "Reason: Git LFS is used, and LFS object sizes are not included in the Git tree size."
    }
    if ($unknownEntries -gt 0) {
        Write-Output "Reason: $unknownEntries tree entries have unknown blob sizes."
    }
    if ($knownBytes -gt $limitBytes) {
        Write-Output "Reason: known Git blobs already exceed the limit."
    }
    exit 2
}

Write-Output "Size check passed. Checking out the working tree."
& git -C $Destination checkout --force
if ($LASTEXITCODE -ne 0) {
    throw "Checkout failed."
}

$workingBytes = (Get-ChildItem -LiteralPath $Destination -Recurse -File -Force |
    Where-Object { $_.FullName -notmatch '\\.git\\' } |
    Measure-Object -Property Length -Sum).Sum
$workingSizeGB = [math]::Round(([int64]$workingBytes) / 1GB, 3)
Write-Output "Checked out successfully: $Destination ($workingSizeGB GB, excluding .git)"
