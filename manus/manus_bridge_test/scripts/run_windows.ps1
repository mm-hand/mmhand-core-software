param(
    [string]$Mode = "debug",
    [string]$Output = "manus_log.csv",
    [string]$Connection = "local",
    [double]$PrintHz = 10.0,
    [string]$Config = "config\manus_bridge_config.json"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ExeCandidates = @(
    (Join-Path $ProjectRoot "build_codex\manus_bridge_test.exe"),
    (Join-Path $ProjectRoot "build\Release\manus_bridge_test.exe"),
    (Join-Path $ProjectRoot "build\manus_bridge_test.exe")
)
$Exe = $ExeCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $Exe) {
    Write-Host "Executable not found."
    Write-Host "Build first with: .\scripts\build_windows.ps1"
    exit 1
}

if ($Mode -eq "record") {
    & $Exe --mode record --output $Output --connection $Connection --print-hz $PrintHz --config $Config
}
else {
    & $Exe --mode debug --connection $Connection --print-hz $PrintHz --config $Config
}
exit $LASTEXITCODE
