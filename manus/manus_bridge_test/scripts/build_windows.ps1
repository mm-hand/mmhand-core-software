param(
    [string]$ManusSdkRoot = "",
    [string]$Configuration = "Release",
    [string]$BuildDir = "build_codex"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$CMake = "cmake"

if (-not (Get-Command $CMake -ErrorAction SilentlyContinue)) {
    $VsCMake = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    if (Test-Path $VsCMake) {
        $CMake = $VsCMake
        Write-Host "Using Visual Studio CMake: $CMake"
    }
    else {
        Write-Host "CMake was not found in PATH."
        Write-Host "Install CMake or add Visual Studio CMake to PATH."
        exit 1
    }
}

if ([string]::IsNullOrWhiteSpace($ManusSdkRoot)) {
    $DefaultSdk = Join-Path $ProjectRoot "third_party\ManusSDK"
    $RepoSdk = Resolve-Path (Join-Path $ProjectRoot "..\SDKClient_Windows\ManusSDK") -ErrorAction SilentlyContinue

    if (Test-Path (Join-Path $DefaultSdk "include\ManusSDK.h")) {
        $ManusSdkRoot = $DefaultSdk
    }
    elseif ($RepoSdk) {
        $ManusSdkRoot = $RepoSdk.Path
        Write-Host "Using SDK from existing sample: $ManusSdkRoot"
    }
    else {
        Write-Host "MANUS SDK was not found."
        Write-Host "Copy it to third_party\ManusSDK or pass -ManusSdkRoot C:\path\to\ManusSDK."
        exit 1
    }
}

$VcVars64 = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
$BuildPath = Join-Path $ProjectRoot $BuildDir

if (Test-Path $VcVars64) {
    $Cmd = "call `"$VcVars64`" && `"$CMake`" -S `"$ProjectRoot`" -B `"$BuildPath`" -G `"NMake Makefiles`" -DCMAKE_BUILD_TYPE=$Configuration -DMANUS_SDK_ROOT=`"$ManusSdkRoot`" && `"$CMake`" --build `"$BuildPath`" --config $Configuration"
    cmd.exe /d /s /c $Cmd
}
else {
    & $CMake -S $ProjectRoot -B $BuildPath -A x64 -DMANUS_SDK_ROOT="$ManusSdkRoot"
    & $CMake --build $BuildPath --config $Configuration
}
