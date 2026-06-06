param(
    [string]$ManusSdkRoot = "",
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$ProjectDir = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$BuildDir = Join-Path $ProjectDir "build\windows"

function Resolve-CMake {
    $fromPath = Get-Command cmake -ErrorAction SilentlyContinue
    if ($fromPath) {
        return $fromPath.Source
    }

    $vsCMakeCandidates = @(
        "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
        "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
        "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    )
    foreach ($candidate in $vsCMakeCandidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    throw "cmake not found. Install CMake or Visual Studio CMake tools, or add cmake.exe to PATH."
}

if (-not $ManusSdkRoot) {
    $ManusSdkRoot = Join-Path (Split-Path -Parent $ProjectDir) "third_party\ManusSDK"
}

if (-not (Test-Path (Join-Path $ManusSdkRoot "include\ManusSDK.h"))) {
    throw "MANUS SDK include not found. Expected: $ManusSdkRoot\include\ManusSDK.h"
}
if (-not (Test-Path (Join-Path $ManusSdkRoot "lib\ManusSDK.lib"))) {
    throw "MANUS SDK import library not found. Expected: $ManusSdkRoot\lib\ManusSDK.lib"
}
if (-not (Test-Path (Join-Path $ManusSdkRoot "lib\ManusSDK.dll"))) {
    throw "MANUS SDK DLL not found. Expected: $ManusSdkRoot\lib\ManusSDK.dll"
}

$CMakeExe = Resolve-CMake
& $CMakeExe -S $ProjectDir -B $BuildDir -A x64 -DMANUS_SDK_ROOT="$ManusSdkRoot"
& $CMakeExe --build $BuildDir --config $Configuration

Write-Host "Built mmhand_manus_sdk. Output:"
Write-Host (Join-Path $BuildDir "$Configuration\mmhand_manus_sdk.dll")
