param(
    [ValidateSet("p4", "s3")]
    [string]$Target = "p4"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# Shared environment for the MMHand ESP32 JTAG debug tooling.
# Dot-source with:  . (Join-Path $PSScriptRoot "debug_env.ps1") [-Target p4|s3]
#
# The whole debug stack ships with the esp32 Arduino core — no ESP-IDF needed:
#   arduino-cli : bundled with the Arduino IDE 2.x installation
#   OpenOCD     : <Arduino15>/packages/esp32/tools/openocd-esp32/<ver>
#   GDB         : <Arduino15>/packages/esp32/tools/{riscv32,xtensa}-esp-elf-gdb/<ver>

$script:RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

function Select-FirstExisting {
    param([string[]]$Paths)
    foreach ($path in $Paths) {
        if (Test-Path -LiteralPath $path) { return $path }
    }
    return $null
}

function Select-LatestTool {
    # Newest version directory wins; directory names sort chronologically
    # for the Espressif version scheme (e.g. v0.12.0-esp32-20251215).
    param(
        [Parameter(Mandatory = $true)][string]$Root,
        [Parameter(Mandatory = $true)][string]$Filter,
        [Parameter(Mandatory = $true)][string]$Leaf
    )
    if (-not (Test-Path -LiteralPath $Root)) { return $null }
    $candidate = Get-ChildItem -LiteralPath $Root -Directory -Filter $Filter |
        Sort-Object Name -Descending |
        Select-Object -First 1
    if ($null -eq $candidate) { return $null }
    $exe = Join-Path $candidate.FullName $Leaf
    if (Test-Path -LiteralPath $exe) { return $exe }
    return $null
}

$script:ArduinoCli = Select-FirstExisting @(
    (Join-Path $env:LOCALAPPDATA "Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"),
    (Join-Path ${env:ProgramFiles} "Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe")
)

$script:Esp32Tools = Join-Path $env:LOCALAPPDATA "Arduino15\packages\esp32\tools"
$script:OpenOcdExe = Select-LatestTool -Root (Join-Path $script:Esp32Tools "openocd-esp32") `
    -Filter "v*" -Leaf "bin\openocd.exe"
$script:OpenOcdScripts = $null
if ($script:OpenOcdExe) {
    $openOcdRoot = Split-Path (Split-Path $script:OpenOcdExe)
    $script:OpenOcdScripts = Join-Path $openOcdRoot "share\openocd\scripts"
}

if ($Target -eq "p4") {
    # Match the deployed Arduino release layout; EraseFlash=none keeps the
    # calibration and multi-turn recovery records intact across uploads.
    $script:Fqbn = (
        "esp32:esp32:esp32p4:" +
        "UploadSpeed=921600," +
        "USBMode=hwcdc," +
        "CDCOnBoot=cdc," +
        "MSCOnBoot=default," +
        "DFUOnBoot=default," +
        "UploadMode=default," +
        "FlashFreq=80," +
        "FlashMode=qio," +
        "FlashSize=4M," +
        "PartitionScheme=default," +
        "DebugLevel=none," +
        "PSRAM=disabled," +
        "EraseFlash=none," +
        "JTAGAdapter=builtin," +
        "ChipVariant=prev3"
    )
    $script:SketchName = "ESP32-P4"
    $script:ElfName = "ESP32-P4.ino.elf"
    $script:OpenOcdBoardCfg = "board/esp32p4-builtin.cfg"
    $script:GdbExe = Select-LatestTool -Root (Join-Path $script:Esp32Tools "riscv32-esp-elf-gdb") `
        -Filter "*" -Leaf "bin\riscv32-esp-elf-gdb.exe"
}
else {
    $script:Fqbn = "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc"
    $script:SketchName = "ESP32-S3"
    $script:ElfName = "ESP32-S3.ino.elf"
    $script:OpenOcdBoardCfg = "board/esp32s3-builtin.cfg"
    $script:GdbExe = Select-LatestTool -Root (Join-Path $script:Esp32Tools "xtensa-esp-elf-gdb") `
        -Filter "*" -Leaf "bin\xtensa-esp32s3-elf-gdb.exe"
}

$script:SketchPath = Join-Path $script:RepoRoot $script:SketchName
# Debug builds live outside git (see .gitignore); the ELF/BIN pair in here is
# the single source of truth shared by upload and GDB.
$script:BuildPath = Join-Path (Join-Path $script:RepoRoot ".debug") $Target
$script:ElfPath = Join-Path $script:BuildPath $script:ElfName
$script:BinPath = Join-Path $script:BuildPath ($script:ElfName -replace "\.elf$", ".bin")
$script:GdbMemoryMapDisabled = $true

function Assert-PathExists {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Description
    )
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Description does not exist: $Path"
    }
}

function Assert-DebugEnvironment {
    if (-not $script:ArduinoCli) { throw "Arduino CLI not found (Arduino IDE 2.x expected under %LOCALAPPDATA%\Programs)." }
    if (-not $script:OpenOcdExe) { throw "openocd-esp32 not found under $script:Esp32Tools (install the esp32 Arduino core first)." }
    if (-not $script:OpenOcdScripts) { throw "OpenOCD scripts directory not found next to $($script:OpenOcdExe)." }
    if (-not $script:GdbExe) { throw "Target GDB not found under $script:Esp32Tools for target '$Target'." }
    Assert-PathExists -Path $script:SketchPath -Description "$script:SketchName sketch"
}
