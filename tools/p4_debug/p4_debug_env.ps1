Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# Build through the ASCII-only junction so GDB can resolve source paths.
$script:P4DebugWorkspace = "C:\Users\g8765\mmhand-debug"
$script:P4SketchPath = Join-Path $script:P4DebugWorkspace "ESP32-P4"
$script:P4BuildPath = Join-Path $script:P4DebugWorkspace ".debug\p4"

# Arduino CLI bundled with the local Arduino IDE 2.3.8 installation.
$script:ArduinoCli = "C:\Users\g8765\arduino-ide\resources\app\lib\backend\resources\arduino-cli.exe"

# ESP-IDF 5.5.2 supplies OpenOCD and RISC-V GDB only.
$script:OpenOcdExe = "D:\esp32\Espressif\tools\openocd-esp32\v0.12.0-esp32-20250707\openocd-esp32\bin\openocd.exe"
$script:OpenOcdScripts = "D:\esp32\Espressif\tools\openocd-esp32\v0.12.0-esp32-20250707\openocd-esp32\share\openocd\scripts"
$script:RiscvGdbExe = "D:\esp32\Espressif\tools\riscv32-esp-elf-gdb\16.3_20250913\riscv32-esp-elf-gdb\bin\riscv32-esp-elf-gdb.exe"

# Match the working Arduino Release layout and preserve the existing partition layout.
$script:P4Fqbn = (
    "esp32:esp32:esp32p4:" +
    "UploadSpeed=921600," +
    "USBMode=default," +
    "CDCOnBoot=default," +
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

$script:P4ElfPath = Join-Path $script:P4BuildPath "ESP32-P4.ino.elf"
$script:P4BinPath = Join-Path $script:P4BuildPath "ESP32-P4.ino.bin"

function Assert-PathExists {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Description does not exist: $Path"
    }
}

function Assert-P4DebugEnvironment {
    Assert-PathExists -Path $script:P4DebugWorkspace -Description "English workspace junction"
    Assert-PathExists -Path $script:P4SketchPath -Description "P4 Arduino sketch"
    Assert-PathExists -Path $script:ArduinoCli -Description "Arduino CLI"
    Assert-PathExists -Path $script:OpenOcdExe -Description "ESP-IDF OpenOCD"
    Assert-PathExists -Path $script:OpenOcdScripts -Description "OpenOCD scripts"
    Assert-PathExists -Path $script:RiscvGdbExe -Description "ESP-IDF RISC-V GDB"
}
