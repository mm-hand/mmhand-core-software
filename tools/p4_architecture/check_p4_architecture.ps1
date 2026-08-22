<#
.SYNOPSIS
检查 P4 当前嵌入式分层、硬件唯一入口、薄任务、安全规则和产品契约单一来源。

.DESCRIPTION
本脚本是提交前快速门禁，只执行文本与生成文件检查，不运行 Arduino/ESP-IDF 编译，不访问硬件。
依赖方向：config/data/generated -> platform/hal -> modules/safety/runtime -> tasks -> app。
#>
[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceRoot = Join-Path $repoRoot "ESP32-P4\ServoBoardMain\src"
$driverHeaderPath = Join-Path $sourceRoot "hal\servo\FeetechServoDriver.h"
$servoServicePath = Join-Path $sourceRoot "tasks\servo_bus\ServoBusService.cpp"
$systemContextPath = Join-Path $sourceRoot "runtime\SystemRuntimeContext.h"
$productFingerprintJsonPath = Join-Path $repoRoot "desktop\resources\config\product_fingerprint.json"
$productFingerprintHeaderPath = Join-Path $sourceRoot "generated\ProductFingerprint.generated.h"
$failures = New-Object "System.Collections.Generic.List[string]"

function Add-Failure([string]$message) {
    $script:failures.Add($message)
}

function Get-RelativeSourcePath([System.IO.FileInfo]$file) {
    return $file.FullName.Substring($sourceRoot.Length + 1).Replace("\", "/")
}

function Find-ForbiddenInclude($files, [string]$pattern, [string]$label) {
    $matches = $files | Select-String -Encoding UTF8 -Pattern $pattern
    foreach ($match in $matches) {
        Add-Failure ("{0}: {1}:{2}" -f $label, $match.Path, $match.LineNumber)
    }
}

$sourceFiles = Get-ChildItem -LiteralPath $sourceRoot -Recurse -File |
    Where-Object { $_.Extension -in @(".c", ".cpp", ".h") }

# R0：顶层目录是架构契约；历史功能型目录不得回流。
$allowedTopLevel = @(
    "app", "config", "data", "generated", "hal", "modules",
    "platform", "runtime", "safety", "tasks"
)
$actualTopLevel = Get-ChildItem -LiteralPath $sourceRoot -Directory |
    Where-Object {
        (Get-ChildItem -LiteralPath $_.FullName -Recurse -File `
            -ErrorAction SilentlyContinue).Count -gt 0
    } | Select-Object -ExpandProperty Name
foreach ($name in $actualTopLevel) {
    if ($name -notin $allowedTopLevel) {
        Add-Failure "Legacy or unregistered top-level source directory: $name"
    }
}
foreach ($legacy in @("core", "domain", "hardware", "shared", "actuator",
                       "calibration", "communication", "control", "persistence",
                       "recovery")) {
    if (Test-Path -LiteralPath (Join-Path $sourceRoot $legacy)) {
        Add-Failure "Legacy source directory still exists: $legacy"
    }
}

# R1：每个源码文件必须在头部说明职责；中文内容由人工 review 保证。
foreach ($file in $sourceFiles) {
    $head = (Get-Content -LiteralPath $file.FullName -Encoding UTF8 `
        -TotalCount 24) -join "`n"
    if ($head -notmatch "@file" -or $head -notmatch "@brief") {
        Add-Failure "Missing @file/@brief header: $(Get-RelativeSourcePath $file)"
    }
}

# R2：data 是纯值类型，不能泄漏 Arduino、RTOS 或上层业务依赖。
$dataRoot = Join-Path $sourceRoot "data"
$dataFiles = $sourceFiles | Where-Object { $_.FullName -like "$dataRoot*" }
Find-ForbiddenInclude $dataFiles `
    '^\s*#include.*(Arduino|FreeRTOS|freertos/|\.\./(hal|modules|platform|safety|system|tasks)/)' `
    "data layer depends upward"

# R3：HAL 只封装物理外设，不得依赖 modules/safety/tasks/system 业务状态。
$halRoot = Join-Path $sourceRoot "hal"
$halFiles = $sourceFiles | Where-Object { $_.FullName -like "$halRoot*" }
Find-ForbiddenInclude $halFiles `
    '^\s*#include.*\.\./\.\./(modules|safety|tasks|system)/' `
    "HAL depends on business layer"

# R4：platform 只提供 RTOS/时钟适配，不得反向依赖硬件或业务模块。
$platformRoot = Join-Path $sourceRoot "platform"
$platformFiles = $sourceFiles | Where-Object { $_.FullName -like "$platformRoot*" }
$platformCoreFiles = $platformFiles | Where-Object {
    $_.FullName -notlike "$(Join-Path $platformRoot 'log')*"
}
Find-ForbiddenInclude $platformCoreFiles `
    '^\s*#include.*\.\./\.\./(hal|modules|safety|tasks|system)/' `
    "platform depends on hardware/business layer"
# 日志与二进制协议共用同一物理串口，platform/log 仅允许依赖 serial HAL。
$platformLogFiles = $platformFiles | Where-Object {
    $_.FullName -like "$(Join-Path $platformRoot 'log')*"
}
Find-ForbiddenInclude $platformLogFiles `
    '^\s*#include.*\.\./\.\./hal/(?!serial/)' `
    "platform/log depends on non-serial HAL"

# FreeRTOS 原生头只有适配头可以直接包含，防止 task.h/FreeRTOS.h 顺序再次漂移。
$freeRtosAdapterPath = Join-Path $sourceRoot "platform\rtos\FreeRtosKernel.h"
$directFreeRtosIncludes = $sourceFiles |
    Where-Object { $_.FullName -ne $freeRtosAdapterPath } |
    Select-String -Encoding UTF8 -Pattern '^\s*#include\s*[<"]freertos/'
foreach ($match in $directFreeRtosIncludes) {
    Add-Failure ("FreeRTOS header bypasses platform adapter: {0}:{1}" -f `
                 $match.Path, $match.LineNumber)
}

$legacyPlatformNamespace = $sourceFiles |
    Select-String -Encoding UTF8 -Pattern '\bp4::hal::rtos\b|\bP4_HAL_RTOS_'
foreach ($match in $legacyPlatformNamespace) {
    Add-Failure ("Legacy HAL/RTOS namespace remains: {0}:{1}" -f `
                 $match.Path, $match.LineNumber)
}

# R5：modules 与 safety 不得包含 task 实现；跨层事件使用 runtime 稳定端口。
$modulesRoot = Join-Path $sourceRoot "modules"
$safetyRoot = Join-Path $sourceRoot "safety"
$moduleSafetyFiles = $sourceFiles | Where-Object {
    $_.FullName -like "$modulesRoot*" -or $_.FullName -like "$safetyRoot*"
}
Find-ForbiddenInclude $moduleSafetyFiles `
    '^\s*#include.*tasks/' `
    "modules/safety depends on tasks"
Find-ForbiddenInclude $moduleSafetyFiles `
    '\b(p4::platform::rtos::createTask|xTaskCreate|vTaskDelete)\b' `
    "modules/safety owns RTOS task lifecycle"
Find-ForbiddenInclude $moduleSafetyFiles `
    '\b(millis|vTaskDelay|vTaskDelayUntil|xTaskGetTickCount)\s*\(' `
    "modules/safety bypasses task-provided scheduling time"

# R6：所有 FreeRTOS Task 仅做薄入口，禁止重新塞入业务 switch/循环。
$taskRoot = Join-Path $sourceRoot "tasks"
$taskCppFiles = Get-ChildItem -LiteralPath $taskRoot -Filter "*Task.cpp" -File -Recurse
foreach ($file in $taskCppFiles) {
    $lineCount = (Get-Content -LiteralPath $file.FullName -Encoding UTF8).Count
    if ($lineCount -gt 60) {
        Add-Failure "Task adapter contains business logic ($lineCount lines): $($file.Name)"
    }
}

# 每个运行任务必须有同目录薄入口和显式编排/服务文件。
$taskPackages = @{
    "host_comm" = @("HostCommTask.cpp", "HostCommService.cpp")
    "system_state" = @("SystemStateTask.cpp", "SystemStateCoordinator.cpp")
    "s3_can_rx" = @("S3CanRxTask.cpp", "S3CanRxService.cpp")
    "servo_bus" = @("ServoBusTask.cpp", "ServoBusService.cpp")
    "motion_control" = @("MotionControlTask.cpp", "MotionControlService.cpp")
    "runtime_monitor" = @("RuntimeMonitorTask.cpp", "RuntimeMonitorService.cpp")
    "auto_calibration" = @("AutoCalibrationTask.cpp", "AutoCalibrationCoordinator.cpp")
}
foreach ($entry in $taskPackages.GetEnumerator()) {
    foreach ($fileName in $entry.Value) {
        $path = Join-Path (Join-Path $taskRoot $entry.Key) $fileName
        if (-not (Test-Path -LiteralPath $path)) {
            Add-Failure "Task package contract is missing: $($entry.Key)/$fileName"
        }
    }
}

# R6.2：服务必须使用 init/step 与纯值快照；历史永久循环入口不得回流。
$serviceContractFiles = @(
    "data\ServiceSnapshots.h",
    "runtime\ports\ServiceOutputPort.h",
    "runtime\ports\SystemEventPort.h",
    "runtime\adapters\ServiceContextAdapter.h",
    "runtime\adapters\ServiceContextAdapter.cpp",
    "modules\runtime\ServiceCyclePolicy.h",
    "modules\runtime\ServiceCyclePolicy.cpp"
)
foreach ($relativePath in $serviceContractFiles) {
    if (-not (Test-Path -LiteralPath (Join-Path $sourceRoot $relativePath))) {
        Add-Failure "Service snapshot/port contract is missing: $relativePath"
    }
}
$serviceBoundaryFiles = @(
    Join-Path $sourceRoot "data\ServiceSnapshots.h"
    Join-Path $sourceRoot "runtime\ports\ServiceOutputPort.h"
    Join-Path $sourceRoot "runtime\ports\SystemEventPort.h"
)
Find-ForbiddenInclude $serviceBoundaryFiles `
    '\b(TaskSharedData_t|QueueHandle_t|SemaphoreHandle_t|TaskHandle_t)\b' `
    "service boundary leaks shared context or RTOS resources"

$legacySharedContext = $sourceFiles | Select-String -Encoding UTF8 -Pattern `
    '\bTaskSharedData_t\b|\bSystemContext\.h\b'
foreach ($match in $legacySharedContext) {
    Add-Failure ("Legacy shared-context name remains: {0}:{1}" -f `
                 $match.Path, $match.LineNumber)
}
$removedServiceLoops = $sourceFiles | Select-String -Encoding UTF8 -Pattern `
    '\b(runControlService|runServoRuntimeService|runSystemStateCoordinator|runUpperCommService|runSensorIngressService|runRuntimeMonitorService|runPersistenceService)\b'
foreach ($match in $removedServiceLoops) {
    Add-Failure ("Blocking service loop returned: {0}:{1}" -f `
                 $match.Path, $match.LineNumber)
}

# R6.1：三个历史超大文件必须保持为编排层，禁止状态机实现重新回流。
$coordinatorBudgets = @{
    "tasks\auto_calibration\AutoCalibrationCoordinator.cpp" = 260
    "tasks\host_comm\HostCommService.cpp" = 560
    "tasks\system_state\SystemStateCoordinator.cpp" = 180
}
foreach ($entry in $coordinatorBudgets.GetEnumerator()) {
    $path = Join-Path $sourceRoot $entry.Key
    if (-not (Test-Path -LiteralPath $path)) {
        Add-Failure "Required coordinator is missing: $($entry.Key)"
        continue
    }
    $lineCount = (Get-Content -LiteralPath $path -Encoding UTF8).Count
    if ($lineCount -gt $entry.Value) {
        Add-Failure "Coordinator exceeds architecture budget ($lineCount > $($entry.Value)): $($entry.Key)"
    }
}

$requiredSplitModules = @(
    "tasks\auto_calibration\CalibrationMotionSupport.cpp",
    "tasks\auto_calibration\single_motor\SingleMotorCalibrationMachine.cpp",
    "tasks\auto_calibration\dual_motor\DualMotorCalibrationMachine.cpp",
    "tasks\host_comm\HostFrameReader.cpp",
    "tasks\host_comm\HostCommandDispatcher.cpp",
    "tasks\host_comm\HostTelemetryPublisher.cpp",
    "tasks\host_comm\HostAckPublisher.cpp",
    "tasks\s3_can_rx\EncoderHealthMonitor.cpp",
    "tasks\servo_bus\ServoBusRouting.cpp",
    "safety\policies\SystemReadinessPolicy.cpp",
    "safety\policies\SystemFaultPolicy.cpp",
    "tasks\system_state\HostLeaseSupervisor.cpp",
    "tasks\system_state\SystemEventDispatcher.cpp",
    "tasks\system_state\RopeInitController.cpp"
)
foreach ($relativePath in $requiredSplitModules) {
    if (-not (Test-Path -LiteralPath (Join-Path $sourceRoot $relativePath))) {
        Add-Failure "Required split module is missing: $relativePath"
    }
}

# Rope Init 已并入状态协调循环，不允许重新创建独立 Task/Runtime/Service。
$removedArchitectureSymbols = $sourceFiles | Select-String -Encoding UTF8 -Pattern `
    '\b(RopeInitTask|RopeInitService|RopeInitTaskRuntime|runRopeInitTask)\b'
foreach ($match in $removedArchitectureSymbols) {
    Add-Failure ("Removed architecture symbol returned: {0}:{1}" -f `
                 $match.Path, $match.LineNumber)
}

# R7：共享上下文只能组合聚合状态，不得重新堆回散落业务字段。
$systemContextText = Get-Content -LiteralPath $systemContextPath -Encoding UTF8 -Raw
foreach ($base in @("InterTaskChannels", "CommandRuntimeState",
                     "ControlRuntimeState", "ServoRuntimeState",
                     "CalibrationRuntimeState")) {
    if ($systemContextText -notmatch [regex]::Escape($base)) {
        Add-Failure "SystemContext lost required state aggregate: $base"
    }
}
$contextBody = [regex]::Match(
    $systemContextText,
    'struct\s+SystemRuntimeContext\s*:[^{]+\{(?<body>[\s\S]*?)\};'
)
if (-not $contextBody.Success) {
    Add-Failure "SystemRuntimeContext composition declaration is missing."
} elseif ($contextBody.Groups['body'].Value -match ';') {
    Add-Failure "SystemRuntimeContext contains business fields instead of aggregates."
}

# R8：飞特、Host 串口和 TWAI 只能通过各自 HAL 唯一入口。
$sdkViolations = & rg -n `
    "SMS_STS|SyncWritePosEx|WritePosEx|EnableTorque|\bFeedBack\s*\(|\bReadPos\s*\(" `
    $sourceRoot -g "*.h" -g "*.cpp" -g "*.c" `
    -g "!**/FeetechServoDriver.h" -g "!**/FeetechServoDriver.cpp"
if ($LASTEXITCODE -gt 1) {
    Add-Failure "Failed to scan Feetech SDK access."
} elseif ($sdkViolations) {
    Add-Failure ("Feetech SDK access outside driver:`n" +
                 ($sdkViolations -join "`n"))
}

$canViolations = & rg -n "\btwai_[a-zA-Z0-9_]+\s*\(" `
    $sourceRoot -g "*.h" -g "*.cpp" -g "*.c" `
    -g "!**/TwaiCanDriver.h" -g "!**/TwaiCanDriver.cpp"
if ($LASTEXITCODE -gt 1) {
    Add-Failure "Failed to scan TWAI access."
} elseif ($canViolations) {
    Add-Failure ("TWAI access outside CAN HAL:`n" +
                 ($canViolations -join "`n"))
}

# 舵机目标邮箱只允许 Gateway 读写；最终发送必须经过结构化 SafetyDecision。
$queueViolations = & rg -n "servoTargetQueue\.(send|receive|overwrite|reset)" `
    $sourceRoot -g "*.h" -g "*.cpp" -g "*.c" `
    -g "!**/ActuatorCommandGateway.cpp"
if ($LASTEXITCODE -gt 1) {
    Add-Failure "Failed to scan servoTargetQueue access."
} elseif ($queueViolations) {
    Add-Failure ("servoTargetQueue access outside gateway:`n" +
                 ($queueViolations -join "`n"))
}
if (-not (Select-String -LiteralPath $servoServicePath `
        -SimpleMatch "p4::actuator::evaluateCommand" -Quiet)) {
    Add-Failure "Servo execution path bypasses ActuatorSafetyPolicy::evaluateCommand."
}
if (-not (Select-String -LiteralPath $driverHeaderPath `
        -SimpleMatch "logicalToHardwareTarget" -Quiet)) {
    Add-Failure "Feetech driver lost final hardware range guard."
}

# R9：协议、URDF、安全目录必须由唯一源生成且无漂移。
$legacyMappingRuntime = $sourceFiles | Select-String -Encoding UTF8 -Pattern `
    '\b(mapEncoderWindowToUrdf|mapJointCalibrationEncoderToUrdf|mapCalibratedEncoderToDeg|encoderAtUrdfMin|encoderAtUrdfMax)\b|urdf_joint_mapping_v2'
foreach ($match in $legacyMappingRuntime) {
    Add-Failure ("Legacy Auto mapping V2 runtime remains: {0}:{1}" -f `
                 $match.Path, $match.LineNumber)
}

# R0：产品指纹不仅要“已生成”，还必须逐项证明 Desktop 与 P4 使用同一组 CRC。
if (-not (Test-Path -LiteralPath $productFingerprintJsonPath) -or
    -not (Test-Path -LiteralPath $productFingerprintHeaderPath)) {
    Add-Failure "Product fingerprint contract is missing."
} else {
    $fingerprint = Get-Content -LiteralPath $productFingerprintJsonPath `
        -Encoding UTF8 -Raw | ConvertFrom-Json
    $fingerprintHeader = Get-Content -LiteralPath $productFingerprintHeaderPath `
        -Encoding UTF8 -Raw
    $fingerprintFields = @{
        "control_model_crc" = "kProductControlModelCrc"
        "visual_model_crc" = "kProductVisualModelCrc"
        "hardware_map_crc" = "kProductHardwareMapCrc"
        "parameter_crc" = "kProductParameterCrc"
        "firmware_source_crc" = "kProductFirmwareSourceCrc"
        "structure_commit_crc" = "kProductStructureCommitCrc"
    }
    foreach ($entry in $fingerprintFields.GetEnumerator()) {
        $expected = ([string]$fingerprint.($entry.Key)).ToUpperInvariant()
        $pattern = [regex]::Escape($entry.Value) + '\s*=\s*0x([0-9A-Fa-f]{8})UL'
        $match = [regex]::Match($fingerprintHeader, $pattern)
        if (-not $match.Success -or $match.Groups[1].Value.ToUpperInvariant() -ne $expected) {
            Add-Failure "Product fingerprint mismatch: $($entry.Key)"
        }
    }
}

# R1：一次安全事实必须能够经过缓存、协议和 Desktop 解析完整送达。
$safetyEventContracts = @{
    "safety\SafetyEventStore.h" = "copySafetyEventsAfter"
    "tasks\host_comm\HostTelemetryPublisher.cpp" = "sendSafetyEventPacket"
    "generated\UpperProtocol.generated.h" = "PACKET_TYPE_SAFETY_EVENT"
}
foreach ($entry in $safetyEventContracts.GetEnumerator()) {
    $path = Join-Path $sourceRoot $entry.Key
    if (-not (Test-Path -LiteralPath $path) -or
        -not (Select-String -LiteralPath $path -SimpleMatch $entry.Value -Quiet)) {
        Add-Failure "Safety event contract is incomplete: $($entry.Key) -> $($entry.Value)"
    }
}

# 安全目录不做中央裁决：Policy 返回判断，业务执行动作。
# 业务只能经 SafetyEventReporter 上报，禁止直接写 EventStore。
$safetyBypassViolations = & rg -n `
    "\b(recordSafetyEvent|recordSafetyRuleEvent|serviceRecordSafety)\s*\(" `
    $sourceRoot -g "*.h" -g "*.cpp" -g "*.c" `
    -g "!**/SafetyEventStore.h" `
    -g "!**/SafetyEventStore.cpp" `
    -g "!**/SafetyEventReporter.cpp" `
    -g "!**/ServiceContextAdapter.cpp" `
    -g "!**/ServiceOutputPort.h"
if ($LASTEXITCODE -gt 1) {
    Add-Failure "Failed to scan direct safety event writes."
} elseif ($safetyBypassViolations) {
    Add-Failure ("Safety event store bypasses SafetyEventReporter:`n" +
                 ($safetyBypassViolations -join "`n"))
}

# 已删除的中央裁决架构不得以“兼容层”名义回流。
$removedSafetyArchitecture = @(
    "safety\SafetySupervisor.h",
    "safety\SafetySupervisor.cpp",
    "safety\SafetyActionExecutor.h",
    "safety\SafetyActionExecutor.cpp",
    "safety\facades"
)
foreach ($relativePath in $removedSafetyArchitecture) {
    if (Test-Path -LiteralPath (Join-Path $sourceRoot $relativePath)) {
        Add-Failure "Removed central safety adjudicator returned: $relativePath"
    }
}
$desktopSafetyParser = Join-Path $repoRoot "desktop\core\protocol.py"
if (-not (Select-String -LiteralPath $desktopSafetyParser `
        -SimpleMatch "parse_safety_event_packet" -Quiet)) {
    Add-Failure "Desktop safety event parser is missing."
}
Push-Location $repoRoot
try {
    foreach ($check in @(
        @("tools\generate_upper_protocol.py", "Upper protocol"),
        @("tools\generate_robot_model.py", "Robot model"),
        @("tools\generate_p4_safety_catalog.py", "P4 safety catalog")
    )) {
        & python $check[0] --check
        if ($LASTEXITCODE -ne 0) {
            Add-Failure "$($check[1]) generated files are stale."
        }
    }
    & python "tools\p4_architecture\check_safety_rule_closure.py"
    if ($LASTEXITCODE -ne 0) {
        Add-Failure "P4 safety rules have not completed registration/execution/event/recovery closure."
    }
} finally {
    Pop-Location
}

if ($failures.Count -gt 0) {
    Write-Host "P4 architecture check FAILED:" -ForegroundColor Red
    $failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}

Write-Host "P4 architecture check passed." -ForegroundColor Green
Write-Host "Checked $($sourceFiles.Count) C/C++ source files; firmware was not compiled."
