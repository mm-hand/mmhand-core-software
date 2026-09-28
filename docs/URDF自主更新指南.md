# URDF 自主更新指南

本指南用于不依赖 Codex，自行从 `mm-hand/structure` 仓库更新 MMHand 的 Desktop 与 ESP32-P4 URDF 数据。

> 本流程不会修改 `wuji_hand_ws`。Wuji 的 URDF、mesh 和契约由独立流程维护。

## 1. 更新前准备

确认以下目录存在：

```text
MMHand 工程：C:\浏览器下周\mmhand-cl\mmhand-cl
结构仓库：  D:\mmhand structure
```

建议先保存当前代码版本：

```powershell
git status
git add -A
git commit -m "chore: backup before URDF update"
```

如果存在不准备提交的临时文件，不要执行 `git add -A`，应先单独处理这些文件。

## 2. 推荐更新命令

在 PowerShell 中进入 MMHand 工程：

```powershell
cd "C:\浏览器下周\mmhand-cl\mmhand-cl"
```

执行模型更新，但暂时不自动编译 P4：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\update_structure_urdf.ps1 `
  -StructureRepo "D:\mmhand structure" `
  -SkipP4Build
```

脚本会自动完成：

1. 对 structure 执行 `git pull --ff-only`；
2. 下载 Git LFS 中的 mesh；
3. 生成 Desktop 使用的 URDF Adapter；
4. 校验21个控制关节和所有 mesh 引用；
5. 生成 Desktop 关节范围和产品指纹；
6. 生成 P4 URDF 范围头文件和固件指纹；
7. 执行模型、架构及相关自动化测试。

## 3. 只使用本地 structure

如果已经手动更新过 `D:\mmhand structure`，或者当前电脑不能联网：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\update_structure_urdf.ps1 `
  -StructureRepo "D:\mmhand structure" `
  -NoPull `
  -SkipP4Build
```

## 4. 判断是否更新成功

成功时终端末尾应包含：

```text
URDF validation passed.
P4 architecture check passed.
Structure URDF and Desktop/P4 generated contracts are ready.
```

随后检查修改文件：

```powershell
git status --short
git diff --stat
git diff -- desktop/resources/config/urdf_joint_mapping_v2.json
```

确认 `wuji_hand_ws` 没有被修改：

```powershell
git diff --name-only -- wuji_hand_ws
```

正常结果应为空。

## 5. 更新后的必要操作

1. 重新启动 MMHand 上位机，使 Python 清除旧模型缓存；
2. 重新编译并烧录 ESP32-P4，否则上下位机 URDF CRC 会不一致；
3. 检查 Device Check 中21个关节的 URDF Min/Max；
4. 如果关节范围 CRC 已变化，旧 Auto/LUT 标定快照不能继续使用，需要重新自动标定；
5. 如果只是 mesh 外形变化而关节范围未变，一般不需要重录 Manual Min/Max；
6. 如果机械结构、磁编码器安装或机械限位发生变化，应重新录入 Manual Min/Max 并重新自动标定。

## 6. 常见失败处理

### structure 存在本地修改

```powershell
git -C "D:\mmhand structure" status
```

先确认这些修改是否需要保留。不要使用 `git reset --hard` 强行清除。

### 找不到 mmhand.urdf

适配器会在 `mmhand_description\urdf` 中寻找 URDF：

- 存在 `mmhand.urdf` 时优先使用；
- 没有标准文件名但只有一个 `.urdf` 时，使用唯一候选并打印警告；
- 存在多个候选时直接停止，避免选择错误模型。

### P4 编译时提示 DLL 解压失败

先检查系统盘空间：

```powershell
Get-PSDrive C
```

系统临时目录需要足够空间。模型更新可以先使用 `-SkipP4Build` 完成，再清理空间并通过 Arduino IDE 编译烧录。

## 7. 禁止手工修改的生成文件

以下文件由脚本生成，不应直接编辑：

```text
desktop/resources/config/urdf_joint_mapping_v2.json
desktop/resources/config/product_fingerprint.json
ESP32-P4/ServoBoardMain/src/generated/UrdfJointLimits.generated.h
ESP32-P4/ServoBoardMain/src/generated/ProductFingerprint.generated.h
```

需要改变关节范围时，应修改 structure 源 URDF，然后重新运行更新脚本。
