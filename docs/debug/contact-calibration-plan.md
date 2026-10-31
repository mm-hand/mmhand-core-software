# 触觉接触标定 V5：实现与使用说明

> 本文替换旧的 V4/147 参数计划。当前实现以 **counts 为安全域、`θ = k·c + b` 为唯一角度解释**。

## 1. 公式与数据

```
c  = signedDirectedCoordinateFromWindowStart(m_lo, raw, S)
θ  = k·c + b
k0 = (U_hi − U_lo) / S ,  b0 = U_lo        # joint calibration 初值
```

- `JointCalibrationResult` 新增 `angleK/angleB`（两 float）。
- `angleMin/angleMax` 仍存在，恒为派生端点 `[b, kS+b]`；用于目标钳位/LUT/兼容。
- `k/b` 由上位机算好，通过 `CMD_CALIB_LOAD_JOINT` 下发；**payload 仍是 188 字节**：
  原先闲置的 `currentMinDeg/currentMaxDeg` 两个 float 槽改为承载 k/b。上行
  calibration snapshot 的 201 字节同理。
- `mappingVersion = 5`。旧 v4 bundle 在 `CalibrationSnapshotStore.load/inspect` 时自动迁移：
  按 `(angleMax−angleMin)/manualSpanCounts` 补 k0、`angleMin` 补 b0，更新 CRC/指纹后
  重新 load 到 P4；不需要重做 joint/motor 标定。
- **counts 安全域完全不变**：`m_lo/m_hi/manualSpan/outsideCounts/READY/Pull into Workspace`
  仍由 counts 决定。k/b 只解释角度。
- 运行时角度端点来自每关节 result：`angleMin/angleMax`；静态 URDF 只是标称模型/
  初始值。`resolve_joint_workspace_range`、snapshot 行、ROS2 校验、request_pose、
  replay 都使用活动端点。

## 2. 标定求解

`desktop/core/contact_calibration.py`：

- 采集：S3 直连 `FULL`、15 模块 valid、`control_mode == IDLE`、静止
  (`max|Δθ| ≤ 0.1°`)、恰好 2 个不同指模块、同姿态去重 + 0.5 s 冷却。
- 事件只存 `c`（counts）+ 模块局部接触点 + 合力。
- 未知量：每关节 5 个几何（3 平移 + 2 垂直轴旋转）+ k + b = 147。
- 数值中心差分 Jacobian + 列归一化 LM；朝初值的 ridge 取最小修正解。
- 输出几何原点（`xyz/rpy`）、k/b、RMS、每关节事件数、weak/rejected。

**可辨识性（重要）**：无触觉模块的中间 link（AA/CMC 链）上的相邻关节原点存在规范
自由度；实测 147 参数接触 Jacobian 的秩约 129。因此：

- 数据能唯一确定的是“sensor 位姿 + 可观测的 k/b/几何组合”；
- 规范零空间分量用 ridge 保持在 joint calibration 的 k/b 和零几何；
- 验收以 **held-out 接触残差**（<0.1 mm）和 k 的可观测误差（<1%）为准，
  不再要求逐项还原随机真值。

## 3. 保存与应用

Contact 页 Solve 后 Save & Apply：

1. 用 `relabel_lut` 把当前 result 的 LUT 角度轴重标（`A=k/k0, B=b−A·b0`），
   motor 表不变；更新 `angleK/B/angleMin/Max`。
2. `build_updated_bundle` 更新活动 bundle 的 21 个 result 并重新封存。
3. 用现有 calibration import 流程（preview → import）把 bundle 写入 P4；
   ACK 成功后写 `DATA_DIR/contact_calibration.json`（几何 + 哈希）。
4. 页面刷新后 Hand 页显示修正模型；设备遥测角度来自新 k/b。

`Clear` 只删除本地几何 JSON；设备端的 k/b/LUT 需用 joint calibration 的
“Load Joint & Motor Calibration” 恢复为初始结果（后续可以补一个“回滚到 k0/b0”按钮）。

## 4. Hand 页

`desktop/ui/web/hand.py` 显示三只手：

| 模型 | origin | 角度 | 可见 |
|---|---|---|---|
| 浅灰 | 原始 URDF | 当前 angle | 始终 |
| 灰 | contact 几何修正 | 当前 angle | 始终 |
| 绿 | contact 几何修正 | target | FEEDBACK/DIRECT + Online |

- 触觉只挂灰模：每模块 `PointCloud`（真实测点，按 taxel 法向 z 着色；非 FULL 隐藏）
  + 一根合力箭头；不显示数值。
- 22 个 STL 文件浏览器只下一份；三棵模型树共 66 个 mesh。

## 5. 使用步骤（实机）

1. S3 直连 USB，P4 正常连接；确认 Contact 页前置条件全绿。
2. 手摆姿势让两指触觉相碰，每个姿势保持约 0.5 s；覆盖所有 10 个指对，至少 150 个事件。
3. Solve；看 before/after RMS、rejected/weak。
4. Save & Apply；进度显示导入完成后刷新页面。
5. Hand 页对比浅灰/灰两只手；灰手应更贴近实机。

## 6. 不做 / 后续

- 不改 P4 控制环/LUT 代码（LUT 只在保存时重标角度轴）。
- 不生成第二份标定 URDF；几何修正存运行时 JSON。
- 不加 Teleop 页；Replay 已移入 Control。
- 后续若要让 P4 自己拥有 k/b（外部控制器/多上位机），需把 k/b 进 calibration result
  的正式字段并定 mappingVersion 契约；当前用保留槽已足够单上位机场景。
- `Clear` 的设备端回滚、Contact 页的实时点云预览可以再补。
