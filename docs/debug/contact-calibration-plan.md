# 接触标定 V6：手动接触点击 + 二分类（物理手指 link）

> 弃用触觉采集与触觉 sensor link。人工点两个按钮标注“接触 / 接近但未接触”，
> 用 Joint Calibration 的 k/b + URDF 物理手指 link 推断接触部位，
> 二分类拟合每关节 `k/b`（可选再加 3 平移 + 2 旋转）。

## 1. 公式与接触体

```
c_j = signedDirectedCoordinateFromWindowStart(m_lo, raw, S)   # 与 P4 一致
θ_j = k_j·c_j + b_j
k0_j = (U_hi − U_lo) / S_j,  b0_j = U_lo                      # Joint Calibration 基值

O_j = O_j^URDF · E(t_j, α_j, β_j),  E = [[R(a₁α_j+a₂β_j), t_j], [0,1]]
```

15 个接触体是物理手指 link（不是 `*_skeleton_sensor`）：

| 手指 | 指尖 | 远节 | 近节 |
|---|---|---|---|
| Pinky | `link_finger_tip_5` | `link_distal_phalanx_5` | `link_proximal_phalanx_5` |
| Ring | `link_finger_tip_4` | `link_distal_phalanx_4` | `link_proximal_phalanx_4` |
| Middle | `link_finger_tip_3` | `link_distal_phalanx_3` | `link_proximal_phalanx_3` |
| Index | `link_finger_tip_2` | `link_distal_phalanx_2` | `link_proximal_phalanx_2` |
| Thumb | `link_finger_tip_1` | `link_distal_phalanx_1` | `link_proximal_phalanx_1` |

- 表面点：现有模块测点用 URDF 里 sensor fixed joint 的原点（纯平移）换到物理 link 坐标系；
  世界坐标与 sensor link 计算完全一致。不读触觉数据。
- 页面先选移动的指尖 f；每个样本只在 f 对另外 4 指 × 3 段（12 对）里推断目标指和段。
  不用全局 60 对：手蜷起来时相邻手指（如中指/无名指）在名义模型里只差 2–3 mm，
  会把真实的小指-拇指接触抢走。
- 位姿间隙 = 两段点云最近点对的距离；12 对里最近的那一对就是推断出的接触部位。
- 手掌不建模；指尖按掌心不会被识别为手掌接触。

```
s_e = (δ − d_e)/τ,  p_e = σ(s_e),  δ = 0.8 mm, τ = 0.4 mm
loss = Σ_e BCE(p_e, y_e) + ½·Σ (ξ/σ_ξ)²
σ_ξ: 平移 2 mm, 旋转 1°, k 2%, b 2°
```

求解：全部内存事件 → 记录时推断的 `(指尖, 目标指, 段)`（测试/旧数据按 fingertip 分组）→
物理 link FK 与最近点 → 数值中心差分 Jacobian + 列归一化 LM 最小化
`[sqrt(2·BCE), 先验]`；初值 = 零几何 + k0/b0。
`optimize_kb` / `optimize_geometry` 控制自由参数子集，默认只拟合 k/b。
越界关节（±5 mm、±3°、±10% k、±5° b）整组回退并记入 `rejected`。

## 2. 页面

工具栏 6 个按钮：

| 按钮 | 行为 |
|---|---|
| `Record Contact` | 记录当前 counts 为接触样本，不自动求解 |
| `Record Close (no contact)` | 记录为接近但未接触样本 |
| `Delete Last Sample` | 删除最后一条样本；已有解则标记需要重新 Solve |
| `Solve` | 按勾选项后台求解一次 |
| `Save Contact Calibration` | 写标定包 |
| `Clear Contact Calibration` | 删除标定包 contact 字段并清空会话 |

- `Fingertip` 下拉框：记录前必须选（Thumb/Index/Middle/Ring/Pinky）；没选时两个记录按钮禁用。
- 勾选框：`Encoder correction (k/b)`（默认选）、`Joint axis translation & rotation`（默认不选）。
- 计数累加两类手指：所选指尖 + 推断出的目标指，各 +1。
  例：选 Thumb、碰 Pinky → `Finger contact/close: Thumb 1/0 … Pinky 1/0`。
  目标指只在所选指尖的 12 个候选里推断，不会出现与所选指尖无关的假计数。
  记录时把 `(指尖, 目标指, 段)` 存进事件，求解器直接用这一对。
- 样本只追加：`Solve` 后继续 `Record` 会提示重新 `Solve`，下一次 `Solve` 用全部样本；
  `Delete Last Sample` 删最后一条，`Clear` 清空会话。
- 3D：灰 = Joint Calibration 位姿（URDF 原点 + k0/b0）；蓝 = 最新解。
- Solve 后状态只显示一句：
  `Average gap between touching fingers: 7.1 mm before, 1.7 mm after.`

## 3. 保存与下发

活动标定包（`calibration_bundle.mmcal.json`）新增：

```json
"contact_calibration": {
  "index_mcp_fe_joint": {
    "k": 0.0143, "b": -14.0,
    "base_k": 0.0140, "base_b": -14.2,
    "xyz": [0, 0, 0], "rpy": [0, 0, 0]
  }
}
```

- `xyz/rpy` 只对真正有几何修正的关节存在；未修正关节没有这两个键。
- `save_contact` 只写这个字段，不改 `joint_calib_results`。
- Motor Calibration 生成每个关节 result 后调用 `apply_contact_refinement` → `relabel_lut`，
  修正 k/b 随 LUT/限位一起下发 P4；超带则保持 Joint Calibration 值。
- `Clear Contact Calibration` 只删字段。之后重做 Motor Calibration 会从 Manual 端点
  重新计算 k0/b0，不读旧 contact 值，可正常下发 P4。
- Plot→Hand：灰 = 最新修正当前位姿（无 contact 修正时即 Joint Calibration）、红 = 目标；
  每次刷新重读活动标定包的几何修正，缺失关节回退 URDF 原点。

## 4. 限制

- 只建模“指尖 → 另一指三段”；目标指仍按 12 对最近表面推断，模型误差大时可能推到相邻手指。
  计数只依赖所选指尖，所以不会出现假计数；目标指要是老错，再加目标指下拉框。
- 点云离散化使分辨率约 1 mm；`δ` 是补偿旋钮。
- 先验限制 k/b 修正幅度；接触覆盖不足的关节保持 Joint Calibration 值。
- 几何修正只影响上位机显示；P4 控制只使用 k/b。
