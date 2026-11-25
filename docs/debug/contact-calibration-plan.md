# 接触标定：指尖合力触发的滑动轨迹

## 1. 工作流

1. 进入 Contact Calibration。手部空载时点 `Tactile sensor force zeroing`。只需连接 P4 USB，S3 逐个标零在线触觉模组并回读合力。
2. 用一个指尖触觉传感器贴住另一根手指，保持接触滑动约 1 cm，再松开。页面自动收录轨迹。可以用 `Delete Last Contact Trajectory` 删除上一条。
3. 选好 `Encoder correction (k/b)` 和可选的 `Joint axis translation & rotation`，点 `Solve`，确认后点 `Save Contact Calibration`。
4. `Clear Contact Calibration` 清空会话并删除标定包中的 contact 字段，回退到 Joint Calibration 值。

灰色手是当前基准位姿，浅蓝色手是拟合后的位姿。蓝色高亮指尖传感器和对侧接触段。箭头从各在线模组的固定中心显示三轴合力。

## 2. 采集

- 用 P4 时间戳将原始合力摘要与最近的有效 21 路编码器帧配对，允许的时间差为 25 ms。
- 指尖合力模长达到 5 个读数单位时触发，降到 3 时释放，两帧确认。多个指尖同时触发时选拇指。多个指尖触发且不含拇指时暂停采样。
- 当前指尖掉线、P4 断开或离开页面时，丢弃未完成的轨迹。
- 每帧对侧取其他四指中最近的表面。按指尖传感器质心移动 2 mm 抽稀，末帧保留。少于 5 个接触样本时提示延长滑动。

## 3. 事件格式

```
{'c': [21 个 signed counts], 'y': 1, 'pair': (f, o, seg), 'traj': 0}
```

| 字段 | 含义 |
|---|---|
| `c` | 21 个 signed counts；不是 raw |
| `y` | 1 = 真接触；0 = 程序生成的 non-contact |
| `pair` | f = 移动指尖（0 小指…4 拇指）；o = 目标手指；seg = 0 指尖 / 1 远节 / 2 近节 |
| `traj` | 自动收录的轨迹编号；连续性只在同一 `traj` 内、列表相邻的 y=1 之间生效 |

## 4. non-contact 样本

对每个接触样本：

1. 用基值模型求目标对间隙 `d` 对移动指尖各关节 count 的偏导（中心差分，`GRAD_STEP_COUNTS=50`）。
2. 沿“让 d 变大”的方向做一维搜索，在每关节最多 `5°` 的前提下，找最小步长使模型间隙达到约 3 mm。
3. 关节角夹在 URDF 范围内；达不到 1.6 mm 就跳过。

**关于“穿模”**：这里的间隙是无符号最近点距离，两个表面叠在一起时它仍是 0，
模型分不清“接触”和“穿透”。把穿透位姿标成 non-contact 会把参数带歪，所以不采用。
若以后要做“接触 + 有符号距离惩罚过穿透”，属于 V8，需要 mesh-based signed distance，不在 V7 范围。

## 5. 损失与求解

```
p_e = 1 / (1 + exp((d_e − δ)/τ))          δ = 0.8 mm, τ = 0.4 mm

L_data = Σ_e −[ y_e·ln p_e + (1−y_e)·ln(1−p_e) ]

L_cont = Σ_{同 traj 相邻 y=1} ½·[ max(0, ‖q_i − q_{i−1}‖ − A) / σ ]²
         q = 两侧最近点的中点，A = 6 mm，σ = 3 mm

L_prior = ½·Σ_θ [(θ − θ0)/s_θ]²
          k: s=0.02·k0     b: s=2°
          t: s=2 mm        α,β: s=1°

J = L_data + L_cont + L_prior
```

- `L_data`：接触样本拉向 d=0，non-contact 样本推离接触；`p` 只是可微的软开关。
- `L_cont`：真实滑动每个抽稀样本的接触点大约移动几毫米；6 mm 以内不罚，超出部分线性/二次惩罚。
- `L_prior`：每个参数系一根弹簧，锚定 Joint Calibration；数据没覆盖的关节保持基值。
- 数值 Jacobian + LM，`solve` 单次最多 12 次迭代、早停；页面用 `solve_contact` 做 3 轮外循环
  （每轮 6 次），每轮重新找最近点对应，避免固定材料点造成的局部极小。
- `solve` 在每个残差评估里按当前几何重新找指定 `pair` 内的最近点（不再固定材料点），
  这是 V7 能真正拟合轨迹的关键。
- 越界（±5 mm、±3°、±10% k、±5° b）整组回退；被回退关节记入 `rejected`。

## 6. 保存与下发

- 活动标定包 `contact_calibration[urdf_joint_name] = {k, b, base_k, base_b[, xyz, rpy]}`。
- 几何修正只影响上位机 3D 显示；P4 控制只用 k/b。
- Motor Calibration 生成关节结果后调用 `apply_contact_refinement` → `relabel_lut`，
  超出 ±10% k / ±5° b / ±5° 端点则不采用。
- `Clear Contact Calibration` 只删字段；之后重做 Motor Calibration 会用 Joint Calibration 值。

## 7. 离线仿真验证（桌面临时目录，已删除）

仿真代码曾放在 `C:\Users\73547\Desktop\mmhand_contact_sim\`（不进 git），
用真值 k/b（±2%/±2°）生成“食指指尖在拇指尖上滑动”的连续轨迹，
再调用仓库真实的 `trajectory_events / add_non_contact_samples / solve`。

结论（10 seeds，默认轨迹约 1 cm、2 mm 抽稀）：

| 项目 | 结果 |
|---|---|
| 自动指尖 | 10/10 正确 |
| 接触/近距样本数 | 中位 11 / 5 |
| k/b 误差 | 中位 0.95% / 0.94° |
| 端点角误差 | 中位 1.8°（A=2mm,σ=1mm）；A=6mm,σ=3mm 时约 1.1° |
| contact 通过率 | 43%（A=2/1）；63%（A=6/3）；79%（A=10/5） |
| non-contact 通过率 | 90% |
| 求解耗时 | 约 1–2 s |

- **样本数量**：样本数不是主要瓶颈；5–21 个 contact 的端点误差都在 1.2–2.1° 波动，没有单调改善。
  推荐一条 ≥1 cm 的滑动、≥8 个 contact 作为实用下限；`MIN_SAMPLES=5` 只是硬下限。
  真正制约精度的是先验和轨迹姿态多样性（一次滑动通常只覆盖约 3 个关节），所以建议一页内录 2–3 个指对。
- **抽稀步长**：1 mm 比 2 mm 慢约一倍，误差没有改善；默认 2 mm。
- **连续性**：A=2 mm/σ=1 mm 对真实滑动太紧；A=6 mm/σ=3 mm 明显提高 contact 通过率，默认采用。
- **non-contact 样本**：在当前理想仿真里对参数误差帮助有限（关掉也差不多），
  保留它主要用于真机的尺度约束；真值间隙有少数落在 0.7 mm 附近，说明生成方法不是绝对可靠。
- **模型失配**：真值 k/b 放大到 ±5%/±5° 时端点误差约 4.5°，说明本方法只适合
  Joint Calibration 之后的小修正，不是重新做关节标定。
- **几何勾选**：仿真里误差略降（1.7°→1.3°），但仍需要更多轨迹覆盖；默认不勾。

## 8. 限制

- 触觉只确认哪个指尖在受力。对侧仍由几何距离推断，因此采集时应只让参与标定的两根手指接触。
- 无符号最近点距离无法检测穿透；穿模需要在 3D 里人工发现。
- 一次滑动主要约束移动指尖的关节；其他关节由先验保持，建议一页内录 2–3 个指对。
- 仿真使用同一套 URDF/点云，验证的是可辨识性和实现正确性，不代表真机绝对精度。

## 9. 测试

```
python -m pytest tests/desktop/core/test_contact_calibration.py \
    tests/desktop/ui/test_native_pages.py \
    tests/desktop/core/test_application_actions.py -q
python tools/check_desktop_architecture.py
```

（本机若默认 temp 目录被系统策略拒绝，加 `--basetemp=.testtmp`。）
