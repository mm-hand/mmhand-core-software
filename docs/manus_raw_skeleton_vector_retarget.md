# MANUS Raw Skeleton 向量 Retarget 流程

本文档整理当前 `MANUS raw skeleton -> robot hand` 的 retarget 和 IK 链路。当前实现的核心思想是：

```text
MANUS raw skeleton
-> MANUS palm frame
-> MANUS palm-local
-> robot URDF palm frame
-> robot palm-local 向量目标
-> 向量 IK + reference
-> J00-J20 目标角
```

当前 raw skeleton 链路不再把一个绝对 fingertip 点直接作为 IK 目标，而是用两个向量作为 IK 目标：

```text
1. anchor -> fingertip
2. thumb fingertip -> other fingertip
```

对应主要代码：

```text
desktop/teleop/manus_raw_skeleton_retarget.py
desktop/teleop/Posture/hand_retargeting/urdf_model.py
desktop/teleop/manus_calibration.py
```

## 1. MANUS Raw Skeleton 输入

MANUS SDK / bridge 读到的数据最终进入 `ManusFrame.skeleton`。当前使用的是语义化 skeleton tree，而不是直接用无名数组：

```text
skeleton["hand"]
skeleton["thumb"]["metacarpal" / "proximal" / "distal" / "tip"]
skeleton["index"]["metacarpal" / "proximal" / "intermediate" / "distal" / "tip"]
skeleton["middle"]["metacarpal" / "proximal" / "intermediate" / "distal" / "tip"]
skeleton["ring"]["metacarpal" / "proximal" / "intermediate" / "distal" / "tip"]
skeleton["pinky"]["metacarpal" / "proximal" / "intermediate" / "distal" / "tip"]
```

每个 node 主要使用：

```text
pos = [x, y, z]
rot = [w, x, y, z]
```

`pos` 用于几何 retarget。`hand.rot` 用于修正 MANUS palm frame 的法向正反。

关键读取函数：

```text
_node_pos()
_node_rot()
_quat_to_rotation()
```

位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py
```

## 2. 建立 MANUS 手掌坐标系

关键函数：

```text
FullRawSkeletonRetargeter._raw_skeleton_palm_frame()
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:258
```

取点：

```text
p_hand
p_index_meta
p_middle_meta
p_ring_meta
p_pinky_meta
```

要求至少要有：

```text
p_index_meta
p_pinky_meta
```

因为这两个点用于确定掌宽方向。

公式：

```text
O_m = p_hand

C_m = mean(
  p_index_meta,
  p_middle_meta,
  p_ring_meta,
  p_pinky_meta
)

x_m = normalize(C_m - O_m)

y_raw_m = normalize(p_pinky_meta - p_index_meta)

z_m = normalize(x_m x y_raw_m)
```

用 `hand.rot` 修正手掌法向：

```text
R_hand = quat_to_rotation(hand.rot)
n_hand = R_hand[:, 2]

if z_m · n_hand < 0:
    z_m = -z_m
```

最后重新计算正交的 y 轴：

```text
y_m = normalize(z_m x x_m)

R_m = [x_m, y_m, z_m]
```

这里 `R_m` 的三列分别是 MANUS palm frame 的 x/y/z 轴在 MANUS 原始坐标系里的方向。

## 3. MANUS Palm-Local

关键函数：

```text
FullRawSkeletonRetargeter._raw_skeleton_to_palm_local_nodes()
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:315
```

对每个 MANUS node：

```text
p_i = node_i.pos
O_m = MANUS palm origin
R_m = [x_m, y_m, z_m]
```

转成 MANUS palm-local：

```text
p_i^m = R_m^T (p_i - O_m)
```

代码里使用 row-vector 写法：

```text
p_i^m = (p_i - O_m)^T R_m
```

这一步的作用是去掉 MANUS 手掌在空间中的全局位姿，只保留“相对手掌”的手形几何。

## 4. 根据新版 URDF 建立 Robot 结构

新版 URDF 已替换到默认路径：

```text
desktop/hku_hand_v2_urdf/urdf/hand.urdf
```

当前 URDF 名称：

```text
<robot name="mmhand">
```

新版 URDF 使用语义 joint 名，例如：

```text
index_aa_joint
middle_aa_joint
ring_aa_joint
little_aa_joint
thumb_cmc1_joint
```

关键函数：

```text
URDFRobotHandModel._build_semantic_mmhand_chains()
```

代码位置：

```text
desktop/teleop/Posture/hand_retargeting/urdf_model.py:154
```

四指链路：

```text
base_link
-> base_link_1_finger_base_N    fixed
-> AA joint
-> MCP joint
-> PIP joint
-> DIP joint
-> fingertip link
```

拇指链路：

```text
base_link
-> thumb_cmc1_joint
-> thumb_aa_joint
-> thumb MCP
-> thumb PIP
-> thumb DIP
-> thumb fingertip link
```

FK 关键函数：

```text
URDFRobotHandModel.forward_tip_position()
```

代码位置：

```text
desktop/teleop/Posture/hand_retargeting/urdf_model.py:266
```

注意：新版四指 FK 必须从 `base_link -> finger_base` 的 fixed transform 开始，否则会漏掉 URDF 里的 finger base 偏移。

## 5. 建立 Robot 手掌坐标系

关键函数：

```text
URDFRobotHandModel.robot_palm_frame()
```

代码位置：

```text
desktop/teleop/Posture/hand_retargeting/urdf_model.py:289
```

这里 `O_r` 使用 URDF 的 `base_link` 坐标系原点。因为所有 robot 点本来就表达在 `base_link` 坐标系下，所以数值是：

```text
O_r = p_base_link = [0, 0, 0]
```

这里的 `[0, 0, 0]` 不是任意选择的点，而是 `base_link frame origin`。

四指 root：

```text
p_index_root
p_middle_root
p_ring_root
p_pinky_root
```

来自每根手指 AA joint 在 `base_link` 坐标系下的位置。

公式：

```text
C_r = mean(
  p_index_root,
  p_middle_root,
  p_ring_root,
  p_pinky_root
)

x_r = normalize(C_r - O_r)

y_raw_r = normalize(p_pinky_root - p_index_root)

z_r = normalize(x_r x y_raw_r)
```

用 URDF/base_link 正法向修正：

```text
n_robot = [0, 0, 1]

if z_r · n_robot < 0:
    z_r = -z_r
```

最后：

```text
y_r = normalize(z_r x x_r)

R_r = [x_r, y_r, z_r]
```

## 6. Robot Palm-Local

base_link 坐标转 robot palm-local：

```text
p_r_local = R_r^T (p_r_base - O_r)
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:394
```

robot palm-local 转回 base_link：

```text
p_r_base = O_r + R_r p_r_local
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:398
```

当前 raw IK 里，最终残差在 `base_link` 坐标中计算，所以 retarget 出来的 robot palm-local 向量会再乘 `R_r` 转回 `base_link` 向量。

## 7. 掌宽得到尺度系数

关键函数：

```text
FullRawSkeletonRetargeter._human_palm_width()
FullRawSkeletonRetargeter._retarget_available_tip_targets()
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:432
desktop/teleop/manus_raw_skeleton_retarget.py:358
```

当前 raw skeleton retarget 不再按每根手指折线长度分别求尺度，而是所有向量统一使用掌宽比例。

```text
p_index_meta^m = MANUS index metacarpal in MANUS palm-local
p_pinky_meta^m = MANUS pinky metacarpal in MANUS palm-local

W_MANUS = ||p_pinky_meta^m - p_index_meta^m||
W_robot = robot_palm_width

s_palm = W_robot / W_MANUS
```

这个 `s_palm` 同时用于 `anchor->tip` 和 `thumb->tip` 两类向量。

## 8. MANUS Palm-Local 到 Robot Palm-Local 的向量 Retarget

关键函数：

```text
FullRawSkeletonRetargeter._retarget_available_tip_targets()
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:402
```

当前使用两个向量作为 IK 目标。

### 向量 1：anchor -> fingertip

每根手指都有：

```text
a_f^m = MANUS finger anchor in MANUS palm-local
t_f^m = MANUS fingertip in MANUS palm-local

v_anchor_tip_f^m = t_f^m - a_f^m
```

当前 anchor：

```text
thumb:  metacarpal
index:  metacarpal
middle: metacarpal
ring:   metacarpal
pinky:  metacarpal
```

缩放到 robot palm-local：

```text
v_anchor_tip_f^r = s_palm v_anchor_tip_f^m
```

再转成 base_link 向量：

```text
v_anchor_tip_f^base = R_r v_anchor_tip_f^r
```

### 向量 2：thumb fingertip -> other fingertip

非拇指额外使用：

```text
t_thumb^m = MANUS thumb fingertip in MANUS palm-local
t_f^m = MANUS finger fingertip in MANUS palm-local

v_thumb_tip_f^m = t_f^m - t_thumb^m
```

这个向量描述的是两根手指之间的空间关系，也使用同一个掌宽尺度：

```text
s_palm = robot_palm_width / MANUS_palm_width

v_thumb_tip_f^r = s_palm v_thumb_tip_f^m
```

再转成 base_link 向量：

```text
v_thumb_tip_f^base = R_r v_thumb_tip_f^r
```

## 9. 向量 IK + Reference

关键函数：

```text
FullRawSkeletonRetargeter.preview()
FullRawSkeletonRetargeter._solve_finger_vector_ik()
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:645
desktop/teleop/manus_raw_skeleton_retarget.py:708
```

求解顺序：

```text
thumb -> index -> middle -> ring -> pinky
```

原因是其他四指需要使用“已解算拇指 tip”来构造 `thumb -> fingertip` 向量残差。

### 拇指 IK 残差

拇指只使用 `anchor -> fingertip`：

```text
e1(q) = (FK_tip_thumb(q) - robot_thumb_root) - v_anchor_tip_thumb^base
```

### 四指 IK 残差

四指使用两个向量：

```text
e1(q) = (FK_tip_f(q) - robot_root_f) - v_anchor_tip_f^base

e2(q) = (FK_tip_f(q) - FK_tip_thumb) - v_thumb_tip_f^base
```

组合残差：

```text
e(q) = [e1(q), w_thumb e2(q)]
```

当前权重：

```text
w_thumb = 0.65
```

优化目标：

```text
min_q ||e(q)||^2 + lambda ||q - q_ref||^2
```

代码里用数值 Jacobian 和 damped least-squares 更新：

```text
J = de / dq

(J^T J + damping I + lambda I) delta_q
  = -(J^T e + lambda (q - q_ref))

q <- q + clamp(delta_q)
```

Reference 来源：

```text
FullRawSkeletonRetargeter._build_raw_reference_joint_angles()
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:545
```

reference 主要用于收束 IK，避免同一个向量目标下关节角乱选解。它是软约束，不是硬限制。

当前 raw skeleton 路径已经不再构造 MediaPipe 风格的 21 点数组，也不再复用视觉版 reference。所有手指统一使用 raw soft reference：

```text
如果没有 ergonomics:
q_ref = 0.7 q_last + 0.3 q_neutral

如果有 ergonomics:
q_ref = 0.6 q_last + 0.2 q_neutral + 0.2 q_ergonomics
```

其中：

```text
q_last     = 上一帧 IK 解
q_neutral  = 0 rad，并按 URDF joint limit clamp
q_ergonomics = MANUS ergonomics 映射出的弱提示
```

这里的 reference 只作为 IK 正则项使用，真正的目标仍然是 raw skeleton 得到的向量约束。

向量 IK 当前参数：

```text
regularization = 5e-3
max_iters = 20
max_step = 0.18 rad
```

这样目标向量较大时，IK 仍会主要跟随向量误差；reference 只负责防跳解和多解收束。

## 10. 输出 J00-J20

IK 得到的是 URDF joint angle。之后转成电机 UI / 发送用的 J00-J20：

```text
_solution_to_encoder_degrees()
_encoder_index_for_joint()
```

代码位置：

```text
desktop/teleop/manus_raw_skeleton_retarget.py:172
desktop/teleop/manus_raw_skeleton_retarget.py:183
```

新版语义 joint 到 J 编号的映射。注意这里是 `URDF joint -> 逻辑关节 Jxx`，不是直接映射到物理电机 Mxx：

```text
little_aa_joint                                      -> J00
mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint     -> J01
mmhand_finger_4_distal_phalanx_1_pip_joint          -> J02
mmhand_finger_4_fingertip_1_dip_joint               -> J03

ring_aa_joint                                       -> J04
...

middle_aa_joint                                     -> J08
...

index_aa_joint                                      -> J12
...

thumb_aa_joint                                      -> J16
thumb MCP                                           -> J17
thumb PIP                                           -> J18
thumb DIP                                           -> J19
thumb_cmc1_joint                                    -> J20
```

拇指 J 编号到物理电机 M 编号的关系由下位机硬件映射继续展开。当前拇指对应关系是：

```text
J16 -> M20 + M21
J17 -> M22
J18 -> M23
J19 -> M24
J20 -> M25 + M26
```

## 11. 当前实现的边界

当前已经实现：

```text
1. 新版 mmhand URDF 替换默认 hand.urdf
2. 新版 mmhand 语义 joint parser
3. robot palm frame
4. MANUS palm-local
5. 掌宽比例作为 raw skeleton 向量统一尺度
6. 每根手指的 anchor->tip 向量目标
7. 四指的 thumb->tip 向量目标
8. raw skeleton 专用向量 IK
```

当前仍保留但 raw path 不再主用：

```text
旧 fingertip absolute target IK
```

后续可以继续优化：

```text
1. 把 thumb->tip 权重从固定 0.65 改成 UI/配置项
2. 使用多点向量目标，例如 MCP/PIP/DIP/tip 多个 FK 点
3. 用 raw skeleton open pose 标定 finger-local 旋转
4. 分开标定 forward/side/depth 三个尺度，而不是单个 s_palm
```
