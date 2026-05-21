# PID ABS 标定数据使用说明

## 文件位置

PID 控制需要的舵机 ABS 锚点单独保存在：

```text
desktop/config/pid_abs_calib.json
```

这个文件不要和 `desktop/config/calib_deg.json` 混用。`calib_deg.json` 保存的是磁编码器 0 度对应的原始值；`pid_abs_calib.json` 保存的是 PID 控制需要的舵机 ABS 控制锚点。

## 如何生成

1. 连接上位机和 ESP32-P4。
2. 执行自动标定，等待右上角 `CALIB` 进入成功状态。
3. 在 Joint Channel 里观察 `0deg ABS` 列。
   - 上位机会先等待该关节完成自动标定，也就是对应电机的 `Limit` valid 位先被清除、随后重新变为有效。
   - 在该关节完成标定之后，实际磁编角第一次接近 `0 ± 0.25°` 时，上位机会记录该关节对应舵机的当前绝对 ABS。
   - 一个关节对应两个拮抗舵机时，会同时记录两个舵机，例如 `M00:12345 M22:22345`。
4. 确认 Motor Channel 的 `Limit` 已经有有效范围。
   - PID 文件使用 `Limit` 的右端，也就是固件上报的 `motor_abs_limit_max[motor]`。
5. 点击 Device 区域的 `Save PID ABS`。
6. 上位机会写入 `desktop/config/pid_abs_calib.json`。

如果某些关节还没有进入过 0 度附近，或者对应舵机没有有效 `Limit`，保存时会跳过这些项，并在底部状态提示缺失数量。

## 文件格式

示例：

```json
{
  "comment": "PID ABS calibration anchors...",
  "schema_version": 1,
  "updated_at_host_time": 1760000000.0,
  "hand_profile": "v2",
  "source_calib_status": "SUCCESS",
  "zero_capture_tolerance_deg": 0.25,
  "source_limit_valid_bitmap": "0x00FFFFFF",
  "source_limit_locked_bitmap": "0x00FFFFFF",
  "joints": {
    "J00": {
      "joint": 0,
      "motors": {
        "M00": {
          "motor": 0,
          "zero_abs_at_encoder_0deg": 12345,
          "tight_abs_right_limit": 15678,
          "span_abs": 3333
        },
        "M22": {
          "motor": 22,
          "zero_abs_at_encoder_0deg": 22345,
          "tight_abs_right_limit": 25678,
          "span_abs": 3333
        }
      }
    }
  }
}
```

字段含义：

- `zero_abs_at_encoder_0deg`：该关节完成自动标定之后，磁编码器实际角第一次进入 `0 ± 0.25°` 时，该舵机读到的绝对 ABS。
- `tight_abs_right_limit`：绝对绷紧时右端 ABS，来自 Motor Channel 的 `Limit` 右端，也就是 `motor_abs_limit_max[motor]`。
- `span_abs`：`tight_abs_right_limit - zero_abs_at_encoder_0deg`，可能为正也可能为负，控制时不能假设方向固定。

## Python 调用示例

```python
import json
from pathlib import Path

PID_ABS_CALIB_PATH = Path("desktop/config/pid_abs_calib.json")


def load_pid_abs_calib(path=PID_ABS_CALIB_PATH):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def get_motor_pid_abs(calib, motor_idx):
    motor_label = f"M{motor_idx:02d}"
    for joint_data in calib.get("joints", {}).values():
        motor_data = joint_data.get("motors", {}).get(motor_label)
        if motor_data is not None:
            return (
                int(motor_data["zero_abs_at_encoder_0deg"]),
                int(motor_data["tight_abs_right_limit"]),
            )
    raise KeyError(f"{motor_label} not found in pid_abs_calib.json")


calib = load_pid_abs_calib()
zero_abs, tight_abs = get_motor_pid_abs(calib, 0)
```

## 控制时怎么用

如果 PID 输出的是舵机 ABS 增量：

```python
def clamp_abs(value, a, b):
    lo = min(a, b)
    hi = max(a, b)
    return max(lo, min(hi, value))


cmd_abs = zero_abs + pid_output_abs_delta
cmd_abs = clamp_abs(cmd_abs, zero_abs, tight_abs)
```

如果 PID 输出的是从 0 到 1 的绷紧比例：

```python
ratio = max(0.0, min(1.0, ratio))
cmd_abs = zero_abs + ratio * (tight_abs - zero_abs)
```

如果需要把当前 ABS 归一化成绷紧比例：

```python
span = tight_abs - zero_abs
if span == 0:
    raise ValueError("invalid PID ABS calibration: zero_abs equals tight_abs")

ratio = (current_abs - zero_abs) / span
ratio = max(0.0, min(1.0, ratio))
```

注意：`tight_abs_right_limit` 的数值不一定大于 `zero_abs_at_encoder_0deg`。方向应由 `span_abs` 的正负决定，限幅时必须用 `min()` 和 `max()`。

## 使用建议

- 每次重新自动标定后，都应该重新点击 `Save PID ABS`。
- PID 程序启动时应检查 `schema_version == 1`。
- 对缺失的关节或舵机，不要使用默认 0 参与控制，应直接报错或禁用该通道。
- 实机控制前建议打印每个通道的 `zero_abs_at_encoder_0deg`、`tight_abs_right_limit`、`span_abs`，确认方向和范围是否符合预期。
