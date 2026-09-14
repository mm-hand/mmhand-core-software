# 多圈与标定 JSON 恢复

最后核对：2026-08-22。

## 数据所有权

P4 只保存本次上电会话的 RAM 状态，不再写入运行期 Flash。跨 P4 重启的事实源全部在上位机 JSON：

| 文件 | 内容 | 更新时机 |
| --- | --- | --- |
| `desktop/resources/config/calibration_bundle.mmcal.json` | 21 路 Manual、Auto/LUT、ABS Limit | Manual 写入、自动标定成功、Calib Load |
| `desktop/resources/config/device_state/multiturn_runtime.json` | 27 路 logical ABS、SavedRaw、有效掩码和设备身份 | 正常运行时更新、Reset Zero、标定结果变化 |

两份文件都支持复制到另一台电脑后再执行 Calib Load/恢复；恢复前必须通过产品指纹和 JSON 完整性校验。

## 多圈恢复

舵机单独掉电后，硬件多圈坐标可能回到 turn 0。上位机对每台 Turn Unknown 舵机，根据最近一次 JSON 锚点和当前 raw 计算跨圈最短差：

```text
delta = ((currentRaw - savedRaw + 2048) mod 4096) - 2048
restoredLogicalAbs = savedLogicalAbs + delta
```

P4 收到的是最终 `restoredLogicalAbs`，只建立 logical offset，不能再次展开 raw 差。恢复失败、身份不匹配或锚点无效时保持 Turn Unknown，相关关节必须重新标定。

## 会话内热插拔

P4 仍会在舵机掉线时冻结最后可信 RAM 锚点，用于不重启 P4 的热插拔恢复。该 RAM 锚点在 P4 断电后消失；跨重启恢复始终使用上位机 JSON。

## Calibrate 与 Reset Zero

`Calibrate` 会清除所选关节对应舵机的 JSON 多圈锚点、下发硬件圈数归零并失效对应 Auto/LUT，然后重新建立新锚点。`Reset Zero` 只用于硬件坐标接近 `+-30715` 时的受控归零；成功后上位机更新 JSON，旧 LUT 必须失效。

## 验收

1. `SavedRaw=4090`、当前 raw=6，恢复差应为 `+12 counts`。
2. 只让 M24 掉电，M24 独立恢复，其它 26 路锚点不变。
3. 删除或篡改 JSON 后，系统保持 Turn Unknown，不能静默进入 READY。
4. P4 重启后先恢复 Manual/Auto/LUT 到 RAM，再恢复多圈锚点，随后才允许负反馈。
