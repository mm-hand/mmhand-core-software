# 舵机反馈失败文本诊断

上位机自动将 P4 的 `[SERVO READ FAIL]` 文本保存在
`logs/session_YYYYMMDD_HHMMSS.log`。无需开启 Save Data 或 JTAG。
校准命令记录使用 `[CALIB_TX]`，ACK 使用 `[PROTO]`，校准流程使用
`[HOST_CALIB]` / `[CALIB_WORKFLOW]`。这些记录统一写入会话日志，
不再创建独立的 `logs/auto_calib` 文件。

示例（仅说明格式，不代表已捕获的故障）：

```text
[SERVO READ FAIL] bus=2 id=5 mode=probe reason=motor_status err=0 status=0x04 status_valid=1 data=15/15 reply_len=17 bus_rx=0/0 elapsed_ms=1 seq=0 suppressed=0
```

| 字段 | 含义 |
| --- | --- |
| `bus`, `id` | 从 0 编号的物理 BUS 与舵机协议 ID |
| `mode` | `probe` 单播完整反馈探测；`sync` 周期批量同步读 |
| `reason` | `timeout` 无响应或回包不完整；`checksum` 校验失败；`wrong_id` 单播回包 ID 不符；`length` 长度字段不符；`motor_status` 完整有效回包带非零状态 |
| `err` | SDK 通信错误码：0 无通信错误，1 无完整响应，2 校验失败，3 ID 不符，4 长度不符 |
| `status`, `status_valid` | 原始电机状态字；只有 `status_valid=1` 才来自完整且校验正确的本次回包，0 时不能当作电机状态证据 |
| `data` | 本次实际收到的数据区字节数 / 期望字节数，不含协议头和校验字节 |
| `reply_len` | 收到的原始 LENGTH 字段；未收到完整回复头时为 0 |
| `bus_rx` | 同步读本轮整条 BUS 的缓冲字节数 / 期望字节数；单播不统计，显示 0/0 |
| `elapsed_ms` | 从请求开始到本次收集的经过时间；包括任务调度间隔 |
| `seq` | 同步反馈轮次；单播为 0 |
| `suppressed` | 上一条日志之后因限频而没有输出的失败记录数量 |

同一 BUS/ID 最多每 5 秒输出一次。扫描会跨四条 BUS 查找相同 ID，
因此不在该 BUS 上的 ID 出现 `timeout` 属于正常探测结果。
应结合实际接线、路由和同一组电机的失败模式判断，不能仅凭一条超时认定硬件故障。

诊断不会额外执行 Ping、改变读超时或在线门槛，不扩展二进制协议。
同步读的文本输出安排在四条 BUS 收集完成之后，使用现有零等待日志锁。
文本仍可能因串口竞争丢弃，`suppressed` 只统计限频，不统计传输丢失。
