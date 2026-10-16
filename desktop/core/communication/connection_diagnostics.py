"""连接失败的只读诊断分类。

职责：将传输观测转成连接失败原因，不执行恢复动作。
线程：应用操作线程调用，无后台线程。
输入：当前连接的字节数、帧数、协议状态及错误。
输出：可读的故障说明。
状态所有权：不持有连接或设备状态。
禁止依赖：禁止依赖 UI、硬件控制、设备复位及文件写入。
"""


def describe_connection_failure(diag):
    port = str(diag.get('port', 'P4'))
    if diag.get('error'):
        return f'{port}: serial transport failed: {diag["error"]}'
    if not diag.get('rx_bytes', 0):
        return f'{port}: USB opened, but P4 sent no bytes within 10 seconds.'
    if not diag.get('rx_packets', 0):
        return f'{port}: received {diag["rx_bytes"]} bytes, but no valid P4 protocol frames.'
    if not diag.get('protocol_compatible', False):
        return f'{port}: protocol frames received, but the protocol handshake was not accepted. Check version and device identity.'
    return f'{port}: protocol handshake succeeded, but no fresh system status arrived within 10 seconds.'
