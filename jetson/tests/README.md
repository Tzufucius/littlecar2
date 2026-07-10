# tests 目录说明

本目录保存 Jetson 上位机 Python 代码的测试。测试分为两类：

- `test_protocol_frame.py`：离线单元测试，覆盖帧编码、CRC、ACK 解析和字节流解析。
- `test_protocol_client.py`：内存串口仿真测试，覆盖当前 STM32 已实现的 SYSTEM、SAFETY 和全部 CHASSIS 高层方法及其 Payload。
- `test_protocol_hardware.py`：实车串口联调测试，默认跳过，供上板后验证 ACK 和查询响应。

运行方式：

```powershell
python -m unittest discover -s tests
```

执行实车通信检查：

```powershell
$env:STM32_SERIAL_PORT = "COM8"       # Jetson 示例：/dev/ttyTHS1
python -m unittest tests.test_protocol_hardware -v
```

只有在车辆已架空或现场满足安全条件时，才允许执行包含运动命令的检查：

```powershell
$env:STM32_ENABLE_MOTION_TESTS = "1"
python -m unittest tests.test_protocol_hardware -v
```
