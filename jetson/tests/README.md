# tests 目录说明

本目录包含 Jetson 视觉函数和协议包测试：

- `test_vision.py`：测试同心圆数字识别、传统彩色物料检测、二维码空输入和 YOLO 参数边界。
- `test_protocol_frame.py`：测试协议帧编码、CRC、ACK 和字节流解析。
- `test_protocol_client.py`：使用内存串口测试协议客户端及其高层接口。
- `test_protocol_hardware.py`：真实串口联调测试，默认跳过。

运行全部测试：

```powershell
conda run -n low_numpy python -m unittest discover -s tests -v
```

真实硬件测试前必须确认串口、供电和运动安全条件。
