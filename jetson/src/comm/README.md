# comm 目录说明

本目录存放 Jetson 与 STM32 的通信代码。

- `interface.py`：通信客户端抽象接口
- `mock_client.py`：无硬件环境下的 mock 客户端
- `serial_client.py`：基于 `pyserial` 的真实串口客户端
- `protocol.py`：`0x5A 0xA5` 帧格式、CRC16 和解析工具
