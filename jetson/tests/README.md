# tests 目录说明

本目录保存 Jetson 上位机 Python 代码的轻量单元测试。当前测试重点覆盖 `protocol` 包的帧编码、CRC、ACK 解析、Payload 小端编码和字节流解析行为。

运行方式：

```powershell
python -m unittest discover -s tests
```
