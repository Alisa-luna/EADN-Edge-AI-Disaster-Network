# 早期版本            ⚠️ **已停止维护**

基于 STM32 + ESP8266 的第一代地震预警系统,灵感和标准库来源于江协科技例程:硬件读取I2C(原视频链接:https://www.bilibili.com/video/BV1th411z7sn?p=35&vd_source=d311fe493f2317e5df5bf8d69a1c8387)

- **STM32F103**：负责 MPU6050 传感器数据采集（I2C 寄存器级驱动）
- **ESP8266**：负责 WiFi 连接和 MQTT 数据透传（TLS 加密）
- **云端 Python**：传感器数据趋势分析 + SiliconFlow LLM AI 模拟震相拾取

目前，该系统直到该版本一共进行了12次不同程度的迭代，已经进行了6个月的改进。

此版本已被 ESP32 双核方案取代，保留作为项目演进参考。

