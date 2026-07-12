# EADN

## Edge AI Enabled Self-Organizing LoRa Network for Earthquake Monitoring and Emergency Response

**EADN** 是一个面向低成本快速部署场景的边缘智能灾害监测网络。

系统结合 ESP32‑S3 边缘计算节点、LoRa 自组织通信网络、4G/WiFi 双模网关以及云端多源融合平台，实现从现场振动感知、边缘 AI 判断、无线组网、震源定位到应急告警的完整链路。当前版本主要面向**低成本分布式地震监测与研究验证**场景，探索边缘 AI、低功耗无线网络和多节点融合技术在地震早期预警中的应用。系统同时具备灾害环境下的自治通信、节点定位和信息广播能力。

---

## 系统架构

```
                    User Layer
           App / Web / Emergency Message
                        │
                 Cloud Service (Python)
         (Fusion / Database / CENC / ShakeMap)
                        │
            Gateway Emergency Hub (ESP32‑S3)
           WiFi / 4G / Local Fusion / DTU
                        │
           LoRa Self‑Organizing Network
        ┌──────────┬──────────┐
     Node 1     Node 2     Node 3  (ESP32‑S3 + MPU6050)
        │
 ┌──────┴──────────┐
 │   Edge AI        │
 │ AE + CNN         │
 │ Dual Encoder     │
 │ P‑Azimuth        │
 └─────────────────┘
```

---

## 核心特性

### 1. Edge AI 地震感知

节点通过 MPU6050 传感器以 **100 Hz** 采集三轴加速度。自编码器 (AE) 在本地学习环境振动基线，一旦振动模式超出正常范围，立即触发 Edge Impulse **CNN 地震确认**。确认后由双编码器神经网络快速估算 **τc**、**S‑P 走时** 和 **峰值加速度 Pd**，同时结合 P 波初动分析计算 **P 波方位角**，实现单站粗定位。

### 2. 自组织 LoRa 灾害网络

节点通过 **DX‑LR22 LoRa 模块** 组成轻量自治网络，支持：

- 节点自动发现与注册
- 多跳中继转发（TTL 去重）
- ACK 确认与异步重传
- **NTP 时间同步广播**
- 节点间短消息收发

### 3. 无公网自治运行

网关在无 WiFi 时自动切换 **4G DTU** 备份链路，并**在本地运行多节点融合算法**，独立完成震中网格搜索定位、烈度估算和报警推送。即使互联网完全中断，LoRa 网络内部仍可维持节点管理、事件融合和本地告警。

### 4. 多源定位与校验

系统综合 **GPS**、**LBS**、**RSSI 测距**、**三边定位** 和 **P 波方位角推算**，实现多节点交叉验证。网关可反向校正节点坐标，服务器支持基于节点观测数据的网关位置一致性校验。节点会定时上报自己到网关的RSSI测距，网关二次测距并与该结果进行对比，形成**双向验证**，防止RSSI通信链路受干扰。

### 5. 云端融合与官方数据联动

服务器 (Python) 接收并存储所有事件到 **SQLite**，多节点触发时通过网格搜索最小二乘法定位震中，生成烈度分布 HTML 地图。同时接入**国家地震科学数据中心 (CENC) 实时烈度速报 API**，当官方速报事件可能影响部署区域时，系统可生成辅助提示信息。

### 6. 多渠道告警与应急交互

- **MQTT** 推送至手机 App（实时显示震中距和 S 波到达倒计时）
- **钉钉机器人** 分级推送（黄 / 橙 / 红）
- **QQ 邮箱** HTML 邮件告警
- **Web 管理页面**：节点和网关均提供 WiFi AP + 网页界面，可查看运行状态、配置参数
- **导航功能**：Web 界面可调起高德地图，支持节点间双向导航

### 7. 关键参数传输架构

传统方案通常需要上传连续波形，由服务器完成分析。EADN 将分析过程前移至边缘节点，仅上传地震事件关键参数：

- τc（特征周期）
- S‑P 走时差
- 峰值加速度 Pd
- AI 置信度
- AE 异常分数
- P 波方位角

单个事件帧约 **37 字节**，大幅降低 LoRa 网络带宽压力，使低速远距离无线通信成为可能。

---

## 硬件要求

| 组件 | 型号 | 用途 |
|------|------|------|
| 节点 MCU | ESP32‑S3 (需 PSRAM) | AI 推理 + LoRa 通信 |
| 加速度计 | MPU6050 (DMP 模式) | 100 Hz 三轴振动采集 |
| 节点 LoRa | DX‑LR22 (UART) | 远距离数据传输 |
| 网关 MCU | ESP32‑S3 | LoRa 接收 + 4G DTU 控制 |
| 4G DTU | Air780EPM 或类似 | 蜂窝网络备份与钉钉透传 |

---

## 快速开始

1. 安装 Arduino IDE 和 ESP32‑S3 开发板支持。
2. 将所有依赖库放入 `libraries/`（MPU6050, PubSubClient, Edge Impulse SDK, ArduinoJson 等）。
3. 将 `node/` 目录下的代码烧录到节点 ESP32‑S3。
4. 将 `gateway/` 目录下的代码烧录到网关 ESP32‑S3。
5. 根据实际网络环境修改 `config.h` 中的 WiFi、MQTT 服务器、钉钉 Token、节点坐标等参数。
6. 上电后节点自动建立 AP (`EQ_Node_ID`, 密码 `12345678`)，手机可连接并访问 `192.168.4.1` 查看状态。网关自动连接 WiFi 或启动 AP (`EQ_Gateway`) 供配置。

---

## 目录结构

```
├── node/                 # 节点端 Arduino 代码
│   ├── sketch_node.ino   # 主程序
│   ├── ae_model.h/cpp    # 自编码器模型及基线管理
│   ├── ae_weights.h      # AE 预训练权重
│   ├── dual_encoder_*.h  # 双编码器 (τc/S-P) 权重
│   └── config.h          # 节点配置
├── gateway/              # 网关端 Arduino 代码
│   ├── sketch_gateway.ino
│   └── NetworkManager.h  # WiFi/AP/4G 网络管理
├── server/               # 云端 Python 服务
│   └── eq_server.py      # MQTT 接收、事件管理、CENC 监听
├── models/               # 模型训练与导出脚本，包含有对边缘AI的一些探索
├── app/                  # React Native 手机 APP
├── docs/                 # 文档与图片
└── README.md
```

---

## 工作原理

### 环境学习
节点上电后静默 30 秒，收集当前小时的振动特征，建立聚类基线。自编码器持续计算异常分数。

### 异常触发
当异常分数超出基线范围时，节点触发 CNN 地震确认。

### AI 确认与参数估算
CNN 模型分类为地震后，双编码器快速估算 τc、S‑P 和 Pd，同时计算 P 波方位角。

### 关键参数传输
37 字节事件帧通过 LoRa Mesh 广播，邻居节点中继转发，TTL 防止循环。

### 网关处理
- **WiFi 模式**：MQTT 全量上传服务器，进行多节点定位和外部告警。
- **4G 模式**：网关本地运行融合算法，直接推送告警。

### 云服务器
- SQLite 存储事件
- 网格搜索定位震中，生成烈度图
- 定期接入 CENC 目录，评估本地影响并生成辅助提示

### 手机 App
订阅 `earthquake/alert`，获取 GPS 坐标，计算震中距和 S 波倒计时，全屏弹窗 + 系统通知。

---

## 致谢

- **MCU‑Quake** — *Real‑time discrimination of earthquake signals by integrating AI technology into IoT devices*, Zhi Geng et al., 2025 | CC BY 4.0 | https://www.nature.com/articles/s43247-025-02003-y
- **PhaseNet / EQTransformer** — [SeisBench](https://github.com/seisbench/seisbench) | MIT License
- **CENC 实时烈度速报 API** — 数据来源：[国家地震科学数据中心](https://api-cencint-public.nowquake.cn)

---

## 许可证

本项目采用 **MIT License**。使用前请阅读各依赖模型和数据源的使用条款。

---

## 免责声明

本系统仅供**研究、实验和教育目的**，不可作为正式的地震预警设备使用。实际地震预警请以**中国地震局**或当地应急管理部门发布的官方信息为准。
