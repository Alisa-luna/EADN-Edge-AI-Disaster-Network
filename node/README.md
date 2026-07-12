**以下将会详细介绍当前节点所具备的所有能力**




## 一、地震预警功能（核心 AI 链）

这是节点最主要的功能，运行在 **Core 0** 上。

| 功能 | 实现函数 | 说明 |
|------|----------|------|
| **AE 异常检测** | `ae_predict()` | 自编码器学习正常环境，检测偏离 |
| **基线管理** | `ae_update_baseline()` | 每小时统计 30 个窗口的聚类基线 |
| **CNN 地震确认** | `runInference()` | Edge Impulse 模型（MCU-Quake）分类 |
| **双编码器参数估算** | `full_inference_chain()` | 输出 τc、S-P 走时、峰值加速度等 |
| **P 波方位角** | `computePAzimuth()` | 计算初动方向，用于单站粗定位 |
| **综合异常分数** | `core0Task` 内计算 | 高频误差 + latent 距离 + 频谱误差 |
| **地震帧发送** | `enqueueFrame(FRAME_DATA_PRED)` | 37 字节关键参数帧通过 LoRa 发出 |

---

## 二、LoRa 通信与 Mesh 网络

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **帧发送** | `enqueueFrame()` → `sendLoRaFromQueue()` | 通过队列异步发送 |
| **帧接收** | `handleLoRaRX()` | 解析帧头、长度、类型、RSSI |
| **中继转发** | `FRAME_DATA_PRED` 分支 | 收到其他节点地震帧 → 去重 → TTL 递减 → 转发 |
| **消息去重** | `isRelayDuplicate()` / `recordRelay()` | 10 秒窗口内同一事件不重复转发 |
| **邻居探测** | `FRAME_PROBE` / `FRAME_PROBE_RESP` | 每 60 秒探测周围节点 |
| **邻居列表广播** | `FRAME_NEIGHBOR_LIST` 处理 | 网关广播邻居拓扑，节点记录 |
| **位置广播** | `FRAME_POSITION` 处理 | 接收网关位置并计算距离 |
| **时间同步** | `FRAME_TIME_SYNC` 处理 | 通过 LoRa 接收 NTP 时间戳 |
| **消息收发** | `FRAME_MESSAGE` 处理 | 节点间文本消息 |
| **ACK 确认** | `case 0x0A` / `waitingForAck` | 异步等待 LoRa ACK |
| **RSSI 测距** | `rssiToDistance()` | 根据信号强度估算距离 |
| **心跳帧** | `enqueueFrame(0x0B)` | 每 60 秒发送心跳 |

---

## 三、定位与位置验证

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **单站震中粗定位** | `computePAzimuth()` + 距离 | P 波方位角 + 震中距 → 推算震中 |
| **三边定位自校验** | `verifyPositionByNeighbors()` | 用 3 个邻居距离加权平均推算自身位置 |
| **网关位置接收** | `FRAME_POSITION` 处理 | 接收网关 GPS 坐标 |
| **部署坐标确认** | `FRAME_DEPLOY_CONFIRM` 处理 | 网关反向验证后更新节点坐标 |
| **WiFi 注册** | `registerViaWiFi()` | 扫描部署 AP → 连接 → TCP 注册 → 获取网关坐标 |

---

## 四、Web 与本地人机接口

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **WiFi AP** | `WiFi.softAP("EQ_Node_ID")` | 节点自建热点，供手机连接 |
| **Web 管理页面** | `genWebPage()` | 显示位置、邻居、统计、日志 |
| **Web API** | `server->on(...)` | `/api/status`, `/api/neighbors`, `/api/time`, `/api/messages` |
| **消息发送** | `/sendMsg` | Web 页面发送 LoRa 消息 |
| **位置上报** | `/sendLocation` | 主动上报当前位置到网关 |
| **导航功能** | Web 页面 JS | 调起高德地图导航到网关/邻居 |
| **配置持久化** | `saveConfig()` / `loadConfig()` | FFat 文件保存坐标等配置 |

---

## 五、数据采集与预处理（Core 1）

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **100Hz 定时采集** | 硬件定时器 + ISR | 定时器中断触发，Core 1 读取 I2C |
| **DMP 姿态校正** | `mpu.dmpGetQuaternion()` | 四元数转旋转矩阵，加速度转到地面坐标系 |
| **毛刺过滤** | `max_abs_val < 3.0f` | 丢弃超过 3g 的异常值 |
| **历史环形缓冲** | `addToHistory()` | 10 秒 × 100Hz = 1000 点窗口 |
| **6 通道窗口构建** | `build_6ch_envelope_window()` | Z/N/E 加速度 + RMS 包络 + 归一化 |
| **窗口信号量** | `windowReadySemaphore` | 每 1 秒给 Core 0 发信号 |

---

## 功能分类总览

| 大类 | 核心功能 |
|------|----------|
| **预警检测** | AE 异常 → CNN 确认 → 双编码器参数 → 帧发送 |
| **LoRa 通信** | 发送/接收/中继/去重/邻居探测/时间同步 |
| **定位验证** | 单站方位角推算、三边校验、网关坐标接收 |
| **Web 接口** | AP 热点、管理页面、API、消息、导航 |
| **数据采集** | 100Hz 采样、DMP 校正、滤波、缓冲 |
