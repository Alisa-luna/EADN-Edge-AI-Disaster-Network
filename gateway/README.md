**以下将详细介绍当前网关具有的功能**


## 一、多模通信与 Mesh 网络

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **LoRa 接收** | `handleLoRaRX()` | 解析帧头、类型、长度、RSSI，支持多种帧类型 |
| **LoRa 发送** | `sendLoRaPacket()` / `sendLoRaFromQueue()` | 通过队列异步发送，含 AUX 忙检测 |
| **中继去重** | `isRelayDuplicate()` / `recordRelay()` | 10 秒窗口内同一事件不重复中继 |
| **时间同步广播** | `broadcastTimeSync()` | 将 NTP/4G 时间通过 LoRa 广播给所有节点 |
| **网关位置广播** | `broadcastGatewayPosition()` | 将自己的 GPS/LBS 坐标广播给节点 |
| **邻居列表广播** | `broadcastNeighborList()` | 每 60 秒广播已注册节点拓扑 |
| **ACK 确认** | `FRAME_DATA_PRED` 分支末尾 | 收到地震帧后回复 ACK |
| **RSSI 测距** | `rssiToDistance()` | 根据信号强度估算节点距离 |

---

## 二、多源数据融合与验证

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **帧解析** | `FRAME_DATA_PRED` 分支 | 解析 37 字节地震参数帧（τc、S‑P、Pd、方位角等） |
| **单节点多指标评分** | `compute_single_node_score()` | AI/AE 一致性、τc‑sPeak 物理一致性、S‑P 距离一致性、时间一致性 |
| **事件缓冲区** | `addNodeToEvent()` / `EventRecord` | 5 秒时间窗口内收集多个节点的报警 |
| **多节点融合定位** | `perform_event_fusion()` | 网格搜索最小二乘法推算震中，融合震级和置信度 |
| **事件过期处理** | `checkEventExpiry()` | 超时未收到新节点则强制融合输出 |
| **单节点高置信度触发** | 融合条件判断 | 单节点评分 > 0.8 时可独自触发融合 |
| **低分过滤** | `single_score < 0.3f` | 评分过低的节点直接丢弃，不进入事件缓冲 |

---

## 三、双通道告警推送

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **WiFi 模式 MQTT 全量上报** | `FRAME_DATA_PRED` 分支 | 将地震参数完整 JSON 发到 `earthquake/data` |
| **WiFi 模式 MQTT 告警** | 同上 | 高烈度时同时发到 `earthquake/alert` |
| **4G 模式融合告警** | `send_fused_alert_dtu()` | 通过 DTU `bbb3` 通道发送融合后的 JSON |
| **4G 模式钉钉通知** | `send_fused_alert_dingtalk()` | 通过 DTU `bbb1` 通道发送钉钉 Markdown 消息 |
| **4G 模式分级告警** | 钉钉消息中 | 根据融合置信度和节点数自动分级（红/橙/黄） |
| **MQTT 心跳** | `core1Task` 中定时发送 | 定期上报网关状态、节点数、网络模式等 |

---

## 四、网关自身定位与节点坐标管理

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **GPS 定位** | `dtuSendCmdAsync("config,get,gpsext")` | 通过 DTU 获取 GPS 坐标，WGS84→GCJ02 转换 |
| **LBS 基站定位** | `dtuSendCmdAsync(DTU_CMD_LOC_QUERY)` | GPS 失败时降级为基站定位 |
| **WiFi 海拔获取** | `fetchAltitude()` | 通过 OpenTopoData API 获取海拔 |
| **DTU 异步海拔** | `requestAltitude()` | 4G 模式下通过 DTU 的 `bbb2` 通道请求海拔 |
| **节点位置自动纠正** | `autoCorrectNodePosition()` | 根据 RSSI 距离反向修正节点坐标， ≥3 个已知节点时通过三点定位验证节点坐标，并通过 LoRa 下发 |
| **网关位置自校验** | `verifyGatewayPosition()` | 用 ≥3 个已知节点反向验证网关自身坐标 |
| **部署模式** | `handleDeploymentClients()` | 接收节点 TCP 注册，分配 IP，返回邻居信息 |

---

## 五、Web 管理后台（人机接口）

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **WiFi AP / STA** | `NetworkManager` | 自动切换 WiFi 客户端或 AP 模式 |
| **Web 页面** | `generateWebPage()` | 响应式管理界面（深色主题） |
| **节点表格** | `/api/nodes` + 前端 JS | 展示在线状态、坐标、RSSI、距离等 |
| **拓扑视图** | `/api/nodes` + 前端渲染 | ASCII 树形图展示网关与节点连接关系 |
| **配置管理** | `/saveConfig` | 修改 WiFi、钉钉 Token、阈值并重启 |
| **地图选点** | 前端 JS + 高德 API | 搜索地址、微调坐标、获取海拔，预分配节点 |
| **导航功能** | 前端 JS | 调起高德地图导航到指定节点 |
| **消息广播** | `/sendGwMsg` | 通过 LoRa 向单个或所有节点发送文本消息 |
| **部署控制** | `/startDeploy` 等 | 一键开启/关闭部署 AP，广播拓扑 |

---

## 六、时间与系统管理

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **WiFi NTP 同步** | `syncNTP()` | 从阿里云 NTP 获取 UTC+8 时间 |
| **4G 网络时间** | `DTU_CMD_NETTIME` 回调 | 解析 DTU 返回的时间字符串，设置系统时钟 |
| **持久化配置** | `saveConfig()` / `loadConfig()` | FFat 保存 WiFi、钉钉等配置 |
| **看门狗** | `esp_task_wdt_init(60)` | 60 秒硬件看门狗 |
| **任务统计** | `core1Task` 定时输出 | 每分钟打印数据包、告警、内存、堆栈余量 |

---

### 网关功能分类总览

| 大类 | 核心功能 |
|------|----------|
| **多模通信** | LoRa 收发、中继去重、ACK、RSSI 测距、时间/位置广播 |
| **数据融合** | 单节点多指标评分、事件缓冲、多节点网格搜索定位、震级融合 |
| **告警推送** | WiFi: MQTT 全量+告警；4G: DTU 融合告警+钉钉通知+心跳 |
| **定位管理** | 网关 GPS/LBS/WiFi 定位、海拔获取、节点坐标自动纠正 |
| **Web 管理** | 响应式页面、节点管理、拓扑视图、地图选点、配置、消息 |
| **系统管理** | NTP/4G 时间同步、配置持久化、看门狗、状态监控 |

### 网关web界面展示（仅部分展示）
<img width="660" height="1434" alt="IMG_4545 (1)" src="https://github.com/user-attachments/assets/a937b778-3888-4e94-8205-464d339e3533" />
