# 下面将详细介绍App的功能
# 您应该使用React Native 来运行该代码


## 一、核心预警功能

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **MQTT 实时接收** | `connectMQTT()` | 通过 WebSocket 连接 MQTT 服务器，订阅 `earthquake/alert` 主题 |
| **震中自动获取** | `client.on('message')` | 从告警 JSON 中提取 `epicenter_lat/lng`，自动更新震中坐标 |
| **单节点震中推算** | `estimateEpicenterFromNode()` | 当收到 `azimuth` + `node_lat/lng` + `distance` 时，推算出震中位置 |
| **S 波到达倒计时** | `calculateDistance()` + `eta = distance / 3.5` | 根据用户 GPS 位置和震中坐标，计算震中距和 S 波预计到达时间 |
| **全屏预警弹窗** | `showAlert` 状态 | 红色全屏弹窗，显示烈度、震中距、预计到达秒数 |
| **系统级通知** | `sendNotification()` | 通过 Notifee 库发送 Android/iOS 系统通知，支持前台服务 |
| **烈度阈值过滤** | `minIntensityRef.current` | 低于设定烈度的告警不弹窗、不通知 |

---

## 二、MQTT 通信与消息处理

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **多种消息类型兼容** | `client.on('message')` 中的 `switch` 逻辑 | 兼容 `trigger`（4G 报警）、`earthquake_alert`（服务器融合）、`external_alert`（CENC）、`state_change`（预警解除）等多种类型 |
| **钉钉格式解析** | 正则匹配 `text` 字段 | 从钉钉推送的文本中提取置信度和烈度 |
| **自动重连** | `reconnectPeriod: 5000` | MQTT 断开后每 5 秒自动重连 |
| **后台任务** | `startBackgroundTask()` | 启动 React Native 后台任务，保持 MQTT 连接 |

---

## 三、用户界面与交互

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **震中手动输入** | `inputLat` / `inputLon` + `saveEpicenter()` | 用户可以手动输入经纬度设置震中 |
| **震中来源显示** | `epicenterSource` 状态 | 显示震中来自"MQTT 自动获取"还是"手动输入" |
| **定位方式显示** | `epicenterMethod` 变量 | 显示"服务器融合定位"、"单节点方位角推算"或"网关位置参考" |
| **预警阈值设置** | `minIntensity` + `saveConfig()` | 用户可设置最小预警烈度（0-12） |
| **实时日志** | `logs` 数组 | 显示 MQTT 连接状态、消息接收、位置获取等实时日志 |
| **震中持久化** | `saveEpicenterToFile()` / `loadEpicenterFromFile()` | 震中坐标和配置保存到本地文件，重启不丢失 |

---

## 四、数据持久化

| 功能 | 实现位置 | 说明 |
|------|----------|------|
| **震中坐标存储** | `EPICENTER_FILE` | 保存震中经纬度和来源（auto/manual）到 JSON 文件 |
| **配置存储** | `CONFIG_FILE` | 保存最小预警烈度到 JSON 文件 |
| **启动自动加载** | `useEffect` → `loadAll()` | App 启动时自动读取上次保存的震中和配置 |

---

## 功能分类总览

| 大类 | 核心功能 |
|------|----------|
| **预警功能** | MQTT 接收、震中自动提取、单节点推算、S 波倒计时、全屏弹窗、系统通知 |
| **消息处理** | 多类型消息兼容、钉钉格式解析、自动重连、后台保活 |
| **用户界面** | 震中手动输入、来源显示、阈值设置、实时日志、定位方式显示 |
| **数据持久化** | 震中坐标存储、配置存储、启动自动加载 |

