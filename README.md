**基于边缘智能的广域地震监测网络 (EEW System)**

本系统已从全量波形传输更新到仅关键参数传输的架构。节点端集成自编码器 (AE) 异常检测、双编码器神经网络 (τc / S-P / Pd 快速估算) 及 P 波方位角计算，实现单站粗定位。网关支持无公网环境下的独立多节点融合与震中定位。服务器已对接国家地震科学数据中心 (CENC) 实时烈度速报 API，可主动发布官方地震对当地的预估影响。

---

**系统架构**

```
节点 (ESP32-S3 + MPU6050) ──LoRa Mesh──→ 网关 (ESP32-S3 + 4G DTU) ──MQTT/4G──→ 云端 (Python)
     │                                           │                              │
     ├── 100Hz 加速度采集                       ├── 多节点融合定位               ├── CENC 实时目录接入
     ├── AE 自学习异常检测                      ├── WiFi: MQTT 全量上报           ├── 震中网格搜索
     ├── Edge Impulse CNN 地震确认              ├── 4G: 本地融合 + 手机App/钉钉/邮件     ├── 烈度分布图生成
     ├── 双编码器 τc/S-P/Pd 估算                └── LoRa 时间广播                 ├── 历史事件数据库
     ├── P 波方位角计算                                                                 └── 手机 App / 钉钉 / 邮件告警
     └── LoRa 关键参数帧传输 (37字节)
```

**核心特性**

1、自学习异常检测：每个节点开机后采集 30 秒环境振动，建立分时段统计基线；自编码器 (AE) 实时监控，一旦振动模式超出基线范围，立即触发后续分析。

2、多级 AI 确认：AE 异常 → Edge Impulse CNN 地震分类 → 双编码器神经网络快速估算 τc、S-P 走时、峰值加速度 Pd 及 P 波方位角。

3、单站粗定位：利用 P 波方位角与震中距，节点可大致推算震中位置；多节点数据汇聚后，网关或服务器通过网格搜索交叉定位。

4、关键参数传输：LoRa 帧仅包含 37 字节地震参数（τc、S-P、Pd、方位角、AI 置信度、AE 分数等），大幅降低带宽需求。

5、断网独立运行：网关在无 WiFi 时自动切换 4G DTU 备份链路，并运行本地多节点融合算法，独立完成报警推送。

6、官方数据联动：服务器接入国家地震科学数据中心 API，实时获取官方烈度速报，对可能影响本地的地震主动发布预警。

7、多渠道告警：通过 MQTT 推送至手机 App、钉钉机器人、QQ 邮箱；手机 App 实时显示震中距和 S 波到达倒计时。

8、时间同步：通过 WiFi NTP 或 4G 网络时间获取 UTC，并通过 LoRa 广播至所有节点，保证时间戳一致。

9、Web 管理页面：节点和网关均提供 WiFi AP + 网页界面，可查看运行状态、配置 WiFi/钉钉 Token 等。

10、导航与通信网络：网络借助Lora提供节点内短消息发送能力，可通过Web界面进行指定/全网广播发送；Web界面可调起高德地图导航，支持参与点之间的双向导航。

**快速开始**

*硬件要求*

组件 型号 用途
节点 MCU ESP32‑S3 (需 PSRAM) AI 推理 + LoRa 通信
加速度计 MPU6050 (DMP 模式) 100Hz 三轴振动采集
节点 LoRa DX‑LR22 (UART) 远距离数据传输
网关 MCU ESP32‑S3 LoRa 接收 + 4G DTU 控制
4G DTU Air780EPM 或类似 蜂窝网络备份与钉钉透传

*烧录步骤*

1. 安装 Arduino IDE 和 ESP32‑S3 开发板支持。
2. 将所有依赖库放入 libraries/（MPU6050, PubSubClient, Edge Impulse SDK, ArduinoJson 等）。
3. 将 node/ 目录下的代码烧录到节点 ESP32‑S3。
4. 将 gateway/ 目录下的代码烧录到网关 ESP32‑S3。
5. 根据实际网络环境修改 config.h 中的 WiFi、MQTT 服务器、钉钉 Token、节点坐标等参数。
6. 上电后节点会自动建立 AP (EQ_Node_ID, 密码 12345678)，手机可连接并访问 192.168.50.1 查看状态。网关会自动连接 WiFi 或启动 AP (EQ_Gateway) 供配置。

*目录结构*

```
├── node/                 # 节点端 Arduino 代码
│   ├── sketch_node.ino   # 主程序
│   ├── ae_model.h/cpp    # 自编码器模型及基线管理
│   ├── ae_weights.h      # AE 预训练权重
│   ├── dual_encoder_*.h  # 双编码器 (τc/S-P) 权重
│   └── config.h          # 节点配置
├── gateway/              # 网关端 Arduino 代码
│   ├── sketch_gateway.ino
│   ├── NetworkManager.h  # WiFi/AP/4G 网络管理
│   
├── server/               # 云端 Python 服务
│   ├── eq_server.py      # MQTT 接收、事件管理、CENC 监听
│   └── earthquake.db     # SQLite 事件数据库（自动生成）
├── models/Dual_PhyNet    # 模型训练与导出脚本
│   ├── train_ae.py       # AE 离线训练
│   ├── export_weights.py # 权重转 C 头文件
│   ├── train.py          # 双编码器训练文件
│   ├── test.py           # 双编码器测试文件
│   └── generate_data.py  # 合成训练数据
├── app/                  # React Native 手机 APP
├── docs/                 # 文档与图片
└── README.md
```

*工作原理*

1. 环境学习
   
   节点上电后静默 30 秒，收集当前小时的振动特征，计算中位数和相对范围（聚类基线）。自编码器持续将输入窗口重建，计算高频重建误差、latent 距离和频谱误差，并融合为一个异常分数。
   
2. 异常触发
   
   当异常分数超出基线范围（低于下界或高于上界）时，节点认为环境发生显著改变，触发 CNN 地震确认。
   
3. AI 确认与参数估算
   
   Edge Impulse 部署的 CNN 模型（MCU‑Quake）对异常窗口进行分类；若置信度 > 0.5，则调用双编码器神经网络快速输出 τc（特征周期）、S‑P 走时差、峰值加速度 Pd。同时计算 P 波初动方位角。
   
4. LoRa 数据传输
   
   37 字节关键参数帧通过 LoRa 广播，包含：NTP 时间戳、s_peak、τc、S‑P、震中距、预警时间、AI 置信度、AE 异常分数、P 波方位角、TTL。邻居节点会中继转发，TTL 递减以防止循环。
   
5. 网关处理
   
   · WiFi 模式：将完整 JSON 通过 MQTT 上传至服务器，由服务器进行多节点定位、烈度图生成和外部告警。
   
   · 4G 模式：网关自身运行融合算法，利用多个节点的距离和方位角进行三角定位，直接通过 DTU 发送钉钉/邮件告警。
   
6. 云服务器

   · 接收并存储所有事件到 SQLite。
   
   · 多节点触发时，通过网格搜索最小二乘法定位震中，生成烈度分布 HTML 地图。
   
   · 每分钟拉取 CENC 最新地震目录，若官方地震对本地预估烈度 ≥ IV 度，主动发布告警。
   
7. 手机 App
   
   订阅 MQTT earthquake/alert 主题，收到告警后获取用户 GPS 坐标，计算震中距和 S 波到达时间，以全屏弹窗和系统通知的方式展示倒计时。
   

*致谢*

· MCU‑Quake 模型
  Real‑time discrimination of earthquake signals by integrating AI technology into IoT devices
  Zhi Geng, Yanfei Wang, Wenyong Pan, Caixia Yu, Zhijing Bai, Hongzhou Zhang
  Communications Earth & Environment, 2025 | CC BY 4.0
  
  模型代码：ScienceDB
  
· PhaseNet / EQTransformer

  预训练模型来自 SeisBench | MIT License
  
· CENC 实时烈度速报 API

  数据来源：国家地震科学数据中心
  

*许可证*

本项目采用 MIT License。使用前请阅读各依赖模型和数据源的使用条款。

*免责声明*

本系统仅供研究、实验和教育目的，不可作为正式的地震预警设备使用。实际地震预警请以中国地震局或当地应急管理部门发布的官方信息为准。
