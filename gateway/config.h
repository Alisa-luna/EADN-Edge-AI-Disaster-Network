#ifndef CONFIG_H
#define CONFIG_H

// ==================== 网关身份 ====================
#define GATEWAY_ID       0xFFFF

// ==================== Wi-Fi ====================
// 默认值，会被FFat中的配置覆盖
#define DEFAULT_WIFI_SSID     "iPhone"
#define DEFAULT_WIFI_PASS     "Lq18992475777"
#define WIFI_RETRY_DELAY      10000   // 重连间隔 (ms)
#define WIFI_RETRY_COUNT      5       // 最大重试次数

// ==================== LoRa ====================
#define LORA_FREQ        915E6
#define LORA_SF          12
#define LORA_BW          125E3
#define LORA_CR          5
#define LORA_TX_POWER    17
#define LORA_PREAMBLE    8

// ==================== PCA ====================
#define PCA_INPUT_DIM    300
#define PCA_OUTPUT_DIM   20

// ==================== AI 阈值 ====================
#define WINDOW_SIZE      300
#define YELLOW_THRESHOLD 0.5f
#define ORANGE_THRESHOLD 0.7f
#define VOTE_COUNT       2

// ==================== 节点管理 ====================
#define MAX_NODES        50
// 单节点报警阈值（多个节点同时报警才触发网关级警报）
#define GATEWAY_ALERT_MIN_NODES  2
#define NODE_TIMEOUT_SEC         120

// ==================== DTU ====================
#define DTU_SERIAL       Serial1
#define DTU_BAUDRATE     115200
#define DTU_RX           16
#define DTU_TX           17

// 默认钉钉token（可被Web配置覆盖）
#define DEFAULT_DINGTALK_TOKEN  "8d8aaaf366694452a076974b719cbc35ee1efd133a41cc5c3a2fe3462ec32d63"


// ==================== 系统 ====================
#define SERIAL_BAUD      115200
#define HEARTBEAT_INTERVAL  60000   // 心跳间隔 (ms)
#define NEIGHBOR_BROADCAST_INTERVAL 60000  // 邻居表广播间隔
#define DTU_LOCATION_INTERVAL 300000      // DTU定位间隔 (5分钟)

// ==================== 网络状态枚举 ====================
enum NetworkState {
    NET_DISCONNECTED,
    NET_WIFI,
    NET_4G_FALLBACK      // 网关特有：DTU 4G回退
};
// ==================== DTU 合宙 Air780EPM config 协议 ====================
#define DTU_SERIAL          Serial2
#define DTU_BAUDRATE        115200
#define DTU_RX              16
#define DTU_TX              17

// config 命令
#define DTU_CMD_AT          "AT\r\n"
#define DTU_CMD_LOC_ENABLE  "config,set,location,1,1,60,2,${GPSLON},${GPSLAT},1,0\r\n"
#define DTU_CMD_LOC_QUERY   "config,get,lbsloc\r\n"
#define DTU_CMD_SMS_TEXT    "AT+CMGF=1\r\n"
#define DTU_CMD_SMS_CSET    "AT+CSCS=\"GSM\"\r\n"

// DTU 指令（DTU固件用 config 格式）
#define DTU_CMD_AT            "config,get,ssta\r\n"
#define DTU_TIMEOUT_AT        2000
#define DTU_CMD_LOC_ENABLE    "config,set,location,1,1,60,0,0,0,1,0\r\n"
#define DTU_TIMEOUT_CMD       3000
#define DTU_CMD_LOC_QUERY     "config,get,lbsloc\r\n"
#define DTU_TIMEOUT_LOC       60000
#define DTU_CMD_CSQ           "config,get,csq\r\n"
#define DTU_CMD_NETTIME       "config,get,nettime\r\n"
#define DTU_CMD_SAVE          "config,set,save\r\n"
#define DTU_LOC_INTERVAL      300000

// 超时
#define DTU_TIMEOUT_AT      1000
#define DTU_TIMEOUT_CMD     3000
#define DTU_TIMEOUT_LOC     60000   // 基站定位最长60秒
#define DTU_LOC_INTERVAL    300000  // 手动查询定位间隔（5分钟）

// 新增全局变量
bool     dtuLocationEnabled = false;
uint32_t lastDTULocRequest  = 0;

#endif