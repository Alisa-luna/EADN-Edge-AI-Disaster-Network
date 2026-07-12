#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <FFat.h>
#include <math.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_task_wdt.h>


#include "NetworkManager.h"
#include <USER01253-project-1_inferencing.h>
#include <sys/time.h>

// ==================== 网关身份 ====================
#define GATEWAY_ID 0xFFFF

// ==================== WiFi ====================
#define DEFAULT_WIFI_SSID "MIFI-BCC6"
#define DEFAULT_WIFI_PASS "1234567890"

// ==================== PCA ====================
#define PCA_INPUT_DIM 300
#define PCA_OUTPUT_DIM 20

// ==================== AI 阈值 ====================
#define WINDOW_SIZE 700
#define YELLOW_THRESHOLD 0.5f
#define ORANGE_THRESHOLD 0.7f
#define VOTE_COUNT 2

// ==================== 节点管理 ====================
#define MAX_NODES 50
#define GATEWAY_ALERT_MIN_NODES 2
#define NODE_TIMEOUT_SEC 120

// ==================== DTU 串口 ====================
#define DTU_SERIAL Serial1
#define DTU_BAUDRATE 115200
#define DTU_RX 16
#define DTU_TX 17

// ==================== DX-LR22 LoRa 串口 ====================
#define DXLR_SERIAL Serial2
#define DXLR_BAUDRATE 9600
#define DXLR_RX 6
#define DXLR_TX 5
#define DXLR_M0 20
#define DXLR_M1 21
#define DXLR_AUX 13

// ==================== DTU 命令 ====================
#define DTU_CMD_AT "config,get,ssta\r\n"
#define DTU_TIMEOUT_AT 2000
#define DTU_CMD_LOC_ENABLE "config,set,location,1,1,60,0,0,0,1,0\r\n"
#define DTU_TIMEOUT_CMD 3000
#define DTU_CMD_LOC_QUERY "config,get,lbsloc\r\n"
#define DTU_TIMEOUT_LOC 60000
#define DTU_CMD_CSQ "config,get,csq\r\n"
#define DTU_CMD_NETTIME "config,get,nettime\r\n"
#define DTU_CMD_SAVE "config,set,save\r\n"

// ==================== MQTT ====================
#define MQTT_SERVER "w8381dbc.ala.cn-hangzhou.emqxsl.cn"
#define MQTT_PORT 8883
#define MQTT_USER "User"
#define MQTT_PASSWORD "1234567890"
#define MQTT_TOPIC_DATA "earthquake/data"
#define MQTT_TOPIC_ALERT "earthquake/alert"
#define MQTT_TOPIC_HEARTBEAT "earthquake/heartbeat"
#define MQTT_CA_CERT \
  "-----BEGIN CERTIFICATE-----\n" \
  "MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n" \
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
  "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
  "MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
  "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
  "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n" \
  "9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n" \
  "2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n" \
  "1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n" \
  "q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n" \
  "tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n" \
  "vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n" \
  "BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n" \
  "5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n" \
  "1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n" \
  "NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n" \
  "Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n" \
  "8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n" \
  "pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n" \
  "MrY=\n" \
  "-----END CERTIFICATE-----"

// ==================== 帧协议 ====================
#define FRAME_HEADER 0xAA
#define FRAME_DATA_PRED 0x01
#define FRAME_DATA_HIST 0x02
#define FRAME_ALERT 0x03
#define FRAME_RELAY 0x04
#define FRAME_REGISTER 0x05
#define FRAME_NEIGHBOR_LIST 0x08
#define FRAME_PROBE_RESP 0x07
#define FRAME_POSITION 0x09
#define FRAME_DEPLOY_CONFIRM 0x10
#define FRAME_PRESENCE 0x20
#define FRAME_MESSAGE 0x30
#define FRAME_TIME_SYNC 0x11  // 新增帧类型

// ==================== 其他常量 ====================
#define QUANT_SCALE 0.001f
#define MAX_QUEUE 20
#define DXLR_MAX_PAYLOAD 200
#define DTU_LOC_INTERVAL 300000
#define SERIAL_BAUD 115200
#define HEARTBEAT_INTERVAL 60000
#define NEIGHBOR_BROADCAST_INTERVAL 60000
#define NTP_SYNC_INTERVAL 3600000  // NTP 同步间隔：1 小时
#define TX_QUEUE_LEN 20
// ==================== DTU 异步命令系统 ====================
#define DTU_CMD_QUEUE_SIZE 5

struct DTUCmd {
  char cmd[64];
  char matchKey[32];
  void (*callback)(const String& response);
};

QueueHandle_t dtuCmdQueue;
char dtuRxBuffer[512] = { 0 };
int dtuRxLen = 0;

const char* DTU_MATCH_AT = "ssta,ok";
const char* DTU_MATCH_LOC = "lbsloc,ok";
const char* DTU_MATCH_CSQ = "csq,ok";
const char* DTU_MATCH_NETTIME = "nettime,ok";
const char* DTU_MATCH_LOC_EN = "location,ok";
const char* DTU_MATCH_GPS = "gps,ok";

bool forceLocRequest = false;

// ==================== 数据结构 ====================
struct NodeInfo {
  uint16_t nodeID;
  float assignedLat, assignedLng, assignedAlt;
  float verifiedLat, verifiedLng;
  bool registered;
  bool positionVerified;
  uint32_t lastSeen;
  uint32_t lastAlertTime;
  uint8_t alertLevel;
  IPAddress assignedIP;
  int8_t lastRSSI;
  float estimatedDistance;
  bool hasEQInfo;  // 是否已收到 0x01 地震信息
  float s_peak_gal;
  float tau_c_sec;
  float sp_time_sec;
  float distance_km;
  float warning_time_sec;
  uint32_t eqEpoch;
  float ai_confidence;  // 新增
  float ae_anomaly_score;
  float p_wave_azimuth;
  // 单节点评分
  float single_node_score;  // 0~1
  float ai_conf;
  float ae_score;
  float tau_consistency;
  float sp_consistency;
  float time_consistency;
  uint32_t eq_epoch;  // 已存在
  bool in_current_event;
  // 是否已加入当前事件
};

#define MAX_EVENT_NODES 10
#define MAX_EVENT_AGE_MS 5000  // 事件有效期5秒

struct EventRecord {
  uint32_t start_epoch;  // 事件起始时间（第一个节点报警的 epoch）
  uint32_t last_update;  // 本地上次更新时间
  uint8_t node_count;
  struct {
    uint16_t node_id;
    float s_peak_gal;
    float tau_c_sec;
    float sp_time_sec;
    float distance_km;
    float ai_confidence;
    float ae_anomaly;
    float single_score;  // 单节点评分
    float lat, lng;      // 节点坐标
  } nodes[MAX_EVENT_NODES];

  // 融合结果
  float network_confidence;
  float epicenter_lat, epicenter_lng;
  float fused_magnitude;  // 修正后震级（可选）
  bool validated;
};

EventRecord currentEvent;
bool eventActive = false;


// 消息存储
#define GW_MAX_STORED_MSGS 50

struct GwStoredMessage {
  uint32_t timestamp;
  uint16_t fromNode;
  uint16_t toNode;
  char text[200];
};
GwStoredMessage gwMsgStore[GW_MAX_STORED_MSGS];
int gwMsgCount = 0;
// ==================== 全局变量 ====================
NodeInfo nodes[MAX_NODES];
int nodeCount = 0;
IPAddress nextNodeIP(192, 168, 4, 100);

float gwLat = 0, gwLng = 0, gwAlt = 0;
bool gwLocationValid = false;
String gwLocationMethod = "none";

String config_ssid = "";
String config_pass = "";
String config_dingtalk = "";
float config_alertThreshold = 0.5f;
int dtuSignal = -1;
String dtuNetTime = "";




WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

NetworkManager networkManager;
WebServer server(80);

volatile uint32_t totalPackets = 0;
volatile uint32_t totalAlerts = 0;
volatile uint32_t totalRelays = 0;
volatile uint32_t totalDTUFails = 0;
uint32_t lastDTUHeartbeat = 0;

float g_gateway_inf_data[WINDOW_SIZE];
float gwRecentZBuf[700];

struct FirstTriggerInfo {
  bool active;
  uint16_t nodeID;
  float confidence;
  int16_t quantizedCoefficients[10];
  uint32_t timestamp;
};
FirstTriggerInfo firstTrigger;
uint32_t firstTriggerLastUpdate = 0;
uint32_t lastDtuCmdTime = 0;

// Z 轴环形缓冲区
#define GW_Z_HISTORY_SIZE 700
float gwZHistory[GW_Z_HISTORY_SIZE] = { 0 };
int gwZHistoryHead = 0;
int gwZHistoryCount = 0;
bool gwZHistoryFull = false;

bool deploymentMode = false;
WiFiServer deploymentServer(5555);
WiFiClient deploymentClients[20];

struct DeploymentRequest {
  uint16_t nodeID;
  float lat, lng, alt;
  IPAddress nodeIP;
  bool confirmed;
};
DeploymentRequest deployRequests[50];
int deployRequestCount = 0;
int nextDHCPIndex = 0;

bool dtuOnline = false;
portMUX_TYPE dtuMux = portMUX_INITIALIZER_UNLOCKED;
bool dtuInitVbattOk = false;

int gpsFailCount = 0;
const int GPS_FAIL_THRESHOLD = 3;

String cachedWebPage = "";
uint32_t cachedWebPageTime = 0;


// 状态机回调标志（跨核安全）

volatile float pendingProbability = 0;


// DTU 异步发送队列

// 共用的静态缓冲区

static char mqttBuf[512];
static char httpBuf[1536];
static char g_dingBuf[1024];

// DTU 串口写入互斥锁


time_t g_epochTime = 0;    // 当前 Unix 时间戳（秒）
bool g_timeValid = false;  // 时间是否有效
uint32_t lastNtpSync = 0;

QueueHandle_t txQueue;


// ==================== 中继去重 ====================
#define RELAY_DEDUP_SIZE 20        // 滑动窗口大小
#define RELAY_DEDUP_TIMEOUT 10000  // 10秒超时

struct RelayRecord {
  uint16_t sourceNode;    // 原始报警节点ID
  uint32_t eqEpoch;       // 地震事件的NTP时间戳
  uint32_t receivedTime;  // 网关收到的时间(ms)
};

RelayRecord relayHistory[RELAY_DEDUP_SIZE];
int relayHistoryCount = 0;
int relayHistoryIndex = 0;
/**
 * @brief 检查是否已经中继过该事件
 * @param sourceNode 原始报警节点ID
 * @param eqEpoch 地震事件时间戳
 * @return true=已中继过，应该跳过；false=新事件，应该中继
 */
bool isRelayDuplicate(uint16_t sourceNode, uint32_t eqEpoch) {
  uint32_t now = millis();

  // 清理过期记录
  int writeIdx = 0;
  for (int i = 0; i < relayHistoryCount; i++) {
    if (now - relayHistory[i].receivedTime < RELAY_DEDUP_TIMEOUT) {
      if (writeIdx != i) {
        relayHistory[writeIdx] = relayHistory[i];
      }
      writeIdx++;
    }
  }
  relayHistoryCount = writeIdx;

  // 检查是否重复
  for (int i = 0; i < relayHistoryCount; i++) {
    if (relayHistory[i].sourceNode == sourceNode) {
      // 同一节点
      if (abs((int32_t)(relayHistory[i].eqEpoch - eqEpoch)) <= 3) {
        // 时间戳相差不超过3秒，认为是同一事件
        return true;
      }
    }
  }

  return false;
}

/**
 * @brief 记录一次中继
 */
void recordRelay(uint16_t sourceNode, uint32_t eqEpoch) {
  // 先清理过期
  isRelayDuplicate(sourceNode, eqEpoch);

  // 添加新记录
  if (relayHistoryCount < RELAY_DEDUP_SIZE) {
    relayHistory[relayHistoryCount].sourceNode = sourceNode;
    relayHistory[relayHistoryCount].eqEpoch = eqEpoch;
    relayHistory[relayHistoryCount].receivedTime = millis();
    relayHistoryCount++;
  } else {
    // 覆盖最旧的记录
    relayHistory[relayHistoryIndex].sourceNode = sourceNode;
    relayHistory[relayHistoryIndex].eqEpoch = eqEpoch;
    relayHistory[relayHistoryIndex].receivedTime = millis();
    relayHistoryIndex = (relayHistoryIndex + 1) % RELAY_DEDUP_SIZE;
  }
}

//====================单节点验证函数===============

// 1. AI与AE双重一致性
float compute_ai_ae_consistency(float ai_conf, float ae_score) {
  // ae_score 已经归一化，假设 0=正常，>3=异常
  float ae_norm = constrain(ae_score / 5.0f, 0.0f, 1.0f);  // 映射到0~1
  return 0.5f * ai_conf + 0.5f * ae_norm;
}

// 2. τc 与 s_peak 物理一致性
// 经验模型：log10(tau_c) ≈ 0.5*log10(s_peak) - 1.5
float compute_tau_consistency(float s_peak, float tau_c) {
  if (s_peak < 1.0f || tau_c < 0.01f) return 0.5f;  // 无效值
  float expected_log = 0.5f * log10f(s_peak) - 1.5f;
  float actual_log = log10f(tau_c);
  float error = fabsf(actual_log - expected_log);
  return expf(-error * 2.0f);  // 误差越大分数越低
}

// 3. S-P 距离一致性
float compute_sp_consistency(float sp_time, float distance_km) {
  if (sp_time < 0.1f) return 0.5f;
  float expected_dist = 8.0f * sp_time;  // 8 km/s S-P 波速差
  float error = fabsf(distance_km - expected_dist);
  return expf(-error / 20.0f);
}

// 4. 时间一致性（对比网关 NTP 时间）
float compute_time_consistency(uint32_t node_epoch) {
  if (!g_timeValid) return 1.0f;  // 网关时间无效则忽略
  uint32_t now_epoch = getEpochTime();
  int32_t diff = abs((int32_t)(now_epoch - node_epoch));
  if (diff <= 2) return 1.0f;
  if (diff <= 10) return 0.7f;
  return 0.3f;
}

// 综合单节点评分
float compute_single_node_score(NodeInfo& node) {
  float ai_ae = compute_ai_ae_consistency(node.ai_confidence, node.ae_anomaly_score);
  float tau = compute_tau_consistency(node.s_peak_gal, node.tau_c_sec);
  float sp = compute_sp_consistency(node.sp_time_sec, node.distance_km);
  float time = compute_time_consistency(node.eqEpoch);

  node.ai_conf = node.ai_confidence;
  node.ae_score = node.ae_anomaly_score;
  node.tau_consistency = tau;
  node.sp_consistency = sp;
  node.time_consistency = time;

  // 权重可调
  return 0.3f * ai_ae + 0.2f * tau + 0.2f * sp + 0.2f * time + 0.1f;
}
//==================== 事件缓冲区 ==================
void addNodeToEvent(uint16_t node_id) {
  if (!eventActive) {
    // 新事件
    memset(&currentEvent, 0, sizeof(currentEvent));
    currentEvent.start_epoch = getEpochTime();
    currentEvent.last_update = millis();
    eventActive = true;
  }

  // 查找节点（避免重复添加）
  for (int i = 0; i < currentEvent.node_count; i++) {
    if (currentEvent.nodes[i].node_id == node_id) {
      // 更新信息
      NodeInfo* n = nullptr;
      for (int j = 0; j < nodeCount; j++) {
        if (nodes[j].nodeID == node_id) {
          n = &nodes[j];
          break;
        }
      }
      if (n) {
        currentEvent.nodes[i].single_score = n->single_node_score;
        // 其他字段也更新...
      }
      currentEvent.last_update = millis();
      return;
    }
  }

  if (currentEvent.node_count >= MAX_EVENT_NODES) return;

  NodeInfo* n = nullptr;
  for (int j = 0; j < nodeCount; j++) {
    if (nodes[j].nodeID == node_id) {
      n = &nodes[j];
      break;
    }
  }
  if (!n) return;

  int idx = currentEvent.node_count++;
  currentEvent.nodes[idx].node_id = node_id;
  currentEvent.nodes[idx].s_peak_gal = n->s_peak_gal;
  currentEvent.nodes[idx].tau_c_sec = n->tau_c_sec;
  currentEvent.nodes[idx].sp_time_sec = n->sp_time_sec;
  currentEvent.nodes[idx].distance_km = n->distance_km;
  currentEvent.nodes[idx].ai_confidence = n->ai_confidence;
  currentEvent.nodes[idx].ae_anomaly = n->ae_anomaly_score;
  currentEvent.nodes[idx].single_score = n->single_node_score;
  currentEvent.nodes[idx].lat = n->assignedLat;
  currentEvent.nodes[idx].lng = n->assignedLng;
  currentEvent.last_update = millis();
}
void checkEventExpiry() {
  if (eventActive && (millis() - currentEvent.last_update > MAX_EVENT_AGE_MS)) {
    // 事件过期，强制融合并上报（即使节点数不够）
    perform_event_fusion();
    eventActive = false;
  }
}
void resetEventBuffer() {
  // 清除所有节点的 in_current_event 标记
  for (int i = 0; i < currentEvent.node_count; i++) {
    for (int j = 0; j < nodeCount; j++) {
      if (nodes[j].nodeID == currentEvent.nodes[i].node_id) {
        nodes[j].in_current_event = false;
        break;
      }
    }
  }

  memset(&currentEvent, 0, sizeof(currentEvent));
  eventActive = false;
}
//===================== 告警函数 ====================
void send_fused_alert_mqtt() {
  if (!mqttClient.connected()) return;
  if (!currentEvent.validated) return;

  // 构造融合报警 JSON（使用局部 String，不占用全局缓冲区）
  String json = "{";
  json += "\"type\":\"fused_event\",";
  json += "\"node_count\":" + String(currentEvent.node_count) + ",";
  json += "\"network_confidence\":" + String(currentEvent.network_confidence, 2) + ",";
  json += "\"epicenter_lat\":" + String(currentEvent.epicenter_lat, 6) + ",";
  json += "\"epicenter_lng\":" + String(currentEvent.epicenter_lng, 6) + ",";
  json += "\"fused_magnitude\":" + String(currentEvent.fused_magnitude, 1) + ",";
  json += "\"timestamp\":" + String(getEpochTime()) + ",";
  json += "\"nodes\":[";
  for (int i = 0; i < currentEvent.node_count; i++) {
    if (i > 0) json += ",";
    json += "{\"id\":" + String(currentEvent.nodes[i].node_id) + ",\"score\":" + String(currentEvent.nodes[i].single_score, 2) + ",\"dist\":" + String(currentEvent.nodes[i].distance_km, 1) + "}";
  }
  json += "]}";

  // 直接调用已有的 MQTT 发布
  mqttClient.publish(MQTT_TOPIC_ALERT, json.c_str());
  Serial.println("[MQTT] Fused alert sent");
}
void send_fused_alert_dtu() {
  if (!dtuOnline) return;
  if (!currentEvent.validated) return;

  // ---- 1. 通过 DTU 发送 MQTT 报警（复用 dtuSendAlert） ----
  String json = "{";
  json += "\"type\":\"fused_event\",";
  json += "\"node_count\":" + String(currentEvent.node_count) + ",";
  json += "\"network_confidence\":" + String(currentEvent.network_confidence, 2) + ",";
  json += "\"epicenter_lat\":" + String(currentEvent.epicenter_lat, 6) + ",";
  json += "\"epicenter_lng\":" + String(currentEvent.epicenter_lng, 6) + ",";
  json += "\"fused_magnitude\":" + String(currentEvent.fused_magnitude, 1) + ",";
  json += "\"timestamp\":" + String(getEpochTime()) + ",";
  json += "\"nodes\":[";
  for (int i = 0; i < currentEvent.node_count; i++) {
    if (i > 0) json += ",";
    json += "{\"id\":" + String(currentEvent.nodes[i].node_id) + ",\"score\":" + String(currentEvent.nodes[i].single_score, 2) + ",\"dist\":" + String(currentEvent.nodes[i].distance_km, 1) + "}";
  }
  json += "]}";

  dtuSendAlert(json);  // 复用原有函数
  Serial.println("[DTU] Fused alert sent via bbb3");

  // ---- 2. 通过 DTU 发送钉钉通知（复用 dtuSendDingTalk） ----
  String ding_json = "{\"msgtype\":\"markdown\",\"markdown\":{";
  ding_json += "\"title\":\"🌍 地震预警融合事件\",";
  ding_json += "\"text\":\"## 🌍 地震预警融合事件\\n\\n";
  ding_json += "- **参与节点**: " + String(currentEvent.node_count) + "\\n";
  ding_json += "- **网络置信度**: " + String(currentEvent.network_confidence * 100, 0) + "%\\n";
  ding_json += "- **估计震中**: " + String(currentEvent.epicenter_lat, 4) + ", " + String(currentEvent.epicenter_lng, 4) + "\\n";
  ding_json += "- **融合震级**: " + String(currentEvent.fused_magnitude, 1) + " gal\\n\\n";
  ding_json += "**节点详情**:\\n";
  for (int i = 0; i < currentEvent.node_count; i++) {
    ding_json += "- 节点 " + String(currentEvent.nodes[i].node_id)
                 + ": 距离 " + String(currentEvent.nodes[i].distance_km, 1) + "km, 评分 "
                 + String(currentEvent.nodes[i].single_score, 2) + "\\n";
  }
  ding_json += "\"}}";

  String http_req = "POST /robot/send?access_token=" + config_dingtalk + " HTTP/1.1\r\n"
                                                                         "Host: oapi.dingtalk.com\r\n"
                                                                         "Content-Type: application/json\r\n"
                                                                         "Content-Length: "
                    + String(ding_json.length()) + "\r\n\r\n"
                    + ding_json;

  dtuSendDingTalk(http_req);  // 复用原有函数
  Serial.println("[DTU] Fused alert DingTalk sent");
}
void send_fused_alert_dingtalk() {
  if (!dtuOnline || config_dingtalk.length() == 0) return;
  if (!currentEvent.validated) return;

  // 根据融合结果判断级别
  String level_str;
  String level_emoji;
  if (currentEvent.network_confidence > 0.8f && currentEvent.node_count >= 3) {
    level_str = "🔴 红色预警";
    level_emoji = "🔴";
  } else if (currentEvent.network_confidence > 0.6f && currentEvent.node_count >= 2) {
    level_str = "🟠 橙色预警";
    level_emoji = "🟠";
  } else {
    level_str = "🟡 黄色预警";
    level_emoji = "🟡";
  }

  String ding_json = "{\"msgtype\":\"markdown\",\"markdown\":{";
  ding_json += "\"title\":\"🌍 地震预警 " + level_str + "\",";
  ding_json += "\"text\":\"## " + level_emoji + " 地震预警融合事件\\n\\n";
  ding_json += "- **预警级别**: " + level_str + "\\n";
  ding_json += "- **参与节点**: " + String(currentEvent.node_count) + "\\n";
  // ... 其他信息 ...
  ding_json += "\"}}";

  // ... DTU 发送 ...
}
//===================== 多节点验证函数 ================
void perform_event_fusion() {
  if (!eventActive || currentEvent.node_count == 0) return;

  // 1. 时间一致性
  float epoch_vals[MAX_EVENT_NODES];
  for (int i = 0; i < currentEvent.node_count; i++) {
    NodeInfo* n = nullptr;
    for (int j = 0; j < nodeCount; j++) {
      if (nodes[j].nodeID == currentEvent.nodes[i].node_id) {
        n = &nodes[j];
        break;
      }
    }
    epoch_vals[i] = n ? n->eqEpoch : currentEvent.start_epoch;
  }
  float mean_epoch = 0;
  for (int i = 0; i < currentEvent.node_count; i++) mean_epoch += epoch_vals[i];
  mean_epoch /= currentEvent.node_count;
  float epoch_std = 0;
  for (int i = 0; i < currentEvent.node_count; i++) {
    epoch_std += (epoch_vals[i] - mean_epoch) * (epoch_vals[i] - mean_epoch);
  }
  epoch_std = sqrtf(epoch_std / currentEvent.node_count);
  float time_consistency = expf(-epoch_std * 2.0f);  // 秒级 std 越小越好

  // 2. 网络置信度（平均单节点评分）
  float avg_single_score = 0;
  for (int i = 0; i < currentEvent.node_count; i++) avg_single_score += currentEvent.nodes[i].single_score;
  avg_single_score /= currentEvent.node_count;

  // 3. 震中粗略估计（网格搜索）
  float best_lat = gwLat, best_lng = gwLng;
  float best_error = 1e9;
  if (gwLocationValid) {
    // 在网关位置周围 ±0.5° 搜索
    for (float dlat = -0.5f; dlat <= 0.5f; dlat += 0.05f) {
      for (float dlng = -0.5f; dlng <= 0.5f; dlng += 0.05f) {
        float test_lat = gwLat + dlat;
        float test_lng = gwLng + dlng;
        float error = 0;
        for (int i = 0; i < currentEvent.node_count; i++) {
          float dx = (currentEvent.nodes[i].lat - test_lat) * 111320.0f;
          float dy = (currentEvent.nodes[i].lng - test_lng) * 111320.0f * cosf(test_lat * M_PI / 180.0f);
          float pred_dist = sqrtf(dx * dx + dy * dy);
          float dist_err = pred_dist - currentEvent.nodes[i].distance_km * 1000.0f;  // 转为米
          error += dist_err * dist_err;
        }
        if (error < best_error) {
          best_error = error;
          best_lat = test_lat;
          best_lng = test_lng;
        }
      }
    }
  }
  currentEvent.epicenter_lat = best_lat;
  currentEvent.epicenter_lng = best_lng;

  // 4. 震级一致性：修正 s_peak 到参考距离
  float ref_distance = 100.0f;  // 100km
  float mag_sum = 0, mag_weight = 0;
  for (int i = 0; i < currentEvent.node_count; i++) {
    float d = currentEvent.nodes[i].distance_km;
    if (d < 1) d = 1;
    // 简单衰减模型：PGA ≈ M0 / d^1.5
    float corrected_pga = currentEvent.nodes[i].s_peak_gal * powf(d / ref_distance, 1.5f);
    float w = currentEvent.nodes[i].single_score;
    mag_sum += corrected_pga * w;
    mag_weight += w;
  }
  currentEvent.fused_magnitude = (mag_weight > 0) ? (mag_sum / mag_weight) : 0;

  // 5. 综合网络置信度
  currentEvent.network_confidence = 0.4f * time_consistency + 0.4f * avg_single_score
                                    + 0.2f * (currentEvent.node_count >= 3 ? 1.0f : 0.5f);
  currentEvent.validated = true;
}
// ==================== 辅助函数 ====================
void sendLoRaFromQueue() {
  uint8_t* buf = NULL;
  if (xQueueReceive(txQueue, &buf, 0) == pdTRUE) {
    if (buf) {
      waitAuxHigh();
      int len = buf[2] + 5;
      DXLR_SERIAL.write(buf, len);
      free(buf);
    }
  }
}

bool syncNTP() {
  if (!WiFi.isConnected()) return false;  // 4G 模式不在这里处理

  configTime(8 * 3600, 0, "ntp.aliyun.com", "ntp1.aliyun.com", "time.windows.com");

  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 20) {
    delay(500);
    retry++;
  }

  if (retry < 20) {
    g_epochTime = mktime(&timeinfo);
    lastNtpSync = millis();
    g_timeValid = true;
    Serial.printf("[NTP] Time synced via WiFi: %s", asctime(&timeinfo));
    return true;
  }

  return false;
}

// 获取格式化的时间字符串
String getTimeString() {
  if (!g_timeValid) return String(millis() / 1000) + "s";

  time_t now = g_epochTime + (millis() - lastNtpSync) / 1000;
  struct tm* timeinfo = localtime(&now);

  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  return String(buf);
}

// 获取 Unix 时间戳
uint32_t getEpochTime() {
  if (!g_timeValid) return millis() / 1000;
  return g_epochTime + (millis() - lastNtpSync) / 1000;
}

// 广播时间同步帧
void broadcastTimeSync() {
  if (!g_timeValid) return;

  uint32_t epoch = getEpochTime();

  uint8_t buf[10];
  buf[0] = FRAME_HEADER;
  buf[1] = FRAME_TIME_SYNC;
  buf[2] = 4;  // 4字节时间戳
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  memcpy(&buf[5], &epoch, 4);
  sendLoRaPacket(buf, 9);

  Serial.printf("[TIME] Broadcast time: %u\n", epoch);
}

void waitAuxHigh() {
  if (DXLR_AUX <= 0) {
    delay(20);
    return;
  }
  uint32_t t = millis();
  while (digitalRead(DXLR_AUX) == LOW) {
    if (millis() - t > 500) break;
    delay(1);
  }
}

float rssiToDistance(int8_t rssi) {
  return pow(10.0f, (-30.0f - rssi) / (10.0f * 2.8f));
}

void wgs84_to_gcj02(float wgs_lat, float wgs_lng, float* gcj_lat, float* gcj_lng) {
  const float pi = 3.141592653589793f;
  const float a = 6378245.0f;
  const float ee = 0.006693421622965943f;

  auto _lat = [pi](float x, float y) -> float {
    float r = -100.0f + 2.0f * x + 3.0f * y + 0.2f * y * y + 0.1f * x * y + 0.2f * sqrt(fabs(x));
    r += (20.0f * sin(6.0f * x * pi) + 20.0f * sin(2.0f * x * pi)) * 2.0f / 3.0f;
    r += (20.0f * sin(y * pi) + 40.0f * sin(y / 3.0f * pi)) * 2.0f / 3.0f;
    r += (160.0f * sin(y / 12.0f * pi) + 320.0f * sin(y * pi / 30.0f)) * 2.0f / 3.0f;
    return r;
  };

  auto _lng = [pi](float x, float y) -> float {
    float r = 300.0f + x + 2.0f * y + 0.1f * x * x + 0.1f * x * y + 0.1f * sqrt(fabs(x));
    r += (20.0f * sin(6.0f * x * pi) + 20.0f * sin(2.0f * x * pi)) * 2.0f / 3.0f;
    r += (20.0f * sin(x * pi) + 40.0f * sin(x / 3.0f * pi)) * 2.0f / 3.0f;
    r += (150.0f * sin(x / 12.0f * pi) + 300.0f * sin(x / 30.0f * pi)) * 2.0f / 3.0f;
    return r;
  };

  float dLat = _lat(wgs_lng - 105.0f, wgs_lat - 35.0f);
  float dLng = _lng(wgs_lng - 105.0f, wgs_lat - 35.0f);
  float radLat = wgs_lat / 180.0f * pi;
  float magic = sin(radLat);
  magic = 1.0f - ee * magic * magic;
  float sqrtMagic = sqrt(magic);
  dLat = (dLat * 180.0f) / ((a * (1.0f - ee)) / (magic * sqrtMagic) * pi);
  dLng = (dLng * 180.0f) / (a / sqrtMagic * cos(radLat) * pi);
  *gcj_lat = wgs_lat + dLat;
  *gcj_lng = wgs_lng + dLng;
}

float fetchAltitude(float lat, float lng) {
  HTTPClient http;
  float altitude = 0.0;
  String url = "https://api.opentopodata.org/v1/aster30m?locations=" + String(lat, 6) + "," + String(lng, 6);
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == 200) {
    String payload = http.getString();
    int idx = payload.indexOf("\"elevation\":");
    if (idx != -1) {
      idx += 12;
      int endIdx = payload.indexOf(",", idx);
      if (endIdx == -1) endIdx = payload.indexOf("}", idx);
      altitude = payload.substring(idx, endIdx).toFloat();
    }
  }
  http.end();
  return altitude;
}

void appendZHistory(float* zData, int len) {

  for (int i = 0; i < len; i++) {
    gwZHistory[gwZHistoryHead] = zData[i];
    gwZHistoryHead = (gwZHistoryHead + 1) % GW_Z_HISTORY_SIZE;
  }
  gwZHistoryCount += len;
  if (gwZHistoryCount >= GW_Z_HISTORY_SIZE) {
    gwZHistoryFull = true;
    gwZHistoryCount = GW_Z_HISTORY_SIZE;
  }
}

void getRecentZHistory(float* out, int count) {

  int actualCount = gwZHistoryFull ? GW_Z_HISTORY_SIZE : gwZHistoryCount;
  if (count > actualCount) count = actualCount;
  int start = (gwZHistoryHead - count + GW_Z_HISTORY_SIZE) % GW_Z_HISTORY_SIZE;
  for (int i = 0; i < count; i++) {
    out[i] = gwZHistory[(start + i) % GW_Z_HISTORY_SIZE];
  }
}



// ==================== PCA ====================
//这里曾经存在过一些函数，但已经没人需要它们了.......

// ==================== FFat 配置 ====================
void saveConfig() {
  File f = FFat.open("/config.json", FILE_WRITE);
  if (!f) return;
  String json = "{";
  json += "\"ssid\":\"" + config_ssid + "\",";
  json += "\"pass\":\"" + config_pass + "\",";
  json += "\"dingtalk\":\"" + config_dingtalk + "\",";
  json += "\"threshold\":" + String(config_alertThreshold);
  json += "}";
  f.print(json);
  f.close();
}

void loadConfig() {
  File f = FFat.open("/config.json", FILE_READ);
  if (!f) {
    config_ssid = "Mywifi";
    config_pass = "1234567890";
    config_dingtalk = "8d8aaaf366694452a076974b719cbc35ee1efd133a41cc5c3a2fe3462ec32d63";
    config_alertThreshold = 0.5f;
    return;
  }
  String content = "";
  while (f.available()) content += (char)f.read();
  f.close();

  auto findStr = [&](String key) -> String {
    int s = content.indexOf("\"" + key + "\":\"");
    if (s < 0) return "";
    s += key.length() + 4;
    int e = content.indexOf("\"", s);
    return content.substring(s, e);
  };

  auto findFloat = [&](String key) -> float {
    int s = content.indexOf("\"" + key + "\":");
    if (s < 0) return 0;
    s += key.length() + 3;
    int e = content.indexOf(",", s);
    if (e < 0) e = content.indexOf("}", s);
    return content.substring(s, e).toFloat();
  };

  config_ssid = findStr("ssid");
  config_pass = findStr("pass");
  config_dingtalk = findStr("dingtalk");
  config_alertThreshold = findFloat("threshold");
  if (config_ssid.length() == 0) config_ssid = "Mywifi";
  if (config_pass.length() == 0) config_pass = "1234567890";
  if (config_dingtalk.length() == 0) config_dingtalk = "8d8aaaf366694452a076974b719cbc35ee1efd133a41cc5c3a2fe3462ec32d63";
  if (config_alertThreshold < 0.1f) config_alertThreshold = 0.5f;
}

// ==================== LoRa ====================
bool sendLoRaPacket(uint8_t* data, int len) {
  waitAuxHigh();
  DXLR_SERIAL.write(data, len);
  return true;
}

void broadcastGatewayPosition() {
  if (!gwLocationValid) return;
  uint8_t buf[18];
  buf[0] = FRAME_HEADER;
  buf[1] = FRAME_POSITION;
  buf[2] = 12;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  memcpy(&buf[5], &gwLat, 4);
  memcpy(&buf[9], &gwLng, 4);
  memcpy(&buf[13], &gwAlt, 4);
  sendLoRaPacket(buf, 17);
}

void broadcastNeighborList() {
  uint8_t buf[256];
  buf[0] = FRAME_HEADER;
  buf[1] = FRAME_NEIGHBOR_LIST;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  int offset = 5;
  buf[offset++] = nodeCount & 0xFF;
  for (int i = 0; i < nodeCount && offset + 14 < 250; i++) {
    buf[offset++] = (nodes[i].nodeID >> 8) & 0xFF;
    buf[offset++] = nodes[i].nodeID & 0xFF;
    memcpy(&buf[offset], &nodes[i].assignedLat, 4);
    offset += 4;
    memcpy(&buf[offset], &nodes[i].assignedLng, 4);
    offset += 4;
    memcpy(&buf[offset], &nodes[i].assignedAlt, 4);
    offset += 4;
  }
  buf[2] = offset - 5;
  sendLoRaPacket(buf, offset);
}

// ==================== DTU ====================
void dtuSendCmdAsync(const char* cmd, const char* matchKey, void (*callback)(const String&)) {
  // 1. 清空接收缓冲区（快速操作，加锁保护）

  while (DTU_SERIAL.available()) DTU_SERIAL.read();


  // 2. 等待 5s 命令间隔（非阻塞延时）
  uint32_t now = millis();
  if (now - lastDtuCmdTime < 5000) {
    vTaskDelay(pdMS_TO_TICKS(5000 - (now - lastDtuCmdTime)));
  }
  lastDtuCmdTime = millis();

  // 3. 发送命令（加互斥锁保护，不关中断）

  DTU_SERIAL.print(cmd);


  // 4. 入队等待响应
  DTUCmd item;
  strncpy(item.cmd, cmd, 63);
  item.cmd[63] = '\0';
  strncpy(item.matchKey, matchKey, 31);
  item.matchKey[31] = '\0';
  item.callback = callback;

  if (xQueueSend(dtuCmdQueue, &item, 0) != pdTRUE) {
    Serial.println("[DTU] Cmd queue full");
  }
}
uint8_t calculateIntensity(float normFactorZ = 2.0f) {
  if (!gwZHistoryFull) return 1;

  float maxZ = 0;
  static float recentZ[700];
  getRecentZHistory(recentZ, 700);
  for (int i = 0; i < 700; i++) {
    float val = fabs(recentZ[i]);
    if (val > maxZ) maxZ = val;
  }

  // recentZ 已经乘过 normFactor，单位是 g
  float accel = maxZ * 9.8f;  // g → m/s²

  // 中国地震烈度表 (GB/T 17742-2008) 单位 m/s²
  if (accel < 0.31f) return 1;
  if (accel < 0.63f) return 2;
  if (accel < 1.25f) return 3;
  if (accel < 2.50f) return 4;
  if (accel < 5.00f) return 5;
  if (accel < 10.0f) return 6;
  if (accel < 25.0f) return 7;
  if (accel < 50.0f) return 8;
  if (accel < 100.0f) return 9;
  if (accel < 250.0f) return 10;
  return 11;
}

void dtuProcessResponse() {
  // 1. 读取串口数据

  while (DTU_SERIAL.available() && dtuRxLen < 511) {
    char c = DTU_SERIAL.read();
    if (c == (char)0xFE) continue;
    dtuRxBuffer[dtuRxLen++] = c;
  }
  dtuRxBuffer[dtuRxLen] = '\0';


  if (dtuRxLen == 0) return;

  // 2. 等完整行
  if (!strchr(dtuRxBuffer, '\n') && dtuRxLen < 500) return;
  // ----- 新增：海拔响应处理 -----
  // 海拔 API 返回的 JSON 包含 "elevation" 字段
  if (strstr(dtuRxBuffer, "\"elevation\"") != NULL) {
    // 找到 "elevation": 后面的数值
    char* p = strstr(dtuRxBuffer, "\"elevation\":");
    if (p) {
      p += 12;              // 跳过 "elevation":
      float alt = atof(p);  // 直接转 float
      if (alt > 0) {
        gwAlt = alt;
        Serial.printf("[ALT] Altitude updated: %.1f m\n", gwAlt);
      }
    }
    // 无论是否成功，丢弃本次数据
    dtuRxLen = 0;
    memset(dtuRxBuffer, 0, sizeof(dtuRxBuffer));
    return;
  }
  // ----- 新增结束 -----

  // ===== 新增：过滤未知响应 =====
  bool isATResponse = (strstr(dtuRxBuffer, ",ok") != NULL)
                      || (strstr(dtuRxBuffer, "ERROR") != NULL)
                      || (strstr(dtuRxBuffer, "ssta,ok") != NULL)
                      || (strstr(dtuRxBuffer, "vbatt,ok") != NULL)
                      || (strstr(dtuRxBuffer, "lbsloc,ok") != NULL)
                      || (strstr(dtuRxBuffer, "csq,ok") != NULL)
                      || (strstr(dtuRxBuffer, "nettime,ok") != NULL)
                      || (strstr(dtuRxBuffer, "gps,ok") != NULL)
                      || (strstr(dtuRxBuffer, "location,ok") != NULL);

  if (!isATResponse) {
    Serial.printf("[DTU] Unknown response discarded (%d bytes)\n", dtuRxLen);
    dtuRxLen = 0;
    memset(dtuRxBuffer, 0, sizeof(dtuRxBuffer));
    return;
  }
  // ===== 过滤结束 =====

  esp_task_wdt_reset();

  // 3. 原有匹配逻辑（不变）
  DTUCmd pending;
  bool matched = false;
  int queueSize = uxQueueMessagesWaiting(dtuCmdQueue);

  for (int i = 0; i < queueSize; i++) {
    if (xQueueReceive(dtuCmdQueue, &pending, 0) == pdTRUE) {
      if (strstr(dtuRxBuffer, pending.matchKey)) {
        matched = true;
        if (!dtuOnline) {
          dtuOnline = true;
          Serial.println("[DTU] Marked as ONLINE");
        }
        if (pending.callback) {
          pending.callback(String(dtuRxBuffer));
        }
        break;
      } else {
        DTUCmd copy;
        memcpy(&copy, &pending, sizeof(DTUCmd));
        if (xQueueSendToBack(dtuCmdQueue, &copy, 0) != pdTRUE) {
          Serial.println("[DTU] Failed to requeue unmatched cmd");
        }
      }
    }
  }

  if (!matched) {
    Serial.printf("[DTU] No match for: '%s'\n", dtuRxBuffer);
  }

  dtuRxLen = 0;
  memset(dtuRxBuffer, 0, sizeof(dtuRxBuffer));
}

void dtuInitAsync() {
  Serial.println("[DTU] Init: waiting 25s then query battery...");
  delay(3000);
  while (DTU_SERIAL.available()) DTU_SERIAL.read();

  uint32_t start = millis();
  while (millis() - start < 25000) {
    delay(100);
  }

  dtuInitVbattOk = false;
  dtuSendCmdAsync("config,get,vbatt\r\n", "vbatt,ok", [](const String& r) {
    dtuInitVbattOk = true;
    int idx = r.indexOf("vbatt,ok,");
    if (idx >= 0) {
      String val = r.substring(idx + 9);
      val.trim();
      float voltage = val.toFloat();
      Serial.printf("[DTU] Battery: %.1f V\n", voltage / 1000.0);
    }
  });

  start = millis();
  while (!dtuInitVbattOk && millis() - start < 10000) {
    dtuProcessResponse();
    delay(100);
  }

  if (dtuInitVbattOk) {
    dtuOnline = true;
    Serial.println("[DTU] Init OK");
  } else {
    Serial.println("[DTU] Init failed");
    dtuOnline = false;
  }
}

void dtuSendDingTalk(String httpRequest) {
  if (!dtuOnline) {
    Serial.println("[DTU] dtuSendDingTalk skipped: offline");
    return;
  }
  Serial.printf("[DTU] dtuSendDingTalk: %d bytes\n", httpRequest.length());

  DTU_SERIAL.print("bbb1");
  DTU_SERIAL.print(httpRequest);
}


void dtuSendAlert(String payload) {
  if (!dtuOnline) {
    Serial.println("[DTU] dtuSendAlert skipped: offline");
    return;
  }
  Serial.printf("[DTU] dtuSendAlert: %d bytes\n", payload.length());

  DTU_SERIAL.print("bbb3");
  DTU_SERIAL.print(payload);
}

void dtuSendHeartbeat(String payload) {
  if (!dtuOnline) return;

  DTU_SERIAL.print("bbb4");
  DTU_SERIAL.print(payload);
}
void requestAltitude(float lat, float lng) {
  if (!dtuOnline) return;

  // 构造 HTTP 请求（和钉钉一模一样，只是 URL 不同）
  String url = "/v1/aster30m?locations=" + String(lat, 6) + "," + String(lng, 6);
  String httpReq = "GET " + url + " HTTP/1.1\r\n";
  httpReq += "Host: api.opentopodata.org\r\n";
  httpReq += "User-Agent: ESP32\r\n";
  httpReq += "Connection: close\r\n\r\n";

  Serial.printf("[ALT] Requesting altitude via bbb2...\n");

  // 发送时加锁，避免和 Core1 的 bbb1 发送冲突

  DTU_SERIAL.print("bbb2");   // 切换到通道2
  DTU_SERIAL.print(httpReq);  // 发送 HTTP 请求
}


// ==================== 节点管理 ====================
void autoCorrectNodePosition(uint16_t nodeID, int8_t rssi) {
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].nodeID != nodeID) continue;
    nodes[i].lastRSSI = rssi;
    nodes[i].estimatedDistance = rssiToDistance(rssi);

    if (gwLocationValid && nodes[i].assignedLat != 0) {
      float dist = nodes[i].estimatedDistance;

      // ===== 海拔获取：WiFi 同步，4G 异步 =====
      if (nodes[i].assignedAlt == 0) {
        if (WiFi.isConnected()) {
          float fetchedAlt = fetchAltitude(nodes[i].assignedLat, nodes[i].assignedLng);
          if (fetchedAlt > 0) {
            nodes[i].assignedAlt = fetchedAlt;
            Serial.printf("[CORRECT] Node %d altitude via WiFi: %.1f m\n", nodeID, fetchedAlt);
          }
        } else if (dtuOnline) {
          requestAltitude(nodes[i].assignedLat, nodes[i].assignedLng);
          Serial.printf("[CORRECT] Node %d altitude requested via bbb2\n", nodeID);
          // 异步，本次不纠正海拔，等响应回来后再纠正
        }
      }

      // ===== 三维距离计算 =====
      float dLat = (nodes[i].assignedLat - gwLat) * 111320.0f;
      float dLng = (nodes[i].assignedLng - gwLng) * 111320.0f * cos(gwLat * M_PI / 180.0f);
      float dAlt = nodes[i].assignedAlt - gwAlt;
      float expectedDist = sqrt(dLat * dLat + dLng * dLng + dAlt * dAlt);
      float deviation = fabs(expectedDist - dist);

      if (deviation > 100) {
        float dx = nodes[i].assignedLat - gwLat;
        float dy = nodes[i].assignedLng - gwLng;
        float norm = sqrt(dx * dx + dy * dy);
        if (norm > 1e-9) {
          dx /= norm;
          dy /= norm;
          float ratio = dist / expectedDist;
          float newLat = gwLat + dx * dist / 111320.0f;
          float newLng = gwLng + dy * dist / (111320.0f * cos(gwLat * M_PI / 180.0f));
          float newAlt = gwAlt + dAlt * ratio;

          nodes[i].verifiedLat = newLat;
          nodes[i].verifiedLng = newLng;
          nodes[i].assignedAlt = newAlt;
          nodes[i].positionVerified = false;

          uint8_t buf[19];
          buf[0] = FRAME_HEADER;
          buf[1] = FRAME_DEPLOY_CONFIRM;
          buf[2] = 14;
          buf[3] = (GATEWAY_ID >> 8) & 0xFF;
          buf[4] = GATEWAY_ID & 0xFF;
          buf[5] = (nodeID >> 8) & 0xFF;
          buf[6] = nodeID & 0xFF;
          memcpy(&buf[7], &newLat, 4);
          memcpy(&buf[11], &newLng, 4);
          memcpy(&buf[15], &newAlt, 4);
          sendLoRaPacket(buf, 19);

          Serial.printf("[CORRECT] Node %d → %.4f,%.4f,%.1f\n",
                        nodeID, newLat, newLng, newAlt);
        }
      } else {
        nodes[i].positionVerified = true;
      }
    }
    break;
  }
}

void verifyGatewayPosition() {
  if (nodeCount < 3) return;
  float sumLat = 0, sumLng = 0, sumAlt = 0, sumWeight = 0;
  int validNodes = 0;
  for (int i = 0; i < nodeCount; i++) {
    if (!nodes[i].positionVerified) continue;
    if (nodes[i].estimatedDistance <= 0) continue;
    if (millis() - nodes[i].lastSeen > 120000) continue;
    float weight = 1.0f / fmax(nodes[i].estimatedDistance, 1.0f);
    sumLat += nodes[i].assignedLat * weight;
    sumLng += nodes[i].assignedLng * weight;
    sumAlt += nodes[i].assignedAlt * weight;
    sumWeight += weight;
    validNodes++;
  }
  if (validNodes < 3) return;
  float estLat = sumLat / sumWeight;
  float estLng = sumLng / sumWeight;
  float estAlt = sumAlt / sumWeight;
  if (gwLocationValid) {
    float dLat = (gwLat - estLat) * 111320.0f;
    float dLng = (gwLng - estLng) * 111320.0f * cos(estLat * M_PI / 180.0f);
    float dAlt = gwAlt - estAlt;
    float dev = sqrt(dLat * dLat + dLng * dLng + dAlt * dAlt);
    if (dev > 500) {
      Serial.printf("[GW-VERIFY] DTU pos deviation: %.1fm\n", dev);
    } else {
      Serial.printf("[GW-VERIFY] DTU pos verified (dev=%.1fm)\n", dev);
    }
  }
}




// ==================== Web ====================

String generateWebPage() {
  String html;
  html.reserve(35000);

  html = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <title>地震预警网关 v6.1</title>
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <style>
        :root {
            --bg: #0f0f1a;
            --card: #1a1a2e;
            --accent: #e94560;
            --gold: #f5c518;
            --green: #4ecca3;
            --text: #e0e0e0;
            --muted: #888;
            --input-bg: #12122a;
            --border: #2a2a45;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            background: var(--bg);
            color: var(--text);
            padding: 10px;
            line-height: 1.5;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { color: var(--accent); margin-bottom: 5px; font-size: 1.5em; }
        h2 { color: var(--gold); margin: 15px 0 10px 0; font-size: 1.2em; }
        h3 { color: var(--text); margin: 10px 0 5px 0; font-size: 1em; }
        .card {
            background: var(--card);
            padding: 15px;
            margin: 10px 0;
            border-radius: 10px;
            border: 1px solid var(--border);
        }
        .stats {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
            gap: 10px;
            margin-bottom: 10px;
        }
        .stat {
            background: var(--card);
            padding: 12px;
            border-radius: 8px;
            text-align: center;
            border: 1px solid var(--border);
        }
        .stat .num { font-size: 1.8em; font-weight: bold; }
        .stat .lbl { font-size: 0.75em; color: var(--muted); }
        .green { color: var(--green); }
        .red { color: var(--accent); }
        .gold { color: var(--gold); }
        .tabs {
            display: flex;
            gap: 4px;
            margin: 15px 0 5px 0;
            flex-wrap: wrap;
        }
        .tab {
            padding: 10px 18px;
            background: var(--card);
            border: 1px solid var(--border);
            color: var(--text);
            border-radius: 8px 8px 0 0;
            cursor: pointer;
            font-size: 0.9em;
            transition: background 0.2s;
        }
        .tab.active { background: var(--accent); border-color: var(--accent); color: #fff; }
        .tab-content { display: none; }
        .tab-content.active { display: block; }
        table {
            width: 100%;
            border-collapse: collapse;
            font-size: 0.85em;
            margin-top: 10px;
        }
        th, td {
            padding: 8px 6px;
            text-align: left;
            border-bottom: 1px solid var(--border);
            white-space: nowrap;
        }
        th { color: var(--muted); font-weight: 600; font-size: 0.8em; text-transform: uppercase; }
        .online { color: var(--green); }
        .offline { color: var(--accent); }
        .coord { font-family: 'Courier New', monospace; color: var(--green); font-size: 0.85em; }
        .tag {
            display: inline-block;
            padding: 2px 8px;
            border-radius: 12px;
            font-size: 0.75em;
            font-weight: bold;
        }
        .tag-ok { background: #1a3a2a; color: var(--green); }
        .tag-warn { background: #3a1a1a; color: var(--accent); }
        .tag-info { background: #1a2a3a; color: #4da6ff; }
        input, select {
            width: 100%;
            padding: 10px;
            margin: 6px 0;
            border-radius: 6px;
            border: 1px solid var(--border);
            background: var(--input-bg);
            color: var(--text);
            font-size: 0.9em;
        }
        input:focus { border-color: var(--accent); outline: none; }
        button {
            width: 100%;
            padding: 12px;
            background: var(--accent);
            color: #fff;
            border: none;
            border-radius: 6px;
            font-size: 0.95em;
            cursor: pointer;
            margin-top: 8px;
            font-weight: bold;
            transition: opacity 0.2s;
        }
        button:hover { opacity: 0.85; }
        button.secondary { background: var(--card); border: 1px solid var(--border); color: var(--text); }
        .toast {
            position: fixed;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            background: var(--green);
            color: #000;
            padding: 12px 24px;
            border-radius: 8px;
            font-weight: bold;
            z-index: 999;
            display: none;
            animation: fadeIn 0.3s;
        }
        @keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } }
        .pulse {
            display: inline-block;
            width: 8px;
            height: 8px;
            border-radius: 50%;
            margin-right: 4px;
            animation: pulse 1.5s infinite;
        }
        .pulse-g { background: var(--green); }
        .pulse-r { background: var(--accent); }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.4; }
        }
        @media (max-width: 500px) {
            .stats { grid-template-columns: repeat(2, 1fr); }
            table { font-size: 0.75em; }
            th, td { padding: 6px 4px; }
        }
    </style>
    
</head>
<body>
    <div class="container">
        <h1>🌍 地震预警网关 v6.1</h1>
        <div style="font-size:0.8em;color:var(--muted)">ID: GATEWAY | Uptime: )rawliteral"
         + String(millis() / 1000) + R"rawliteral( s</div>
        
        <!-- 状态卡片 -->
        <div class="stats">
            <div class="stat">
                <div class="lbl">📦 数据包</div>
                <div class="num green">)rawliteral"
         + String(totalPackets) + R"rawliteral(</div>
            </div>
            <div class="stat">
                <div class="lbl">🚨 预警</div>
                <div class="num red">)rawliteral"
         + String(totalAlerts) + R"rawliteral(</div>
            </div>
            <div class="stat">
                <div class="lbl">🔄 中继</div>
                <div class="num gold">)rawliteral"
         + String(totalRelays) + R"rawliteral(</div>
            </div>
            <div class="stat">
                <div class="lbl">🔗 节点</div>
                <div class="num green">)rawliteral"
         + String(nodeCount) + R"rawliteral(</div>
            </div>
            <div class="stat">
                <div class="lbl">📡 DTU</div>
                <div class="num )rawliteral"
         + String(dtuOnline ? "green" : "red") + R"rawliteral(">)rawliteral"
         + String(dtuOnline ? "在线" : "离线") + R"rawliteral(</div>
            </div>
            <div class="stat">
                <div class="lbl">💾 内存</div>
                <div class="num">)rawliteral"
         + String(ESP.getFreeHeap() / 1024) + R"rawliteral(<small>KB</small></div>
            </div>
            <div class="card">
            <h2>💬 消息广播</h2>
            <input type="number" id="msgTarget" placeholder="节点ID (0=全部, 填1-65534=指定)" value="0" style="width:100%;padding:8px;margin:5px 0">
            <input type="text" id="msgText" placeholder="输入消息..." style="width:100%;padding:8px;margin:5px 0">
            <button onclick="sendGwMsg()">📤 发送</button>
            <p id="msgStatus" style="font-size:12px;color:#888;margin-top:5px"></p>
            </div>
        </div>
        <div class="card">
    <h2>📨 已收消息</h2>
    <div id="msgList" style="max-height:150px;overflow-y:auto;font-size:12px"></div>
</div>
                <div class="stats" style="margin-top:5px">
            <div class="stat">
                <div class="lbl">📶 4G信号</div>
                <div class="num )rawliteral"
         + String(dtuSignal >= 0 ? (dtuSignal >= 17 ? "green" : "gold") : "red") + R"rawliteral(">)rawliteral"
         + String(dtuSignal >= 0 ? String(dtuSignal) + "<small>/31</small>" : "--") + R"rawliteral(</div>
            </div>
            <div class="stat">
                <div class="lbl">🕐 网络时间</div>
                <div class="num" style="font-size:0.85em">)rawliteral"
         + (dtuNetTime.length() > 0 ? dtuNetTime : "--") + R"rawliteral(</div>
            </div>
        </div>
        <!-- 标签页切换 -->
        <div class="tabs">
            <button class="tab active" onclick="showTab('nodes')">🔗 节点管理</button>
            <button class="tab" onclick="showTab('map')">🗺️ 拓扑</button>
            <button class="tab" onclick="showTab('config')">⚙️ 配置</button>
            <button class="tab" onclick="showTab('location')">📍 定位</button>
            <button class="tab" onclick="showTab('log')">📋 日志</button>
            <button class="tab" onclick="showTab('deploy')">🚀 部署</button>
        </div>
        
        <!-- Toast 消息 -->
        <div class="toast" id="toast"></div>
        
        <!-- 标签页1: 节点管理 -->
        <div id="nodes" class="tab-content active">
            <div class="card">
                <h2>🔗 已注册节点</h2>
                <table>
                    <thead>
                        <tr>
                            <th>ID</th>
                            <th>IP</th>
                            <th>预分配坐标</th>
                            <th>RSSI</th>
                            <th>距离</th>
                            <th>状态</th>
                            <th>预警</th>
                        </tr>
                    </thead>
                    <tbody id="nodeTable">
                    </tbody>
                </table>
                <div id="noNodes" style="color:var(--muted);padding:20px;text-align:center;display:none">
                    暂无注册节点
                </div>
            </div>
            
            <div class="card">
                <h2>➕ 预分配节点坐标</h2>
                <p style="font-size:0.8em;color:var(--muted);margin-bottom:10px">
                    🔍 输入地址搜索，或直接点击地图微调位置
                </p>
                
                <!-- 地址搜索 -->
                <div style="display:flex;gap:8px;margin-bottom:10px">
                    <input type="text" id="addressInput" placeholder="输入地址,如:村口老槐树" style="flex:1">
                    <button onclick="searchAddress()" style="width:auto;padding:10px 20px">🔍 搜索</button>
                </div>
                
                <!-- 静态地图 + 准心 -->
                <div id="mapWrapper" style="position:relative;width:100%;height:300px;margin-bottom:10px;background:#0a1628;border-radius:6px;overflow:hidden">
                    <img id="staticMap" src="" style="width:100%;height:100%;object-fit:cover;display:none">
                    <!-- 准心 -->
                    <div style="position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:40px;height:40px;pointer-events:none">
                        <div style="position:absolute;top:50%;left:0;width:100%;height:2px;background:#e94560"></div>
                        <div style="position:absolute;top:0;left:50%;width:2px;height:100%;background:#e94560"></div>
                        <div style="position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:12px;height:12px;border:2px solid #e94560;border-radius:50%"></div>
                    </div>
                    <p id="mapHint" style="position:absolute;bottom:10px;width:100%;text-align:center;color:#888;font-size:12px">输入地址后显示地图</p>
                </div>
                
                <!-- 坐标显示 -->
                <input type="text" id="selectedCoord" readonly placeholder="选中坐标" style="background:#0a1628;text-align:center;margin-bottom:10px">
                
                <!-- 微调按钮 -->
                <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;margin-bottom:10px">
                    <button onclick="nudgeMap(0, 0.0001)" style="padding:8px">⬆️ 北</button>
                    <button onclick="nudgeMap(0.0001, 0)" style="padding:8px">➡️ 东</button>
                    <button onclick="nudgeMap(0, -0.0001)" style="padding:8px">⬇️ 南</button>
                    <button onclick="nudgeMap(-0.0001, 0)" style="padding:8px">⬅️ 西</button>
                    <button onclick="updateMapZoom(1)" style="padding:8px">🔍+</button>
                    <button onclick="updateMapZoom(-1)" style="padding:8px">🔍-</button>
                </div>
                
                
                <form onsubmit="addNode(event)">
                <!-- 节点ID -->
                <input type="number" id="newID" placeholder="节点ID (1-65534)" min="1" max="65534" required>
    
                <!-- 经纬度 + 海拔在同一行 -->
                <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin:6px 0">
                <input type="text" id="newLat" placeholder="纬度" value="">
                <input type="text" id="newLng" placeholder="经度" value="">
                <input type="number" id="newAlt" placeholder="海拔(m)" step="0.1" value="0">
                </div>
    
                <!-- 获取海拔按钮 -->
                <button type="button" onclick="fetchAltitudeForNode()">🗻 获取海拔</button>
    
                <!-- 预分配按钮 -->
                <button type="submit">📌 预分配</button>
                </form>
            </div>
        </div>
        
        <!-- 标签页2: 拓扑 -->
        <div id="map" class="tab-content">
            <div class="card">
                <h2>🗺️ 网络拓扑</h2>
                <div id="topology" style="font-family:monospace;font-size:0.85em;line-height:2;white-space:pre-wrap"></div>
            </div>
        </div>
        
        <!-- 标签页3: 配置 -->
        <div id="config" class="tab-content">
            <div class="card">
                <h2>📶 WiFi 设置</h2>
                <input type="text" id="cfgSSID" placeholder="WiFi SSID" value=")rawliteral"
         + config_ssid + R"rawliteral(">
                <input type="password" id="cfgPass" placeholder="WiFi 密码">
            </div>
            
            <div class="card">
                <h2>📱 报警设置</h2>
                <input type="text" id="cfgDingtalk" placeholder="钉钉机器人 Token" value=")rawliteral"
         + config_dingtalk + R"rawliteral(">
                
                <input type="number" id="cfgThreshold" placeholder="AI阈值 (默认0.5)" step="0.01" min="0" max="1" value=")rawliteral"
         + String(config_alertThreshold, 2) + R"rawliteral(">
            </div>
            
            <div class="card">
                <h2>⚡ 操作</h2>
                <button onclick="saveConfig()">💾 保存配置并重启</button>
                <button class="secondary" onclick="if(confirm('确定重启网关？')){location.href='/restart'}">🔄 重启网关</button>
            </div>
        </div>
        
        <!-- 标签页4: 定位 -->
        <div id="location" class="tab-content">
            <div class="card">
                <h2>📍 网关定位</h2>
                <table>
                    <tr><td>纬度</td><td class="coord" id="locLat">)rawliteral"
         + String(gwLat, 6) + R"rawliteral(</td></tr>
                    <tr><td>经度</td><td class="coord" id="locLng">)rawliteral"
         + String(gwLng, 6) + R"rawliteral(</td></tr>
                    <tr><td>海拔</td><td>)rawliteral"
         + String(gwAlt, 1) + R"rawliteral( m</td></tr>
                    <tr><td>定位方式</td><td><span class="tag tag-info">)rawliteral"
         + gwLocationMethod + R"rawliteral(</span></td></tr>
                    <tr><td>状态</td><td>)rawliteral"
         + String(gwLocationValid ? "<span class='tag tag-ok'>✅ 已定位</span>" : "<span class='tag tag-warn'>⚠️ 未定位</span>") + R"rawliteral(</td></tr>
                </table>
                <button onclick="location.href='/requestLoc'">🔄 刷新定位 (基站/WiFi)</button>
                <button class="secondary" onclick="location.href='/broadcast'">📡 广播位置给节点</button>
                <!-- 🔥 新增：导航到节点 -->
                <h3 style="margin-top:15px">🗺️ 导航到节点</h3>
                <select id="navNodeSelect" style="width:100%;padding:10px;margin:6px 0;border-radius:6px;border:1px solid var(--border);background:var(--input-bg);color:var(--text);font-size:0.9em">
                    <option value="">-- 选择节点 --</option>
                </select>
                <button onclick="navigateToNode()" style="background:#4ecca3">🚶 导航到该节点</button>
            </div>
        </div>
        
        <!-- 标签页5: 日志 -->
        <div id="log" class="tab-content">
            <div class="card">
                <h2>📋 运行统计</h2>
                <table>
                    <tr><td>数据包</td><td class="green">)rawliteral"
         + String(totalPackets) + R"rawliteral(</td></tr>
                    <tr><td>预警触发</td><td class="red">)rawliteral"
         + String(totalAlerts) + R"rawliteral(</td></tr>
                    <tr><td>中继转发</td><td class="gold">)rawliteral"
         + String(totalRelays) + R"rawliteral(</td></tr>
                    <tr><td>DTU掉线</td><td>)rawliteral"
         + String(totalDTUFails) + R"rawliteral( 次</td></tr>
                    <tr><td>MQTT</td><td>)rawliteral"
         + String(mqttClient.connected() ? "<span class='online'>已连接</span>" : "<span class='offline'>断开</span>") + R"rawliteral(</td></tr>
                    <tr><td>WiFi</td><td>)rawliteral"
         + String(WiFi.isConnected() ? "<span class='online'>" + WiFi.localIP().toString() + "</span>" : "<span class='offline'>断开</span>") + R"rawliteral(</td></tr>
                    <tr><td>运行时间</td><td>)rawliteral"
         + String(millis() / 1000) + R"rawliteral( 秒</td></tr>
                    <tr><td>剩余内存</td><td>)rawliteral"
         + String(ESP.getFreeHeap() / 1024) + R"rawliteral( KB</td></tr>
                </table>
            </div>
        </div>
                <!-- 标签页6: 部署 -->
        <div id="deploy" class="tab-content">
            <div class="card">
                <h2>🚀 部署模式</h2>
                <p>WiFi AP: <strong id="apStatus">未开启</strong></p>
                <p>注册节点: <strong id="deployCount">0</strong></p>
                <button onclick="location.href='/startDeploy'">📡 开启部署AP</button>
                <button class="secondary" onclick="location.href='/stopDeploy'">⏹️ 停止部署</button>
                <button class="secondary" onclick="location.href='/deployBroadcast'">📢 广播拓扑</button>
            </div>
            
            <div class="card">
                <h2>📋 注册列表</h2>
                <table>
                    <thead><tr><th>节点ID</th><th>分配IP</th><th>纬度</th><th>经度</th><th>海拔</th></tr></thead>
                    <tbody id="deployTable"></tbody>
                </table>
            </div>
        </div>
    </div>
    
    <script>
        var currentLat = window.gwLat || 31.2304;
        var currentLng = window.gwLng || 121.4737;
        var currentZoom = 15;
        var amapKey = 'a8c640d3f0ea6c654a66cc22e0ed6106';  // 你的高德Key

        function fetchAltitudeForNode() {
    var lat = parseFloat(document.getElementById('newLat').value);
    var lng = parseFloat(document.getElementById('newLng').value);
    if (isNaN(lat) || isNaN(lng)) {
        toast('请先输入有效的经纬度');
        return;
    }
    
    var btn = event.target;
    btn.disabled = true;
    btn.textContent = '查询中...';
    
    // 改成请求 ESP32 的代理接口
    fetch('/api/altitude?lat=' + lat.toFixed(6) + '&lng=' + lng.toFixed(6))
        .then(function(r) { return r.json(); })
        .then(function(d) {
            if (d.elevation !== undefined) {
                document.getElementById('newAlt').value = d.elevation.toFixed(1);
                toast('海拔：' + d.elevation.toFixed(1) + ' m');
            } else {
                toast('未获取到海拔数据');
            }
        })
        .catch(function(e) {
            toast('请求失败：' + e.message);
        })
        .finally(function() {
            btn.disabled = false;
            btn.textContent = '🗻 获取海拔';
        });
}
        
        function updateStaticMap() {
            var url = 'https://restapi.amap.com/v3/staticmap?location=' 
                    + currentLng + ',' + currentLat 
                    + '&zoom=' + currentZoom 
                    + '&size=800*600'
                    + '&markers=mid,,A:' + currentLng + ',' + currentLat
                    + '&key=' + amapKey;
            
            document.getElementById('staticMap').src = url;
            document.getElementById('staticMap').style.display = 'block';
            document.getElementById('mapHint').style.display = 'none';
            document.getElementById('selectedCoord').value = currentLat.toFixed(6) + ', ' + currentLng.toFixed(6);
            document.getElementById('newLat').value = currentLat.toFixed(6);
            document.getElementById('newLng').value = currentLng.toFixed(6);
        }
        
        function searchAddress() {
            var addr = document.getElementById('addressInput').value;
            if (!addr) return;
            
            fetch('https://restapi.amap.com/v3/geocode/geo?address=' + encodeURIComponent(addr) + '&key=' + amapKey)
                .then(r => r.json())
                .then(d => {
                    if (d.geocodes && d.geocodes.length > 0) {
                        var loc = d.geocodes[0].location.split(',');
                        currentLng = parseFloat(loc[0]);
                        currentLat = parseFloat(loc[1]);
                        currentZoom = 15;
                        updateStaticMap();
                    }
                });
        }
        
        function nudgeMap(dLat, dLng) {
            currentLat += dLat;
            currentLng += dLng;
            updateStaticMap();
        }
        
        function updateMapZoom(delta) {
            currentZoom = Math.max(5, Math.min(18, currentZoom + delta));
            updateStaticMap();
        }
        // 高德地图懒加载器
        var _amap_script_loaded = false;
        var _amap_ready = false;

        function loadAMapScript(callback) {
            if (_amap_ready) { callback(); return; }
            if (_amap_script_loaded) {
                var check = setInterval(function() {
                    if (_amap_ready) { clearInterval(check); callback(); }
                }, 100);
                return;
            }
            _amap_script_loaded = true;
            var script = document.createElement('script');
            script.src = 'https://webapi.amap.com/maps?v=2.0&key=a8c640d3f0ea6c654a66cc22e0ed6106';
             script.onload = function() {
                _amap_ready = true;
                if (callback) callback();
        };
            document.head.appendChild(script);
        }

        // 原来的 initMap 函数不需要改，只需要调用上述异步加载
        var mapReady = false;
        function tryInitMap() {
            if (mapReady || typeof AMap === 'undefined') return;
            mapReady = true;
            map = new AMap.Map('mapContainer', {
                zoom: 15,
                center: [window.gwLng || 121.4737, window.gwLat || 31.2304],
            });
            map.on('click', function(e) {
                var lng = e.lnglat.getLng();
                var lat = e.lnglat.getLat();
                document.getElementById('selectedCoord').value = lat.toFixed(6) + ', ' + lng.toFixed(6);
                document.getElementById('newLat').value = lat.toFixed(6);
                document.getElementById('newLng').value = lng.toFixed(6);
                 if (marker) { map.remove(marker); }
                marker = new AMap.Marker({ position: [lng, lat], map: map });
            });
        }
        // ===== 填充节点下拉框 =====
        function fillNodeSelect() {
            fetch('/api/nodes')
                .then(r => r.json())
                .then(data => {
                    let sel = document.getElementById('navNodeSelect');
                    if (!sel) return;
                    sel.innerHTML = '<option value="">-- 选择节点 --</option>';
                    data.forEach(n => {
                        sel.innerHTML += '<option value="' + n.id + '|' + n.a_lat + '|' + n.a_lng + '">'
                            + 'Node ' + n.id + ' (' + n.a_lat.toFixed(4) + ',' + n.a_lng.toFixed(4) + ')'
                            + '</option>';
                    });
                });
        }
        
        // ===== 导航到选中节点 =====
        function navigateToNode() {
    let sel = document.getElementById('navNodeSelect');
    if (!sel || !sel.value) { alert('请先选择节点'); return; }

    let parts = sel.value.split('|');
    let nlat = parseFloat(parts[1]);
    let nlng = parseFloat(parts[2]);

    // 网关坐标作为起点
    var glat = window.gwLat;
    var glng = window.gwLng;

    var ua = navigator.userAgent.toLowerCase();
    var isIOS = /iphone|ipad|ipod/.test(ua);
    var isAndroid = /android/.test(ua);

    // Web版高德地图兜底（任何设备都能打开）
    var fallback = 'https://uri.amap.com/navigation?';
    if (glat && glng) {
        fallback += 'from=' + glng + ',' + glat + ',起点&';
    }
    fallback += 'to=' + nlng + ',' + nlat + ',Node&mode=walk&callnative=1';

    if (isAndroid) {
        var s = 'androidamap://navi?sourceApplication=EQGateway&lat=' + nlat + '&lon=' + nlng + '&dev=0&style=2';
        var t = Date.now(); window.location.href = s;
        setTimeout(function() {
            if (Date.now() - t < 2500) window.location.href = fallback;
        }, 2000);
    } else if (isIOS) {
        var s = 'iosamap://navi?sourceApplication=EQGateway&lat=' + nlat + '&lon=' + nlng + '&dev=0&style=2';
        var t = Date.now(); window.location.href = s;
        setTimeout(function() {
            if (Date.now() - t < 2500) window.location.href = fallback;
        }, 2000);
    } else {
        window.open(fallback, '_blank');
    }
}
        function initMap() {
            map = new AMap.Map('mapContainer', {
                zoom: 15,
                center: [window.gwLng || 121.4737, window.gwLat || 31.2304],
            });
            map.on('click', function(e) {
                var lng = e.lnglat.getLng();
                var lat = e.lnglat.getLat();
                document.getElementById('selectedCoord').value = lat.toFixed(6) + ', ' + lng.toFixed(6);
                document.getElementById('newLat').value = lat.toFixed(6);
                document.getElementById('newLng').value = lng.toFixed(6);
                if (marker) { map.remove(marker); }
                marker = new AMap.Marker({ position: [lng, lat], map: map });
            });
        }
        // ===== 标签页切换 =====
        function showTab(name) {
            document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            document.getElementById(name).classList.add('active');
            event.target.classList.add('active');
            if (name === 'nodes' && !map) {
            loadAMapScript(function() {
                setTimeout(tryInitMap, 200);
            });
        }
        }
        
        // ===== Toast 消息 =====
        function toast(msg) {
            const t = document.getElementById('toast');
            t.textContent = msg;
            t.style.display = 'block';
            setTimeout(() => t.style.display = 'none', 2500);
        }
        
        // ===== 节点表格刷新 =====
        function refreshNodes() {
            fetch('/api/nodes')
                .then(r => r.json())
                .then(data => {
                    let tb = document.getElementById('nodeTable');
                    let noN = document.getElementById('noNodes');
                    
                    if (data.length === 0) {
                        tb.innerHTML = '';
                        noN.style.display = 'block';
                    } else {
                        noN.style.display = 'none';
                        tb.innerHTML = data.map(n => {
                            let status = n.online 
                                ? '<span class="online"><span class="pulse pulse-g"></span>在线</span>'
                                : '<span class="offline"><span class="pulse pulse-r"></span>离线</span>';
                            let alert = n.alert > 0 
                                ? '<span class="tag tag-warn">🔴 Lv' + n.alert + '</span>'
                                : '<span class="tag tag-ok">🟢 正常</span>';
                            let verified = n.verified 
                                ? ''
                                : ' <span class="tag tag-warn" title="位置待验证">⚠️</span>';
                            
                            return '<tr>' +
                                '<td>' + n.id + '</td>' +
                                '<td class="coord">' + (n.ip || '--') + '</td>' +
                                '<td class="coord">' + (n.v_lat && n.v_lat != 0 ? n.v_lat.toFixed(4) : (n.a_lat ? n.a_lat.toFixed(4) : '--')) + ',' 
    + (n.v_lng && n.v_lng != 0 ? n.v_lng.toFixed(4) : (n.a_lng ? n.a_lng.toFixed(4) : '--')) + verified + '</td>' +
                                '<td>' + (n.rssi || 0) + ' dBm</td>' +
                                '<td>' + (n.dist > 0 ? n.dist.toFixed(0) + 'm' : '--') + '</td>' +
                                '<td>' + status + '</td>' +
                                '<td>' + alert + '</td>' +
                                '</tr>';
                        }).join('');
                    }
                });
        }
        
        // ===== 拓扑渲染 =====
        function refreshTopology() {
            fetch('/api/nodes')
                .then(r => r.json())
                .then(data => {
                    let div = document.getElementById('topology');
                    if (data.length === 0) {
                        div.innerHTML = '<span style="color:#888">暂无节点数据</span>';
                        return;
                    }
                    
                    let out = '<span style="color:#4ecca3">[GATEWAY]</span> '
                        + window.gwLat.toFixed(4) + ', ' + window.gwLng.toFixed(4) + '\n';
                    out += '  ├── 状态: ' + (window.gwOnline ? '✅ 在线' : '⚠️ 未定位') + '\n';
                    out += '  │\n';
                    
                    data.forEach((n, i) => {
                        let prefix = (i === data.length - 1) ? '  └──' : '  ├──';
                        let dist = n.dist > 0 ? n.dist.toFixed(0) + 'm' : '?m';
                        out += prefix + ' <span style="color:#f5c518">[Node ' + n.id + ']</span> '
                            + (n.a_lat ? n.a_lat.toFixed(4) : '?') + ', ' 
                            + (n.a_lng ? n.a_lng.toFixed(4) : '?')
                            + ' <span style="color:#888">(' + dist + ')</span>\n';
                    });
                    
                    div.innerHTML = out;
                });
        }
                
        function refreshDeploy() {
            fetch('/api/deploy')
                .then(r => r.json())
                .then(d => {
                    document.getElementById('apStatus').textContent = d.active ? '✅ EQ_Deploy' : '未开启';
                    document.getElementById('deployCount').textContent = d.count;
                    let tb = document.getElementById('deployTable');
                    if (d.nodes.length === 0) {
                        tb.innerHTML = '<tr><td colspan="5" style="color:#888">暂无注册</td></tr>';
                    } else {
                        tb.innerHTML = d.nodes.map(n => 
                            '<tr><td>' + n.id + '</td><td class="coord">' + n.ip + '</td><td class="coord">' + n.lat.toFixed(4) + '</td><td class="coord">' + n.lng.toFixed(4) + '</td><td>' + n.alt + 'm</td></tr>'
                        ).join('');
                    }
                });
        }
        // ===== 添加节点 =====
        function addNode(e) {
            e.preventDefault();
            let params = new URLSearchParams({
                id: document.getElementById('newID').value,
                lat: document.getElementById('newLat').value,
                lng: document.getElementById('newLng').value,
                alt: document.getElementById('newAlt').value
            });
            
            fetch('/addNode?' + params)
                .then(r => r.text())
                .then(msg => {
                    toast(msg);
                    document.getElementById('newID').value = '';
                    document.getElementById('newLat').value = '';
                    document.getElementById('newLng').value = '';
                    refreshNodes();
                    refreshTopology();
                });
            return false;
        }
        
        // ===== 保存配置 =====
        function saveConfig() {
            let params = new URLSearchParams({
                ssid: document.getElementById('cfgSSID').value,
                pass: document.getElementById('cfgPass').value,
                dingtalk: document.getElementById('cfgDingtalk').value,
                
                threshold: document.getElementById('cfgThreshold').value
            });
            
            fetch('/saveConfig?' + params)
                .then(r => r.text())
                .then(msg => {
                    toast(msg);
                    setTimeout(() => location.reload(), 3000);
                });
        }
        function sendGwMsg() {
            var t = document.getElementById('msgTarget').value;
            var m = document.getElementById('msgText').value;
            fetch('/sendGwMsg?target=' + t + '&text=' + encodeURIComponent(m))
                .then(r => r.text())
                .then(a => {
                    document.getElementById('msgStatus').textContent = a;
                    document.getElementById('msgText').value = '';
                });
        }
        function refreshMessages() {
    fetch('/api/messages')
        .then(r => r.json())
        .then(data => {
            let html = '';
            data.forEach(m => {
                html += '<div style="padding:3px 0;border-bottom:1px solid #333;font-size:12px">';
                html += '<span style="color:#f5c518">[节点 ' + m.from + ']</span> ';
                html += m.text;
                html += '</div>';
            });
            document.getElementById('msgList').innerHTML = html || '暂无消息';
        });
}
setInterval(refreshMessages, 5000);
        
        // ===== 全局变量初始化 =====
        window.gwLat = )rawliteral"
         + String(gwLat, 6) + R"rawliteral(;
        window.gwLng = )rawliteral"
         + String(gwLng, 6) + R"rawliteral(;
        window.gwOnline = )rawliteral"
         + String(gwLocationValid ? "true" : "false") + R"rawliteral(;
        
        // ===== 初始化 =====
        refreshNodes();
        refreshTopology();
        setInterval(refreshNodes, 10000);
        setInterval(refreshTopology, 30000);
        refreshDeploy();
        setInterval(refreshDeploy, 5000);
        fillNodeSelect();
        setInterval(fillNodeSelect, 10000);
        
        // ===== 从URL参数切换标签页 =====
        (function() {
            let tab = new URLSearchParams(location.search).get('tab');
            if (tab) showTab(tab);
        })();
    </script>
</body>
</html>
    )rawliteral";

  return html;
}

void setupWebServer() {
  server.on("/", []() {
    // 每 60 秒重新生成一次缓存
    if (cachedWebPage.length() == 0 || millis() - cachedWebPageTime > 60000) {
      cachedWebPage = generateWebPage();
      cachedWebPageTime = millis();
      Serial.printf("[Web] Cached homepage (%d bytes)\n", cachedWebPage.length());
    }
    server.send(200, "text/html", cachedWebPage);
  });
  server.on("/api/altitude", []() {
    if (server.hasArg("lat") && server.hasArg("lng")) {
      float lat = server.arg("lat").toFloat();
      float lng = server.arg("lng").toFloat();
      float alt = fetchAltitude(lat, lng);
      String json = "{\"elevation\":" + String(alt, 1) + "}";
      server.send(200, "application/json", json);
    } else {
      server.send(400, "application/json", "{\"error\":\"missing lat/lng\"}");
    }
  });

  server.on("/api/status", []() {
    String json = "{";
    json += "\"packets\":" + String(totalPackets) + ",";
    json += "\"alerts\":" + String(totalAlerts) + ",";
    json += "\"relays\":" + String(totalRelays) + ",";
    json += "\"nodes\":" + String(nodeCount) + ",";
    json += "\"gwValid\":" + String(gwLocationValid ? "true" : "false") + ",";
    json += "\"gwLat\":" + String(gwLat, 6) + ",";
    json += "\"gwLng\":" + String(gwLng, 6) + ",";
    json += "\"gwMethod\":\"" + gwLocationMethod + "\",";
    json += "\"dtuOnline\":" + String(dtuOnline ? "true" : "false") + ",";
    json += "\"mqttConnected\":" + String(mqttClient.connected() ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/api/nodes", []() {
    String json;
    json.reserve(2000);
    json = "[";

    for (int i = 0; i < nodeCount; i++) {
      if (i > 0) json += ",";
      json += "{";
      json += "\"id\":" + String(nodes[i].nodeID) + ",";

      // 优先使用验证后的坐标
      float displayLat = (nodes[i].positionVerified && nodes[i].verifiedLat != 0)
                           ? nodes[i].verifiedLat
                           : nodes[i].assignedLat;
      float displayLng = (nodes[i].positionVerified && nodes[i].verifiedLng != 0)
                           ? nodes[i].verifiedLng
                           : nodes[i].assignedLng;

      json += "\"a_lat\":" + String(displayLat, 6) + ",";
      json += "\"a_lng\":" + String(displayLng, 6) + ",";
      json += "\"v_lat\":" + String(nodes[i].verifiedLat, 6) + ",";
      json += "\"v_lng\":" + String(nodes[i].verifiedLng, 6) + ",";
      json += "\"ip\":\"" + nodes[i].assignedIP.toString() + "\",";
      json += "\"rssi\":" + String(nodes[i].lastRSSI) + ",";
      json += "\"dist\":" + String(nodes[i].estimatedDistance, 1) + ",";
      json += "\"online\":" + String((millis() - nodes[i].lastSeen < 120000) ? "true" : "false") + ",";
      json += "\"verified\":" + String(nodes[i].positionVerified ? "true" : "false");
      json += "}";
    }
    json += "]";
    server.send(200, "application/json", json);
  });

  // 节点预分配
  server.on("/addNode", []() {
    if (server.hasArg("id") && server.hasArg("lat") && server.hasArg("lng")) {
      uint16_t id = server.arg("id").toInt();
      float lat = server.arg("lat").toFloat();
      float lng = server.arg("lng").toFloat();
      float alt = server.hasArg("alt") ? server.arg("alt").toFloat() : 0;

      for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].nodeID == id) {
          nodes[i].assignedLat = lat;
          nodes[i].assignedLng = lng;
          nodes[i].assignedAlt = alt;
          server.send(200, "text/plain", "Node " + String(id) + " updated");
          return;
        }
      }

      if (nodeCount < MAX_NODES) {
        nodes[nodeCount].nodeID = id;
        nodes[nodeCount].assignedLat = lat;
        nodes[nodeCount].assignedLng = lng;
        nodes[nodeCount].assignedAlt = alt;
        nodes[nodeCount].assignedIP = IPAddress(192, 168, 4, 100 + nodeCount);
        nodeCount++;
        server.send(200, "text/plain", "Node " + String(id) + " added (IP: 192.168.4." + String(100 + nodeCount - 1) + ")");
      } else {
        server.send(400, "text/plain", "Max nodes reached");
      }
    } else {
      server.send(400, "text/plain", "Missing parameters");
    }
  });

  server.on("/saveConfig", []() {
    if (server.hasArg("ssid")) config_ssid = server.arg("ssid");
    if (server.hasArg("pass")) config_pass = server.arg("pass");
    if (server.hasArg("dingtalk")) config_dingtalk = server.arg("dingtalk");
    if (server.hasArg("threshold")) config_alertThreshold = server.arg("threshold").toFloat();

    saveConfig();
    networkManager.setConfig(config_ssid, config_pass);
    server.send(200, "text/plain", "Saved. Restarting...");
    delay(100);  // 不要 delay，直接重启
    ESP.restart();
  });

  server.on("/restart", []() {
    server.send(200, "text/plain", "Restarting...");
    delay(100);
    ESP.restart();
  });

  server.on("/requestLoc", []() {
    server.send(200, "text/plain", "Location requested");  // 先返回
    // 用标志位通知 Core0 去查询（Core0 会自己定期查，或设置一个标志）
    forceLocRequest = true;  // 强制下次立刻查询
  });

  server.on("/broadcast", []() {
    broadcastGatewayPosition();
    broadcastNeighborList();
    server.send(200, "text/plain", "Broadcasted");
  });

  // ===== 部署模式 =====
  server.on("/startDeploy", []() {
    deploymentMode = true;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("EQ_Deploy", "12345678");
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    deploymentServer.begin();
    deploymentServer.setNoDelay(true);
    deployRequestCount = 0;
    nextDHCPIndex = 0;
    Serial.println("[DEPLOY] AP started: EQ_Deploy / 12345678");
    server.send(200, "text/plain", "✅ AP started. SSID: EQ_Deploy, PW: 12345678");
  });

  server.on("/stopDeploy", []() {
    deploymentMode = false;
    deploymentServer.stop();
    WiFi.softAPdisconnect(true);
    server.send(200, "text/plain", "Deployment stopped");
  });

  server.on("/deployBroadcast", []() {
    for (int i = 0; i < deployRequestCount; i++) {
      uint8_t buf[18];
      buf[0] = FRAME_HEADER;
      buf[1] = 0x10;
      buf[2] = 14;
      buf[3] = (deployRequests[i].nodeID >> 8) & 0xFF;
      buf[4] = deployRequests[i].nodeID & 0xFF;
      memcpy(&buf[5], &deployRequests[i].lat, 4);
      memcpy(&buf[9], &deployRequests[i].lng, 4);
      memcpy(&buf[13], &deployRequests[i].alt, 4);
      sendLoRaPacket(buf, 17);
      delay(100);
    }
    server.send(200, "text/plain", "Broadcasted to " + String(deployRequestCount) + " nodes");
  });

  server.on("/api/deploy", []() {
    String json;
    json.reserve(1000);
    json = "{";
    json += "\"active\":" + String(deploymentMode ? "true" : "false") + ",";
    json += "\"count\":" + String(deployRequestCount) + ",";
    json += "\"nodes\":[";
    for (int i = 0; i < deployRequestCount; i++) {
      if (i > 0) json += ",";
      json += "{";
      json += "\"id\":" + String(deployRequests[i].nodeID) + ",";
      json += "\"ip\":\"" + deployRequests[i].nodeIP.toString() + "\",";
      json += "\"lat\":" + String(deployRequests[i].lat, 6) + ",";
      json += "\"lng\":" + String(deployRequests[i].lng, 6) + ",";
      json += "\"alt\":" + String(deployRequests[i].alt, 1);
      json += "}";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });
  server.on("/sendGwMsg", []() {
    if (server.hasArg("target") && server.hasArg("text")) {
      uint16_t target = server.arg("target").toInt();
      String text = server.arg("text");

      uint8_t* payload = (uint8_t*)malloc(200);
      if (!payload) {
        server.send(500, "text/plain", "Memory error");
        return;
      }
      payload[0] = (target >> 8) & 0xFF;
      payload[1] = target & 0xFF;
      payload[2] = 0;
      int len = text.length() > 196 ? 196 : text.length();
      memcpy(payload + 3, text.c_str(), len);

      uint8_t* buf = (uint8_t*)malloc(len + 8);
      if (!buf) {
        free(payload);
        server.send(500, "text/plain", "Memory error");
        return;
      }
      buf[0] = FRAME_HEADER;
      buf[1] = FRAME_MESSAGE;
      buf[2] = len + 3;
      buf[3] = 0xFF;
      buf[4] = 0xFF;
      memcpy(buf + 5, payload, len + 3);
      free(payload);

      // 放入 Core0 的发送队列
      if (xQueueSend(txQueue, &buf, 0) != pdTRUE) {
        free(buf);
        server.send(500, "text/plain", "Queue full");
        return;
      }
      server.send(200, "text/plain", "✅ 已发送");
    }
  });
  server.on("/api/messages", []() {
    String json = "[";
    for (int i = 0; i < gwMsgCount; i++) {
      if (i > 0) json += ",";
      json += "{";
      json += "\"from\":" + String(gwMsgStore[i].fromNode) + ",";
      json += "\"to\":" + String(gwMsgStore[i].toNode) + ",";
      json += "\"time\":" + String(gwMsgStore[i].timestamp) + ",";
      json += "\"text\":\"" + String(gwMsgStore[i].text) + "\"";
      json += "}";
    }
    json += "]";
    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("[Web] Server started");
}

void syncDeployToNodeTable(uint16_t nid, float lat, float lng, float alt, IPAddress ip) {
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].nodeID == nid) {
      nodes[i].assignedLat = lat;
      nodes[i].assignedLng = lng;
      nodes[i].assignedAlt = alt;
      nodes[i].assignedIP = ip;
      nodes[i].registered = true;
      return;
    }
  }
  if (nodeCount < MAX_NODES) {
    nodes[nodeCount].nodeID = nid;
    nodes[nodeCount].assignedLat = lat;
    nodes[nodeCount].assignedLng = lng;
    nodes[nodeCount].assignedAlt = alt;
    nodes[nodeCount].assignedIP = ip;
    nodes[nodeCount].registered = true;
    nodes[nodeCount].lastSeen = millis();
    nodeCount++;
  }
}

void handleDeploymentClients() {
  // 接受新连接
  WiFiClient newClient = deploymentServer.accept();
  if (newClient) {
    for (int i = 0; i < 20; i++) {
      if (!deploymentClients[i] || !deploymentClients[i].connected()) {
        deploymentClients[i] = newClient;
        Serial.printf("[DEPLOY] New client: %s\n",
                      newClient.remoteIP().toString().c_str());
        break;
      }
    }
  }

  // 处理每个客户端的数据
  for (int i = 0; i < 20; i++) {
    if (!deploymentClients[i] || !deploymentClients[i].connected()) continue;

    while (deploymentClients[i].available() > 0) {
      String line = deploymentClients[i].readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;

      // 解析: {"cmd":"register","id":3,"lat":31.23,"lng":121.47,"alt":15}
      if (line.indexOf("\"cmd\":\"register\"") > 0) {
        auto findVal = [&](String key) -> String {
          int s = line.indexOf("\"" + key + "\":");
          if (s < 0) return "";
          s += key.length() + 3;
          int e = line.indexOf(",", s);
          if (e < 0) e = line.indexOf("}", s);
          return line.substring(s, e);
        };

        uint16_t nid = findVal("id").toInt();
        float nlat = findVal("lat").toFloat();
        float nlng = findVal("lng").toFloat();
        float nalt = findVal("alt").toFloat();

        if (nid > 0 && nlat != 0 && nlng != 0) {
          IPAddress assignedIP(192, 168, 4, 100 + nextDHCPIndex);
          nextDHCPIndex++;

          bool found = false;
          for (int j = 0; j < deployRequestCount; j++) {
            if (deployRequests[j].nodeID == nid) {
              deployRequests[j].lat = nlat;
              deployRequests[j].lng = nlng;
              deployRequests[j].alt = nalt;
              deployRequests[j].nodeIP = assignedIP;
              deployRequests[j].confirmed = true;
              found = true;
              break;
            }
          }
          if (!found && deployRequestCount < 50) {
            deployRequests[deployRequestCount].nodeID = nid;
            deployRequests[deployRequestCount].lat = nlat;
            deployRequests[deployRequestCount].lng = nlng;
            deployRequests[deployRequestCount].alt = nalt;
            deployRequests[deployRequestCount].nodeIP = assignedIP;
            deployRequests[deployRequestCount].confirmed = true;
            deployRequestCount++;
          }

          syncDeployToNodeTable(nid, nlat, nlng, nalt, assignedIP);

          // 构造响应（含邻居信息）
          String resp = "{\"status\":\"ok\",";
          resp += "\"assigned_ip\":\"" + assignedIP.toString() + "\",";
          resp += "\"gateway_lat\":" + String(gwLat, 6) + ",";
          resp += "\"gateway_lng\":" + String(gwLng, 6) + ",";
          resp += "\"neighbors\":[";
          bool first = true;
          for (int j = 0; j < deployRequestCount; j++) {
            if (deployRequests[j].nodeID != nid && deployRequests[j].confirmed) {
              if (!first) resp += ",";
              resp += "{\"id\":" + String(deployRequests[j].nodeID) + ",";
              resp += "\"lat\":" + String(deployRequests[j].lat, 6) + ",";
              resp += "\"lng\":" + String(deployRequests[j].lng, 6) + "}";
              first = false;
            }
          }
          resp += "]}";
          deploymentClients[i].println(resp);

          // LoRa预广播
          uint8_t buf[18];
          buf[0] = FRAME_HEADER;
          buf[1] = 0x10;
          buf[2] = 14;
          buf[3] = (nid >> 8) & 0xFF;
          buf[4] = nid & 0xFF;
          memcpy(&buf[5], &nlat, 4);
          memcpy(&buf[9], &nlng, 4);
          memcpy(&buf[13], &nalt, 4);
          sendLoRaPacket(buf, 17);

          Serial.printf("[DEPLOY] Node %d → IP %s (%.4f,%.4f)\n",
                        nid, assignedIP.toString().c_str(), nlat, nlng);
        }
      }
    }
  }
}




// ==================== LoRa 接收处理 ====================
void handleLoRaRX() {
  // ===== 调试 =====
  static uint32_t lastDbg = 0;
  if (DXLR_SERIAL.available() > 0) {
    if (millis() - lastDbg > 200) {
      Serial.printf("[LoRa RX] Available: %d bytes\n", DXLR_SERIAL.available());
      lastDbg = millis();
    }
  }
  // ================

  // =====================================
  while (DXLR_SERIAL.available() >= 5) {
    uint8_t buf[256];
    int idx = 0;

    while (DXLR_SERIAL.available() && idx < 256) {
      buf[idx] = DXLR_SERIAL.read();
      if (buf[0] != FRAME_HEADER) {
        idx = 0;
        continue;
      }
      idx++;
      if (idx >= 5) break;
    }

    if (idx < 5) return;

    uint8_t len = buf[2];
    uint16_t srcID = (buf[3] << 8) | buf[4];

    int totalLen = 5 + len;
    while (idx < totalLen && DXLR_SERIAL.available()) {
      buf[idx++] = DXLR_SERIAL.read();
    }

    if (idx < totalLen) return;

    int8_t rssi = 0;
    if (DXLR_SERIAL.available()) {
      rssi = -(0xFF - DXLR_SERIAL.read());
    }

    uint8_t type = buf[1];
    Serial.printf("[LoRa RX] type=0x%02X, srcID=%d, len=%d\n", type, srcID, len);

    switch (type) {


      case FRAME_DATA_PRED:
        {
          // 最少 32 字节（旧格式），新格式 37 字节
          if (len < 32) break;

          // ===== 1. 解析 32 字节基础字段 =====
          uint32_t epoch;
          float s_peak, tau_c, sp_time, dist, warn, ai_conf, ae_score;
          memcpy(&epoch, &buf[5], 4);
          memcpy(&s_peak, &buf[9], 4);
          memcpy(&tau_c, &buf[13], 4);
          memcpy(&sp_time, &buf[17], 4);
          memcpy(&dist, &buf[21], 4);
          memcpy(&warn, &buf[25], 4);
          memcpy(&ai_conf, &buf[29], 4);
          memcpy(&ae_score, &buf[33], 4);

          // ===== 2. 解析扩展字段（36/37 字节） =====
          float azimuth = -1;  // -1 表示无效
          uint8_t ttl = 0;

          if (len >= 36) {
            memcpy(&azimuth, &buf[37], 4);  // 偏移 37 = 5(头) + 32
          }
          if (len >= 37) {
            ttl = buf[41];  // 偏移 41 = 5(头) + 36
          }

          // ===== 3. 计算简易烈度 =====
          uint8_t intensity = 1;
          float accel = s_peak;
          if (accel >= 250.0f) intensity = 11;
          else if (accel >= 100.0f) intensity = 10;
          else if (accel >= 50.0f) intensity = 9;
          else if (accel >= 25.0f) intensity = 8;
          else if (accel >= 10.0f) intensity = 7;
          else if (accel >= 5.0f) intensity = 6;
          else if (accel >= 2.5f) intensity = 5;
          else if (accel >= 1.25f) intensity = 4;
          else if (accel >= 0.63f) intensity = 3;
          else if (accel >= 0.31f) intensity = 2;

          Serial.printf("🌍 [EQ EVENT] Node %d: epoch=%u s_peak=%.1f gal τc=%.3f s "
                        "S-P=%.3f s dist=%.1f km warn=%.1f s ai=%.2f ae=%.2f az=%.1f° TTL=%d\n",
                        srcID, epoch, s_peak, tau_c, sp_time, dist, warn,
                        ai_conf, ae_score, azimuth, ttl);

          // ===== 4. 更新/创建节点记录 =====
          NodeInfo* node = nullptr;
          for (int i = 0; i < nodeCount; i++) {
            if (nodes[i].nodeID == srcID) {
              node = &nodes[i];
              break;
            }
          }
          if (!node && nodeCount < MAX_NODES) {
            node = &nodes[nodeCount];
            memset(node, 0, sizeof(NodeInfo));
            node->nodeID = srcID;
            nodeCount++;
            Serial.printf("[NODE] New node %d registered\n", srcID);
          }

          if (node) {
            node->lastSeen = millis();
            node->lastAlertTime = millis();
            node->alertLevel = (intensity >= 6) ? 3 : (intensity >= 4 ? 2 : 1);
            node->hasEQInfo = true;
            node->s_peak_gal = s_peak;
            node->tau_c_sec = tau_c;
            node->sp_time_sec = sp_time;
            node->distance_km = dist;
            node->warning_time_sec = warn;
            node->eqEpoch = epoch;
            node->ai_confidence = ai_conf;
            node->ae_anomaly_score = ae_score;
            node->p_wave_azimuth = azimuth;  // ← 新增
            node->lastRSSI = rssi;
            node->estimatedDistance = rssiToDistance(rssi);
          }

          // ===== 5. 更新统计 =====
          totalPackets++;
          totalAlerts++;
          autoCorrectNodePosition(srcID, rssi);

          // ===== 6. WiFi 模式：上传到 MQTT =====
          if (WiFi.isConnected() && mqttClient.connected()) {

            // 6a. 全量数据 → earthquake/data
            String dataJson = "{";
            dataJson += "\"type\":\"eq_event\",";
            dataJson += "\"node\":" + String(srcID) + ",";
            dataJson += "\"epoch\":" + String(epoch) + ",";
            dataJson += "\"s_peak\":" + String(s_peak, 2) + ",";
            dataJson += "\"tau_c\":" + String(tau_c, 3) + ",";
            dataJson += "\"sp_time\":" + String(sp_time, 3) + ",";
            dataJson += "\"distance\":" + String(dist, 1) + ",";
            dataJson += "\"warning\":" + String(warn, 1) + ",";
            dataJson += "\"ai_confidence\":" + String(ai_conf, 3) + ",";
            dataJson += "\"ae_anomaly\":" + String(ae_score, 2) + ",";
            dataJson += "\"intensity\":" + String(intensity) + ",";
            dataJson += "\"rssi\":" + String(rssi) + ",";

            // 单节点震中推算信息
            if (azimuth >= 0 && node) {
              dataJson += "\"azimuth\":" + String(azimuth, 1) + ",";
              dataJson += "\"node_lat\":" + String(node->assignedLat, 6) + ",";
              dataJson += "\"node_lng\":" + String(node->assignedLng, 6) + ",";
            }

            dataJson += "\"gw_lat\":" + String(gwLat, 6) + ",";
            dataJson += "\"gw_lng\":" + String(gwLng, 6) + ",";
            dataJson += "\"timestamp\":" + String(getEpochTime());
            dataJson += "}";

            mqttClient.publish(MQTT_TOPIC_DATA, dataJson.c_str());
            Serial.printf("[MQTT] → earthquake/data (%d bytes)\n", dataJson.length());

            // 6b. 高烈度时 → earthquake/alert
            if (intensity >= 4) {
              String alertJson = "{";
              alertJson += "\"type\":\"alert\",";
              alertJson += "\"node\":" + String(srcID) + ",";
              alertJson += "\"intensity\":" + String(intensity) + ",";
              alertJson += "\"s_peak\":" + String(s_peak, 1) + ",";
              alertJson += "\"distance\":" + String(dist, 1) + ",";
              alertJson += "\"warning\":" + String(warn, 1) + ",";

              if (azimuth >= 0 && node) {
                alertJson += "\"azimuth\":" + String(azimuth, 1) + ",";
                alertJson += "\"node_lat\":" + String(node->assignedLat, 6) + ",";
                alertJson += "\"node_lng\":" + String(node->assignedLng, 6) + ",";
              }

              alertJson += "\"gw_lat\":" + String(gwLat, 6) + ",";
              alertJson += "\"gw_lng\":" + String(gwLng, 6) + ",";
              alertJson += "\"timestamp\":" + String(getEpochTime());
              alertJson += "}";

              mqttClient.publish(MQTT_TOPIC_ALERT, alertJson.c_str());
              Serial.printf("[MQTT] → earthquake/alert\n");
            }
          }

          // ===== 7. 4G 模式：本地融合 =====
          else if (!WiFi.isConnected() && dtuOnline && node) {

            // 计算单节点评分
            float single_score = compute_single_node_score(*node);
            node->single_node_score = single_score;

            Serial.printf("  [4G SCORE] node=%d score=%.3f\n", srcID, single_score);

            // 低分过滤
            if (single_score < 0.3f) {
              Serial.printf("  [4G FILTER] Score too low, dropped\n");
              break;
            }

            // 加入事件缓冲区
            if (!node->in_current_event) {
              addNodeToEvent(srcID);
              node->in_current_event = true;
              Serial.printf("  [4G EVENT] Node %d added (total %d)\n",
                            srcID, currentEvent.node_count);
            }

            // 融合触发条件
            bool should_fuse = false;
            if (currentEvent.node_count >= GATEWAY_ALERT_MIN_NODES) {
              should_fuse = true;
            } else if (currentEvent.node_count == 1 && single_score > 0.8f) {
              should_fuse = true;
            }

            if (should_fuse) {
              Serial.printf("  [4G FUSION] Triggering (%d nodes)\n", currentEvent.node_count);
              perform_event_fusion();

              if (currentEvent.validated && currentEvent.network_confidence > 0.5f) {
                send_fused_alert_dtu();
                if (config_dingtalk.length() > 0) {
                  send_fused_alert_dingtalk();
                }
              }
              resetEventBuffer();
            }
          }

          // ===== 8. 中继转发（网关不转发） =====
          // 网关是终点，不做中继

          // ===== 9. ACK 确认 =====
          uint8_t ack[5];
          ack[0] = FRAME_HEADER;
          ack[1] = 0x0A;
          ack[2] = 0;
          ack[3] = (srcID >> 8) & 0xFF;
          ack[4] = srcID & 0xFF;
          sendLoRaPacket(ack, 5);

          break;
        }

      case FRAME_RELAY:
        {
          totalRelays++;
          if (buf[14] > 0) {
            buf[14]--;
            sendLoRaPacket(buf, idx);
          }
          break;
        }

      case FRAME_REGISTER:
        {
          float nlat, nlng, nalt;
          memcpy(&nlat, &buf[5], 4);
          memcpy(&nlng, &buf[9], 4);
          memcpy(&nalt, &buf[13], 4);

          bool found = false;
          for (int i = 0; i < nodeCount; i++) {
            if (nodes[i].nodeID == srcID) {
              nodes[i].verifiedLat = nlat;
              nodes[i].verifiedLng = nlng;
              nodes[i].registered = true;
              nodes[i].lastSeen = millis();
              found = true;
              break;
            }
          }

          if (!found && nodeCount < MAX_NODES) {
            nodes[nodeCount].nodeID = srcID;
            nodes[nodeCount].assignedLat = nlat;
            nodes[nodeCount].assignedLng = nlng;
            nodes[nodeCount].assignedAlt = nalt;
            nodes[nodeCount].registered = true;
            nodes[nodeCount].lastSeen = millis();
            nodes[nodeCount].assignedIP = nextNodeIP;
            nextNodeIP[3]++;
            nodeCount++;
          }
          break;
        }

      case FRAME_PROBE_RESP:
        break;

      case FRAME_PRESENCE:
        {
          float pLat, pLng, pAlt;
          memcpy(&pLat, &buf[5], 4);
          memcpy(&pLng, &buf[9], 4);
          memcpy(&pAlt, &buf[13], 4);

          if (mqttClient.connected()) {
            String json = "{\"type\":\"presence\",\"node\":" + String(srcID)
                          + ",\"lat\":" + String(pLat, 6)
                          + ",\"lng\":" + String(pLng, 6) + "}";
            mqttClient.publish(MQTT_TOPIC_ALERT, json.c_str());
          }

          if (dtuOnline && config_dingtalk.length() > 0) {
            String dingJson = "{\"msgtype\":\"markdown\",\"markdown\":{";
            dingJson += "\"title\":\"📍 人员到场通知\",";
            dingJson += "\"text\":\"### 📍 人员到场通知\\n";
            dingJson += "- **节点**: " + String(srcID) + "\\n";
            dingJson += "- **坐标**: " + String(pLat, 4) + ", " + String(pLng, 4) + "\\n";
            dingJson += "- **网关**: " + String(gwLat, 4) + ", " + String(gwLng, 4);
            dingJson += "\"}}";

            String httpReq = "POST /robot/send?access_token=" + config_dingtalk + " HTTP/1.1\r\n"
                                                                                  "Host: oapi.dingtalk.com\r\n"
                                                                                  "Content-Type: application/json\r\n"
                                                                                  "Content-Length: "
                             + String(dingJson.length()) + "\r\n\r\n"
                             + dingJson;

            dtuSendDingTalk(httpReq);
          }
          break;
        }

      case FRAME_MESSAGE:
        {
          uint16_t targetID = (buf[5] << 8) | buf[6];
          if (targetID == 0xFFFF || targetID == 0) {  // 发给网关或广播
            if (gwMsgCount < GW_MAX_STORED_MSGS) {
              gwMsgStore[gwMsgCount].timestamp = getEpochTime();
              gwMsgStore[gwMsgCount].fromNode = srcID;
              gwMsgStore[gwMsgCount].toNode = targetID;
              int textLen = len - 3 > 199 ? 199 : len - 3;
              memcpy(gwMsgStore[gwMsgCount].text, &buf[8], textLen);
              gwMsgStore[gwMsgCount].text[textLen] = '\0';
              gwMsgCount++;
              Serial.printf("[MSG] Stored from Node %d: %s\n", srcID, gwMsgStore[gwMsgCount - 1].text);
            }
          }
          // 如果目标不是网关，转发
          if (targetID != 0xFFFF && targetID != 0) {
            sendLoRaPacket(buf, idx);
          }
          break;
        }

      case 0x0B:
        {
          if (len >= 4) {
            float nodeDist;
            memcpy(&nodeDist, &buf[5], 4);
            float gwDist = rssiToDistance(rssi);

            for (int i = 0; i < nodeCount; i++) {
              if (nodes[i].nodeID == srcID) {
                nodes[i].lastSeen = millis();
                nodes[i].estimatedDistance = nodeDist;
                nodes[i].lastRSSI = rssi;
                if (nodeDist > 0 && gwDist > 0) {
                  float dev = fabs(nodeDist - gwDist);
                  if (dev > 500) {
                    nodes[i].positionVerified = false;
                    Serial.printf("[VERIFY] Node %d distance mismatch: node=%.1fm gw=%.1fm dev=%.1fm\n",
                                  srcID, nodeDist, gwDist, dev);
                  } else {
                    nodes[i].positionVerified = true;
                  }
                } else if (nodeDist == 0 && gwDist > 0) {
                  Serial.printf("[VERIFY] Node %d reported 0 but gw RSSI=%.1fm\n", srcID, gwDist);
                } else if (nodeDist > 0 && gwDist == 0) {
                  Serial.printf("[VERIFY] Node %d reported %.1fm but gw RSSI=0\n", srcID, nodeDist);
                }
                break;
              }
            }
          }
          break;
        }
    }
    esp_task_wdt_reset();
  }
}

// ==================== Core 0: LoRa + DTU ====================
void core0Task(void* pv) {
  Serial.println("[Core0] LoRa RX/TX");

  checkEventExpiry();


  uint32_t lastNB = 0;
  uint32_t lastLoc = 7000;
  uint32_t lastSignal = 13000;
  uint32_t lastNetTime = 19000;
  uint32_t lastGwPosBroadcast = 0;
  uint32_t lastGWVerify = 0;

  for (;;) {

    handleLoRaRX();
    dtuProcessResponse();
    esp_task_wdt_reset();
    sendLoRaFromQueue();

    // 处理 DTU 异步发送队列


    if (millis() - lastNB >= 60000) {
      broadcastNeighborList();
      lastNB = millis();
    }

    if (gwLocationValid && millis() - lastGwPosBroadcast >= 120000) {
      Serial.println("[DTU] Position timer fired!");
      broadcastGatewayPosition();
      lastGwPosBroadcast = millis();
    }

    if (millis() - lastSignal >= 127000) {
      Serial.println("[DTU] CSQ timer fired!");
      dtuSendCmdAsync(DTU_CMD_CSQ, DTU_MATCH_CSQ, [](const String& r) {
        int idx = r.indexOf("csq,ok,");
        if (idx >= 0) dtuSignal = r.substring(idx + 7).toInt();
      });
      lastSignal = millis();
    }

    if (millis() - lastNetTime >= 251000) {
      if (!WiFi.isConnected()) {
        dtuSendCmdAsync(DTU_CMD_NETTIME, DTU_MATCH_NETTIME, [](const String& r) {
          int idx = r.indexOf("nettime,ok,");
          if (idx >= 0) {
            String data = r.substring(idx + 11);
            data.trim();
            int comma1 = data.indexOf(',');
            int comma2 = data.indexOf(',', comma1 + 1);
            int comma3 = data.indexOf(',', comma2 + 1);
            int comma4 = data.indexOf(',', comma3 + 1);
            int comma5 = data.indexOf(',', comma4 + 1);

            int y = data.substring(0, comma1).toInt();
            int mo = data.substring(comma1 + 1, comma2).toInt();
            int d = data.substring(comma2 + 1, comma3).toInt();
            int h = data.substring(comma3 + 1, comma4).toInt();
            int mi = data.substring(comma4 + 1, comma5).toInt();
            int s = data.substring(comma5 + 1).toInt();

            char buf[20];
            snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
            dtuNetTime = String(buf);

            struct tm timeinfo = {};
            timeinfo.tm_year = y - 1900;
            timeinfo.tm_mon = mo - 1;
            timeinfo.tm_mday = d;
            timeinfo.tm_hour = h;
            timeinfo.tm_min = mi;
            timeinfo.tm_sec = s;

            time_t epoch = mktime(&timeinfo);
            if (epoch > 1000000000) {
              g_epochTime = epoch;
              lastNtpSync = millis();
              g_timeValid = true;
              Serial.printf("[NTP] Time synced via DTU: %s\n", buf);
              broadcastTimeSync();
            }
          }  // if (idx >= 0) 结束
        });  // lambda 结束 + dtuSendCmdAsync 结束
      }
      lastNetTime = millis();
    }

    if (millis() - lastLoc >= 310000 || forceLocRequest) {
      forceLocRequest = false;

      dtuSendCmdAsync("config,get,gpsext\r\n", DTU_MATCH_GPS, [](const String& r) {
        int idx = r.indexOf("gps,ok,");
        bool gpsOk = false;

        if (idx >= 0) {
          String data = r.substring(idx + 7);
          int comma1 = data.indexOf(',');
          if (comma1 >= 0 && data.substring(0, comma1).toInt() == 1) {
            gpsFailCount = 0;
            data = data.substring(comma1 + 1);
            int comma2 = data.indexOf(',');
            String lngStr = data.substring(comma2 + 1, data.indexOf(',', comma2 + 1));
            int comma3 = data.indexOf(',', comma2 + 1);
            int comma4 = data.indexOf(',', comma3 + 1);
            String latStr = data.substring(comma3 + 1, comma4);
            int comma5 = data.indexOf(',', comma4 + 1);
            int comma6 = data.indexOf(',', comma5 + 1);
            int comma7 = data.indexOf(',', comma6 + 1);
            String altStr = data.substring(comma6 + 1, comma7);

            float lng = lngStr.toFloat();
            float lat = latStr.toFloat();
            float alt = altStr.toFloat();

            if (lng > 0 && lat > 0) {
              wgs84_to_gcj02(lat, lng, &lat, &lng);
              gwLat = lat;
              gwLng = lng;
              gwAlt = alt;
              gwLocationValid = true;
              gwLocationMethod = "GPS";
              broadcastGatewayPosition();
              gpsOk = true;
            }
          }
        }

        if (!gpsOk) {
          gpsFailCount++;
          if (gpsFailCount >= GPS_FAIL_THRESHOLD) gwLocationMethod = "none";

          dtuSendCmdAsync(DTU_CMD_LOC_QUERY, DTU_MATCH_LOC, [](const String& r2) {
            int idx2 = r2.indexOf("lbsloc,ok,");
            if (idx2 >= 0) {
              String data = r2.substring(idx2 + 10);
              int comma = data.indexOf(',');
              if (comma >= 0) {
                float lng = data.substring(0, comma).toFloat();
                float lat = data.substring(comma + 1).toFloat();
                if (lng > 0 && lat > 0) {
                  wgs84_to_gcj02(lat, lng, &lat, &lng);
                  gwLat = lat;
                  gwLng = lng;
                  gwAlt = fetchAltitude(gwLat, gwLng);
                  gwLocationValid = true;
                  gwLocationMethod = "CELL";
                  // 海拔未知或无效时，主动请求
                  if (gwAlt == 0) {
                    requestAltitude(gwLat, gwLng);
                  } else {
                    broadcastGatewayPosition();
                  }
                }
              }
            }
          });
        }
      });

      lastLoc = millis();
    }

    if (millis() - lastGWVerify >= 300000) {
      verifyGatewayPosition();
      lastGWVerify = millis();
    }

    // NTP 时间同步（每小时一次）
    if (millis() - lastNtpSync >= 3600000) {
      lastNtpSync = millis();
      if (syncNTP()) {
        broadcastTimeSync();  // 同步成功后广播一次
      }
    }

    if (deploymentMode) { handleDeploymentClients(); }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== Core 1: Web + MQTT + AI ====================
void core1Task(void* pv) {
  Serial.println("[Core1] Web + MQTT + AI");

  uint32_t lastMqtt = 0, lastHb = 0, lastStat = 0;
  uint32_t mqttReconnectTime = 0;
  bool mqttReconnectPending = false;
  bool mqttInitDone = false;
  uint32_t lastStateUpdate = 0;


  for (;;) {


    esp_task_wdt_reset();


    // 处理 PCA 数据


    mqttClient.loop();

    if (!mqttClient.connected()) {
      if (!mqttReconnectPending) {
        mqttReconnectPending = true;
        mqttReconnectTime = millis() + 60000;
      } else if (millis() >= mqttReconnectTime) {
        if (mqttClient.connect(("GW-" + String(random(0xFFFF), HEX)).c_str(), MQTT_USER, MQTT_PASSWORD)) {
          mqttClient.setKeepAlive(30);
          mqttReconnectPending = false;
          Serial.println("[MQTT] Connected");
        } else {
          mqttReconnectTime = millis() + 60000;
          Serial.printf("[MQTT] Connect failed, state=%d\n", mqttClient.state());
        }
      }
    } else {
      mqttReconnectPending = false;
    }

    if (millis() - lastHb >= 60000) {
      lastHb = millis();
      String hb = "{";
      hb += "\"type\":\"heartbeat\",";
      hb += "\"nodes\":" + String(nodeCount) + ",";
      hb += "\"packets\":" + String(totalPackets) + ",";
      hb += "\"alerts\":" + String(totalAlerts) + ",";
      hb += "\"wifi\":" + String(WiFi.isConnected() ? "true" : "false") + ",";
      hb += "\"dtu\":" + String(dtuOnline ? "true" : "false") + ",";
      hb += "\"uptime\":" + String(millis() / 1000) + ",";
      hb += "\"freeHeap\":" + String(ESP.getFreeHeap() / 1024) + ",";
      hb += "\"gwLat\":" + String(gwLat, 6) + ",";
      hb += "\"gwLng\":" + String(gwLng, 6);
      hb += "}";

      if (WiFi.isConnected() && mqttClient.connected()) {
        mqttClient.publish(MQTT_TOPIC_HEARTBEAT, hb.c_str());
      }
    }

    if (millis() - lastStat >= 10000) {
      lastStat = millis();
      Serial.printf("[GW] P=%lu A=%lu DTU=%s MQTT=%s N=%d Free=%d\n",
                    totalPackets, totalAlerts,
                    dtuOnline ? "Y" : "N",
                    mqttClient.connected() ? "Y" : "N",
                    nodeCount, ESP.getFreeHeap());
      Serial.printf(
        "MinFreeHeap=%u\n",
        ESP.getMinFreeHeap());

      Serial.printf(
        "Stack=%u\n",
        uxTaskGetStackHighWaterMark(NULL));
      Serial.printf(
        "Stack=%u\n",
        uxTaskGetStackHighWaterMark(NULL));
    }
    static uint32_t lastNMUpdate = 0;
    if (millis() - lastNMUpdate >= 10000) {
      lastNMUpdate = millis();
      networkManager.update();  // ← 检查连接状态，断线自动重连
    }





    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


// ==================== 启动 ====================
void setup() {
  Serial.begin(115200);
  DTU_SERIAL.begin(DTU_BAUDRATE, SERIAL_8N1, DTU_RX, DTU_TX);
  delay(2000);
  DXLR_SERIAL.begin(9600, SERIAL_8N1, 1, 2);

  Serial.println("\n╔══════════════════════════════════╗");
  Serial.println("║  Earthquake Gateway v6.1        ║");
  Serial.println("╚══════════════════════════════════╝\n");

  if (!FFat.begin(true)) {
    Serial.println("❌ FFat failed");
  }
  loadConfig();

  networkManager.setConfig(config_ssid, config_pass);
  networkManager.begin();

  Serial.println("[WiFi] Waiting for connection...");
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 30000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Not connected, continuing...");
  }

  // DX-LR22 初始化
  delay(2000);
  DXLR_SERIAL.print("+++\r\n");
  delay(2000);
  Serial.print("[DX-LR22] +++: ");
  while (DXLR_SERIAL.available()) {
    uint8_t c = DXLR_SERIAL.read();
    Serial.printf("%c", c);
  }
  Serial.println();

  DXLR_SERIAL.print("AT+DRSSI1\r\n");
  delay(2000);
  Serial.print("[DX-LR22] DRSSI1: ");
  while (DXLR_SERIAL.available()) {
    uint8_t c = DXLR_SERIAL.read();
    Serial.printf("%c", c);
  }
  Serial.println();

  DXLR_SERIAL.println("AT+LBT1");
  delay(2000);
  Serial.print("[DX-LR22] LBT1: ");
  while (DXLR_SERIAL.available()) {
    uint8_t c = DXLR_SERIAL.read();
    Serial.printf("%c", c);
  }
  Serial.println();

  DXLR_SERIAL.print("AT+LEVEL0\r\n");
  delay(2000);
  Serial.print("[DX-LR22] LEVEL0: ");
  while (DXLR_SERIAL.available()) {
    uint8_t c = DXLR_SERIAL.read();
    Serial.printf("%c", c);
  }
  Serial.println();

  DXLR_SERIAL.print("AT+CHANNEL00\r\n");
  delay(2000);
  Serial.print("[DX-LR22] CHANNEL00: ");
  while (DXLR_SERIAL.available()) {
    uint8_t c = DXLR_SERIAL.read();
    Serial.printf("%c", c);
  }
  Serial.println();

  DXLR_SERIAL.print("+++\r\n");
  delay(2000);
  Serial.print("[DX-LR22] +++: ");
  while (DXLR_SERIAL.available()) {
    uint8_t c = DXLR_SERIAL.read();
    Serial.printf("%c", c);
  }
  Serial.println();

  pinMode(DXLR_M0, OUTPUT);
  pinMode(DXLR_M1, OUTPUT);
  digitalWrite(DXLR_M0, LOW);
  digitalWrite(DXLR_M1, LOW);

  delay(100);
  while (DXLR_SERIAL.available()) DXLR_SERIAL.read();

  Serial.println("[DX-LR22] Init done");

  // MQTT
  mqttClient.setClient(espClient);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  espClient.setCACert(MQTT_CA_CERT);

  // Web
  setupWebServer();




  // 队列
  // ===== 保留这些队列创建 =====
  txQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(uint8_t*));
  dtuCmdQueue = xQueueCreate(DTU_CMD_QUEUE_SIZE, sizeof(DTUCmd));









  while (DXLR_SERIAL.available()) DXLR_SERIAL.read();

  xTaskCreatePinnedToCore(core0Task, "Core0", 32768, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(core1Task, "Core1", 32768, NULL, 2, NULL, 1);

  esp_task_wdt_init(60, true);

  dtuInitAsync();





  Serial.println("✅ Gateway Ready\n");
  vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL);
}