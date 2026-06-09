#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <BearSSLHelpers.h>

// WiFi设置
const char *ssid = "Mywifi";
const char *password = "1234567890";

// MQTT设置
const int mqtt_port = 8883;
const char *mqtt_broker = "192.168.1.100";
const char *mqtt_username = "***";
const char *mqtt_password = "***";

// 串口设置
#define STM32_BAUDRATE 460800     // 与STM32通信的高速串口

// 数据包结构（保持不变）
#pragma pack(push, 1)
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint16_t checksum;
} MPU6050_DataPacket;
#pragma pack(pop)

// 全局变量
String receive_topic;
String publish_topic;
uint8_t serialBuffer[18];
uint8_t bufferIndex = 0;
uint8_t rxState = 0;
unsigned long lastDataTime = 0;

// NTP时间同步
const char *ntp_server = "pool.ntp.org";
const long gmt_offset_sec = 8 * 3600;
const int daylight_offset_sec = 0;

// MQTT客户端
BearSSL::WiFiClientSecure espClient;
PubSubClient mqtt_client(espClient);

// CA证书内容（你的证书保持不变）
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIEHzCCAwegAwIBAgIUB1A1Kug5dkQqqFOzphotLdEGVHswDQYJKoZIhvcNAQEL
BQAwgZ4xCzAJBgNVBAYTAkNOMRAwDgYDVQQIDAdCZWlqaW5nMRAwDgYDVQQHDAdC
ZWlqaW5nMRMwEQYDVQQKDApNeSBDb21wYW55MRYwFAYDVQQLDA1JVCBEZXBhcnRt
ZW50MRkwFwYDVQQDDBBNeSBMb2NhbCBNUVRUIENBMSMwIQYJKoZIhvcNAQkBFhRM
MTM3MDkxMjEyODJAMTYzLmNvbTAeFw0yNjAyMjMwNzQxMTZaFw0zNjAyMjEwNzQx
MTZaMIGeMQswCQYDVQQGEwJDTjEQMA4GA1UECAwHQmVpamluZzEQMA4GA1UEBwwH
QmVpamluZzETMBEGA1UECgwKTXkgQ29tcGFueTEWMBQGA1UECwwNSVQgRGVwYXJ0
bWVudDEZMBcGA1UEAwwQTXkgTG9jYWwgTVFUVCBDQTEjMCEGCSqGSIb3DQEJARYU
TDEzNzA5MTIxMjgyQDE2My5jb20wggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK
AoIBAQDh8S20uTKGHNAoffKyTS8owWq9kOr+f4f1iOTEkpsyiZjrw9F5yav+5Ozw
l2rbMb1WNbkXkp12fvVHjEoxusVrSlJMiajNbcLjQXXP9PvsYqt6xr0EIJYY2+9i
rD1DooUCZNRozyXWXrRugUD/0UsDDwOI35ejt0WSP+d1D3spOzw2oP540WGWa2cY
JENbcScQmpZMMQUfQr1DA6c2xa2b4XwIDU7ooGAYFN6bIG+RxR5DJ2VPO7iEWDTa
trBnik7bXeLtYVxZIRm5eCszRBgJFx5DBrCRYyOlauro9JSfal4/wARVMVySzUx6
jq0JFwyUqup//O6cRmRFf+3+xwr7AgMBAAGjUzBRMB0GA1UdDgQWBBRglb8fssgv
XLFT1Zwh3Bxgq/IH2jAfBgNVHSMEGDAWgBRglb8fssgvXLFT1Zwh3Bxgq/IH2jAP
BgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3DQEBCwUAA4IBAQB5PVCoYceiazUSrWGn
zBxAdoDC6Z7Tn0El998B4+cKikNLIwy1K/M9sFbxvCecq89JPx9SIfydg6x+UEA1
76VM2UBxf2EqG3VqhVUb6TrXAi0eib7VWxXXveJVXr8MOYkpiWYTren9wjH8myrD
u2k02/IYRoBNCauKuwoyqjlgSFElOCKBwt4eZ94UmH8YIov0v3IhPSZpBN0itU4c
9TbVwcpLOmDPsDbqDghO0iNWCV90b4nIN593AcVglUQ1b+yLNl5LKtpXMdYIWYr3
XZTTCSOp7j7+O17ny4vI3ulqSIzsXZotxnkdtax9VTSxD2PjP808ZyF8Sh64PsZB
eNVT
-----END CERTIFICATE-----
)EOF";

// 函数声明
void connectToWiFi();
void connectToMQTT();
void syncTime();
void mqttCallback(char *topic, byte *payload, unsigned int length);
String getMacAddress();
void processMPU6050Data();
uint16_t CalculateChecksum(uint8_t *data, uint16_t length);
void sendSensorDataToMQTT(MPU6050_DataPacket &packet);

void setup() {
    // ===== 修改1：两个串口分别初始化 =====
    
    // 串口1：接电脑看日志（UART1，只有TX）
    Serial1.begin(115200);  // 用115200，和电脑串口助手保持一致
    Serial1.setDebugOutput(true);  // 把所有的调试输出重定向到Serial1
    Serial1.println("ESP8266 Starting...");  // 这条会从GPIO2发出
    
    // 串口0：接STM32通信（UART0，用GPIO3接收）
    Serial.begin(STM32_BAUDRATE, SERIAL_8N1, SERIAL_RX_ONLY, 3);  
    // 参数说明：波特率，数据格式，只接收模式，指定RX引脚为GPIO3
    
    // 初始化主题名称
    String mac = getMacAddress();
    receive_topic = mac + "-receive";
    publish_topic = mac + "-publish";
    
    Serial1.print("Receive topic: ");
    Serial1.println(receive_topic);
    Serial1.print("Publish topic: ");
    Serial1.println(publish_topic);
    
    connectToWiFi();
    syncTime();
    
    // 设置MQTT
    mqtt_client.setServer(mqtt_broker, mqtt_port);
    mqtt_client.setCallback(mqttCallback);
    mqtt_client.setBufferSize(1024);
    
    connectToMQTT();
}

String getMacAddress() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

uint16_t CalculateChecksum(uint8_t *data, uint16_t length) {
    uint16_t sum = 0;
    for(uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

void connectToWiFi() {
    Serial1.print("Connecting to WiFi: ");
    Serial1.println(ssid);
    
    WiFi.begin(ssid, password);
    unsigned long startTime = millis();
    
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > 30000) {
            Serial1.println("\nWiFi connection failed!");
            ESP.restart();
            return;
        }
        delay(500);
        Serial1.print(".");
    }
    
    Serial1.println("\nConnected to WiFi");
    Serial1.print("IP Address: ");
    Serial1.println(WiFi.localIP());
    Serial1.print("MAC Address: ");
    Serial1.println(getMacAddress());
}

void syncTime() {
    Serial1.print("Syncing time");
    configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
    
    unsigned long startTime = millis();
    while (time(nullptr) < 1000000000) {
        if (millis() - startTime > 30000) {
            Serial1.println("\nTime sync failed!");
            break;
        }
        delay(500);
        Serial1.print(".");
    }
    Serial1.println("\nTime synced");
}

void connectToMQTT() {
    BearSSL::X509List serverTrustedCA(ca_cert);
    espClient.setTrustAnchors(&serverTrustedCA);
    espClient.setBufferSizes(1024, 1024);
    
    String client_id = "esp8266-" + getMacAddress();
    Serial1.print("Connecting to MQTT broker: ");
    Serial1.println(mqtt_broker);
    
    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial1.println(" Connected to MQTT broker");
        
        if (mqtt_client.subscribe(receive_topic.c_str())) {
            Serial1.print(" Subscribed to: ");
            Serial1.println(receive_topic);
        } else {
            Serial1.println(" Subscription failed");
        }
        
        String message = "Device connected: " + getMacAddress();
        if (mqtt_client.publish(publish_topic.c_str(), message.c_str())) {
            Serial1.println(" Initial message published");
        } else {
            Serial1.println(" Message publish failed");
        }
        
    } else {
        Serial1.print(" MQTT connection failed, rc=");
        Serial1.println(mqtt_client.state());
        
        char err_buf[256];
        espClient.getLastSSLError(err_buf, sizeof(err_buf));
        Serial1.print("SSL error: ");
        Serial1.println(err_buf);
        
        delay(5000);
    }
}

void processMPU6050Data() {
    while (Serial.available()) {  // 从Serial（STM32）读取数据
        uint8_t incomingByte = Serial.read();
        
        switch (rxState) {
            case 0:
                if (incomingByte == 0xFF) {
                    rxState = 1;
                    bufferIndex = 0;
                }
                break;
                
            case 1:
                serialBuffer[bufferIndex++] = incomingByte;
                if (bufferIndex >= 14) {
                    rxState = 2;
                }
                break;
                
            case 2:
                if (incomingByte == 0xFE) {
                    uint16_t calculatedChecksum = CalculateChecksum(serialBuffer, 12);
                    uint16_t receivedChecksum = serialBuffer[12] | (serialBuffer[13] << 8);
                    
                    if (calculatedChecksum == receivedChecksum) {
                        MPU6050_DataPacket packet;
                        memcpy(&packet, serialBuffer, 14);
                        
                        sendSensorDataToMQTT(packet);
                        lastDataTime = millis();
                        
                        // 日志通过Serial1发出
                        Serial1.println("Checksum OK");
                    } else {
                        Serial1.print("Checksum error: calc=");
                        Serial1.print(calculatedChecksum, HEX);
                        Serial1.print(" recv=");
                        Serial1.println(receivedChecksum, HEX);
                    }
                }
                rxState = 0;
                bufferIndex = 0;
                break;
        }
    }
}

void sendSensorDataToMQTT(MPU6050_DataPacket &packet) {
    String jsonData = "{";
    jsonData += "\"AX\":" + String(packet.accel_x) + ",";
    jsonData += "\"AY\":" + String(packet.accel_y) + ",";
    jsonData += "\"AZ\":" + String(packet.accel_z) + ",";
    jsonData += "\"GX\":" + String(packet.gyro_x) + ",";
    jsonData += "\"GY\":" + String(packet.gyro_y) + ",";
    jsonData += "\"GZ\":" + String(packet.gyro_z);
    jsonData += "}";
    
    if (mqtt_client.connected()) {
        if (mqtt_client.publish(publish_topic.c_str(), jsonData.c_str())) {
            Serial1.println("MPU6050 data published to MQTT");
        } else {
            Serial1.println("MQTT publish failed");
        }
    }
    
    Serial1.print("Accel: ");
    Serial1.print(packet.accel_x); Serial1.print(", ");
    Serial1.print(packet.accel_y); Serial1.print(", ");
    Serial1.print(packet.accel_z);
    Serial1.print(" | Gyro: ");
    Serial1.print(packet.gyro_x); Serial1.print(", ");
    Serial1.print(packet.gyro_y); Serial1.print(", ");
    Serial1.println(packet.gyro_z);
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    Serial1.print("Message received [");
    Serial1.print(topic);
    Serial1.print("]: ");
    
    char message[length + 1];
    for (int i = 0; i < length; i++) {
        message[i] = (char)payload[i];
    }
    message[length] = '\0';
    
    Serial1.println(message);
}

void loop() {
    if (!mqtt_client.connected()) {
        Serial1.println("MQTT disconnected, reconnecting...");
        connectToMQTT();
    }
    
    mqtt_client.loop();
    processMPU6050Data();
    
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 60000) {
        lastHeartbeat = millis();
        
        if (mqtt_client.connected()) {
            String status = "{\"status\":\"online\",\"uptime\":" + String(millis() / 1000) + ",\"last_data\":" + String(lastDataTime / 1000)+ "}";
            mqtt_client.publish(publish_topic.c_str(), status.c_str());
            Serial1.println("Heartbeat sent");
        }
    }
    
    delay(50);
}
