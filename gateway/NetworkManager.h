#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <WiFi.h>

enum NetworkState {
    NET_DISCONNECTED,
    NET_WIFI,
    NET_4G_FALLBACK
};

#define WIFI_RETRY_DELAY  3000
#define WIFI_RETRY_COUNT  3

class NetworkManager {
private:
    NetworkState currentState;
    int wifiRetryCount;
    unsigned long lastRetryTime;
    bool wifiConnected;
    bool dtuAvailable;
    String wifi_ssid;
    String wifi_password;
    bool apModeActive;

public:
    NetworkManager() {
        currentState = NET_DISCONNECTED;
        wifiRetryCount = 0;
        lastRetryTime = 0;
        wifiConnected = false;
        dtuAvailable = true;
        wifi_ssid = "";
        wifi_password = "";
        apModeActive = false;
    }

    void setConfig(String ssid, String password) {
        wifi_ssid = ssid;
        wifi_password = password;
    }

    void setDTUAvailable(bool available) {
        dtuAvailable = available;
    }

    void begin() {
        Serial.println("[WiFi] NetworkManager begin()");
        if (wifi_ssid.length() == 0) {
            Serial.println("[WiFi] No WiFi config, starting AP only...");
            WiFi.mode(WIFI_AP);
            WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
            WiFi.softAP("EQ_Gateway", "12345678");
            apModeActive = true;
            Serial.printf("[WiFi] AP started: EQ_Gateway, IP: %s\n", WiFi.softAPIP().toString().c_str());
            currentState = dtuAvailable ? NET_4G_FALLBACK : NET_DISCONNECTED;
            return;
        }
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
        wifiRetryCount = 0;
        Serial.printf("[WiFi] Connecting to %s...\n", wifi_ssid.c_str());
    }

    NetworkState update() {
        if (wifi_ssid.length() == 0) {
            currentState = dtuAvailable ? NET_4G_FALLBACK : NET_DISCONNECTED;
            return currentState;
        }

        if (WiFi.status() == WL_CONNECTED) {
            if (!wifiConnected) {
                wifiConnected = true;
                wifiRetryCount = 0;
                currentState = NET_WIFI;
                Serial.println("[WiFi] Connected!");
                Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
                if (apModeActive) {
                    stopAPMode();
                }
            }
            return currentState;
        }

        if (wifiConnected) {
            wifiConnected = false;
            Serial.println("[WiFi] Disconnected!");
            lastRetryTime = millis();
        }

        if (millis() - lastRetryTime >= WIFI_RETRY_DELAY) {
            if (wifiRetryCount < WIFI_RETRY_COUNT) {
                wifiRetryCount++;
                Serial.printf("[WiFi] Reconnect %d/%d\n", wifiRetryCount, WIFI_RETRY_COUNT);
                WiFi.reconnect();
                lastRetryTime = millis();
            } else {
                if (!apModeActive) {
                    startAPMode();
                    apModeActive = true;
                    
                }
                currentState = dtuAvailable ? NET_4G_FALLBACK : NET_DISCONNECTED;
            }
        }

        if (currentState == NET_4G_FALLBACK && !apModeActive) {
            if (millis() - lastRetryTime >= WIFI_RETRY_DELAY * 6) {
                Serial.println("[WiFi] Periodic check for WiFi...");
                WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
                lastRetryTime = millis();

                uint32_t start = millis();
                while (millis() - start < 5000) {
                    if (WiFi.status() == WL_CONNECTED) {
                        currentState = NET_WIFI;
                        wifiConnected = true;
                        wifiRetryCount = 0;
                        Serial.println("[WiFi] WiFi restored!");
                        Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
                        return currentState;
                    }
                    delay(100);
                }
            }
        }

        return currentState;
    }

    void startAPMode() {
        if (apModeActive) return;
        apModeActive = true;
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
        WiFi.softAP("EQ_Gateway", "12345678");
        Serial.println("[WiFi] AP started: EQ_Gateway");
        Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    }

    void stopAPMode() {
        if (!apModeActive) return;
        apModeActive = false;
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);
        Serial.println("[WiFi] AP stopped, STA mode only");
    }

    NetworkState getState() { return currentState; }
    bool isWiFi() { return currentState == NET_WIFI; }
    bool is4G() { return currentState == NET_4G_FALLBACK; }
    String getSSID() { return wifi_ssid; }
    String getPassword() { return wifi_password; }
    bool isAPActive() { return apModeActive; }
};

#endif