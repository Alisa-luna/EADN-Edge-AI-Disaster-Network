#ifndef EARTHQUAKE_STATE_MACHINE_H
#define EARTHQUAKE_STATE_MACHINE_H

#include <Arduino.h>
#include <functional>

enum class AlertLevel { 
    NONE = 0, 
    YELLOW = 1, 
    ORANGE = 2, 
    RED = 3 
};

// 回调函数类型：参数为新级别和概率
typedef std::function<void(AlertLevel, float)> AlertCallback;

class EarthquakeStateMachine {
private:
    AlertLevel currentLevel;
    uint32_t eventStartTime;
    uint32_t lastTriggerTime;
    float lastProbability;
    
    // 多节点触发记录
    static const int MAX_TRIGGERS = 20;
    struct NodeTrigger {
        uint16_t nodeID;
        float probability;
        uint32_t time;
    };
    NodeTrigger triggers[MAX_TRIGGERS];
    int triggerCount;
    
    // 通知回调
    AlertCallback onLevelChange;
    
public:
    EarthquakeStateMachine() { 
        reset();
        onLevelChange = nullptr;
    }
    
    // 设置状态变化回调
    void setCallback(AlertCallback callback) {
        onLevelChange = callback;
    }
    
    void reset() {
        currentLevel = AlertLevel::NONE;
        eventStartTime = 0;
        lastTriggerTime = 0;
        lastProbability = 0;
        triggerCount = 0;
    }
    
    // 记录节点触发
    void recordNodeTrigger(uint16_t nodeID, float probability) {
        uint32_t now = millis();
        
        // 去重：同一节点在 5 秒内只记录一次
        for (int i = 0; i < triggerCount; i++) {
            if (triggers[i].nodeID == nodeID && now - triggers[i].time < 5000) {
                if (probability > triggers[i].probability) {
                    triggers[i].probability = probability;
                }
                lastTriggerTime = now;
                lastProbability = probability;
                return;
            }
        }
        
        // 新增记录
        if (triggerCount < MAX_TRIGGERS) {
            triggers[triggerCount].nodeID = nodeID;
            triggers[triggerCount].probability = probability;
            triggers[triggerCount].time = now;
            triggerCount++;
        } else {
            // 队列满，覆盖最旧的
            int oldest = 0;
            for (int i = 1; i < MAX_TRIGGERS; i++) {
                if (triggers[i].time < triggers[oldest].time) oldest = i;
            }
            triggers[oldest].nodeID = nodeID;
            triggers[oldest].probability = probability;
            triggers[oldest].time = now;
        }
        
        lastTriggerTime = now;
        lastProbability = probability;
        
        Serial.printf("[StateMachine] Record trigger: Node=%d, prob=%.2f\n", nodeID, probability);
        
        // 立即评估状态变化
        evaluateState();
    }
    
    // 评估并更新状态
    void evaluateState() {
        AlertLevel prevLevel = currentLevel;
        int count = countRecentNodes(5000);
        
        if (count >= 2 && currentLevel < AlertLevel::ORANGE) {
            currentLevel = AlertLevel::ORANGE;
            eventStartTime = millis();
            Serial.printf("🟠 [StateMachine] %d nodes triggered → ORANGE\n", count);
        }
        else if (count == 1 && lastProbability > 0.5f && currentLevel < AlertLevel::ORANGE) {
            currentLevel = AlertLevel::ORANGE;
            eventStartTime = millis();
            Serial.printf("🟠 [StateMachine] Single node %.2f → ORANGE\n", lastProbability);
        }
        else if (count == 1 && lastProbability > 0.3f && currentLevel < AlertLevel::YELLOW) {
            currentLevel = AlertLevel::YELLOW;
            eventStartTime = millis();
            Serial.printf("🟡 [StateMachine] Single node %.2f → YELLOW\n", lastProbability);
        }
        
        // 状态变化时自动调用回调
        if (currentLevel != prevLevel && onLevelChange != nullptr) {
            Serial.printf("[StateMachine] Level changed: %s → %s\n",
                         levelToString(prevLevel), levelToString(currentLevel));
            onLevelChange(currentLevel, lastProbability);
        }
    }
    
    // 每 5 秒调用一次，检查是否需要降级
    void update() {
        AlertLevel prevLevel = currentLevel;
        checkTimeout();
        
        // 降级时也触发回调
        if (currentLevel != prevLevel && onLevelChange != nullptr) {
            Serial.printf("[StateMachine] Level changed: %s → %s\n",
                         levelToString(prevLevel), levelToString(currentLevel));
            onLevelChange(currentLevel, lastProbability);
        }
    }
    
    // 超时降级
    void checkTimeout() {
        if (currentLevel == AlertLevel::NONE) return;
        
        uint32_t now = millis();
        
        // 60 秒无任何触发则强制归零
        if (now - lastTriggerTime > 60000) {
            Serial.printf("[StateMachine] Force reset (no trigger for %d ms)\n", now - lastTriggerTime);
            currentLevel = AlertLevel::NONE;
            eventStartTime = 0;
            return;
        }
        
        // 30 秒内无新触发则降一级
        if (now - eventStartTime > 30000 && countRecentNodes(30000) == 0) {
            switch (currentLevel) {
                case AlertLevel::RED:
                    currentLevel = AlertLevel::ORANGE;
                    eventStartTime = now;
                    Serial.println("🔴→🟠 [StateMachine] RED timeout → ORANGE");
                    break;
                case AlertLevel::ORANGE:
                    currentLevel = AlertLevel::YELLOW;
                    eventStartTime = now;
                    Serial.println("🟠→🟡 [StateMachine] ORANGE timeout → YELLOW");
                    break;
                case AlertLevel::YELLOW:
                    currentLevel = AlertLevel::NONE;
                    eventStartTime = 0;
                    Serial.println("🟡→✅ [StateMachine] YELLOW timeout → NONE");
                    break;
                default:
                    break;
            }
        }
    }
    
    // 统计最近 windowMs 毫秒内有多少个不同节点触发
    int countRecentNodes(uint32_t windowMs) {
        uint32_t now = millis();
        uint16_t seen[MAX_TRIGGERS];
        int seenCount = 0;
        
        for (int i = 0; i < triggerCount; i++) {
            if (now - triggers[i].time > windowMs) continue;
            
            bool found = false;
            for (int j = 0; j < seenCount; j++) {
                if (seen[j] == triggers[i].nodeID) { found = true; break; }
            }
            if (!found && seenCount < MAX_TRIGGERS) {
                seen[seenCount++] = triggers[i].nodeID;
            }
        }
        return seenCount;
    }
    
    // 清除超过 maxAgeMs 的旧记录
    void cleanOldTriggers(uint32_t maxAgeMs) {
        uint32_t now = millis();
        int writeIdx = 0;
        for (int i = 0; i < triggerCount; i++) {
            if (now - triggers[i].time < maxAgeMs) {
                triggers[writeIdx++] = triggers[i];
            }
        }
        triggerCount = writeIdx;
    }
    
    AlertLevel getLevel() { return currentLevel; }
    float getLastProbability() { return lastProbability; }
    
    const char* getLevelString() {
        return levelToString(currentLevel);
    }
    
private:
    const char* levelToString(AlertLevel level) {
        switch (level) {
            case AlertLevel::NONE:   return "NONE";
            case AlertLevel::YELLOW: return "YELLOW";
            case AlertLevel::ORANGE: return "ORANGE";
            case AlertLevel::RED:    return "RED";
            default: return "?";
        }
    }
};

#endif