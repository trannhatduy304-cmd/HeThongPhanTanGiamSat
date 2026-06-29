/*
 * ================================================================
 * CENTER HUB v6.4 — Static Hardware Mapping (LED 1->N1, LED 2->N2)
 * ================================================================
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

bool isEspNowReady = false;
#define ESPNOW_CHANNEL 1      
#define NODE_TIMEOUT 120000UL // 2 phút

const float VAR_TEMP = 0.5;    
const float VAR_HUM = 2.0;     
const int VAR_GAS = 50;        
const unsigned long MUTE_DURATION = 60000; 

uint8_t gEdgeMACs[2][6] = {0};
bool gEdgeMacReady[2] = {false, false};

// ========================= Struct Gói Dữ Liệu =========================
struct __attribute__((packed)) DataPacket { byte id; float t; float h; int g; bool r; bool alarm; };
struct __attribute__((packed)) AckPacket { bool mute; };
struct __attribute__((packed)) MutePacket { byte id; byte mute; };
struct __attribute__((packed)) TimePacket { byte id; byte hr; byte mn; byte sc; bool hasRtc; };
struct __attribute__((packed)) TimeSyncPacket { byte hr; byte mn; byte sc; byte mute; };

#define I2C_SDA 21
#define I2C_SCL 22
#define BUZZER_PIN 16 
#define LED1_PIN 12   
#define LED2_PIN 14   

Adafruit_SSD1306 display(128, 64, &Wire, -1);

struct NodeData {
  byte nodeId; 
  float temp, hum; int gas;
  bool relayState, alarm, valid;
  byte edgeHr, edgeMn, edgeSc;
  bool isTimeMaster; 
};

// KHÓA CỨNG: Slot 0 luôn là Node 1, Slot 1 luôn là Node 2
NodeData gNode[2] = {{1, 0, 0, 0, false, false, false, 0, 0, 0, false}, 
                     {2, 0, 0, 0, false, false, false, 0, 0, 0, false}};
                     
bool gMuteNode[2] = {false, false};
bool gHardMuteNode[2] = {false, false}; 
int gViewNode = 0;
unsigned long gLastNodeSeen[2] = {0, 0};

// ĐỒNG HỒ ẢO CỦA CENTER
volatile int gRtcH = 0, gRtcM = 0, gRtcS = 0;
volatile bool gRtcReady = false; 
unsigned long gLastTick = 0;

unsigned long gMuteStartTime[2] = {0, 0};
float gSnapTemp[2] = {0}, gSnapHum[2] = {0};
int gSnapGas[2] = {0};

SemaphoreHandle_t xMutexData;
volatile bool gNeedAck[2] = {false, false};
volatile bool gAckMute[2] = {false, false};
volatile bool gNeedMuteSync[2] = {false, false}; 

void TaskRadio(void *pv);
void TaskButtons(void *pv);
void TaskAlarm(void *pv);
void TaskDisplay(void *pv);

// HÀM TÌM SLOT CỐ ĐỊNH CHỐNG LỘN XỘN
int getSlot(byte id) {
  if (id == 1) return 0;
  if (id == 2) return 1;
  return -1; // Từ chối tất cả các ID lạ khác
}

void autoLearnEdgeMAC(const uint8_t *mac, int idx, byte realId) {
  if (idx < 0 || idx > 1) return;
  bool isNewMac = false;
  
  // Tự động cập nhật nếu thay board mạch Edge khác (MAC khác)
  if (!gEdgeMacReady[idx]) {
    isNewMac = true;
  } else if (memcmp(gEdgeMACs[idx], mac, 6) != 0) {
    esp_now_del_peer(gEdgeMACs[idx]);
    isNewMac = true;
  }
  
  if (isNewMac) {
    memcpy(gEdgeMACs[idx], mac, 6);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, gEdgeMACs[idx], 6);
    peerInfo.channel = ESPNOW_CHANNEL; peerInfo.encrypt = false; peerInfo.ifidx = WIFI_IF_STA; 
    if(esp_now_add_peer(&peerInfo) == ESP_OK || esp_now_add_peer(&peerInfo) == ESP_ERR_ESPNOW_EXIST) {
        gEdgeMacReady[idx] = true;
        Serial.printf("[SYSTEM] Slot %d cap nhat MAC cua Node #%d\n", idx, realId);
    }
  }
}

void sendAckESPNOW(int idx, bool mute) {
  if (!gEdgeMacReady[idx]) return; 
  AckPacket ack = {mute};
  esp_now_send(gEdgeMACs[idx], (uint8_t *)&ack, sizeof(ack));
}

void sendTimeSyncESPNOW(int idx) {
  if (!gEdgeMacReady[idx]) return;
  TimeSyncPacket ts;
  ts.hr = (byte)gRtcH; ts.mn = (byte)gRtcM; ts.sc = (byte)gRtcS;
  ts.mute = gMuteNode[idx] ? 1 : 0;
  esp_now_send(gEdgeMACs[idx], (uint8_t*)&ts, sizeof(ts));
}

void onESPNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  if (data_len < 1) return;
  byte senderId = data[0]; 
  
  int idx = -1;
  if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(50)) == pdTRUE) {
    idx = getSlot(senderId); xSemaphoreGive(xMutexData);
  }
  if (idx == -1) return; 
  autoLearnEdgeMAC(recv_info->src_addr, idx, senderId);

  if (data_len >= (int)sizeof(DataPacket)) {
    DataPacket pkg; memcpy(&pkg, data, sizeof(DataPacket));
    if (xSemaphoreTake(xMutexData, 0) == pdTRUE) {
      gNode[idx].temp = pkg.t; gNode[idx].hum = pkg.h; gNode[idx].gas = pkg.g;
      gNode[idx].relayState = pkg.r; gNode[idx].alarm = pkg.alarm;
      gNode[idx].valid = true; gLastNodeSeen[idx] = millis();
      bool mute = gMuteNode[idx];
      xSemaphoreGive(xMutexData);
      gAckMute[idx] = mute; gNeedAck[idx] = true;
    }
  }
  else if (data_len == (int)sizeof(TimePacket)) {
    TimePacket tp; memcpy(&tp, data, sizeof(TimePacket));
    if (xSemaphoreTake(xMutexData, 0) == pdTRUE) {
      gNode[idx].edgeHr = tp.hr; gNode[idx].edgeMn = tp.mn; gNode[idx].edgeSc = tp.sc;
      gNode[idx].isTimeMaster = tp.hasRtc; 
      
      // LOGIC TIME RELAY: Chỉ lấy giờ làm chuẩn nếu thằng Edge gửi qua CÓ RTC
      if (tp.hasRtc) {
        gRtcH = tp.hr; gRtcM = tp.mn; gRtcS = tp.sc;
        gLastTick = millis(); 
        gRtcReady = true;
      }
      xSemaphoreGive(xMutexData);
    }
  }
  else if (data_len == (int)sizeof(MutePacket)) {
    MutePacket mp; memcpy(&mp, data, sizeof(MutePacket));
    if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (mp.mute == 1) {
        if (!gMuteNode[idx]) { 
          gMuteStartTime[idx] = millis();
          gSnapTemp[idx] = gNode[idx].temp; gSnapHum[idx] = gNode[idx].hum; gSnapGas[idx] = gNode[idx].gas;
        }
        gMuteNode[idx] = true; gHardMuteNode[idx] = false; 
      } else {
        if (!gHardMuteNode[idx]) gMuteNode[idx] = false;
      }
      gNeedMuteSync[idx] = true;
      gAckMute[idx] = gMuteNode[idx]; gNeedAck[idx] = true;
      xSemaphoreGive(xMutexData);
    }
  }
}

void onESPNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

bool initESPNOW() {
  WiFi.mode(WIFI_STA); WiFi.disconnect(false); delay(200);
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onESPNowRecv);
  esp_now_register_send_cb(onESPNowSent);
  return true;
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH);
  Serial.begin(115200); Wire.begin(I2C_SDA, I2C_SCL); Wire.setTimeOut(1000); 
  
  pinMode(LED1_PIN, OUTPUT); digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT); digitalWrite(LED2_PIN, LOW);
  const int btnPins[] = {25, 26, 27};
  for (int i = 0; i < 3; i++) pinMode(btnPins[i], INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("[ERR] OLED fail!");
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0, 0); display.println("Center Hub v6.4"); 
  display.setCursor(0, 20); display.println("Time Relayer Mode");
  display.setCursor(0, 35); display.println("Fixed LED Mapping");
  display.display(); delay(1000);

  gLastTick = millis(); 

  isEspNowReady = initESPNOW();
  xMutexData = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(TaskRadio, "Radio", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskButtons, "Buttons", 2048, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(TaskAlarm, "Alarm", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }

void TaskRadio(void *pv) {
  unsigned long lastTimeSync = 0;
  for (;;) {
    if (!isEspNowReady) {
      isEspNowReady = initESPNOW();
      if (!isEspNowReady) { vTaskDelay(pdMS_TO_TICKS(3000)); continue; }
    }
    bool nodeActive[2] = { (gNode[0].valid && gLastNodeSeen[0] > 0), 
                           (gNode[1].valid && gLastNodeSeen[1] > 0) };

    for (int i = 0; i < 2; i++) {
      if (gNeedAck[i]) {
        gNeedAck[i] = false;
        if (nodeActive[i]) { sendAckESPNOW(i, gAckMute[i]); vTaskDelay(pdMS_TO_TICKS(20)); }
      }
      if (gNeedMuteSync[i]) {
        gNeedMuteSync[i] = false;
        if (nodeActive[i]) { sendTimeSyncESPNOW(i); vTaskDelay(pdMS_TO_TICKS(20)); }
      }
    }
    
    // Đổ bộ thời gian cho các SLAVE 
    if (gRtcReady && millis() - lastTimeSync >= 3000) {
      lastTimeSync = millis();
      for (int i = 0; i < 2; i++) {
        if (nodeActive[i]) { 
          sendTimeSyncESPNOW(i); 
          vTaskDelay(pdMS_TO_TICKS(30)); 
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// === CÁC TÁC VỤ NÚT BẤM VÀ BÁO ĐỘNG ===
#define BTN_MUTE_SLOT0 25 // Nút Mute dành riêng cho Node 1
#define BTN_MUTE_SLOT1 26 // Nút Mute dành riêng cho Node 2
#define BTN_VIEW 27
const int BTN_PINS_C[3] = {BTN_MUTE_SLOT0, BTN_MUTE_SLOT1, BTN_VIEW};
static int btnState[3] = {HIGH, HIGH, HIGH}, btnLast[3] = {HIGH, HIGH, HIGH};
static uint32_t btnDebounce[3] = {0};
static uint32_t btnPressTime[3] = {0, 0, 0};
static bool btnLongPressHandled[3] = {false, false, false};

void TaskButtons(void *pv) {
  TickType_t xLastWake = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    for (int i = 0; i < 3; i++) {
      int pin = BTN_PINS_C[i]; int rd = digitalRead(pin);
      if (rd != btnLast[i]) btnDebounce[i] = now;
      if ((now - btnDebounce[i]) > 50) {
        if (rd != btnState[i]) {
          btnState[i] = rd;
          if (rd == LOW) {
            btnPressTime[i] = now; btnLongPressHandled[i] = false;
          } else {
            if (!btnLongPressHandled[i]) {
              if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(50)) == pdTRUE) {
                int nIdx = -1;
                // Áp đặt Nút bấm vào thẳng Slot
                if (pin == BTN_MUTE_SLOT0) nIdx = 0;
                else if (pin == BTN_MUTE_SLOT1) nIdx = 1;
                else if (pin == BTN_VIEW) { gViewNode = (gViewNode + 1) % 2; }

                if (nIdx != -1) {
                  gMuteNode[nIdx] = !gMuteNode[nIdx]; gHardMuteNode[nIdx] = false; 
                  if (gMuteNode[nIdx]) {
                    gMuteStartTime[nIdx] = millis();
                    gSnapTemp[nIdx] = gNode[nIdx].temp; gSnapHum[nIdx] = gNode[nIdx].hum; gSnapGas[nIdx] = gNode[nIdx].gas;
                  }
                  gNeedMuteSync[nIdx] = true;
                  gAckMute[nIdx] = gMuteNode[nIdx]; gNeedAck[nIdx] = true;
                } 
                xSemaphoreGive(xMutexData);
              }
            }
          }
        } 
        else if (rd == LOW && !btnLongPressHandled[i]) {
          if (now - btnPressTime[i] >= 3000) {
            btnLongPressHandled[i] = true; 
            if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(50)) == pdTRUE) {
              int nIdx = -1;
              if (pin == BTN_MUTE_SLOT0) nIdx = 0;
              else if (pin == BTN_MUTE_SLOT1) nIdx = 1;
              
              if (nIdx != -1) {
                gMuteNode[nIdx] = true; gHardMuteNode[nIdx] = true; 
                gNeedMuteSync[nIdx] = true; gAckMute[nIdx] = true; gNeedAck[nIdx] = true;
              }
              xSemaphoreGive(xMutexData);
            }
          }
        }
      }
      btnLast[i] = rd;
    }
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(20));
  }
}

void TaskAlarm(void *pv) {
  TickType_t xLastWake = xTaskGetTickCount();
  bool blinkOn = false; uint32_t lastBlink = 0;
  for (;;) {
    if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint32_t now = millis();
      for(int i = 0; i < 2; i++) {
        if (gNode[i].valid && gNode[i].alarm) {
          if (gMuteNode[i] && !gHardMuteNode[i]) {
            float dT = abs(gNode[i].temp - gSnapTemp[i]); float dH = abs(gNode[i].hum - gSnapHum[i]); int dG = abs(gNode[i].gas - gSnapGas[i]);
            if (dT >= VAR_TEMP || dH >= VAR_HUM || dG >= VAR_GAS || (now - gMuteStartTime[i] >= MUTE_DURATION)) {
              gMuteNode[i] = false; gNeedMuteSync[i] = true; gAckMute[i] = false; gNeedAck[i] = true;
            }
          }
        } else {
          if (gMuteNode[i]) { gMuteNode[i] = false; gHardMuteNode[i] = false; gNeedMuteSync[i] = true; }
        }
      }
      
      // LOGIC LED TÁCH BIỆT (LED 1 luôn của Node 1, LED 2 luôn của Node 2)
      bool a1 = gNode[0].valid && gNode[0].alarm; 
      bool a2 = gNode[1].valid && gNode[1].alarm;
      bool m1 = gMuteNode[0]; 
      bool m2 = gMuteNode[1];
      
      digitalWrite(BUZZER_PIN, ((a1 && !m1) || (a2 && !m2)) ? LOW : HIGH);
      if (now - lastBlink >= 300) { lastBlink = now; blinkOn = !blinkOn; }
      
      digitalWrite(LED1_PIN, (a1 && !m1 && blinkOn) ? HIGH : LOW);
      digitalWrite(LED2_PIN, (a2 && !m2 && blinkOn) ? HIGH : LOW);
      
      xSemaphoreGive(xMutexData);
    } 
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(100));
  }
}

void TaskDisplay(void *pv) {
  TickType_t xLastWake = xTaskGetTickCount();
  for (;;) {
    unsigned long now = millis();
    if (now - gLastTick >= 1000) {
      unsigned long elapsed = (now - gLastTick) / 1000;
      gLastTick += elapsed * 1000;
      gRtcS += elapsed;
      while (gRtcS >= 60) {
        gRtcS -= 60; gRtcM++;
        if (gRtcM >= 60) { gRtcM = 0; gRtcH++; if (gRtcH >= 24) gRtcH = 0; }
      }
    }

    if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(50)) == pdTRUE) {
      int v = gViewNode; 
      byte idV = gNode[v].nodeId; float tV = gNode[v].temp, hV = gNode[v].hum; int gV = gNode[v].gas;
      bool rlyV = gNode[v].relayState, mutedV = gMuteNode[v], hardMutedV = gHardMuteNode[v], alarmV = gNode[v].valid && gNode[v].alarm;
      unsigned long seenV = gLastNodeSeen[v]; byte eHrV = gNode[v].edgeHr, eMnV = gNode[v].edgeMn, eScV = gNode[v].edgeSc;
      bool isMasterV = gNode[v].isTimeMaster;
      
      int ot = 1 - v; byte idOt = gNode[ot].nodeId;
      bool mutedOt = gMuteNode[ot], hardMutedOt = gHardMuteNode[ot], alarmOt = gNode[ot].valid && gNode[ot].alarm;
      unsigned long seenOt = gLastNodeSeen[ot];
      xSemaphoreGive(xMutexData);

      bool blink = (millis() / 500) % 2; 
      bool onlineV = (seenV > 0) && ((now - seenV) < 30000UL); bool timedOutV = (seenV == 0) || ((now - seenV) >= NODE_TIMEOUT);
      bool onlineOt = (seenOt > 0) && ((now - seenOt) < 30000UL);

      display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);

      if (timedOutV) {
        display.setCursor(0, 0); display.printf("%02d:%02d:%02d  [EN]", gRtcH, gRtcM, gRtcS);
        display.setCursor(0, 18); display.print("---- CENTER HUB ----");
        display.setCursor(0, 32); display.printf("Node #%d: MAT KET NOI", idV); display.setCursor(0, 46); display.print("Dang cho du lieu...");
        for (int d = 0; d < (now / 500) % 4; d++) display.print(".");
      }
      else if (!onlineV) {
        display.setCursor(0, 0); display.printf("%02d:%02d N%d [OFF]", gRtcH, gRtcM, idV); 
        display.setCursor(0, 12); display.printf("Gas : %4d", gV); display.setCursor(0, 23); display.printf("Temp: %.1f C", tV);
        display.setCursor(0, 32); display.printf("Hum : %.0f %%", hV); display.setCursor(0, 41); display.printf("Relay: %s", rlyV ? "ON " : "OFF");
        display.setCursor(0, 53); display.printf(blink ? "N%d: MAT KET NOI" : "N%d AN TOAN", idV);
      }
      else {
        display.setCursor(0, 0); display.printf("%02d:%02d:%02d N%d%s", eHrV, eMnV, eScV, idV, isMasterV ? "[MASTER]" : "[SLAVE]");
        display.setCursor(0, 12); display.printf("Gas : %4d", gV); if (alarmV && !mutedV && blink) display.print(" [!]");
        display.setCursor(0, 23); display.printf("Temp: %.1f C", tV); display.setCursor(0, 32); display.printf("Hum : %.0f %%", hV);
        display.setCursor(0, 41); display.printf("Relay: %s", rlyV ? "ON " : "OFF");
        display.setCursor(0, 53);
        if (alarmV) { display.printf(hardMutedV ? "N%d [H.MUTED]" : mutedV ? "N%d [MUTED]" : "N%d [ALARM!]", idV); } 
        else { display.printf("N%d AN TOAN", idV); }
      }

      display.setCursor(80, 53);
      if (!onlineOt) display.printf("N%d:---", idOt);
      else if (alarmOt && !mutedOt && blink) display.printf("N%d:ALM", idOt);
      else if (mutedOt) { display.printf(hardMutedOt ? "N%d:[HM]" : "N%d:[M]", idOt); }
      else display.printf("N%d: OK", idOt);
      display.display();
    }
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(300));
  }
}