/*
 * ================================================================
 * EDGE NODE v10.4 — Master/Slave RTC (Thêm NONE) | Fix Traffic Jam
 * ================================================================
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <RTClib.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <EEPROM.h>

const uint8_t HUB_MAC[6] = {0x70, 0x4B, 0xCA, 0x8F, 0x51, 0xA8}; // Điền MAC Center
#define ESPNOW_CHANNEL 1

const float VAR_TEMP = 0.5;    
const float VAR_HUM = 2.0;     
const int VAR_GAS = 50;        
const unsigned long MUTE_DURATION = 60000; 

#define I2C_SDA 21
#define I2C_SCL 22
#define DHTPIN 15
#define MQ2_PIN 34
#define RELAY_PIN 12        
#define BUZZER_PIN 13       
#define LED_ALARM_PIN 14

#define BTN_MUTE 16   
#define BTN_RELAY 17  
#define BTN_MODE 32   
#define BTN_UP 33     
#define BTN_DOWN 25   
#define BTN_SELECT 26 

#define MODE_HOME 0
#define MODE_TIMER 1
#define MODE_TEMP 2
#define MODE_HUM 3
#define MODE_GAS 4
#define MODE_NODEID 5  
#define MODE_COUNT 6
#define MODE_BOOT_SETUP 99 

struct __attribute__((packed)) DataPacket { byte id; float t; float h; int g; bool r; bool alarm; };
struct __attribute__((packed)) AckPacket { bool mute; };
struct __attribute__((packed)) MutePacket { byte id; byte mute; };
struct __attribute__((packed)) TimePacket { byte id; byte hr; byte mn; byte sc; bool hasRtc; };
struct __attribute__((packed)) TimeSyncPacket { byte hr; byte mn; byte sc; byte mute; };

Adafruit_SSD1306 display(128, 64, &Wire, -1);

DHT* dht = nullptr;       
RTC_DS3231 rtc3231;
RTC_DS1307 rtc1307;

int dhtType = 22;         
int rtcType = 3231;       // 0 = NONE, 3231 = DS3231, 1307 = DS1307
int tempDhtType = 22;
int tempRtcType = 3231;

byte nodeId = 1; 
byte tempNodeId = 1; 
float curTemp = 0, curHum = 0; int curGas = 0;
int curHr = 0, curMin = 0, curSec = 0;
bool sensorValid = false, rtcOK = false;

bool relayState = false; 
bool alarmTriggered = false, alarmMuted = false, hardMuted = false;

unsigned long muteStartTime = 0;
float snapTemp = 0, snapHum = 0; int snapGas = 0;

float thresTempMax = 35.0, thresTempMin = 20.0;
float thresHum = 80.0; int thresGas = 2000;
int timerOnHr = 8, timerOnMin = 0, timerOffHr = 17, timerOffMin = 0;
int currentMode = MODE_HOME, subSelect = 0; 

unsigned long lastSensorRead = 0, lastOledUpdate = 0, lastRadioSend = 0;
int lastTimerCheckMin = -1;
volatile unsigned long lastCenterSync = 0xFFFF0000UL;
unsigned long lastSecondTick = 0; 

volatile bool radioDataReady = false; DataPacket radioPacket;
volatile bool radioTimeReady = false; TimePacket timePacket;
volatile bool radioMuteReady = false; MutePacket mutePacket;

struct Button {
  uint8_t pin; bool lastReading, stableState;
  unsigned long debounceTime, pressTime, repeatTime;
  bool longPressHandled;
};
Button btns[6] = {
    {BTN_MUTE, HIGH, HIGH, 0, 0, 0, false}, {BTN_RELAY, HIGH, HIGH, 0, 0, 0, false},
    {BTN_MODE, HIGH, HIGH, 0, 0, 0, false}, {BTN_UP, HIGH, HIGH, 0, 0, 0, false},
    {BTN_DOWN, HIGH, HIGH, 0, 0, 0, false}, {BTN_SELECT, HIGH, HIGH, 0, 0, 0, false}
};

String getRtcName(int type) {
  if (type == 3231) return "DS3231";
  if (type == 1307) return "DS1307";
  return "NONE";
}

void initDHT() {
  if (dht != nullptr) { delete dht; }
  dht = new DHT(DHTPIN, (dhtType == 22) ? DHT22 : DHT11);
  dht->begin();
}

void initRTC() {
  rtcOK = false;
  if (rtcType == 0) {
    Serial.println("[SYSTEM] RTC Disabled (NONE Mode). Node is TIME SLAVE.");
    return;
  }
  
  if (rtcType == 3231) {
    if (rtc3231.begin()) {
      if (rtc3231.lostPower()) rtc3231.adjust(DateTime(F(__DATE__), F(__TIME__)));
      rtcOK = true;
    }
  } else if (rtcType == 1307) {
    if (rtc1307.begin()) {
      if (!rtc1307.isrunning()) rtc1307.adjust(DateTime(F(__DATE__), F(__TIME__)));
      rtcOK = true;
    }
  }
  Serial.printf("[SYSTEM] Initialized %s | Status: %s\n", getRtcName(rtcType).c_str(), rtcOK ? "OK" : "FAIL");
}

DateTime getRTCTime() {
  if (rtcType == 3231) return rtc3231.now();
  return rtc1307.now();
}

void onESPNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (currentMode == MODE_BOOT_SETUP) return; 
  if (len == (int)sizeof(TimeSyncPacket)) {
    TimeSyncPacket ts; memcpy(&ts, data, sizeof(ts));
    
    // NẾU LÀ SLAVE (KHÔNG CÓ RTC), LUÔN CẬP NHẬT GIỜ TỪ CENTER
    if (!rtcOK) {
      curHr = ts.hr; curMin = ts.mn; curSec = ts.sc;
      lastSecondTick = millis(); 
    }
    lastCenterSync = millis(); 
    
    if (ts.mute == 1) {
      if (!alarmMuted) {
        alarmMuted = true; hardMuted = false; muteStartTime = millis();
        snapTemp = curTemp; snapHum = curHum; snapGas = curGas;
      }
    } else { if (!hardMuted) alarmMuted = false; }
  } else if (len == (int)sizeof(AckPacket)) {
    AckPacket ack; memcpy(&ack, data, sizeof(AckPacket));
    if (ack.mute) {
      if (!alarmMuted) {
        alarmMuted = true; hardMuted = false; muteStartTime = millis();
        snapTemp = curTemp; snapHum = curHum; snapGas = curGas;
      }
    } else { if (!hardMuted) alarmMuted = false; }
  }
}
void onESPNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) { }

bool initESPNOW() {
  WiFi.mode(WIFI_STA); WiFi.disconnect(false); delay(200);
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(onESPNowSent);
  esp_now_register_recv_cb(onESPNowRecv);
  esp_now_peer_info_t peer = {}; memcpy(peer.peer_addr, HUB_MAC, 6);
  peer.channel = ESPNOW_CHANNEL; peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { esp_now_deinit(); return false; }
  return true;
}

void TaskRadio(void *pv) {
  bool ready = initESPNOW(); int failCount = 0;
  for (;;) {
    if (currentMode == MODE_BOOT_SETUP) { vTaskDelay(pdMS_TO_TICKS(500)); continue; } 
    if (!ready) {
      ready = initESPNOW();
      if (!ready) { vTaskDelay(pdMS_TO_TICKS(3000)); continue; }
    }
    
    bool sentAnything = false;

    if (radioDataReady) {
      radioDataReady = false;
      esp_err_t res = esp_now_send(HUB_MAC, (uint8_t *)&radioPacket, sizeof(radioPacket));
      if (res == ESP_OK) { failCount = 0; } else { failCount++; }
      sentAnything = true;
    }

    if (sentAnything) vTaskDelay(pdMS_TO_TICKS(30)); // HÍT THỞ CHỐNG KẸT XE

    if (radioTimeReady) {
      radioTimeReady = false;
      esp_now_send(HUB_MAC, (uint8_t *)&timePacket, sizeof(timePacket));
      sentAnything = true;
    }

    if (sentAnything) vTaskDelay(pdMS_TO_TICKS(30));

    if (radioMuteReady) {
      radioMuteReady = false;
      esp_now_send(HUB_MAC, (uint8_t *)&mutePacket, sizeof(mutePacket));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200); delay(200);
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setTimeOut(1000); 

  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH); 
  pinMode(LED_ALARM_PIN, OUTPUT); digitalWrite(LED_ALARM_PIN, LOW);
  for (int i = 0; i < 6; i++) pinMode(btns[i].pin, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("[ERR] OLED fail!");
  display.clearDisplay(); display.display();

  EEPROM.begin(4); 
  byte isConfigured = EEPROM.read(3);
  
  if (isConfigured != 0xAB) {
    currentMode = MODE_BOOT_SETUP;
    tempNodeId = 1; tempDhtType = 22; tempRtcType = 3231; 
  } else {
    nodeId = EEPROM.read(0); tempNodeId = nodeId;
    dhtType = EEPROM.read(1); tempDhtType = dhtType;
    byte sRtc = EEPROM.read(2);
    rtcType = (sRtc == 1) ? 3231 : ((sRtc == 2) ? 1307 : 0);
    tempRtcType = rtcType;
    
    initDHT();
    initRTC();
    if (rtcOK) {
      DateTime dt = getRTCTime();
      curHr = dt.hour(); curMin = dt.minute(); curSec = dt.second();
    }
    currentMode = MODE_HOME;
  }
  
  lastSecondTick = millis();
  xTaskCreatePinnedToCore(TaskRadio, "Radio", 8192, NULL, 1, NULL, 0);
}

void readSensors() {
  if (currentMode == MODE_BOOT_SETUP) return; 
  if (millis() - lastSensorRead < 2000) return;
  lastSensorRead = millis();
  
  float t = dht->readTemperature(); 
  float h = dht->readHumidity(); 
  curGas = analogRead(MQ2_PIN);
  
  if (!isnan(t) && !isnan(h)) { curTemp = t; curHum = h; sensorValid = true; }
  else { sensorValid = false; }
}

// BỘ ĐẾM THỜI GIAN ĐƯỢC LÀM MỚI HOÀN TOÀN
void tickClock() {
  if (currentMode == MODE_BOOT_SETUP) return;
  unsigned long now = millis();
  if (now - lastSecondTick >= 1000) {
    unsigned long elapsed = (now - lastSecondTick) / 1000;
    lastSecondTick += elapsed * 1000; 
    
    if (rtcOK) {
      // MASTER: Luôn lấy giờ chuẩn từ mạch RTC vật lý
      DateTime dt = getRTCTime();
      curHr = dt.hour(); curMin = dt.minute(); curSec = dt.second();
    } else {
      // SLAVE: Tự đếm giờ ảo, đợi Center gửi gói tin sang để chốt lại
      curSec += (int)elapsed;
      while (curSec >= 60) {
        curSec -= 60; curMin++;
        if (curMin >= 60) { curMin = 0; curHr++; if (curHr >= 24) curHr = 0; }
      }
    }
  }
}

void checkAlarms() {
  if (currentMode == MODE_BOOT_SETUP) return; 
  bool cond = sensorValid && (curTemp > thresTempMax || curTemp < thresTempMin || curHum > thresHum || curGas > thresGas);
  if (cond) {
    alarmTriggered = true;
    if (alarmMuted) {
      if (!hardMuted) {
        float dT = abs(curTemp - snapTemp); float dH = abs(curHum - snapHum); int dG = abs(curGas - snapGas);
        if (dT >= VAR_TEMP || dH >= VAR_HUM || dG >= VAR_GAS || (millis() - muteStartTime >= MUTE_DURATION)) {
          alarmMuted = false;
          mutePacket.id = nodeId; mutePacket.mute = 0; radioMuteReady = true;
        }
      }
    }
  } else { 
    alarmTriggered = false; alarmMuted = false; hardMuted = false; 
  }
}

void controlActuators() {
  if (currentMode == MODE_BOOT_SETUP) return;
  digitalWrite(RELAY_PIN, relayState);

  if (alarmTriggered && !alarmMuted) {
    unsigned long cycle = millis() % 300;
    digitalWrite(BUZZER_PIN, (cycle < 200) ? LOW : HIGH); 
    digitalWrite(LED_ALARM_PIN, (millis() / 200) % 2);
  } else {
    digitalWrite(BUZZER_PIN, HIGH); digitalWrite(LED_ALARM_PIN, LOW);
  }

  if (curMin != lastTimerCheckMin) {
    lastTimerCheckMin = curMin;
    if (curHr == timerOnHr && curMin == timerOnMin) relayState = true;
    if (curHr == timerOffHr && curMin == timerOffMin) relayState = false;
  }
}

void handleButtonAction(uint8_t pin, bool isLongPress);

void processButtons() {
  uint32_t now = millis();
  for (int i = 0; i < 6; i++) {
    int rd = digitalRead(btns[i].pin);
    if (rd != btns[i].lastReading) btns[i].debounceTime = now;
    
    if ((now - btns[i].debounceTime) > 50) {
      if (rd != btns[i].stableState) {
        btns[i].stableState = rd;
        if (rd == LOW) {
          btns[i].pressTime = now; btns[i].repeatTime = now; btns[i].longPressHandled = false;
        } else {
          if (!btns[i].longPressHandled) { handleButtonAction(btns[i].pin, false); }
        }
      } 
      else if (rd == LOW) {
        if (!btns[i].longPressHandled && (now - btns[i].pressTime >= 3000)) {
          btns[i].longPressHandled = true; handleButtonAction(btns[i].pin, true); 
        }
        
        if (btns[i].pin == BTN_UP || btns[i].pin == BTN_DOWN) {
          if ((now - btns[i].pressTime > 600) && (now - btns[i].repeatTime >= 150)) {
            btns[i].repeatTime = now; handleButtonAction(btns[i].pin, false);
          }
        }
      }
    }
    btns[i].lastReading = rd;
  }
}

void handleButtonAction(uint8_t pin, bool isLongPress) {
  if (currentMode == MODE_BOOT_SETUP) {
    if (pin == BTN_MODE || pin == BTN_SELECT) {
      if (!isLongPress) subSelect = (subSelect + 1) % 3;
      else {
        EEPROM.write(0, tempNodeId);
        EEPROM.write(1, tempDhtType);
        EEPROM.write(2, (tempRtcType == 3231) ? 1 : ((tempRtcType == 1307) ? 2 : 0));
        EEPROM.write(3, 0xAB); 
        EEPROM.commit();
        
        nodeId = tempNodeId; dhtType = tempDhtType; rtcType = tempRtcType;
        display.clearDisplay(); display.setCursor(0, 25);
        display.print("Saving & Booting..."); display.display();
        delay(800);
        initDHT(); initRTC(); 
        currentMode = MODE_HOME; subSelect = 0;
      }
    }
    else if (pin == BTN_UP || pin == BTN_DOWN) {
      if (subSelect == 0) tempDhtType = (tempDhtType == 22) ? 11 : 22;
      else if (subSelect == 1) {
        if (pin == BTN_UP) {
          if (tempRtcType == 3231) tempRtcType = 1307;
          else if (tempRtcType == 1307) tempRtcType = 0;
          else tempRtcType = 3231;
        } else {
          if (tempRtcType == 3231) tempRtcType = 0;
          else if (tempRtcType == 0) tempRtcType = 1307;
          else tempRtcType = 3231;
        }
      }
      else if (subSelect == 2) {
        if (pin == BTN_UP) tempNodeId = (tempNodeId < 9) ? tempNodeId + 1 : 1;
        else tempNodeId = (tempNodeId > 1) ? tempNodeId - 1 : 9;
      }
    }
    return; 
  }

  if (isLongPress && pin != BTN_MUTE) return;

  if (pin == BTN_MUTE) {
    if (alarmTriggered) {
      if (isLongPress) {
        alarmMuted = true; hardMuted = true;
        mutePacket.id = nodeId; mutePacket.mute = 1; radioMuteReady = true;
      } else {
        alarmMuted = true; hardMuted = false;
        muteStartTime = millis(); snapTemp = curTemp; snapHum = curHum; snapGas = curGas;
        mutePacket.id = nodeId; mutePacket.mute = 1; radioMuteReady = true;
      }
    }
  }
  else if (pin == BTN_RELAY) { relayState = !relayState; }
  else if (pin == BTN_MODE) {
    bool needSave = false;
    if (tempDhtType != dhtType) {
      dhtType = tempDhtType; EEPROM.write(1, dhtType); initDHT(); needSave = true;
    }
    if (tempRtcType != rtcType) {
      rtcType = tempRtcType; EEPROM.write(2, (rtcType == 3231) ? 1 : ((rtcType == 1307) ? 2 : 0)); initRTC(); needSave = true;
    }
    if (needSave) EEPROM.commit();

    currentMode = (currentMode + 1) % MODE_COUNT; subSelect = 0; 
    tempNodeId = nodeId; 
  }
  else if (pin == BTN_SELECT) {
    if (currentMode == MODE_HOME) { /* Tab Home */ } 
    else if (currentMode == MODE_TIMER) { subSelect = (subSelect + 1) % 5; } 
    else if (currentMode == MODE_TEMP) { subSelect = (subSelect + 1) % 3; }  
    else if (currentMode == MODE_NODEID) {
      if (tempNodeId != nodeId) {
        EEPROM.write(0, tempNodeId); EEPROM.commit();
        display.clearDisplay(); display.setCursor(0, 20); display.print("Saving & Rebooting..."); display.display();
        delay(1000); ESP.restart(); 
      }
    }
  }
  else if (pin == BTN_UP || pin == BTN_DOWN) {
    if (currentMode == MODE_HOME) return;
    if (currentMode == MODE_TIMER) {
      if (subSelect == 0) { if (pin == BTN_UP) timerOnHr = (timerOnHr + 1) % 24; else timerOnHr = (timerOnHr == 0) ? 23 : timerOnHr - 1; }
      else if (subSelect == 1) { if (pin == BTN_UP) timerOnMin = (timerOnMin + 1) % 60; else timerOnMin = (timerOnMin == 0) ? 59 : timerOnMin - 1; }
      else if (subSelect == 2) { if (pin == BTN_UP) timerOffHr = (timerOffHr + 1) % 24; else timerOffHr = (timerOffHr == 0) ? 23 : timerOffHr - 1; }
      else if (subSelect == 3) { if (pin == BTN_UP) timerOffMin = (timerOffMin + 1) % 60; else timerOffMin = (timerOffMin == 0) ? 59 : timerOffMin - 1; }
      else if (subSelect == 4) { 
        if (pin == BTN_UP) {
          if (tempRtcType == 3231) tempRtcType = 1307; else if (tempRtcType == 1307) tempRtcType = 0; else tempRtcType = 3231;
        } else {
          if (tempRtcType == 3231) tempRtcType = 0; else if (tempRtcType == 0) tempRtcType = 1307; else tempRtcType = 3231;
        }
      } 
    } else if (currentMode == MODE_TEMP) {
      if (subSelect == 0) { if (pin == BTN_UP) thresTempMax += 0.5; else thresTempMax -= 0.5; }
      else if (subSelect == 1) { if (pin == BTN_UP) thresTempMin += 0.5; else thresTempMin -= 0.5; }
      else if (subSelect == 2) { tempDhtType = (tempDhtType == 22) ? 11 : 22; } 
    } else if (currentMode == MODE_HUM) { 
      if (pin == BTN_UP) thresHum += 1.0; else thresHum -= 1.0; 
    } else if (currentMode == MODE_GAS) { 
      if (pin == BTN_UP) thresGas += 50; else thresGas = (thresGas <= 50) ? 0 : thresGas - 50; 
    } else if (currentMode == MODE_NODEID) { 
      if (pin == BTN_UP) tempNodeId = (tempNodeId < 9) ? tempNodeId + 1 : 1;
      else tempNodeId = (tempNodeId > 1) ? tempNodeId - 1 : 9;
    }
  }
}

void updateOLED() {
  if (millis() - lastOledUpdate < 100) return;
  lastOledUpdate = millis();
  bool blink = (millis() / 500) % 2 == 0;
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);

  if (currentMode == MODE_BOOT_SETUP) {
    display.fillRect(0, 0, 128, 9, WHITE); display.setTextColor(BLACK, WHITE);
    display.setCursor(2, 1); display.print("HARDWARE SETUP WIZARD"); display.setTextColor(WHITE);
    
    display.setCursor(0, 15); display.printf("DHT : %s%s", (subSelect == 0 && blink) ? ">" : " ", (tempDhtType == 22) ? "DHT22" : "DHT11");
    display.setCursor(0, 27); display.printf("RTC : %s%s", (subSelect == 1 && blink) ? ">" : " ", getRtcName(tempRtcType).c_str());
    display.setCursor(0, 39); display.printf("NODE: %s%d", (subSelect == 2 && blink) ? ">" : " ", tempNodeId);
    
    display.setCursor(0, 52); display.print("HOLD 'SELECT' TO SAVE");
  } 
  else if (currentMode == MODE_HOME) {
    display.fillRect(0, 0, 128, 9, WHITE); display.setTextColor(BLACK, WHITE);
    display.setCursor(2, 1); display.printf("NODE #%d [EN] %02d:%02d:%02d", nodeId, curHr, curMin, curSec);
    display.setTextColor(WHITE);

    bool tHi = sensorValid && (curTemp > thresTempMax); bool tLo = sensorValid && (curTemp < thresTempMin);
    bool hA = sensorValid && (curHum > thresHum); bool gA = sensorValid && (curGas > thresGas);

    display.setCursor(0, 11);
    display.printf("TEMP : %s", sensorValid ? (String(curTemp, 1) + "C " + (tHi ? "[HI]" : tLo ? "[LO]" : "   ")).c_str() : "---");
    display.setCursor(0, 21);
    display.printf("HUM  : %s", sensorValid ? (String(curHum, 0) + "%%  " + (hA ? "[!]" : "   ")).c_str() : "---");
    display.setCursor(0, 31);
    display.printf("GAS  : %s", sensorValid ? (String(curGas) + "   " + (gA ? "[!]" : "   ")).c_str() : "---");
    display.setCursor(0, 41); display.printf("RELAY: %s", relayState ? "ON" : "OFF");
    
    display.setCursor(0, 52);
    if (alarmTriggered) {
      if (hardMuted) display.print(">> [HARD MUTED] <<");
      else if (alarmMuted) display.print(">> [MUTED SNOOZE] <<");
      else display.print(">> [ALARM ACTIVE] <<");
    }
    if ((tHi || tLo || hA || gA) && blink) display.fillRect(115, 52, 10, 10, WHITE);
  } else if (currentMode == MODE_TIMER) {
    display.fillRect(0, 0, 128, 9, WHITE); display.setTextColor(BLACK, WHITE);
    display.setCursor(2, 1); display.print("TIMER SETTINGS"); display.setTextColor(WHITE);
    display.setCursor(0, 12); display.printf("ON  : %s%02d : %s%02d", (subSelect == 0 && blink) ? ">" : " ", timerOnHr, (subSelect == 1 && blink) ? ">" : " ", timerOnMin);
    display.setCursor(0, 25); display.printf("OFF : %s%02d : %s%02d", (subSelect == 2 && blink) ? ">" : " ", timerOffHr, (subSelect == 3 && blink) ? ">" : " ", timerOffMin);
    display.setCursor(0, 38); display.printf("RTC : %s%s", (subSelect == 4 && blink) ? ">" : " ", getRtcName(tempRtcType).c_str());
    display.setCursor(0, 50); display.printf("Relay now: %s", relayState ? "ON" : "OFF");
  } else if (currentMode == MODE_TEMP) {
    display.fillRect(0, 0, 128, 9, WHITE); display.setTextColor(BLACK, WHITE);
    display.setCursor(2, 1); display.print("TEMP THRESHOLD"); display.setTextColor(WHITE);
    display.setCursor(0, 12); display.printf("Now: %s", sensorValid ? (String(curTemp, 1) + " C").c_str() : "---");
    display.setCursor(0, 25); display.printf("%sMAX: %.1f C", (subSelect == 0 && blink) ? ">" : " ", thresTempMax);
    display.setCursor(0, 38); display.printf("%sMIN: %.1f C", (subSelect == 1 && blink) ? ">" : " ", thresTempMin);
    display.setCursor(0, 50); display.printf("DHT: %s%s", (subSelect == 2 && blink) ? ">" : " ", (tempDhtType == 22) ? "DHT22" : "DHT11");
  } else if (currentMode == MODE_HUM) {
    display.fillRect(0, 0, 128, 9, WHITE); display.setTextColor(BLACK, WHITE);
    display.setCursor(2, 1); display.print("HUM  THRESHOLD"); display.setTextColor(WHITE);
    display.setCursor(0, 15); display.printf("Now  : %s", sensorValid ? (String(curHum, 0) + " %%").c_str() : "---");
    display.setCursor(0, 35); display.printf("> Set: %.0f %%", thresHum);
  } else if (currentMode == MODE_GAS) {
    display.fillRect(0, 0, 128, 9, WHITE); display.setTextColor(BLACK, WHITE);
    display.setCursor(2, 1); display.print("GAS  THRESHOLD"); display.setTextColor(WHITE);
    display.setCursor(0, 15); display.printf("Now  : %s", sensorValid ? String(curGas).c_str() : "---");
    display.setCursor(0, 35); display.printf("> Set: %d", thresGas);
  } else if (currentMode == MODE_NODEID) {
    display.fillRect(0, 0, 128, 9, WHITE); display.setTextColor(BLACK, WHITE);
    display.setCursor(2, 1); display.print("SYSTEM SETTING"); display.setTextColor(WHITE);
    display.setCursor(0, 25); display.printf("EDGE NODE ID: [%d]", tempNodeId);
    display.setCursor(0, 42); display.print("UP/DOWN: Change ID");
    display.setCursor(0, 52); display.print("SELECT : Save & Boot");
  }
  display.display();
}

void prepareRadioData() {
  if (currentMode == MODE_BOOT_SETUP) return; 
  if (millis() - lastRadioSend < 3000) return;
  lastRadioSend = millis();
  radioPacket.id = nodeId; radioPacket.t = curTemp; radioPacket.h = curHum;
  radioPacket.g = curGas; radioPacket.r = relayState; radioPacket.alarm = alarmTriggered;
  radioDataReady = true;

  // BÁO CÁO GIỜ ĐỒNG THỜI ĐÍNH KÈM CỜ XÁC NHẬN "TÔI CÓ RTC"
  timePacket.id = nodeId; timePacket.hr = (byte)curHr; timePacket.mn = (byte)curMin; timePacket.sc = (byte)curSec;
  timePacket.hasRtc = rtcOK;
  radioTimeReady = true;
}

void loop() {
  tickClock(); 
  readSensors();
  processButtons();
  checkAlarms();
  controlActuators();
  updateOLED();
  prepareRadioData();
  delay(10);
}