#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ### KONFIGURACJA SIECI I MQTT - UZUPEŁNIJ! ###
const char* WIFI_SSID = "TWOJA_NAZWA_SIECI_WIFI";
const char* WIFI_PASSWORD = "TWOJE_HASLO_DO_WIFI";

const char* MQTT_SERVER = "192.168.1.100"; // Adres IP Twojego serwera Home Assistant
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt_user";       // Użytkownik MQTT (jeśli masz)
const char* MQTT_PASSWORD = "mqtt_password"; // Hasło MQTT (jeśli masz)

const char* MQTT_STATE_TOPIC = "ups/state";
const char* MQTT_AVAILABILITY_TOPIC = "ups/status";
// #################################################


// Definicje pinów i stałych
#define OLED_SDA_PIN 8
#define OLED_SCL_PIN 9
#define RELAY_PIN 7
#define POWER_LOSS_PIN 6
#define LED_RED_PIN 2
#define LED_GREEN_PIN 3
#define LED_BLUE_PIN 4
#define BATTERY_VOLTAGE_PIN 0
#define OUTPUT_VOLTAGE_PIN 1

// Stałe kalibracyjne i progowe
#define BATTERY_VOLTAGE_RATIO 0.2117 // Poprzednio 0.2099
#define OUTPUT_VOLTAGE_RATIO 0.2132 // Bez zmian
#define ADC_REF_VOLTAGE 3.3
#define ADC_MAX_VALUE 4095.0

#define VOLTAGE_RECOVERY 11.0
#define VOLTAGE_UNDER 10.0
#define VOLTAGE_OVER_RECOVERY 13.0
#define VOLTAGE_OVER 13.5
#define VOLTAGE_OVER_SHUTDOWN 14.0

// Stałe czasowe
#define SHUTDOWN_DELAY_MS 120000
#define RECOVERY_DELAY_MS 15000
#define MQTT_SHUTDOWN_DELAY_MS 30000
#define MESSAGE_DISPLAY_TIME 2000
#define SCREEN_SWITCH_INTERVAL 1000
#define SAMPLES_COUNT 20
#define MQTT_HEARTBEAT_INTERVAL 300000

// Instancje
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// === Zmienne globalne chronione przez Mutex ===
SemaphoreHandle_t dataMutex;
float avgBatteryVoltage = 0.0;
float avgOutputVoltage = 0.0;
bool isPowerLost = false;
bool isUndervoltage = false;
bool isOvervoltage = false;
bool isMqttShutdownTriggered = false;
uint32_t undervoltageStartTime = 0;
bool undervoltageShutdownActive = false;
uint32_t undervoltagePostShutdownStartTime = 0;
uint32_t undervoltageRecoveryStartTime = 0;
uint32_t overvoltageStartTime = 0;
bool overvoltageShutdownActive = false;
uint32_t overvoltagePostShutdownStartTime = 0;
uint32_t overvoltageRecoveryStartTime = 0;
// ===============================================

enum DisplayState { NORMAL, SHOWING_MESSAGE };

// Prototypy
void sensorsTask(void *pvParameters);
void displayTask(void *pvParameters);
void logicAndLedsTask(void *pvParameters);
void mqttTask(void *pvParameters);
void drawMainScreen(float batV, float outV, bool powerLost, bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, uint32_t uvStartTime, uint32_t ovStartTime, uint32_t uvPostShutdownTime, uint32_t ovPostShutdownTime);
void handleLeds(float voltage);

void setup() {
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(POWER_LOSS_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT); pinMode(LED_GREEN_PIN, OUTPUT); pinMode(LED_BLUE_PIN, OUTPUT);
  u8g2.begin();
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) { return; }
  xTaskCreate(sensorsTask, "Sensors", 2048, NULL, 2, NULL);
  xTaskCreate(displayTask, "Display", 4096, NULL, 1, NULL);
  xTaskCreate(logicAndLedsTask, "Logic/LEDs", 2048, NULL, 3, NULL);
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
}

void loop() { vTaskDelete(NULL); }

// ### ZADANIA RTOS ###

void sensorsTask(void *pvParameters) {
  float batterySamples[SAMPLES_COUNT] = {0}; float outputSamples[SAMPLES_COUNT] = {0}; int sampleIndex = 0;
  for (;;) {
    float rawBatteryVoltage = (analogRead(BATTERY_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / BATTERY_VOLTAGE_RATIO; batterySamples[sampleIndex] = rawBatteryVoltage;
    float rawOutputVoltage = (analogRead(OUTPUT_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / OUTPUT_VOLTAGE_RATIO; outputSamples[sampleIndex] = rawOutputVoltage;
    sampleIndex = (sampleIndex + 1) % SAMPLES_COUNT;
    float batterySum = 0, outputSum = 0;
    for (int i = 0; i < SAMPLES_COUNT; i++) { batterySum += batterySamples[i]; outputSum += outputSamples[i]; }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { avgBatteryVoltage = batterySum / SAMPLES_COUNT; avgOutputVoltage = outputSum / SAMPLES_COUNT; xSemaphoreGive(dataMutex); }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void logicAndLedsTask(void *pvParameters) {
  for (;;) {
    float localBatV; if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { localBatV = avgBatteryVoltage; xSemaphoreGive(dataMutex); }
    bool currentPowerState = (digitalRead(POWER_LOSS_PIN) == LOW);
    bool undervoltage = (localBatV < VOLTAGE_UNDER);
    bool overvoltage = (localBatV > VOLTAGE_OVER_SHUTDOWN);
    uint32_t uv_startTime, uv_postShutdownTime, uv_recoveryTime; bool uv_shutdown;
    uint32_t ov_startTime, ov_postShutdownTime, ov_recoveryTime; bool ov_shutdown;
    bool mqtt_shutdown_flag;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      uv_startTime = undervoltageStartTime; uv_shutdown = undervoltageShutdownActive; uv_postShutdownTime = undervoltagePostShutdownStartTime; uv_recoveryTime = undervoltageRecoveryStartTime;
      ov_startTime = overvoltageStartTime; ov_shutdown = overvoltageShutdownActive; ov_postShutdownTime = overvoltagePostShutdownStartTime; ov_recoveryTime = overvoltageRecoveryStartTime;
      mqtt_shutdown_flag = isMqttShutdownTriggered;
      xSemaphoreGive(dataMutex);
    }
    if (overvoltage) {
      if (ov_startTime == 0) { ov_startTime = millis(); }
      ov_recoveryTime = 0;
      if (!ov_shutdown && (millis() - ov_startTime > SHUTDOWN_DELAY_MS)) { ov_shutdown = true; ov_postShutdownTime = millis(); }
    } else {
      ov_startTime = 0;
      if (ov_shutdown) { if (localBatV < VOLTAGE_OVER_RECOVERY) { if (ov_recoveryTime == 0) { ov_recoveryTime = millis(); } if (millis() - ov_recoveryTime > RECOVERY_DELAY_MS) { ov_shutdown = false; ov_postShutdownTime = 0; ov_recoveryTime = 0; } } else { ov_recoveryTime = 0; } }
    }
    if (undervoltage) {
      if (uv_startTime == 0) { uv_startTime = millis(); }
      uv_recoveryTime = 0;
      if (!uv_shutdown && (millis() - uv_startTime > SHUTDOWN_DELAY_MS)) { uv_shutdown = true; uv_postShutdownTime = millis(); }
    } else {
      uv_startTime = 0;
      if (uv_shutdown) { if (localBatV > VOLTAGE_RECOVERY) { if (uv_recoveryTime == 0) { uv_recoveryTime = millis(); } if (millis() - uv_recoveryTime > RECOVERY_DELAY_MS) { uv_shutdown = false; uv_postShutdownTime = 0; uv_recoveryTime = 0; } } else { uv_recoveryTime = 0; } }
    }
    if ((uv_startTime != 0 && millis() - uv_startTime > MQTT_SHUTDOWN_DELAY_MS) || (ov_startTime != 0 && millis() - ov_startTime > MQTT_SHUTDOWN_DELAY_MS)) { mqtt_shutdown_flag = true; } else { mqtt_shutdown_flag = false; }
    digitalWrite(RELAY_PIN, (uv_shutdown || ov_shutdown) ? HIGH : LOW);
    handleLeds(localBatV);
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      isPowerLost = currentPowerState; isUndervoltage = undervoltage; isOvervoltage = overvoltage; isMqttShutdownTriggered = mqtt_shutdown_flag;
      undervoltageStartTime = uv_startTime; undervoltageShutdownActive = uv_shutdown; undervoltagePostShutdownStartTime = uv_postShutdownTime; undervoltageRecoveryStartTime = uv_recoveryTime;
      overvoltageStartTime = ov_startTime; overvoltageShutdownActive = ov_shutdown; overvoltagePostShutdownStartTime = ov_postShutdownTime; overvoltageRecoveryStartTime = ov_recoveryTime;
      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void displayTask(void *pvParameters) {
  DisplayState currentDisplayState = NORMAL; unsigned long messageStartTime = 0; char messageLine1[20] = ""; char messageLine2[20] = ""; bool lastPowerState = false;
  for (;;) {
    float localBatV, localOutV; bool localPowerLost, localUndervoltage, localOvervoltage, localUvShutdown, localOvShutdown;
    uint32_t localUvStartTime, localUvPostShutdownTime, localOvStartTime, localOvPostShutdownTime;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      localBatV = avgBatteryVoltage; localOutV = avgOutputVoltage; localPowerLost = isPowerLost;
      localUndervoltage = isUndervoltage; localUvShutdown = undervoltageShutdownActive; localUvStartTime = undervoltageStartTime; localUvPostShutdownTime = undervoltagePostShutdownStartTime;
      localOvervoltage = isOvervoltage; localOvShutdown = overvoltageShutdownActive; localOvStartTime = overvoltageStartTime; localOvPostShutdownTime = overvoltagePostShutdownStartTime;
      xSemaphoreGive(dataMutex);
    }
    if (localPowerLost && !lastPowerState) { strcpy(messageLine1, "POWER LOST"); strcpy(messageLine2, "RUNNING ON BAT"); messageStartTime = millis(); currentDisplayState = SHOWING_MESSAGE; }
    else if (!localPowerLost && lastPowerState) { strcpy(messageLine1, "POWER RESTORED"); strcpy(messageLine2, ""); messageStartTime = millis(); currentDisplayState = SHOWING_MESSAGE; }
    lastPowerState = localPowerLost;
    u8g2.clearBuffer();
    switch (currentDisplayState) {
      case SHOWING_MESSAGE: u8g2.setFont(u8g2_font_6x12_tf); u8g2.drawStr((128-strlen(messageLine1)*6)/2, 12, messageLine1); u8g2.drawStr((128-strlen(messageLine2)*6)/2, 28, messageLine2); if (millis() - messageStartTime >= MESSAGE_DISPLAY_TIME) { currentDisplayState = NORMAL; } break;
      case NORMAL: drawMainScreen(localBatV, localOutV, localPowerLost, localUndervoltage, localOvervoltage, localUvShutdown, localOvShutdown, localUvStartTime, localOvStartTime, localUvPostShutdownTime, localOvPostShutdownTime); break;
    }
    u8g2.sendBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void mqttTask(void *pvParameters) {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  char lastPublishedSystemState[32] = ""; unsigned long lastHeartbeatTime = 0;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
    if (!mqttClient.connected()) { if (mqttClient.connect("esp32-ups", MQTT_USER, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 1, true, "offline")) { mqttClient.publish(MQTT_AVAILABILITY_TOPIC, "online", true); } else { vTaskDelay(pdMS_TO_TICKS(5000)); continue; } }
    mqttClient.loop();
    float batV, outV; bool pwrLost, undervoltage, overvoltage, uvShutdown, ovShutdown, mqttShutdown; char currentSystemState[32];
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      batV = avgBatteryVoltage; outV = avgOutputVoltage; pwrLost = isPowerLost;
      undervoltage = isUndervoltage; overvoltage = isOvervoltage;
      uvShutdown = undervoltageShutdownActive; ovShutdown = overvoltageShutdownActive;
      mqttShutdown = isMqttShutdownTriggered;
      xSemaphoreGive(dataMutex);
    }
    if (uvShutdown) strcpy(currentSystemState, "UNDERVOLTAGE_SHUTDOWN");
    else if (undervoltage && mqttShutdown) strcpy(currentSystemState, "UNDERVOLTAGE_SHUTDOWN_NOW");
    else if (undervoltage) strcpy(currentSystemState, "UNDERVOLTAGE_WARNING");
    else if (ovShutdown) strcpy(currentSystemState, "OVERVOLTAGE_SHUTDOWN");
    else if (overvoltage && mqttShutdown) strcpy(currentSystemState, "OVERVOLTAGE_SHUTDOWN_NOW");
    else if (overvoltage) strcpy(currentSystemState, "OVERVOLTAGE_WARNING");
    else if (pwrLost) strcpy(currentSystemState, "ON_BATTERY");
    else strcpy(currentSystemState, "ONLINE");
    bool stateChanged = strcmp(currentSystemState, lastPublishedSystemState) != 0; bool heartbeatDue = millis() - lastHeartbeatTime > MQTT_HEARTBEAT_INTERVAL;
    if (stateChanged || heartbeatDue) {
      char jsonPayload[200];
      snprintf(jsonPayload, sizeof(jsonPayload), "{\"battery_v\":%.2f, \"output_v\":%.2f, \"state\":\"%s\"}", batV, outV, currentSystemState);
      mqttClient.publish(MQTT_STATE_TOPIC, jsonPayload);
      strcpy(lastPublishedSystemState, currentSystemState);
      lastHeartbeatTime = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void drawMainScreen(float batV, float outV, bool powerLost, bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, uint32_t uvStartTime, uint32_t ovStartTime, uint32_t uvPostShutdownTime, uint32_t ovPostShutdownTime) {
  // Lewa połowa: BAT i OUT
  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[8];
  int batWhole = floor(batV);
  int batDecimal = round((batV - batWhole) * 10);
  if (batDecimal == 10) {
    batWhole += 1;
    batDecimal = 0;
  }
  sprintf(buf, "%d", batWhole);
  u8g2.drawStr(0, 14, "BAT");
  u8g2.setFont(u8g2_font_VCR_OSD_mn);
  int wholeWidth = u8g2.getStrWidth(buf);
  sprintf(buf + strlen(buf), "%d", batDecimal);
  int decimalWidth = u8g2.getStrWidth(buf + strlen(buf) - 1); // szerokość jednej cyfry
  int totalWidth = wholeWidth + 3 + decimalWidth;
  int x = 56 - totalWidth;
  sprintf(buf, "%d", batWhole);
  u8g2.drawStr(x, 15, buf);
  sprintf(buf, "%d", batDecimal);
  u8g2.drawStr(x + wholeWidth + 2, 15, buf);
  u8g2.drawBox(41, 12, 2, 2);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(57, 14, "V");
  int outWhole = floor(outV);
  int outDecimal = round((outV - outWhole) * 10);
  if (outDecimal == 10) {
    outWhole += 1;
    outDecimal = 0;
  }
  sprintf(buf, "%d", outWhole);
  u8g2.drawStr(0, 32, "OUT");
  u8g2.setFont(u8g2_font_VCR_OSD_mn);
  wholeWidth = u8g2.getStrWidth(buf);
  sprintf(buf + strlen(buf), "%d", outDecimal);
  decimalWidth = u8g2.getStrWidth(buf + strlen(buf) - 1);
  totalWidth = wholeWidth + 3 + decimalWidth;
  x = 56 - totalWidth;
  sprintf(buf, "%d", outWhole);
  u8g2.drawStr(x, 33, buf);
  sprintf(buf, "%d", outDecimal);
  u8g2.drawStr(x + wholeWidth + 2, 33, buf);
  u8g2.drawBox(41, 30, 2, 2);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(57, 32, "V");

  // Prawa połowa: timery na górze, komunikaty na dole
  u8g2.setFont(u8g2_font_VCR_OSD_mn);
  char timerMsg[16] = "";
  char statusMsg[16] = "";
  if (overvoltage) {
    if (ovShutdown) {
      int s = (millis() - ovPostShutdownTime) / 1000;
      sprintf(timerMsg, "DOWN %d:%02d", s/60, s%60);
    } else {
      int rem_s = (SHUTDOWN_DELAY_MS - (millis() - ovStartTime)) / 1000;
      if (rem_s < 0) rem_s = 0;
      sprintf(timerMsg, "SHUT %d:%02d", rem_s/60, rem_s%60);
    }
    strcpy(statusMsg, "OVER");
  } else if (undervoltage) {
    if (uvShutdown) {
      int s = (millis() - uvPostShutdownTime) / 1000;
      sprintf(timerMsg, "DOWN %d:%02d", s/60, s%60);
    } else {
      int rem_s = (SHUTDOWN_DELAY_MS - (millis() - uvStartTime)) / 1000;
      if (rem_s < 0) rem_s = 0;
      sprintf(timerMsg, "SHUT %d:%02d", rem_s/60, rem_s%60);
    }
    strcpy(statusMsg, "UNDER");
  } else if (powerLost) {
    strcpy(statusMsg, "ON BAT");
  } else {
    strcpy(statusMsg, "SYS OK");
  }
  int timerWidth = u8g2.getStrWidth(timerMsg);
  u8g2.drawStr(127 - timerWidth, 15, timerMsg);
  int statusWidth = u8g2.getStrWidth(statusMsg);
  u8g2.drawStr(127 - statusWidth, 15, statusMsg);
  u8g2.drawFrame(67, 0, 61, 32);
  u8g2.drawHLine(0, 15, 128);
}

void handleLeds(float voltage) {
    const int maxBrightness = 255; int redValue = 0, greenValue = 0, blueValue = 0; const float VOLTAGE_MID = 12.0;
    if (voltage < VOLTAGE_UNDER) { unsigned long currentTime = millis(); float angle = (currentTime % 4000) / 4000.0 * 2 * PI; redValue = (int)(((sin(angle) + 1.0) / 2.0) * (maxBrightness / 2));
    } else if (voltage >= VOLTAGE_UNDER && voltage < VOLTAGE_MID) { float ratio = constrain((voltage - VOLTAGE_UNDER) / (VOLTAGE_MID - VOLTAGE_UNDER), 0.0, 1.0); redValue = (int)((1.0 - ratio) * maxBrightness); greenValue = (int)(ratio * maxBrightness);
    } else if (voltage >= VOLTAGE_MID && voltage <= VOLTAGE_OVER) { float ratio = constrain((voltage - VOLTAGE_MID) / (VOLTAGE_OVER - VOLTAGE_MID), 0.0, 1.0); redValue = (int)(ratio * maxBrightness); greenValue = (int)((1.0 - ratio) * maxBrightness); blueValue = (int)(ratio * maxBrightness);
    } else { redValue = maxBrightness; blueValue = maxBrightness; }
    analogWrite(LED_RED_PIN, redValue); analogWrite(LED_GREEN_PIN, greenValue); analogWrite(LED_BLUE_PIN, blueValue);
}