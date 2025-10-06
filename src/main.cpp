// Version
const char* Version = "1.6.0";

#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ### KONFIGURACJA SIECI I MQTT ###
const char* WIFI_SSID = "Blacksmith";
const char* WIFI_PASSWORD = "Malinka1973";
const char* MQTT_SERVER = "192.168.1.100";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt_user";
const char* MQTT_PASSWORD = "Reguly66";
const char* MQTT_STATE_TOPIC = "ups/state";
const char* MQTT_AVAILABILITY_TOPIC = "ups/status";

// ### DEFINICJE PIN√ìW I STA≈ÅYCH ###
#define OLED_SDA_PIN 8
#define OLED_SCL_PIN 9
#define RELAY_PIN 7
#define POWER_LOSS_PIN 6
#define LED_RED_PIN 2
#define LED_GREEN_PIN 3
#define LED_BLUE_PIN 4
#define BATTERY_VOLTAGE_PIN 0
#define OUTPUT_VOLTAGE_PIN 1

// ### STA≈ÅE KALIBRACYJNE I PROGOWE ###
#define BATTERY_VOLTAGE_RATIO 0.2117
#define OUTPUT_VOLTAGE_RATIO 0.2132
#define ADC_REF_VOLTAGE 3.3
#define ADC_MAX_VALUE 4095.0

#define VOLTAGE_RECOVERY 11.5
#define VOLTAGE_UNDER 11.0
#define VOLTAGE_OVER_RECOVERY 12.5
#define VOLTAGE_OVER 13.0

// ### STA≈ÅE CZASOWE ###
#define SHUTDOWN_DELAY_MS 30000
#define RECOVERY_DELAY_MS 30000
#define MESSAGE_DISPLAY_TIME 2000
#define SAMPLES_COUNT 50
#define MQTT_HEARTBEAT_INTERVAL 300000

// ### INSTANCJE ###
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
uint32_t systemStartTime = 0;

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
void drawMainScreen(float batV, float outV, bool powerLost, bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, uint32_t uvStartTime, uint32_t ovStartTime, uint32_t uvPostShutdownTime, uint32_t ovPostShutdownTime, uint32_t uvRecoveryStartTime, uint32_t ovRecoveryStartTime);
void handleLeds(bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, float voltage);

void setup() {
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(POWER_LOSS_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT); pinMode(LED_GREEN_PIN, OUTPUT); pinMode(LED_BLUE_PIN, OUTPUT);

  // Poprawka: Inicjalizacja PWM (LEDC) dla diod RGB
  const int ledc_freq = 5000;
  const int ledc_resolution = 8;
  ledcSetup(0, ledc_freq, ledc_resolution); ledcAttachPin(LED_RED_PIN, 0);
  ledcSetup(1, ledc_freq, ledc_resolution); ledcAttachPin(LED_GREEN_PIN, 1);
  ledcSetup(2, ledc_freq, ledc_resolution); ledcAttachPin(LED_BLUE_PIN, 2);

  u8g2.begin();
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) { return; }
  systemStartTime = millis();

  xTaskCreate(sensorsTask, "Sensors", 2048, NULL, 2, NULL);
  xTaskCreate(displayTask, "Display", 4096, NULL, 1, NULL);
  xTaskCreate(logicAndLedsTask, "Logic/LEDs", 2048, NULL, 3, NULL);
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
}

void loop() { 
  vTaskDelete(NULL); 
}

// ### ZADANIA RTOS ###

void sensorsTask(void *pvParameters) {
  float batterySamples[SAMPLES_COUNT] = {0}; 
  float outputSamples[SAMPLES_COUNT] = {0}; 
  int sampleIndex = 0;
  
  // Initialize with first readings
  for (int i = 0; i < SAMPLES_COUNT; i++) {
    float rawBatteryVoltage = (analogRead(BATTERY_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / BATTERY_VOLTAGE_RATIO;
    float rawOutputVoltage = (analogRead(OUTPUT_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / OUTPUT_VOLTAGE_RATIO;
    batterySamples[i] = rawBatteryVoltage;
    outputSamples[i] = rawOutputVoltage;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  for (;;) {
    // Read new values
    float rawBatteryVoltage = (analogRead(BATTERY_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / BATTERY_VOLTAGE_RATIO;
    float rawOutputVoltage = (analogRead(OUTPUT_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / OUTPUT_VOLTAGE_RATIO;
    
    // Apply exponential moving average filter
    batterySamples[sampleIndex] = rawBatteryVoltage;
    outputSamples[sampleIndex] = rawOutputVoltage;
    sampleIndex = (sampleIndex + 1) % SAMPLES_COUNT;
    
    // Calculate median instead of mean for better noise rejection
    float batteryValues[SAMPLES_COUNT];
    float outputValues[SAMPLES_COUNT];
    memcpy(batteryValues, batterySamples, sizeof(batterySamples));
    memcpy(outputValues, outputSamples, sizeof(outputSamples));
    
    // Sort arrays to find median
    for (int i = 0; i < SAMPLES_COUNT-1; i++) {
      for (int j = i+1; j < SAMPLES_COUNT; j++) {
        if (batteryValues[i] > batteryValues[j]) {
          float temp = batteryValues[i];
          batteryValues[i] = batteryValues[j];
          batteryValues[j] = temp;
        }
        if (outputValues[i] > outputValues[j]) {
          float temp = outputValues[i];
          outputValues[i] = outputValues[j];
          outputValues[j] = temp;
        }
      }
    }
    
    // Use median value (middle of sorted array)
    float batteryMedian = batteryValues[SAMPLES_COUNT/2];
    float outputMedian = outputValues[SAMPLES_COUNT/2];
    
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { 
      // Apply additional low-pass filter
      avgBatteryVoltage = 0.9 * avgBatteryVoltage + 0.1 * batteryMedian;
      avgOutputVoltage = 0.9 * avgOutputVoltage + 0.1 * outputMedian;
      xSemaphoreGive(dataMutex); 
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void logicAndLedsTask(void *pvParameters) {
  // Add static variables for hysteresis
  static bool wasOvervoltage = false;
  static bool wasUndervoltage = false;
  
  for (;;) {
    // Poprawka: Ca≥a logika stanu wewnπtrz jednej sekcji krytycznej (Mutex)
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      float localOutV = avgOutputVoltage;
      isPowerLost = (digitalRead(POWER_LOSS_PIN) == LOW);
      
      // Apply hysteresis for overvoltage
      if (wasOvervoltage) {
        // If already in overvoltage state, use recovery threshold to exit
        isOvervoltage = (localOutV > VOLTAGE_OVER_RECOVERY);
      } else {
        // If not in overvoltage state, use higher threshold to enter
        isOvervoltage = (localOutV > VOLTAGE_OVER);
      }
      
      // Apply hysteresis for undervoltage
      if (wasUndervoltage) {
        // If already in undervoltage state, use recovery threshold to exit
        isUndervoltage = (localOutV < VOLTAGE_RECOVERY);
      } else {
        // If not in undervoltage state, use lower threshold to enter
        isUndervoltage = (localOutV < VOLTAGE_UNDER);
      }
      
      // Save current states for next iteration
      wasOvervoltage = isOvervoltage;
      wasUndervoltage = isUndervoltage;

      // Logika Overvoltage
      if (isOvervoltage) {
        if (overvoltageStartTime == 0) { overvoltageStartTime = millis(); }
        overvoltageRecoveryStartTime = 0;
        if (!overvoltageShutdownActive && (millis() - overvoltageStartTime > SHUTDOWN_DELAY_MS)) {
          overvoltageShutdownActive = true;
          overvoltagePostShutdownStartTime = millis();
        }
      } else {
        overvoltageStartTime = 0;
        if (overvoltageShutdownActive) {
          if (localOutV < VOLTAGE_OVER_RECOVERY) {
            if (overvoltageRecoveryStartTime == 0) { overvoltageRecoveryStartTime = millis(); }
            if (millis() - overvoltageRecoveryStartTime > RECOVERY_DELAY_MS) {
              overvoltageShutdownActive = false;
              overvoltagePostShutdownStartTime = 0;
              overvoltageRecoveryStartTime = 0;
            }
          } else {
            overvoltageRecoveryStartTime = 0;
          }
        }
      }

      // Logika Undervoltage
      if (isUndervoltage) {
        if (undervoltageStartTime == 0) { undervoltageStartTime = millis(); }
        undervoltageRecoveryStartTime = 0;
        if (!undervoltageShutdownActive && (millis() - undervoltageStartTime > SHUTDOWN_DELAY_MS)) {
          undervoltageShutdownActive = true;
          undervoltagePostShutdownStartTime = millis();
        }
      } else {
        undervoltageStartTime = 0;
        if (undervoltageShutdownActive) {
          if (localOutV > VOLTAGE_RECOVERY) {
            if (undervoltageRecoveryStartTime == 0) { undervoltageRecoveryStartTime = millis(); }
            if (millis() - undervoltageRecoveryStartTime > RECOVERY_DELAY_MS) {
              undervoltageShutdownActive = false;
              undervoltagePostShutdownStartTime = 0;
              undervoltageRecoveryStartTime = 0;
            }
          } else {
            undervoltageRecoveryStartTime = 0;
          }
        }
      }

      digitalWrite(RELAY_PIN, (undervoltageShutdownActive || overvoltageShutdownActive) ? HIGH : LOW);
      handleLeds(isUndervoltage, isOvervoltage, undervoltageShutdownActive, overvoltageShutdownActive, localOutV);

      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void displayTask(void *pvParameters) {
  DisplayState currentDisplayState = NORMAL; unsigned long messageStartTime = 0; char messageLine1[20] = ""; char messageLine2[20] = ""; bool lastPowerState = false;
  for (;;) {
    float localBatV, localOutV; 
    bool localPowerLost, localUndervoltage, localOvervoltage, localUvShutdown, localOvShutdown;
    uint32_t localUvStartTime, localUvPostShutdownTime, localOvStartTime, localOvPostShutdownTime, localUvRecoveryStartTime, localOvRecoveryStartTime;
    
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      localBatV = avgBatteryVoltage; localOutV = avgOutputVoltage; localPowerLost = isPowerLost;
      localUndervoltage = isUndervoltage; localUvShutdown = undervoltageShutdownActive; localUvStartTime = undervoltageStartTime; localUvPostShutdownTime = undervoltagePostShutdownStartTime; localUvRecoveryStartTime = undervoltageRecoveryStartTime;
      localOvervoltage = isOvervoltage; localOvShutdown = overvoltageShutdownActive; localOvStartTime = overvoltageStartTime; localOvPostShutdownTime = overvoltagePostShutdownStartTime; localOvRecoveryStartTime = overvoltageRecoveryStartTime;
      xSemaphoreGive(dataMutex);
    }
    
    if (localPowerLost && !lastPowerState) { strcpy(messageLine1, "POWER LOST"); strcpy(messageLine2, "RUNNING ON BAT"); messageStartTime = millis(); currentDisplayState = SHOWING_MESSAGE; }
    else if (!localPowerLost && lastPowerState) { strcpy(messageLine1, "POWER RESTORED"); strcpy(messageLine2, ""); messageStartTime = millis(); currentDisplayState = SHOWING_MESSAGE; }
    lastPowerState = localPowerLost;
    
    u8g2.clearBuffer();
    switch (currentDisplayState) {
      case SHOWING_MESSAGE: u8g2.setFont(u8g2_font_6x12_tf); u8g2.drawStr((128-strlen(messageLine1)*6)/2, 12, messageLine1); u8g2.drawStr((128-strlen(messageLine2)*6)/2, 28, messageLine2); if (millis() - messageStartTime >= MESSAGE_DISPLAY_TIME) { currentDisplayState = NORMAL; } break;
      case NORMAL: drawMainScreen(localBatV, localOutV, localPowerLost, localUndervoltage, localOvervoltage, localUvShutdown, localOvShutdown, localUvStartTime, localOvStartTime, localUvPostShutdownTime, localOvPostShutdownTime, localUvRecoveryStartTime, localOvRecoveryStartTime); break;
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
    
    float batV, outV; bool pwrLost, undervoltage, overvoltage, uvShutdown, ovShutdown; char currentSystemState[32];
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      batV = avgBatteryVoltage; outV = avgOutputVoltage; pwrLost = isPowerLost;
      undervoltage = isUndervoltage; overvoltage = isOvervoltage;
      uvShutdown = undervoltageShutdownActive; ovShutdown = overvoltageShutdownActive;
      xSemaphoreGive(dataMutex);
    }
    
    if (uvShutdown) strcpy(currentSystemState, "UNDERVOLTAGE_SHUTDOWN");
    else if (ovShutdown) strcpy(currentSystemState, "OVERVOLTAGE_SHUTDOWN");
    else if (undervoltage) strcpy(currentSystemState, "UNDERVOLTAGE_WARNING");
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

void drawMainScreen(float batV, float outV, bool powerLost, bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, uint32_t uvStartTime, uint32_t ovStartTime, uint32_t uvPostShutdownTime, uint32_t ovPostShutdownTime, uint32_t uvRecoveryStartTime, uint32_t ovRecoveryStartTime) {
  // Wy≈õwietlanie napiƒôƒá - oryginalna logika bez zmian
  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[8];
  int batWhole = floor(batV);
  int batDecimal = round((batV - batWhole) * 10);
  if (batDecimal == 10) { batWhole += 1; batDecimal = 0; }
  u8g2.drawStr(0, 14, "BAT");
  u8g2.setFont(u8g2_font_VCR_OSD_mn);
  sprintf(buf, "%d", batWhole);
  int wholeWidth = u8g2.getStrWidth(buf);
  sprintf(buf + strlen(buf), ".%d", batDecimal);
  int totalWidth = u8g2.getStrWidth(buf);
  int x = 56 - totalWidth;
  sprintf(buf, "%d", batWhole);
  u8g2.drawStr(x, 15, buf);
  u8g2.drawStr(x + wholeWidth, 15, ".");
  sprintf(buf, "%d", batDecimal);
  u8g2.drawStr(x + wholeWidth + 3, 15, buf);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(57, 14, "V");

  int outWhole = floor(outV);
  int outDecimal = round((outV - outWhole) * 10);
  if (outDecimal == 10) { outWhole += 1; outDecimal = 0; }
  u8g2.drawStr(0, 32, "OUT");
  u8g2.setFont(u8g2_font_VCR_OSD_mn);
  sprintf(buf, "%d", outWhole);
  wholeWidth = u8g2.getStrWidth(buf);
  sprintf(buf + strlen(buf), ".%d", outDecimal);
  totalWidth = u8g2.getStrWidth(buf);
  x = 56 - totalWidth;
  sprintf(buf, "%d", outWhole);
  u8g2.drawStr(x, 33, buf);
  u8g2.drawStr(x + wholeWidth, 33, ".");
  sprintf(buf, "%d", outDecimal);
  u8g2.drawStr(x + wholeWidth + 3, 33, buf);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(57, 32, "V");
  
  // Prawa strona ekranu
  u8g2.setFont(u8g2_font_VCR_OSD_mn);
  char statusMsg[16] = "";
  int timerMinutes = 0, timerSeconds = 0;
  bool hasTimer = false;
  
  // Improved timer display logic
  // For recovery states
  if (uvRecoveryStartTime != 0 || ovRecoveryStartTime != 0) {
      strcpy(statusMsg, "RECOVERY");
      uint32_t startTime = (uvRecoveryStartTime != 0) ? uvRecoveryStartTime : ovRecoveryStartTime;
      int elapsed_ms = millis() - startTime;
      int remaining_ms = RECOVERY_DELAY_MS - elapsed_ms;
      if (remaining_ms < 0) remaining_ms = 0;
      timerSeconds = remaining_ms / 1000;
      timerMinutes = timerSeconds / 60;
      timerSeconds %= 60;
      hasTimer = true;
  } 
  // For overvoltage states
  else if (overvoltage || ovShutdown) {
    if (ovShutdown) { 
      strcpy(statusMsg, "PWR OFF"); 
      // Calculate elapsed time since shutdown
      int elapsed_ms = millis() - ovPostShutdownTime;
      timerSeconds = elapsed_ms / 1000;
      timerMinutes = timerSeconds / 60;
      timerSeconds %= 60;
    } 
    else { 
      strcpy(statusMsg, "HIGH V"); 
      // Calculate countdown to shutdown
      int elapsed_ms = millis() - ovStartTime;
      int remaining_ms = SHUTDOWN_DELAY_MS - elapsed_ms;
      if (remaining_ms < 0) remaining_ms = 0;
      timerSeconds = remaining_ms / 1000;
      timerMinutes = timerSeconds / 60;
      timerSeconds %= 60;
    }
    hasTimer = true;
  } 
  // For undervoltage states
  else if (undervoltage || uvShutdown) {
    if (uvShutdown) { 
      strcpy(statusMsg, "PWR OFF"); 
      // Calculate elapsed time since shutdown
      int elapsed_ms = millis() - uvPostShutdownTime;
      timerSeconds = elapsed_ms / 1000;
      timerMinutes = timerSeconds / 60;
      timerSeconds %= 60;
    } 
    else { 
      strcpy(statusMsg, "LOW V"); 
      // Calculate countdown to shutdown
      int elapsed_ms = millis() - uvStartTime;
      int remaining_ms = SHUTDOWN_DELAY_MS - elapsed_ms;
      if (remaining_ms < 0) remaining_ms = 0;
      timerSeconds = remaining_ms / 1000;
      timerMinutes = timerSeconds / 60;
      timerSeconds %= 60;
    }
    hasTimer = true;
  } 
  // Normal operation states
  else if (powerLost) {
    strcpy(statusMsg, "ON BATT");
  } else {
    strcpy(statusMsg, "SYS OK");
  }
  
  if (hasTimer) {
    char minStr[4], secStr[4];
    sprintf(minStr, "%d", timerMinutes); sprintf(secStr, "%02d", timerSeconds);
    int minWidth = u8g2.getStrWidth(minStr); int totalWidth = minWidth + 4 + u8g2.getStrWidth(secStr); int x_timer = 128 - totalWidth;
    u8g2.drawStr(x_timer, 15, minStr); int colonX = x_timer + minWidth + 1;
    u8g2.drawBox(colonX, 4, 2, 2); u8g2.drawBox(colonX, 9, 2, 2);
    u8g2.drawStr(x_timer + minWidth + 4, 15, secStr);
  } else {
    uint32_t uptimeMs = millis() - systemStartTime; int uptimeSeconds = uptimeMs / 1000; int uptimeMinutes = uptimeSeconds / 60; int uptimeSecs = uptimeSeconds % 60;
    char minStr[4], secStr[4];
    sprintf(minStr, "%d", uptimeMinutes); sprintf(secStr, "%02d", uptimeSecs);
    int minWidth = u8g2.getStrWidth(minStr); int totalWidth = minWidth + 4 + u8g2.getStrWidth(secStr); int x_timer = 128 - totalWidth;
    u8g2.drawStr(x_timer, 15, minStr); int colonX = x_timer + minWidth + 1;
    u8g2.drawBox(colonX, 4, 2, 2); u8g2.drawBox(colonX, 9, 2, 2);
    u8g2.drawStr(x_timer + minWidth + 4, 15, secStr);
  }
  
  // Poprawka: Wyr√≥wnanie statusu do prawej krawƒôdzi
  u8g2.setFont(u8g2_font_8x13_tf);
  int statusWidth = u8g2.getStrWidth(statusMsg);
  u8g2.drawStr(128 - statusWidth, 32, statusMsg);
  
  for(int y=0; y<32; y+=4) {
    u8g2.drawPixel(65, y);
    u8g2.drawPixel(65, y+1);
  }
}

void handleLeds(bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, float voltage) {
  // Logika dla diody ze wspÛlnπ katodπ: 255 = max jasnoúÊ, 0 = wy≥πczona
  const int maxBrightness = 255;
  int redValue = 0, greenValue = 0, blueValue = 0;
  
  if (overvoltage || ovShutdown) {
      // Make blinking more pronounced by using full on/off cycle
      unsigned long currentTime = millis();
      if (currentTime % 200 < 100) {  // Increased period for more visible blinking
          redValue = maxBrightness;
          greenValue = 0;
          blueValue = 0;
      } else {
          redValue = 0;
          greenValue = 0;
          blueValue = 0;
      }
  } else if (undervoltage || uvShutdown) {
      unsigned long currentTime = millis();
      float phase = (currentTime % 2000) / 2000.0f; 
      if (phase < 0.3f) {
        redValue = (int)((phase / 0.3f) * maxBrightness * 0.7);
      } else {
        redValue = (int)(((1.0f - phase) / 0.7f) * maxBrightness * 0.7);
      }
  } else {
      // Nowa funkcja: P≥ynna zmiana koloru w zaleønoúci od napiÍcia
      float v = constrain(voltage, 11.0, 13.0);
      if (v <= 12.0) {
        // Przejúcie z pomaraÒczowego (11V) do zielonego (12V)
        redValue = map(v * 100, 1100, 1200, maxBrightness, 0);
        greenValue = maxBrightness;
        blueValue = 0;
      } else {
        // Przejúcie z zielonego (12V) do fioletowego (13V)
        redValue = map(v * 100, 1200, 1300, 0, maxBrightness);
        greenValue = map(v * 100, 1200, 1300, maxBrightness, 0);
        blueValue = map(v * 100, 1200, 1300, 0, maxBrightness);
      }
  }
  
  ledcWrite(0, redValue);
  ledcWrite(1, greenValue);
  ledcWrite(2, blueValue);
}

// Version
// Version
// const char* Version = "1.0.0";

// #include <Wire.h>
// #include <U8g2lib.h>
// #include <WiFi.h>
// #include <PubSubClient.h>

// // ### KONFIGURACJA SIECI I MQTT - UZUPE≈ÅNIJ! ###
// const char* WIFI_SSID = "Blacksmith";
// const char* WIFI_PASSWORD = "Malinka1973";

// const char* MQTT_SERVER = "192.168.1.100"; // Adres IP Twojego serwera Home Assistant
// const int MQTT_PORT = 1883;
// const char* MQTT_USER = "mqtt_user";       // U≈ºytkownik MQTT (je≈õli masz)
// const char* MQTT_PASSWORD = "Reguly66"; // Has≈Ço MQTT (je≈õli masz)

// const char* MQTT_STATE_TOPIC = "ups/state";
// const char* MQTT_AVAILABILITY_TOPIC = "ups/status";
// // #################################################


// // Definicje pin√≥w i sta≈Çych
// #define OLED_SDA_PIN 8
// #define OLED_SCL_PIN 9
// #define RELAY_PIN 7
// #define POWER_LOSS_PIN 6
// #define LED_RED_PIN 2
// #define LED_GREEN_PIN 3
// #define LED_BLUE_PIN 4
// #define BATTERY_VOLTAGE_PIN 0
// #define OUTPUT_VOLTAGE_PIN 1

// // Sta≈Çe kalibracyjne i progowe
// #define BATTERY_VOLTAGE_RATIO 0.2117 // Poprzednio 0.2099
// #define OUTPUT_VOLTAGE_RATIO 0.2132 // Bez zmian
// #define ADC_REF_VOLTAGE 3.3
// #define ADC_MAX_VALUE 4095.0

// #define VOLTAGE_RECOVERY 11.5
// #define VOLTAGE_UNDER 11.0
// #define VOLTAGE_OVER_RECOVERY 12.5
// #define VOLTAGE_OVER 13.0
// #define VOLTAGE_OVER_SHUTDOWN 13.0

// // Sta≈Çe czasowe
// #define SHUTDOWN_DELAY_MS 30000
// #define RECOVERY_DELAY_MS 30000
// #define MQTT_SHUTDOWN_DELAY_MS 30000
// #define MESSAGE_DISPLAY_TIME 2000
// #define SCREEN_SWITCH_INTERVAL 1000
// #define SAMPLES_COUNT 50
// #define MQTT_HEARTBEAT_INTERVAL 300000

// // Instancje
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);
// WiFiClient espClient;
// PubSubClient mqttClient(espClient);

// // === Zmienne globalne chronione przez Mutex ===
// SemaphoreHandle_t dataMutex;
// float avgBatteryVoltage = 0.0;
// float avgOutputVoltage = 0.0;
// bool isPowerLost = false;
// bool isUndervoltage = false;
// bool isOvervoltage = false;
// bool isMqttShutdownTriggered = false;
// uint32_t systemStartTime = 0;
// uint32_t undervoltageStartTime = 0;
// bool undervoltageShutdownActive = false;
// uint32_t undervoltagePostShutdownStartTime = 0;
// uint32_t undervoltageRecoveryStartTime = 0;
// uint32_t overvoltageStartTime = 0;
// bool overvoltageShutdownActive = false;
// uint32_t overvoltagePostShutdownStartTime = 0;
// uint32_t overvoltageRecoveryStartTime = 0;
// // ===============================================

// enum DisplayState { NORMAL, SHOWING_MESSAGE };

// // Prototypy
// void sensorsTask(void *pvParameters);
// void displayTask(void *pvParameters);
// void logicAndLedsTask(void *pvParameters);
// void mqttTask(void *pvParameters);
// void drawMainScreen(float batV, float outV, bool powerLost, bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, uint32_t uvStartTime, uint32_t ovStartTime, uint32_t uvPostShutdownTime, uint32_t ovPostShutdownTime);
// void handleLeds(float voltage);

// void setup() {
//   pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
//   pinMode(POWER_LOSS_PIN, INPUT_PULLUP);
//   pinMode(LED_RED_PIN, OUTPUT); pinMode(LED_GREEN_PIN, OUTPUT); pinMode(LED_BLUE_PIN, OUTPUT);
//   u8g2.begin();
//   dataMutex = xSemaphoreCreateMutex();
//   if (dataMutex == NULL) { return; }
//   systemStartTime = millis();
//   xTaskCreate(sensorsTask, "Sensors", 2048, NULL, 2, NULL);
//   xTaskCreate(displayTask, "Display", 4096, NULL, 1, NULL);
//   xTaskCreate(logicAndLedsTask, "Logic/LEDs", 2048, NULL, 3, NULL);
//   xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
// }

// void loop() { vTaskDelete(NULL); }

// // ### ZADANIA RTOS ###

// void sensorsTask(void *pvParameters) {
//   float batterySamples[SAMPLES_COUNT] = {0}; float outputSamples[SAMPLES_COUNT] = {0}; int sampleIndex = 0;
//   for (;;) {
//     float rawBatteryVoltage = (analogRead(BATTERY_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / BATTERY_VOLTAGE_RATIO; batterySamples[sampleIndex] = rawBatteryVoltage;
//     float rawOutputVoltage = (analogRead(OUTPUT_VOLTAGE_PIN) * (ADC_REF_VOLTAGE / ADC_MAX_VALUE)) / OUTPUT_VOLTAGE_RATIO; outputSamples[sampleIndex] = rawOutputVoltage;
//     sampleIndex = (sampleIndex + 1) % SAMPLES_COUNT;
//     float batterySum = 0, outputSum = 0;
//     for (int i = 0; i < SAMPLES_COUNT; i++) { batterySum += batterySamples[i]; outputSum += outputSamples[i]; }
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { avgBatteryVoltage = batterySum / SAMPLES_COUNT; avgOutputVoltage = outputSum / SAMPLES_COUNT; xSemaphoreGive(dataMutex); }
//     vTaskDelay(pdMS_TO_TICKS(50));
//   }
// }

// void logicAndLedsTask(void *pvParameters) {
//   for (;;) {
//     float localBatV, localOutV; if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { localBatV = avgBatteryVoltage; localOutV = avgOutputVoltage; xSemaphoreGive(dataMutex); }
//     bool currentPowerState = (digitalRead(POWER_LOSS_PIN) == LOW);
//     bool undervoltage = (localOutV < VOLTAGE_UNDER);
//     bool overvoltage = (localOutV > VOLTAGE_OVER_SHUTDOWN);
//     uint32_t uv_startTime, uv_postShutdownTime, uv_recoveryTime; bool uv_shutdown;
//     uint32_t ov_startTime, ov_postShutdownTime, ov_recoveryTime; bool ov_shutdown;
//     bool mqtt_shutdown_flag;
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//       uv_startTime = undervoltageStartTime; uv_shutdown = undervoltageShutdownActive; uv_postShutdownTime = undervoltagePostShutdownStartTime; uv_recoveryTime = undervoltageRecoveryStartTime;
//       ov_startTime = overvoltageStartTime; ov_shutdown = overvoltageShutdownActive; ov_postShutdownTime = overvoltagePostShutdownStartTime; ov_recoveryTime = overvoltageRecoveryStartTime;
//       mqtt_shutdown_flag = isMqttShutdownTriggered;
//       xSemaphoreGive(dataMutex);
//     }
//     if (overvoltage) {
//       if (ov_startTime == 0) { ov_startTime = millis(); }
//       ov_recoveryTime = 0;
//       if (!ov_shutdown && (millis() - ov_startTime > SHUTDOWN_DELAY_MS)) { ov_shutdown = true; ov_postShutdownTime = millis(); }
//     } else {
//       ov_startTime = 0;
//       if (ov_shutdown) { if (localOutV < VOLTAGE_OVER_RECOVERY) { if (ov_recoveryTime == 0) { ov_recoveryTime = millis(); } if (millis() - ov_recoveryTime > RECOVERY_DELAY_MS) { ov_shutdown = false; ov_postShutdownTime = 0; ov_recoveryTime = 0; } } else { ov_recoveryTime = 0; } }
//     }
//     if (undervoltage) {
//       if (uv_startTime == 0) { uv_startTime = millis(); }
//       uv_recoveryTime = 0;
//       if (!uv_shutdown && (millis() - uv_startTime > SHUTDOWN_DELAY_MS)) { uv_shutdown = true; uv_postShutdownTime = millis(); }
//     } else {
//       uv_startTime = 0;
//       if (uv_shutdown) { if (localOutV > VOLTAGE_RECOVERY) { if (uv_recoveryTime == 0) { uv_recoveryTime = millis(); } if (millis() - uv_recoveryTime > RECOVERY_DELAY_MS) { uv_shutdown = false; uv_postShutdownTime = 0; uv_recoveryTime = 0; } } else { uv_recoveryTime = 0; } }
//     }
//     if ((uv_startTime != 0 && millis() - uv_startTime > MQTT_SHUTDOWN_DELAY_MS) || (ov_startTime != 0 && millis() - ov_startTime > MQTT_SHUTDOWN_DELAY_MS)) { mqtt_shutdown_flag = true; } else { mqtt_shutdown_flag = false; }
//     digitalWrite(RELAY_PIN, (uv_shutdown || ov_shutdown) ? HIGH : LOW);
//     handleLeds(localOutV);
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//       isPowerLost = currentPowerState; isUndervoltage = undervoltage; isOvervoltage = overvoltage; isMqttShutdownTriggered = mqtt_shutdown_flag;
//       undervoltageStartTime = uv_startTime; undervoltageShutdownActive = uv_shutdown; undervoltagePostShutdownStartTime = uv_postShutdownTime; undervoltageRecoveryStartTime = uv_recoveryTime;
//       overvoltageStartTime = ov_startTime; overvoltageShutdownActive = ov_shutdown; overvoltagePostShutdownStartTime = ov_postShutdownTime; overvoltageRecoveryStartTime = ov_recoveryTime;
//       xSemaphoreGive(dataMutex);
//     }
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }

// void displayTask(void *pvParameters) {
//   DisplayState currentDisplayState = NORMAL; unsigned long messageStartTime = 0; char messageLine1[20] = ""; char messageLine2[20] = ""; bool lastPowerState = false;
//   for (;;) {
//     float localBatV, localOutV; bool localPowerLost, localUndervoltage, localOvervoltage, localUvShutdown, localOvShutdown;
//     uint32_t localUvStartTime, localUvPostShutdownTime, localOvStartTime, localOvPostShutdownTime;
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//       localBatV = avgBatteryVoltage; localOutV = avgOutputVoltage; localPowerLost = isPowerLost;
//       localUndervoltage = isUndervoltage; localUvShutdown = undervoltageShutdownActive; localUvStartTime = undervoltageStartTime; localUvPostShutdownTime = undervoltagePostShutdownStartTime;
//       localOvervoltage = isOvervoltage; localOvShutdown = overvoltageShutdownActive; localOvStartTime = overvoltageStartTime; localOvPostShutdownTime = overvoltagePostShutdownStartTime;
//       xSemaphoreGive(dataMutex);
//     }
//     if (localPowerLost && !lastPowerState) { strcpy(messageLine1, "POWER LOST"); strcpy(messageLine2, "RUNNING ON BAT"); messageStartTime = millis(); currentDisplayState = SHOWING_MESSAGE; }
//     else if (!localPowerLost && lastPowerState) { strcpy(messageLine1, "POWER RESTORED"); strcpy(messageLine2, ""); messageStartTime = millis(); currentDisplayState = SHOWING_MESSAGE; }
//     lastPowerState = localPowerLost;
//     u8g2.clearBuffer();
//     switch (currentDisplayState) {
//       case SHOWING_MESSAGE: u8g2.setFont(u8g2_font_6x12_tf); u8g2.drawStr((128-strlen(messageLine1)*6)/2, 12, messageLine1); u8g2.drawStr((128-strlen(messageLine2)*6)/2, 28, messageLine2); if (millis() - messageStartTime >= MESSAGE_DISPLAY_TIME) { currentDisplayState = NORMAL; } break;
//       case NORMAL: drawMainScreen(localBatV, localOutV, localPowerLost, localUndervoltage, localOvervoltage, localUvShutdown, localOvShutdown, localUvStartTime, localOvStartTime, localUvPostShutdownTime, localOvPostShutdownTime); break;
//     }
//     u8g2.sendBuffer();
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }

// void mqttTask(void *pvParameters) {
//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//   mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
//   char lastPublishedSystemState[32] = ""; unsigned long lastHeartbeatTime = 0;
//   for (;;) {
//     if (WiFi.status() != WL_CONNECTED) { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
//     if (!mqttClient.connected()) { if (mqttClient.connect("esp32-ups", MQTT_USER, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 1, true, "offline")) { mqttClient.publish(MQTT_AVAILABILITY_TOPIC, "online", true); } else { vTaskDelay(pdMS_TO_TICKS(5000)); continue; } }
//     mqttClient.loop();
//     float batV, outV; bool pwrLost, undervoltage, overvoltage, uvShutdown, ovShutdown, mqttShutdown; char currentSystemState[32];
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//       batV = avgBatteryVoltage; outV = avgOutputVoltage; pwrLost = isPowerLost;
//       undervoltage = isUndervoltage; overvoltage = isOvervoltage;
//       uvShutdown = undervoltageShutdownActive; ovShutdown = overvoltageShutdownActive;
//       mqttShutdown = isMqttShutdownTriggered;
//       xSemaphoreGive(dataMutex);
//     }
//     if (uvShutdown) strcpy(currentSystemState, "UNDERVOLTAGE_SHUTDOWN");
//     else if (undervoltage && mqttShutdown) strcpy(currentSystemState, "UNDERVOLTAGE_SHUTDOWN_NOW");
//     else if (undervoltage) strcpy(currentSystemState, "UNDERVOLTAGE_WARNING");
//     else if (ovShutdown) strcpy(currentSystemState, "OVERVOLTAGE_SHUTDOWN");
//     else if (overvoltage && mqttShutdown) strcpy(currentSystemState, "OVERVOLTAGE_SHUTDOWN_NOW");
//     else if (overvoltage) strcpy(currentSystemState, "OVERVOLTAGE_WARNING");
//     else if (pwrLost) strcpy(currentSystemState, "ON_BATTERY");
//     else strcpy(currentSystemState, "ONLINE");
//     bool stateChanged = strcmp(currentSystemState, lastPublishedSystemState) != 0; bool heartbeatDue = millis() - lastHeartbeatTime > MQTT_HEARTBEAT_INTERVAL;
//     if (stateChanged || heartbeatDue) {
//       char jsonPayload[200];
//       snprintf(jsonPayload, sizeof(jsonPayload), "{\"battery_v\":%.2f, \"output_v\":%.2f, \"state\":\"%s\"}", batV, outV, currentSystemState);
//       mqttClient.publish(MQTT_STATE_TOPIC, jsonPayload);
//       strcpy(lastPublishedSystemState, currentSystemState);
//       lastHeartbeatTime = millis();
//     }
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }

// void drawMainScreen(float batV, float outV, bool powerLost, bool undervoltage, bool overvoltage, bool uvShutdown, bool ovShutdown, uint32_t uvStartTime, uint32_t ovStartTime, uint32_t uvPostShutdownTime, uint32_t ovPostShutdownTime) {
//   // Lewa po≈Çowa: BAT i OUT
//   u8g2.setFont(u8g2_font_6x12_tf);
//   char buf[8];
//   int batWhole = floor(batV);
//   int batDecimal = round((batV - batWhole) * 10);
//   if (batDecimal == 10) {
//     batWhole += 1;
//     batDecimal = 0;
//   }
//   sprintf(buf, "%d", batWhole);
//   u8g2.drawStr(0, 14, "BAT");
//   u8g2.setFont(u8g2_font_VCR_OSD_mn);
//   int wholeWidth = u8g2.getStrWidth(buf);
//   sprintf(buf + strlen(buf), "%d", batDecimal);
//   int decimalWidth = u8g2.getStrWidth(buf + strlen(buf) - 1); // szeroko≈õƒá jednej cyfry
//   int totalWidth = wholeWidth + 3 + decimalWidth;
//   int x = 56 - totalWidth;
//   sprintf(buf, "%d", batWhole);
//   u8g2.drawStr(x, 15, buf);
//   sprintf(buf, "%d", batDecimal);
//   u8g2.drawStr(x + wholeWidth + 2, 15, buf);
//   u8g2.drawBox(41, 12, 2, 2);
//   u8g2.setFont(u8g2_font_6x12_tf);
//   u8g2.drawStr(57, 14, "V");
//   int outWhole = floor(outV);
//   int outDecimal = round((outV - outWhole) * 10);
//   if (outDecimal == 10) {
//     outWhole += 1;
//     outDecimal = 0;
//   }
//   sprintf(buf, "%d", outWhole);
//   u8g2.drawStr(0, 32, "OUT");
//   u8g2.setFont(u8g2_font_VCR_OSD_mn);
//   wholeWidth = u8g2.getStrWidth(buf);
//   sprintf(buf + strlen(buf), "%d", outDecimal);
//   decimalWidth = u8g2.getStrWidth(buf + strlen(buf) - 1);
//   totalWidth = wholeWidth + 3 + decimalWidth;
//   x = 56 - totalWidth;
//   sprintf(buf, "%d", outWhole);
//   u8g2.drawStr(x, 33, buf);
//   sprintf(buf, "%d", outDecimal);
//   u8g2.drawStr(x + wholeWidth + 2, 33, buf);
//   u8g2.drawBox(41, 30, 2, 2);
//   u8g2.setFont(u8g2_font_6x12_tf);
//   u8g2.drawStr(57, 32, "V");

//   // Prawa po≈Çowa: timery na g√≥rze, komunikaty na dole
//   u8g2.setFont(u8g2_font_VCR_OSD_mn);
//   char statusMsg[16] = "";
//   int timerMinutes = 0, timerSeconds = 0;
//   bool hasTimer = false;
//   if (overvoltage) {
//     if (ovShutdown) {
//       int s = (millis() - ovPostShutdownTime) / 1000;
//       timerMinutes = s / 60;
//       timerSeconds = s % 60;
//       hasTimer = true;
//     } else {
//       int rem_s = (SHUTDOWN_DELAY_MS - (millis() - ovStartTime)) / 1000;
//       if (rem_s < 0) rem_s = 0;
//       timerMinutes = rem_s / 60;
//       timerSeconds = rem_s % 60;
//       hasTimer = true;
//     }
//     if (ovShutdown) {
//       strcpy(statusMsg, "POWER OFF");
//     } else {
//       strcpy(statusMsg, "OVER V");
//     }
//   } else if (undervoltage) {
//     if (uvShutdown) {
//       strcpy(statusMsg, "POWER OFF");
//     } else {
//       strcpy(statusMsg, "UNDER V");
//     }
//     if (uvShutdown) {
//       int s = (millis() - uvPostShutdownTime) / 1000;
//       timerMinutes = s / 60;
//       timerSeconds = s % 60;
//       hasTimer = true;
//     } else {
//       int rem_s = (SHUTDOWN_DELAY_MS - (millis() - uvStartTime)) / 1000;
//       if (rem_s < 0) rem_s = 0;
//       timerMinutes = rem_s / 60;
//       timerSeconds = rem_s % 60;
//       hasTimer = true;
//     }
//   } else if (powerLost) {
//     strcpy(statusMsg, "ON BATT");
//   } else {
//     strcpy(statusMsg, "SYS OK");
//   }
//   if (hasTimer) {
//     if (timerSeconds == 60) {
//       timerMinutes++;
//       timerSeconds = 0;
//     }
//     char minStr[4], secStr[4];
//     sprintf(minStr, "%d", timerMinutes);
//     sprintf(secStr, "%02d", timerSeconds);
//     int minWidth = u8g2.getStrWidth(minStr);
//     int secWidth = u8g2.getStrWidth(secStr);
//     int totalWidth = minWidth + 4 + secWidth;
//     int x = 128 - totalWidth;
//     u8g2.drawStr(x, 15, minStr);
//     // Dwukropek jako 2 kwadraty 2x2 jeden nad drugim
//     int colonX = x + minWidth + 1;
//     u8g2.drawBox(colonX, 4, 2, 2);
//     u8g2.drawBox(colonX, 9, 2, 2);
//     u8g2.drawStr(x + minWidth + 4, 15, secStr);
//   } else {
//     // Uptime w trybie normalnym
//     uint32_t uptimeMs = millis() - systemStartTime;
//     int uptimeSeconds = uptimeMs / 1000;
//     int uptimeMinutes = uptimeSeconds / 60;
//     int uptimeSecs = uptimeSeconds % 60;
//     char minStr[4], secStr[4];
//     sprintf(minStr, "%d", uptimeMinutes);
//     sprintf(secStr, "%02d", uptimeSecs);
//     int minWidth = u8g2.getStrWidth(minStr);
//     int secWidth = u8g2.getStrWidth(secStr);
//     int totalWidth = minWidth + 4 + secWidth;
//     int x = 128 - totalWidth;
//     u8g2.drawStr(x, 15, minStr);
//     // Dwukropek jako 2 kwadraty 2x2 jeden nad drugim
//     int colonX = x + minWidth + 1;
//     u8g2.drawBox(colonX, 4, 2, 2);
//     u8g2.drawBox(colonX, 9, 2, 2);
//     u8g2.drawStr(x + minWidth + 4, 15, secStr);
//   }
//   u8g2.setFont(u8g2_font_8x13_tf);
//   u8g2.drawStr(68, 32, statusMsg);
//   // Pionowa linia przerywana 2 pixele za "V"
//   for(int y=0; y<32; y+=4) {
//     u8g2.drawPixel(65, y);
//     u8g2.drawPixel(65, y+1);
//   }
// }

// void handleLeds(float voltage) {
//     const int maxBrightness = 255; int redValue = 0, greenValue = 0, blueValue = 0; const float VOLTAGE_MID = 12.0;
//     if (voltage < VOLTAGE_UNDER) { unsigned long currentTime = millis(); float phase = (currentTime % 2000) / 2000.0; if (phase < 0.3) redValue = (int)((phase / 0.3) * (maxBrightness / 2)); else redValue = (int)(((1.0 - phase) / 0.7) * (maxBrightness / 2));
//     } else if (voltage >= VOLTAGE_UNDER && voltage < VOLTAGE_MID) { float ratio = constrain((voltage - VOLTAGE_UNDER) / (VOLTAGE_MID - VOLTAGE_UNDER), 0.0, 1.0); redValue = (int)((1.0 - ratio) * maxBrightness); greenValue = (int)(ratio * maxBrightness);
//     } else if (voltage >= VOLTAGE_MID && voltage <= VOLTAGE_OVER) { float ratio = constrain((voltage - VOLTAGE_MID) / (VOLTAGE_OVER - VOLTAGE_MID), 0.0, 1.0); redValue = (int)(ratio * maxBrightness); greenValue = (int)((1.0 - ratio) * maxBrightness); blueValue = (int)(ratio * maxBrightness);
//     } else if (voltage > VOLTAGE_OVER) {
//       // Bardzo szybkie miganie na czerwono przy overvoltage
//       unsigned long currentTime = millis();
//       if (currentTime % 100 < 50) {
//         redValue = maxBrightness;
//         greenValue = 0;
//         blueValue = 0;
//       } else {
//         redValue = 0;
//         greenValue = 0;
//         blueValue = 0;
//       }
//     }
//     analogWrite(LED_RED_PIN, redValue); analogWrite(LED_GREEN_PIN, greenValue); analogWrite(LED_BLUE_PIN, blueValue);
// }