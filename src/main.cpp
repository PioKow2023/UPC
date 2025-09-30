#include <Wire.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Display dimensions
#define OLED_WIDTH 128
#define OLED_HEIGHT 32

// Pin definitions
#define OLED_SDA_PIN 8
#define OLED_SCL_PIN 9
#define OLED_RESET_PIN U8X8_PIN_NONE
#define RELAY_PIN 5  // Define relay pin - can be changed as needed
#define RESET_BUTTON_PIN 6  // Define reset button pin

// RGB LED pins
#define LED_RED_PIN 2
#define LED_GREEN_PIN 3
#define LED_BLUE_PIN 4

// ADC pins for voltage measurement
#define BATTERY_VOLTAGE_PIN 0
#define OUTPUT_VOLTAGE_PIN 1

// Voltage calculation constants
// Voltage divider: R1=47k, R2=10k => ratio = 10/(47+10) = 0.175439
// Calibrated ratios based on actual measurements
#define BATTERY_VOLTAGE_RATIO 0.1968
#define OUTPUT_VOLTAGE_RATIO 0.1991
#define ADC_REF_VOLTAGE 3.3
#define ADC_MAX_VALUE 4095.0

// Voltage thresholds for LED colors
#define BATTERY_VOLTAGE_RECOVERY 11.0  // Recovery threshold - relay turns back on
#define BATTERY_VOLTAGE_SHUTDOWN 10.0  // Shutdown threshold
#define BATTERY_VOLTAGE_VERY_HIGH 14.0
#define BATTERY_VOLTAGE_HIGH 11.8
#define BATTERY_VOLTAGE_LOW 11.4
#define BATTERY_VOLTAGE_VERY_LOW 10.0

// LED pulse parameters (in seconds for full cycle: brighten + dim)
#define PULSE_VERY_SLOW 4     // 4 seconds for green
#define PULSE_SLOW 2          // 2 seconds for blue and red (very low voltage)
#define PULSE_MEDIUM 1        // 1 second for red (low voltage)
#define PULSE_FAST 0.25       // 0.25 seconds (4 times per second) for red (very high voltage)

// LED brightness levels (0-255)
#define LED_BRIGHTNESS_MAX 255
#define LED_BRIGHTNESS_MIN 0

// Filter settings
#define SAMPLES_COUNT 20

// Text positions
#define BAT_LABEL_X 1
#define BAT_LABEL_Y_START 10
#define BAT_LABEL_Y_INCREMENT 8

#define OUT_LABEL_X 69
#define OUT_LABEL_Y_START 10
#define OUT_LABEL_Y_INCREMENT 8

#define BAT_VOLTAGE_X 10
#define BAT_VOLTAGE_Y 24

#define OUT_VOLTAGE_X 79
#define OUT_VOLTAGE_Y 24

// Voltage values
float batteryVoltage = 0.0;
float outputVoltage = 0.0;

// Arrays for filtering
float batterySamples[SAMPLES_COUNT] = {0};
float outputSamples[SAMPLES_COUNT] = {0};
int sampleIndex = 0;

// LED state variables
unsigned long lastLedUpdate = 0;
float pulsePeriod = PULSE_VERY_SLOW; // Convert seconds to milliseconds
int ledBrightness = 0;
bool ledDirection = true; // true = increasing, false = decreasing
unsigned long pulseStartTime = 0;
bool shutdownActivated = false; // Flag to track if shutdown has been activated
unsigned long lastResetButtonPress = 0; // Debounce tracking for reset button

// Font definitions
#define LABEL_FONT u8g2_font_5x8_tf
#define VOLTAGE_FONT u8g2_font_helvR18_tf

// Horizontal lines positions
#define HORIZONTAL_LINE_Y 31
#define HORIZONTAL_LINE_X1_START 1
#define HORIZONTAL_LINE_X1_END 60
#define HORIZONTAL_LINE_X2_START 69
#define HORIZONTAL_LINE_X2_END 128

// Create display object for SSD1306 128x32 OLED
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, OLED_RESET_PIN, OLED_SCL_PIN, OLED_SDA_PIN);

// Function to calculate average value
float getAverage(float samples[]) {
  float sum = 0;
  for (int i = 0; i < SAMPLES_COUNT; i++) {
    sum += samples[i];
  }
  return sum / SAMPLES_COUNT;
}

// Function to calculate maximum LED brightness based on battery voltage
int getMaxLedBrightness(float voltage) {
  if (voltage < BATTERY_VOLTAGE_VERY_LOW) {
    // Below 10V: 1/4 of maximum brightness
    return LED_BRIGHTNESS_MAX / 4;
  } else if (voltage <= BATTERY_VOLTAGE_VERY_HIGH) {
    // 10V to 14V: 1/2 of maximum brightness
    return LED_BRIGHTNESS_MAX / 2;
  } else {
    // Above 14V: full brightness
    return LED_BRIGHTNESS_MAX;
  }
}

// Function to update LED pulse parameters based on battery voltage
void updateLedParameters(float voltage) {
  // Calculate maximum brightness for current voltage
  int maxBrightness = getMaxLedBrightness(voltage);
  
  if (voltage > BATTERY_VOLTAGE_VERY_HIGH) {
    // Very high voltage: Red, pulse 4 times per second
    pulsePeriod = PULSE_FAST;
    analogWrite(LED_RED_PIN, (ledBrightness * maxBrightness) / LED_BRIGHTNESS_MAX);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
  } else if (voltage > BATTERY_VOLTAGE_HIGH) {
    // High voltage: Green, pulse once every 4 seconds
    pulsePeriod = PULSE_VERY_SLOW;
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, (ledBrightness * maxBrightness) / LED_BRIGHTNESS_MAX);
    analogWrite(LED_BLUE_PIN, 0);
  } else if (voltage > BATTERY_VOLTAGE_LOW) {
    // Medium voltage: Yellow (red + green), pulse once every 2 seconds
    // Reduce green intensity to get proper yellow color
    pulsePeriod = PULSE_SLOW;
    int adjustedRed = (ledBrightness * maxBrightness) / LED_BRIGHTNESS_MAX;
    int adjustedGreen = (adjustedRed / 3); // One third of red brightness for proper yellow
    analogWrite(LED_RED_PIN, adjustedRed);
    analogWrite(LED_GREEN_PIN, adjustedGreen);
    analogWrite(LED_BLUE_PIN, 0);
  } else if (voltage > BATTERY_VOLTAGE_VERY_LOW) {
    // Low voltage: Red, pulse once every 1 second
    pulsePeriod = PULSE_MEDIUM;
    analogWrite(LED_RED_PIN, (ledBrightness * maxBrightness) / LED_BRIGHTNESS_MAX);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
  } else {
    // Very low voltage: Red, pulse once every 4 seconds (half the previous speed)
    pulsePeriod = PULSE_VERY_SLOW;
    analogWrite(LED_RED_PIN, (ledBrightness * maxBrightness) / LED_BRIGHTNESS_MAX);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
  }
}

// Function to handle LED pulsing
void handleLedPulsing(float voltage) {
  unsigned long currentTime = millis();
  
  // Check if we need to start a new pulse cycle
  unsigned long pulsePeriodMs = (unsigned long)(pulsePeriod * 1000);
  if (currentTime - pulseStartTime >= pulsePeriodMs) {
    pulseStartTime = currentTime;
  }
  
  // Calculate position within the current pulse cycle (0 to pulsePeriodMs-1)
  unsigned long timeInCycle = currentTime - pulseStartTime;
  
  // Calculate LED brightness based on position in cycle
  // First half: brighten (0 to pulsePeriodMs/2-1)
  // Second half: dim (pulsePeriodMs/2 to pulsePeriodMs-1)
  if (timeInCycle < pulsePeriodMs/2) {
    // Brighten: map timeInCycle from 0..pulsePeriodMs/2-1 to 0..255
    ledBrightness = (int)((timeInCycle * 255) / (pulsePeriodMs/2 - 1));
  } else {
    // Dim: map timeInCycle from pulsePeriodMs/2..pulsePeriodMs-1 to 255..0
    ledBrightness = (int)(((pulsePeriodMs - 1 - timeInCycle) * 255) / (pulsePeriodMs/2 - 1));
  }
  
  // Update LED with new brightness
  updateLedParameters(voltage);
}

// Function to handle relay and shutdown procedure
void handleRelayAndShutdown(float voltage) {
  // Check if battery voltage is below shutdown threshold
  if (voltage < BATTERY_VOLTAGE_SHUTDOWN && !shutdownActivated) {
    // Activate shutdown sequence
    shutdownActivated = true;
    
    // Turn on relay to disconnect load
    digitalWrite(RELAY_PIN, HIGH);
    
    // Display shutdown message
    u8g2.clearBuffer();
    u8g2.setFont(LABEL_FONT);
    u8g2.drawStr(10, 10, "BATTERY LOW");
    u8g2.drawStr(10, 25, "SHUTTING DOWN");
    u8g2.sendBuffer();
    
    // Delay to show message
    delay(3000);
  }
  // Check if battery has recovered and we can turn load back on
  else if (voltage >= BATTERY_VOLTAGE_RECOVERY && shutdownActivated) {
    // Deactivate shutdown
    shutdownActivated = false;
    
    // Turn off relay to reconnect load
    digitalWrite(RELAY_PIN, LOW);
    
    // Display recovery message
    u8g2.clearBuffer();
    u8g2.setFont(LABEL_FONT);
    u8g2.drawStr(10, 10, "BATTERY OK");
    u8g2.drawStr(10, 25, "SYSTEM ON");
    u8g2.sendBuffer();
    
    // Delay to show message
    delay(2000);
  }
}

void setup() {
  // Initialize the OLED display
  u8g2.begin();
  
  // Initialize relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Start with relay off
  
  // Initialize reset button pin
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize LED pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  
  // Turn off LED initially
  analogWrite(LED_RED_PIN, 0);
  analogWrite(LED_GREEN_PIN, 0);
  analogWrite(LED_BLUE_PIN, 0);
  
  // Clear the display
  u8g2.clearBuffer();
  
  // Set ADC to 12-bit resolution
  analogReadResolution(12);
  
  // Initialize samples arrays with first readings
  int adcValueBattery = analogRead(BATTERY_VOLTAGE_PIN);
  float adcVoltageBattery = adcValueBattery * (ADC_REF_VOLTAGE / ADC_MAX_VALUE);
  batteryVoltage = adcVoltageBattery / BATTERY_VOLTAGE_RATIO;
  
  int adcValueOutput = analogRead(OUTPUT_VOLTAGE_PIN);
  float adcVoltageOutput = adcValueOutput * (ADC_REF_VOLTAGE / ADC_MAX_VALUE);
  outputVoltage = adcVoltageOutput / OUTPUT_VOLTAGE_RATIO;
  
  for (int i = 0; i < SAMPLES_COUNT; i++) {
    batterySamples[i] = batteryVoltage;
    outputSamples[i] = outputVoltage;
  }
}

void loop() {
  // Read battery voltage
  int adcValueBattery = analogRead(BATTERY_VOLTAGE_PIN);
  float adcVoltageBattery = adcValueBattery * (ADC_REF_VOLTAGE / ADC_MAX_VALUE);
  batteryVoltage = adcVoltageBattery / BATTERY_VOLTAGE_RATIO;
  
  // Read output voltage
  int adcValueOutput = analogRead(OUTPUT_VOLTAGE_PIN);
  float adcVoltageOutput = adcValueOutput * (ADC_REF_VOLTAGE / ADC_MAX_VALUE);
  outputVoltage = adcVoltageOutput / OUTPUT_VOLTAGE_RATIO;
  
  // Check for reset button press (with debouncing)
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    unsigned long currentTime = millis();
    if (currentTime - lastResetButtonPress > 1000) { // 1 second debounce
      lastResetButtonPress = currentTime;
      
      // Reset shutdown state if activated
      if (shutdownActivated) {
        shutdownActivated = false;
        digitalWrite(RELAY_PIN, LOW); // Turn relay off
        
        // Display reset message
        u8g2.clearBuffer();
        u8g2.setFont(LABEL_FONT);
        u8g2.drawStr(10, 10, "MANUAL RESET");
        u8g2.drawStr(10, 25, "SYSTEM ON");
        u8g2.sendBuffer();
        delay(2000);
      }
    }
  }
  
  // Add readings to samples arrays
  batterySamples[sampleIndex] = batteryVoltage;
  outputSamples[sampleIndex] = outputVoltage;
  sampleIndex = (sampleIndex + 1) % SAMPLES_COUNT;
  
  // Calculate average values
  float avgBatteryVoltage = getAverage(batterySamples);
  float avgOutputVoltage = getAverage(outputSamples);
  
  // Handle relay and shutdown procedure
  handleRelayAndShutdown(avgBatteryVoltage);
  
  // Handle LED pulsing (only if shutdown not activated)
  if (!shutdownActivated) {
    handleLedPulsing(avgBatteryVoltage);
    
    // Clear the display
    u8g2.clearBuffer();
    
    // Set font for labels - using a narrow font
    u8g2.setFont(LABEL_FONT);
    
    // Draw left side - "BAT" text vertically
    u8g2.drawStr(BAT_LABEL_X, BAT_LABEL_Y_START, "B");
    u8g2.drawStr(BAT_LABEL_X, BAT_LABEL_Y_START + BAT_LABEL_Y_INCREMENT, "A");
    u8g2.drawStr(BAT_LABEL_X, BAT_LABEL_Y_START + 2 * BAT_LABEL_Y_INCREMENT, "T");
    
    // Draw right side - "OUT" text vertically
    u8g2.setFont(LABEL_FONT);
    u8g2.drawStr(OUT_LABEL_X, OUT_LABEL_Y_START, "O");
    u8g2.drawStr(OUT_LABEL_X, OUT_LABEL_Y_START + OUT_LABEL_Y_INCREMENT, "U");
    u8g2.drawStr(OUT_LABEL_X, OUT_LABEL_Y_START + 2 * OUT_LABEL_Y_INCREMENT, "T");
    
    // Draw battery voltage with a bold font
    u8g2.setFont(VOLTAGE_FONT);
    char batteryVoltageStr[6];
    dtostrf(avgBatteryVoltage, 4, 1, batteryVoltageStr);
    u8g2.drawStr(BAT_VOLTAGE_X, BAT_VOLTAGE_Y, batteryVoltageStr);
    
    // Draw output voltage with a bold font
    u8g2.setFont(VOLTAGE_FONT);
    char outputVoltageStr[6];
    dtostrf(avgOutputVoltage, 4, 1, outputVoltageStr);
    u8g2.drawStr(OUT_VOLTAGE_X, OUT_VOLTAGE_Y, outputVoltageStr);
    
    // Draw horizontal lines
    u8g2.drawHLine(HORIZONTAL_LINE_X1_START, HORIZONTAL_LINE_Y, HORIZONTAL_LINE_X1_END - HORIZONTAL_LINE_X1_START);
    u8g2.drawHLine(HORIZONTAL_LINE_X2_START, HORIZONTAL_LINE_Y, HORIZONTAL_LINE_X2_END - HORIZONTAL_LINE_X2_START);
    
    // Send buffer to display
    u8g2.sendBuffer();
  }
  
  // Wait before next update
  delay(50);
}