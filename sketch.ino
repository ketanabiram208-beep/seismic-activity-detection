/*
 * =========================================================================
 * Seismic Activity Detection System
 * Compatible with BOTH ESP32 and Arduino Uno / Nano
 * 
 * Target Boards:
 *  - ESP32 DevKit V1 (30-pin / 38-pin)
 *  - Arduino Uno R3 / Nano / Mega
 *
 * Sensors & Peripherals:
 *  - MPU-6050 3-Axis Accelerometer/Gyro (I2C 0x68)
 *  - SSD1306 128x64 0.96" OLED Display (I2C 0x3C)
 *  - Green LED   -> NORMAL status
 *  - Yellow LED  -> WARNING status (Vibration >= 1.2 m/s^2)
 *  - Red LED     -> ALERT status   (Vibration >= 2.5 m/s^2 sustained)
 *  - Buzzer      -> Audible alarm during ALERT
 * =========================================================================
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- PIN DEFINITIONS (Auto-detects board architecture) ---
#if defined(ESP32)
  #define BOARD_NAME      "ESP32"
  #define GREEN_LED_PIN   25
  #define YELLOW_LED_PIN  26
  #define RED_LED_PIN     27
  #define BUZZER_PIN      18
  #define I2C_SDA_PIN     21
  #define I2C_SCL_PIN     22
#else // Arduino Uno, Nano, Pro Mini, Mega
  #define BOARD_NAME      "Arduino Uno"
  #define GREEN_LED_PIN   2
  #define YELLOW_LED_PIN  3
  #define RED_LED_PIN     4
  #define BUZZER_PIN      8
  #define I2C_SDA_PIN     A4
  #define I2C_SCL_PIN     A5
#endif

// --- DISPLAY CONFIGURATION ---
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define OLED_RESET        -1
#define OLED_I2C_ADDRESS  0x3C

// --- SEISMIC THRESHOLDS (m/s^2 deviation from 1g) ---
#define WARNING_THRESHOLD     1.2f
#define ALERT_THRESHOLD       2.5f
#define ALERT_CLEAR_THRESHOLD 0.8f

// --- TIMING INTERVALS (milliseconds) ---
#define SAMPLE_INTERVAL_MS    50
#define DISPLAY_INTERVAL_MS   200
#define ALERT_CONFIRM_MS      500
#define ALERT_CLEAR_MS        1000
#define SAMPLE_COUNT          5

enum SystemState {
  STATE_NORMAL,
  STATE_WARNING,
  STATE_ALERT
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

SystemState currentState = STATE_NORMAL;
float vibrationSamples[SAMPLE_COUNT] = {0};
int sampleIndex = 0;
int samplesStored = 0;
float sampleTotal = 0.0f;

bool highTimerActive = false;
bool quietTimerActive = false;
unsigned long highVibrationStart = 0;
unsigned long quietStart = 0;
unsigned long lastSampleTime = 0;
unsigned long lastDisplayTime = 0;

float calculateMovingAverage(float newSample) {
  if (samplesStored == SAMPLE_COUNT) {
    sampleTotal -= vibrationSamples[sampleIndex];
  } else {
    samplesStored++;
  }
  vibrationSamples[sampleIndex] = newSample;
  sampleTotal += newSample;
  sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
  return sampleTotal / (float)samplesStored;
}

void updateOutputs() {
  digitalWrite(GREEN_LED_PIN,  currentState == STATE_NORMAL  ? HIGH : LOW);
  digitalWrite(YELLOW_LED_PIN, currentState == STATE_WARNING ? HIGH : LOW);
  digitalWrite(RED_LED_PIN,    currentState == STATE_ALERT   ? HIGH : LOW);
  digitalWrite(BUZZER_PIN,     currentState == STATE_ALERT   ? HIGH : LOW);
}

void processStateMachine(float vibration, unsigned long nowMs) {
  if (currentState == STATE_ALERT) {
    if (vibration < ALERT_CLEAR_THRESHOLD) {
      if (!quietTimerActive) {
        quietStart = nowMs;
        quietTimerActive = true;
      }
      if (nowMs - quietStart >= ALERT_CLEAR_MS) {
        currentState = STATE_NORMAL;
        quietTimerActive = false;
        highTimerActive = false;
      }
    } else {
      quietTimerActive = false;
    }
    return;
  }

  if (vibration >= ALERT_THRESHOLD) {
    if (!highTimerActive) {
      highVibrationStart = nowMs;
      highTimerActive = true;
    }
    if (nowMs - highVibrationStart >= ALERT_CONFIRM_MS) {
      currentState = STATE_ALERT;
      quietTimerActive = false;
      return;
    }
  } else {
    highTimerActive = false;
  }

  currentState = (vibration >= WARNING_THRESHOLD) ? STATE_WARNING : STATE_NORMAL;
}

const __FlashStringHelper* getStateName(SystemState state) {
  switch (state) {
    case STATE_ALERT:   return F("ALERT");
    case STATE_WARNING: return F("WARNING");
    default:            return F("NORMAL");
  }
}

void renderDisplay(float vibration) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("SEISMIC ("));
  display.print(BOARD_NAME);
  display.println(F(")"));

  display.setCursor(0, 14);
  display.println(F("VIBRATION M/S2:"));

  // Large numerical value
  display.setTextSize(2);
  display.setCursor(0, 26);
  display.print(vibration, 2);

  // Status indicator
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print(F("STATUS: "));
  display.println(getStateName(currentState));

  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.print(F("[INIT] Starting Seismic Monitor on "));
  Serial.println(BOARD_NAME);

  // Initialize GPIO pins
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  updateOutputs();

  // Initialize I2C bus
#if defined(ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
#else
  Wire.begin();
#endif

  // Initialize MPU6050
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println(F("[ERROR] MPU-6050 not found! Check I2C wiring (SDA/SCL) and address 0x68."));
    while (1) {
      digitalWrite(RED_LED_PIN, HIGH);
      delay(300);
      digitalWrite(RED_LED_PIN, LOW);
      delay(300);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("[OK] MPU-6050 initialized (+/- 4g range)."));

  // Initialize SSD1306 OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("[ERROR] SSD1306 OLED not found! Check wiring and address 0x3C."));
    while (1) {
      digitalWrite(YELLOW_LED_PIN, HIGH);
      delay(300);
      digitalWrite(YELLOW_LED_PIN, LOW);
      delay(300);
    }
  }

  // Welcome splash screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.println(F("Seismic Activity"));
  display.setCursor(10, 36);
  display.println(F("Monitor Ready"));
  display.display();
  delay(1000);

  Serial.println(F("[OK] System running. Monitoring vibrations..."));
}

void loop() {
  unsigned long nowMs = millis();

  // 1. High-frequency sensor sample rate (every 50 ms = 20 Hz)
  if (nowMs - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = nowMs;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    // Calculate total acceleration magnitude
    float totalAcceleration = sqrt(ax * ax + ay * ay + az * az);
    // Dynamic deviation from Earth's gravity (9.80665 m/s^2)
    float rawVibration = fabs(totalAcceleration - 9.80665f);
    // Smooth readings using moving average
    float avgVibration = calculateMovingAverage(rawVibration);

    processStateMachine(avgVibration, nowMs);
    updateOutputs();

    // Print to serial monitor
    int hundredths = (int)(avgVibration * 100.0f);
    Serial.print(F("Vibration: "));
    Serial.print(hundredths / 100);
    Serial.print(F("."));
    if ((hundredths % 100) < 10) Serial.print(F("0"));
    Serial.print(hundredths % 100);
    Serial.print(F(" m/s^2 | State: "));
    Serial.println(getStateName(currentState));
  }

  // 2. Display refresh rate (every 200 ms = 5 Hz)
  if (nowMs - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = nowMs;
    float currentVib = sampleTotal / (samplesStored > 0 ? (float)samplesStored : 1.0f);
    renderDisplay(currentVib);
  }
}
