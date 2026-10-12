/*
 * ESP32 Seismic Activity Detection
 * Arduino C/C++ Implementation for Arduino IDE / Wokwi Simulator
 *
 * Hardware:
 *  - ESP32 DevKit V1
 *  - MPU6050 Accelerometer / Gyroscope (I2C: SDA=GPIO21, SCL=GPIO22, Addr=0x68)
 *  - SSD1306 128x64 OLED Display (I2C: SDA=GPIO21, SCL=GPIO22, Addr=0x3C)
 *  - Green LED (GPIO 25)  -> NORMAL status
 *  - Yellow LED (GPIO 26) -> WARNING status (>= 1.2 m/s^2)
 *  - Red LED (GPIO 27)    -> ALERT status   (>= 2.5 m/s^2 sustained)
 *  - Buzzer (GPIO 18)     -> Sound alarm during ALERT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

#define GREEN_LED_PIN   25
#define YELLOW_LED_PIN  26
#define RED_LED_PIN     27
#define BUZZER_PIN      18

#define WARNING_THRESHOLD     1.2f
#define ALERT_THRESHOLD       2.5f
#define ALERT_CLEAR_THRESHOLD 0.8f

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

float averageVibration(float newSample) {
  if (samplesStored == SAMPLE_COUNT) {
    sampleTotal -= vibrationSamples[sampleIndex];
  } else {
    samplesStored++;
  }
  vibrationSamples[sampleIndex] = newSample;
  sampleTotal += newSample;
  sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
  return sampleTotal / samplesStored;
}

void updateOutputs() {
  digitalWrite(GREEN_LED_PIN,  currentState == STATE_NORMAL  ? HIGH : LOW);
  digitalWrite(YELLOW_LED_PIN, currentState == STATE_WARNING ? HIGH : LOW);
  digitalWrite(RED_LED_PIN,    currentState == STATE_ALERT   ? HIGH : LOW);
  digitalWrite(BUZZER_PIN,     currentState == STATE_ALERT   ? HIGH : LOW);
}

void updateState(float vibration, unsigned long nowMs) {
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

const char* getStateName(SystemState state) {
  switch (state) {
    case STATE_ALERT:   return "ALERT";
    case STATE_WARNING: return "WARNING";
    default:            return "NORMAL";
  }
}

void updateDisplay(float vibration) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("SEISMIC MONITOR"));

  display.setCursor(0, 16);
  display.println(F("VIBRATION M/S2:"));

  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print(vibration, 2);

  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print(F("STATUS: "));
  display.println(getStateName(currentState));

  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  updateOutputs();

  Wire.begin(21, 22); // SDA=21, SCL=22

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println(F("MPU6050 connection failed! Check wiring."));
    while (1) { delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed! Check wiring."));
    while (1) { delay(1000); }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(F("Seismic Monitor"));
  display.println(F("Starting up..."));
  display.display();
  delay(500);

  Serial.println(F("Seismic activity monitor initialized successfully."));
}

void loop() {
  unsigned long nowMs = millis();

  if (nowMs - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = nowMs;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float totalAccel = sqrt(ax * ax + ay * ay + az * az);
    float rawVibration = fabs(totalAccel - 9.80665f);
    float avgVib = averageVibration(rawVibration);

    updateState(avgVib, nowMs);
    updateOutputs();

    Serial.print(F("Vibration: "));
    Serial.print(avgVib, 2);
    Serial.print(F(" m/s^2 | State: "));
    Serial.println(getStateName(currentState));
  }

  if (nowMs - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = nowMs;
    float currentVib = sampleTotal / (samplesStored > 0 ? samplesStored : 1);
    updateDisplay(currentVib);
  }
}
