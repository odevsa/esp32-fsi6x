#include <Arduino.h>
#include <BleGamepad.h>
#include "driver/rtc_io.h"
#include "esp32-hal-cpu.h"

// #define DEBUG

#define CPU_FREQ_MHZ 80
#define SLEEP_TIMEOUT_MS 300000 // 1000 * 60 * 5 = 5 minutes

unsigned long lastActivityTime = 0;

// Axes
#define AXIS_DEADZONE_RAW 0
#define AXIS_DEADZONE_MAPPED 250
#define AXIS_SMOOTHING_ALPHA 0.40f
#define AXIS_AMOUNT 6

#define AXIS_INPUT_MIN 0
#define AXIS_INPUT_MAX 4095
#define AXIS_INPUT_CENTER 2048

#define AXIS_OUTPUT_MIN -32767
#define AXIS_OUTPUT_MAX 32767

#define PIN_AXIS_ROLL 13
#define PIN_AXIS_PITCH 12
#define PIN_AXIS_THROTTLE 14
#define PIN_AXIS_YAW 27
#define PIN_AXIS_VRA 26
#define PIN_AXIS_VRB 25

#define AXIS_ROLL_INVERTED false
#define AXIS_PITCH_INVERTED true
#define AXIS_THROTTLE_INVERTED false
#define AXIS_YAW_INVERTED false
#define AXIS_VRA_INVERTED false
#define AXIS_VRB_INVERTED false

// Buttons
#define PIN_BUTTON_SWA 4
#define PIN_BUTTON_SWB 16
#define PIN_BUTTON_SWC 2
#define PIN_BUTTON_SWD 15

#define BUTTON_SWA_INVERTED false
#define BUTTON_SWB_INVERTED false
#define BUTTON_SWC_INVERTED true
#define BUTTON_SWD_INVERTED true

// Battery
#define PIN_BATTERY_LEVEL 34
#define BATTERY_VOLTAGE_MAX 4.2f
#define BATTERY_VOLTAGE_MIN 3.0f
#define BATTERY_ADC_REFERENCE 3.575f
#define BATTERY_ADC_MAX 4095.0f
#define BATTERY_DIVIDER_RATIO ((100.0f + 100.0f) / 100.0f) // 2 resistors of 100k ohms
#define BATTERY_AVG_AMOUNT 10
#define BATTERY_READ_INTERVAL 1000
#define BATTERY_READ_PERCENT_DEVIATION_MAX 5

unsigned long lastBatteryRead = 0;
float lastBatteryPercent = 0.0f;
float batteryPercents[BATTERY_AVG_AMOUNT];
bool batteryFirstReport = true;

// Gamepad
#define NUMBER_OF_BUTTONS 4

byte physicalButtons[NUMBER_OF_BUTTONS] = {1, 2, 3, 4};
bool statusChanged = false;

BleGamepad bleGamepad("Flysky i6X", "FlySky", 100);

void resetActivityTimer() { lastActivityTime = millis(); }

void setButton(int buttonIndex, bool pressed)
{
  if (!pressed)
  {
    bleGamepad.release(physicalButtons[buttonIndex]);
    statusChanged = true;
    return;
  }

  resetActivityTimer();
  bleGamepad.press(physicalButtons[buttonIndex]);
  statusChanged = true;
}

// Read buttons
void loadButtons()
{
  #ifdef DEBUG
    Serial.printf("Button States: SWA=%d, SWB=%d, SWC=%d, SWD=%d\n",
                  digitalRead(PIN_BUTTON_SWA), digitalRead(PIN_BUTTON_SWB),
                  digitalRead(PIN_BUTTON_SWC), digitalRead(PIN_BUTTON_SWD));
  #endif
  bool pressedSwa = (digitalRead(PIN_BUTTON_SWA) == LOW) != BUTTON_SWA_INVERTED;
  bool pressedSwb = (digitalRead(PIN_BUTTON_SWB) == LOW) != BUTTON_SWB_INVERTED;
  bool pressedSwc = (digitalRead(PIN_BUTTON_SWC) == LOW) != BUTTON_SWC_INVERTED;
  bool pressedSwd = (digitalRead(PIN_BUTTON_SWD) == LOW) != BUTTON_SWD_INVERTED;

  setButton(0, pressedSwa);
  setButton(1, pressedSwb);
  setButton(2, pressedSwc);
  setButton(3, pressedSwd);
}

// Axes handling
long lastAxisValue[AXIS_AMOUNT] = {0, 0, 0, 0, 0, 0};
float smoothedRawAxis[AXIS_AMOUNT] = {AXIS_INPUT_CENTER, AXIS_INPUT_CENTER, AXIS_INPUT_CENTER, AXIS_INPUT_CENTER, AXIS_INPUT_CENTER, AXIS_INPUT_CENTER};

int16_t mapAxis(int raw) {
  int delta = raw - AXIS_INPUT_CENTER;
  if (abs(delta) < AXIS_DEADZONE_RAW) return 0;

  return (int16_t) map(raw, AXIS_INPUT_MIN, AXIS_INPUT_MAX, AXIS_OUTPUT_MIN, AXIS_OUTPUT_MAX);
}

void loadAxes()
{
  smoothedRawAxis[0] = AXIS_SMOOTHING_ALPHA * analogRead(PIN_AXIS_ROLL)     + (1.0f - AXIS_SMOOTHING_ALPHA) * smoothedRawAxis[0];
  smoothedRawAxis[1] = AXIS_SMOOTHING_ALPHA * analogRead(PIN_AXIS_PITCH)    + (1.0f - AXIS_SMOOTHING_ALPHA) * smoothedRawAxis[1];
  smoothedRawAxis[2] = AXIS_SMOOTHING_ALPHA * analogRead(PIN_AXIS_THROTTLE) + (1.0f - AXIS_SMOOTHING_ALPHA) * smoothedRawAxis[2];
  smoothedRawAxis[3] = AXIS_SMOOTHING_ALPHA * analogRead(PIN_AXIS_YAW)      + (1.0f - AXIS_SMOOTHING_ALPHA) * smoothedRawAxis[3];
  smoothedRawAxis[4] = AXIS_SMOOTHING_ALPHA * analogRead(PIN_AXIS_VRA)      + (1.0f - AXIS_SMOOTHING_ALPHA) * smoothedRawAxis[4];
  smoothedRawAxis[5] = AXIS_SMOOTHING_ALPHA * analogRead(PIN_AXIS_VRB)      + (1.0f - AXIS_SMOOTHING_ALPHA) * smoothedRawAxis[5];

#ifdef DEBUG
  Serial.printf("Smoothed Axis Readings: ROLL=%d, PITCH=%d, THROTTLE=%d, YAW=%d, VRA=%d, VRB=%d\n",
                (int)smoothedRawAxis[0], (int)smoothedRawAxis[1], (int)smoothedRawAxis[2],
                (int)smoothedRawAxis[3], (int)smoothedRawAxis[4], (int)smoothedRawAxis[5]);
#endif
  int axisRollRaw = mapAxis((int)smoothedRawAxis[0]);
  int axisPitchRaw = mapAxis((int)smoothedRawAxis[1]);
  int axisThrottleRaw = mapAxis((int)smoothedRawAxis[2]);
  int axisYawRaw = mapAxis((int)smoothedRawAxis[3]);
  int axisVraRaw = mapAxis((int)smoothedRawAxis[4]);
  int axisVrbRaw = mapAxis((int)smoothedRawAxis[5]);

  int axisRoll = AXIS_ROLL_INVERTED ? (AXIS_OUTPUT_MAX + AXIS_OUTPUT_MIN - axisRollRaw) : axisRollRaw;
  int axisPitch = AXIS_PITCH_INVERTED ? (AXIS_OUTPUT_MAX + AXIS_OUTPUT_MIN - axisPitchRaw) : axisPitchRaw;
  int axisThrottle = AXIS_THROTTLE_INVERTED ? (AXIS_OUTPUT_MAX + AXIS_OUTPUT_MIN - axisThrottleRaw) : axisThrottleRaw;
  int axisYaw = AXIS_YAW_INVERTED ? (AXIS_OUTPUT_MAX + AXIS_OUTPUT_MIN - axisYawRaw) : axisYawRaw;
  int axisVra = AXIS_VRA_INVERTED ? (AXIS_OUTPUT_MAX + AXIS_OUTPUT_MIN - axisVraRaw) : axisVraRaw;
  int axisVrb = AXIS_VRB_INVERTED ? (AXIS_OUTPUT_MAX + AXIS_OUTPUT_MIN - axisVrbRaw) : axisVrbRaw;

    if (labs(axisRoll - lastAxisValue[0]) > AXIS_DEADZONE_MAPPED ||
      labs(axisPitch - lastAxisValue[1]) > AXIS_DEADZONE_MAPPED ||
      labs(axisThrottle - lastAxisValue[2]) > AXIS_DEADZONE_MAPPED ||
      labs(axisYaw - lastAxisValue[3]) > AXIS_DEADZONE_MAPPED ||
      labs(axisVra - lastAxisValue[4]) > AXIS_DEADZONE_MAPPED ||
      labs(axisVrb - lastAxisValue[5]) > AXIS_DEADZONE_MAPPED)
  {
    resetActivityTimer();
    bleGamepad.setAxes(
      axisRoll,
      axisPitch,
      axisThrottle,
      axisYaw,
      axisVra,
      axisVrb
    );
    statusChanged = true;

    lastAxisValue[0] = axisRoll;
    lastAxisValue[1] = axisPitch;
    lastAxisValue[2] = axisThrottle;
    lastAxisValue[3] = axisYaw;
    lastAxisValue[4] = axisVra;
    lastAxisValue[5] = axisVrb;
  }
}

float loadAverageBatteryPercent()
{
  if (batteryFirstReport)
  {
    for (int i = 0; i < BATTERY_AVG_AMOUNT; i++)
    {
      float percent = loadBatteryPercent();
      delayMicroseconds(100);
      batteryPercents[i] = percent;
    }

    lastBatteryPercent = batteryPercents[BATTERY_AVG_AMOUNT - 1];
    batteryFirstReport = false;
  }

  float percent = loadBatteryPercent();
  float sum = 0.0f;

  if (abs(percent - lastBatteryPercent) < BATTERY_READ_PERCENT_DEVIATION_MAX)
  {
    for (int i = 1; i < BATTERY_AVG_AMOUNT; i++)
      batteryPercents[i - 1] = batteryPercents[i];

    batteryPercents[BATTERY_AVG_AMOUNT - 1] = percent;
  }

  lastBatteryPercent = percent;

  for (int i = 0; i < BATTERY_AVG_AMOUNT; i++)
    sum += batteryPercents[i];

  return sum / BATTERY_AVG_AMOUNT;
}

float loadBatteryPercent()
{
  int adcValue = analogRead(PIN_BATTERY_LEVEL);

  float batteryVoltage = (adcValue / BATTERY_ADC_MAX) * BATTERY_ADC_REFERENCE * BATTERY_DIVIDER_RATIO;
  float percent = (batteryVoltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0f;

#ifdef DEBUG
  Serial.println("Calculated Voltage: " + String(batteryVoltage, 3));
#endif

  if (percent > 100.0f)
    percent = 100.0f;
  if (percent < 0.0f)
    percent = 0.0f;

  lastBatteryRead = millis();

  return percent;
}

void loadBatteryLevel()
{
  if (millis() - lastActivityTime > SLEEP_TIMEOUT_MS)
    enterDeepSleep();
  if (millis() - lastBatteryRead < BATTERY_READ_INTERVAL)
    return;

  float percent = loadAverageBatteryPercent();

  bleGamepad.setBatteryLevel((int)percent);
  statusChanged = true;

  if (percent <= 0.0f)
  {
    bleGamepad.sendReport();
    delay(500);
    enterDeepSleep();
  }
}

void sendReport()
{
  if (!statusChanged || !bleGamepad.isConnected())
    return;

  bleGamepad.sendReport();
  statusChanged = false;
}

void enterDeepSleep()
{
  const int buttonPins[] = {PIN_BUTTON_SWA, PIN_BUTTON_SWB, PIN_BUTTON_SWC, PIN_BUTTON_SWD};
  for (int i = 0; i < (int)(sizeof(buttonPins) / sizeof(buttonPins[0])); i++)
  {
    pinMode(buttonPins[i], OUTPUT);
    digitalWrite(buttonPins[i], HIGH);
    gpio_hold_en((gpio_num_t)buttonPins[i]);
  }

  uint64_t gpio_mask = 0;
  for (int i = 0; i < (int)(sizeof(buttonPins) / sizeof(buttonPins[0])); i++)
  {
    int pin = buttonPins[i];
    gpio_mask |= (1ULL << pin);
    rtc_gpio_pulldown_en((gpio_num_t)pin);
    rtc_gpio_pullup_dis((gpio_num_t)pin);
  }

  esp_sleep_enable_ext1_wakeup(gpio_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Set CPU Frequency
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  // Disable GPIO hold if woke up from deep sleep
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
  {
    const int buttonPins[] = {PIN_BUTTON_SWA, PIN_BUTTON_SWB, PIN_BUTTON_SWC, PIN_BUTTON_SWD};
    for (int i = 0; i < (int)(sizeof(buttonPins) / sizeof(buttonPins[0])); i++)
      gpio_hold_dis((gpio_num_t)buttonPins[i]);
  }

  // Setup Buttons
  pinMode(PIN_BUTTON_SWA, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SWB, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SWC, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SWD, INPUT_PULLUP);

  // Battery pin
  pinMode(PIN_BATTERY_LEVEL, INPUT);

  // Setup BLE Gamepad
  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  bleGamepadConfig.setButtonCount(NUMBER_OF_BUTTONS);
  bleGamepadConfig.setHatSwitchCount(0);
  bleGamepadConfig.setAxesMin(AXIS_OUTPUT_MIN);
  bleGamepadConfig.setAxesMax(AXIS_OUTPUT_MAX);
  bleGamepadConfig.setWhichAxes(true, true, true, true, true, true, false, false);

  bleGamepad.begin(&bleGamepadConfig);

  resetActivityTimer();
}

void loop()
{
  loadButtons();
  loadAxes();
  loadBatteryLevel();
  sendReport();

  delay(10);
#ifdef DEBUG
  delay(100);
#endif
}