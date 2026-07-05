/*
 * PICO 2 — F1 Wheel Buttons (TinyUSB)
 * 3x pot → 12-way rotary (3D printed ratchet)
 * KY-040 → 12-way rotary + SW button
 * 8x physical buttons + 2x paddle shifters (10 total)
 * 16x2 I2C LCD status display
 *
 * HID button map:
 *   0  - 11  → Pot1  position 1-12
 *   12 - 23  → Pot2  position 1-12
 *   24 - 35  → Pot3  position 1-12
 *   36 - 47  → KY-040 encoder position 1-12
 *   48       → KY-040 SW push
 *   49 - 56  → Physical buttons 1-8
 *   57       → Paddle shifter Left  (Downshift)
 *   58       → Paddle shifter Right (Upshift)
 *   (59 total — padded to 64 for byte alignment)
 */

#include "Adafruit_TinyUSB.h"
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== CONFIG =====
#define POT_POSITIONS  12
#define ENC_POSITIONS  12
#define LCD_ADDR       0x27
#define NUM_PHYS_BTN   8      // regular physical buttons
#define NUM_PADDLES    2      // gear shifter paddles
#define TOTAL_BUTTONS  59     // 12*3 + 12 + 1 + 8 + 2
#define REPORT_BUTTONS 64     // padded to 8-byte boundary

// ===== CUSTOM HID DESCRIPTOR — 64 buttons, no axes =====
uint8_t const desc_hid_report[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x05,        // Usage (Gamepad)
  0xA1, 0x01,        // Collection (Application)

  // 64 buttons
  0x05, 0x09,        //   Usage Page (Button)
  0x19, 0x01,        //   Usage Minimum (Button 1)
  0x29, 0x40,        //   Usage Maximum (Button 64)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x75, 0x01,        //   Report Size (1)
  0x95, 0x40,        //   Report Count (64)
  0x81, 0x02,        //   Input (Data, Variable, Absolute)

  0xC0               // End Collection
};

// ===== HID REPORT =====
// 64 buttons = 8 bytes
struct __attribute__((packed)) WheelReport {
  uint8_t buttons[8];  // bit-packed, button N → byte N/8 bit N%8
};

Adafruit_USBD_HID usb_hid;
WheelReport report;

void setReportButton(int n, bool val) {
  if (n < 0 || n >= REPORT_BUTTONS) return;
  if (val) report.buttons[n / 8] |=  (1 << (n % 8));
  else     report.buttons[n / 8] &= ~(1 << (n % 8));
}

// ===== PINS =====
const uint8_t PIN_ENC_A   = 10;
const uint8_t PIN_ENC_B   = 11;
const uint8_t PIN_ENC_SW  = 12;
const uint8_t PIN_POT[3]  = {A0, A1, A2};
const uint8_t BUTTON_PINS[NUM_PHYS_BTN] = {2, 3, 13, 14, 15, 16, 17, 18};

// Paddle shifters (behind the wheel — left = downshift, right = upshift)
const uint8_t PADDLE_PINS[NUM_PADDLES] = {19, 20};  // GP19 = left/down, GP20 = right/up

// ===== TUNING =====
const float    POT_ALPHA      = 0.20f;
const int      POT_DEADBAND   = 60;
const float    POT_HYSTERESIS = 0.35f;
const uint32_t DEBOUNCE_MS    = 20;

// ===== LCD =====
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ===== ENCODER =====
volatile int     encRaw = 0;
volatile uint8_t lastAB = 0;

const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void encoderISR() {
  uint8_t AB  = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  uint8_t idx = (lastAB << 2) | AB;
  encRaw += QEM[idx];
  lastAB  = AB;
}

int encPosition = 0;

int getEncPos() {
  int steps = encRaw / 2;
  return ((steps % ENC_POSITIONS) + ENC_POSITIONS) % ENC_POSITIONS;
}

// ===== POTS =====
float potSmooth[3]   = {0, 0, 0};
int   potPosition[3] = {0, 0, 0};

int adcToPosition(float smoothVal, int currentPos, int positions) {
  float mapped = constrain(
    (smoothVal - POT_DEADBAND) / (4095.0f - 2.0f * POT_DEADBAND) * 1023.0f,
    0.0f, 1023.0f
  );
  float segWidth   = 1024.0f / positions;
  float hyst       = segWidth * POT_HYSTERESIS;
  float currCentre = (currentPos + 0.5f) * segWidth;
  float dist       = mapped - currCentre;

  if (dist >  (segWidth * 0.5f + hyst)) return min(currentPos + 1, positions - 1);
  if (dist < -(segWidth * 0.5f + hyst)) return max(currentPos - 1, 0);
  return currentPos;
}

int potBase(int i) { return i * POT_POSITIONS; }

void updatePot(int i) {
  float raw    = analogRead(PIN_POT[i]);
  potSmooth[i] = POT_ALPHA * raw + (1.0f - POT_ALPHA) * potSmooth[i];
  int newPos   = adcToPosition(potSmooth[i], potPosition[i], POT_POSITIONS);
  if (newPos != potPosition[i]) {
    setReportButton(potBase(i) + potPosition[i], false);
    setReportButton(potBase(i) + newPos,         true);
    potPosition[i] = newPos;
  }
}

// ===== BUTTONS (physical buttons + KY-040 SW) =====
// Index layout in these debounce arrays: 0-7 = physical buttons, 8 = KY-040 SW
uint8_t  btnState[9]    = {};
uint8_t  btnLastRaw[9]  = {};
uint32_t btnDebounce[9] = {};

// ===== PADDLE SHIFTERS =====
uint8_t  paddleState[NUM_PADDLES]    = {};
uint8_t  paddleLastRaw[NUM_PADDLES]  = {};
uint32_t paddleDebounce[NUM_PADDLES] = {};

int physBase()   { return POT_POSITIONS * 3 + ENC_POSITIONS + 1; }               // 49
int paddleBase() { return physBase() + NUM_PHYS_BTN; }                            // 57

void updateButtons() {
  for (int i = 0; i < NUM_PHYS_BTN; i++) {
    uint8_t raw = !digitalRead(BUTTON_PINS[i]);
    if (raw != btnLastRaw[i]) { btnDebounce[i] = millis(); btnLastRaw[i] = raw; }
    if ((millis() - btnDebounce[i]) >= DEBOUNCE_MS && raw != btnState[i]) {
      btnState[i] = raw;
      setReportButton(physBase() + i, raw);
    }
  }
  // KY-040 SW → button 48
  uint8_t sw = !digitalRead(PIN_ENC_SW);
  if (sw != btnLastRaw[8]) { btnDebounce[8] = millis(); btnLastRaw[8] = sw; }
  if ((millis() - btnDebounce[8]) >= DEBOUNCE_MS && sw != btnState[8]) {
    btnState[8] = sw;
    setReportButton(POT_POSITIONS * 3 + ENC_POSITIONS, sw);
  }
}

void updatePaddles() {
  for (int i = 0; i < NUM_PADDLES; i++) {
    uint8_t raw = !digitalRead(PADDLE_PINS[i]);
    if (raw != paddleLastRaw[i]) { paddleDebounce[i] = millis(); paddleLastRaw[i] = raw; }
    if ((millis() - paddleDebounce[i]) >= DEBOUNCE_MS && raw != paddleState[i]) {
      paddleState[i] = raw;
      setReportButton(paddleBase() + i, raw);
    }
  }
}

// ===== LCD =====
void updateLCD() {
  lcd.setCursor(0, 0);
  for (int i = 0; i < 3; i++) {
    lcd.print("P"); lcd.print(i + 1); lcd.print(":");
    if (potPosition[i] + 1 < 10) lcd.print('0');
    lcd.print(potPosition[i] + 1);
    if (i < 2) lcd.print(' ');
  }
  lcd.setCursor(0, 1);
  lcd.print("EC:");
  if (encPosition + 1 < 10) lcd.print('0');
  lcd.print(encPosition + 1);
  lcd.print(" SW:");
  lcd.print(btnState[8] ? '*' : '-');
  lcd.print(paddleState[0] ? '<' : '-');  // downshift indicator
  lcd.print(paddleState[1] ? '>' : '-');  // upshift indicator
}

// ===== SETUP =====
void setup() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("  F1 WHEEL v2.1 ");
  lcd.setCursor(0, 1); lcd.print(" Initialising...");
  delay(1200);
  lcd.clear();

  pinMode(PIN_ENC_A,  INPUT_PULLUP);
  pinMode(PIN_ENC_B,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  lastAB = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  for (int i = 0; i < NUM_PHYS_BTN; i++) pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  for (int i = 0; i < NUM_PADDLES;  i++) pinMode(PADDLE_PINS[i], INPUT_PULLUP);

  analogReadResolution(12);


  memset(&report, 0, sizeof(report));

  
  for (int i = 0; i < 3; i++) {
    potSmooth[i]   = analogRead(PIN_POT[i]);
    potPosition[i] = adcToPosition(potSmooth[i], 0, POT_POSITIONS);
    setReportButton(potBase(i) + potPosition[i], true);
  }

  setReportButton(POT_POSITIONS * 3 + encPosition, true);
}


void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  if (!TinyUSBDevice.mounted()) return;

  // HID @ 1 kHz
  static uint32_t lastHID = 0;
  if (millis() - lastHID >= 1) {
    lastHID = millis();

    // Encoder rotary
    int pos = getEncPos();
    if (pos != encPosition) {
      int encBase = POT_POSITIONS * 3;
      setReportButton(encBase + encPosition, false);
      setReportButton(encBase + pos,         true);
      encPosition = pos;
    }

    for (int i = 0; i < 3; i++) updatePot(i);
    updateButtons();
    updatePaddles();

    if (usb_hid.ready()) {
      usb_hid.sendReport(0, &report, sizeof(report));
    }
  }

  // LCD @ 10 Hz
  static uint32_t lastLCD = 0;
  if (millis() - lastLCD >= 100) {
    lastLCD = millis();
    updateLCD();
  }
}
