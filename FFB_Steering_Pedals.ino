/*
 * PICO 1 — FFB Steering + Pedals
 * Encoder: Configurable PPR on GP10/GP11 (5V via voltage divider)
 * Motor:   BTS7960/IBT-2 on GP2/GP3
 * Pedals:  Clutch=A0, Brake=A1, Accel=A2
 * HID:     TinyUSB Gamepad — 4 axes @ 250 Hz
 */

#include "Adafruit_TinyUSB.h"
#include <Arduino.h>

// ===== HID DESCRIPTOR =====
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;

// ===== PINS =====
const uint8_t PIN_ENC_A  = 10;
const uint8_t PIN_ENC_B  = 11;
const uint8_t RPWM       = 2;
const uint8_t LPWM       = 3;
const uint8_t PIN_CLUTCH = A0;
const uint8_t PIN_BRAKE  = A1;
const uint8_t PIN_ACCEL  = A2;

// ===== ENCODER CONFIG (edit these two to match your hardware) =====
// PPR = Pulses Per Revolution, printed on the encoder datasheet/label
// (e.g. 100, 200, 360, 600, 1000, 2000...)
const uint16_t encoderPPR              = 1000;   // <-- change this per encoder
const float    steeringDegreesOfRotation = 300.0f; // lock-to-lock rotation you want

// Quadrature decoding counts 4 edges per pulse (x4 mode), so:
const long countsPerRev  = (long)encoderPPR * 4;
const long maxSteerCounts = (long)(countsPerRev * (steeringDegreesOfRotation / 360.0f));

float Kp             = 0.5f;
float Kd             = 0.0003f;
int   minPWM         = 20;
int   maxPWM         = 55;
int   deadband       = 3;
float brakeForceMax  = 5.0f;
float accelBias      = 3.0f;
int   pedalDeadband  = 50;


volatile long    encCount = 0;
volatile uint8_t lastAB   = 0;

const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void encoderISR() {
  uint8_t AB  = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  uint8_t idx = (lastAB << 2) | AB;
  encCount += QEM[idx];
  lastAB   = AB;
}


void motorStop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void motorDrive(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, pwm);
  } else if (pwm < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -pwm);
  } else {
    motorStop();
  }
}

// ===== PEDALS =====
int readPedal(uint8_t pin) {
  int raw = analogRead(pin);
  if (raw < pedalDeadband) return 0;
  return map(raw, pedalDeadband, 4095, 0, 1023);
}

inline int8_t pedalToAxis(int val) {
  return (int8_t)map(val, 0, 1023, -127, 127);
}


float   lastError = 0.0f;
int32_t lastFFBus = 0;

void updateFFB() {
  int32_t now   = micros();
  int32_t dt_us = now - lastFFBus;
  if (dt_us < 50) return;
  lastFFBus = now;

  float dt     = dt_us * 1e-6f;
  float error  = (float)(-encCount);
  float dError = (error - lastError) / dt;
  lastError    = error;

  float brakeVal = readPedal(PIN_BRAKE) / 1023.0f;
  float accelVal = readPedal(PIN_ACCEL) / 1023.0f;

  float output = Kp * error
               + Kd * dError
               + brakeVal * brakeForceMax
               - accelVal * accelBias;

  if (fabsf(output) < deadband) { motorStop(); return; }

  int sign   = (output > 0) ? 1 : -1;
  int pwmMag = (int)constrain(fabsf(output), minPWM, maxPWM);
  motorDrive(sign * pwmMag);
}


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

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  lastAB = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  motorStop();

  analogReadResolution(12);
  memset(&gp, 0, sizeof(gp));
  lastFFBus = micros();
}

// ===== LOOP =====
void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  if (!TinyUSBDevice.mounted()) return;

  updateFFB();

  static uint32_t lastHID = 0;
  if (millis() - lastHID >= 4) {
    lastHID = millis();

    if (!usb_hid.ready()) return;

    long cnt = constrain(encCount, -maxSteerCounts, maxSteerCounts);
    gp.x     = (int8_t)map(cnt, -maxSteerCounts, maxSteerCounts, -127, 127);

    gp.y  = pedalToAxis(readPedal(PIN_CLUTCH));
    gp.z  = pedalToAxis(readPedal(PIN_BRAKE));
    gp.rz = pedalToAxis(readPedal(PIN_ACCEL));

    gp.rx      = 0;
    gp.ry      = 0;
    gp.hat     = 0;
    gp.buttons = 0;

    usb_hid.sendReport(0, &gp, sizeof(gp));
  }
}