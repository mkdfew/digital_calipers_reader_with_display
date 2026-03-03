/*
Copyright © 2026, GitHub.com/mkdfew
License: MIT

ESP32 digital caliper for 3 axises and the display Nokia 5110

ESP32 | Display Nokia 5110
  pin 14 - Serial clock out: CLK (SCLK)
  pin 12 - Serial data out: DIN
  pin 13 - Data/Command select: DC (D/C)
  pin 33 - LCD chip select: CE (CS)
  pin 02 - LCD reset: RST
  5V    - VCC
  GND   - GND

Buttons:
- Three momentary buttons, one per axis (X/Y/Z)
- Wire each button between the ESP32 GPIO pin and GND
- Uses INPUT_PULLUP (active LOW)
- On press: stores current raw distance as the new zero offset for that axis
- Displayed value = filtered(raw) - filtered(offset reference)
*/

#include <cstdint>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

Adafruit_PCD8544 display = Adafruit_PCD8544(14, 12, 13, 33, 2);

unsigned long time_now;   // For storing the time when the clock signal is changed from HIGH to LOW

#define AXIS_COUNT 3
#define READ_MAX_CONT 5000

char axis_names[AXIS_COUNT] = {'X','Y','Z'};
const uint8_t CLOCK_PIN = 0, DATA_PIN = 1;

uint8_t pins[AXIS_COUNT][2] = {
  {21,32},//X
  {17,16},//Y
  {26,5}  //Z
};

// Button GPIOs (change to your preferred free pins)
const uint8_t BTN_PINS[AXIS_COUNT] = {
  25, // X zero button
  27, // Y zero button
  4   // Z zero button
};

const uint32_t DEBOUNCE_MS = 35;

// Filtering: moving average over ~0.5 s
// With loop delay(100), 0.5 s ~ 5 samples
static const uint8_t AVG_SAMPLES = 5;

float distances[AXIS_COUNT];        // Raw distances from calipers
float zeroOffsets[AXIS_COUNT];      // Stored zero offsets (raw distance at "zero" moment)

// Averaging buffers for raw distance
float avgBuf[AXIS_COUNT][AVG_SAMPLES];
uint8_t avgIdx[AXIS_COUNT];
uint8_t avgCount[AXIS_COUNT];
float avgSum[AXIS_COUNT];

// Averaging buffers for zero offset reference (so zero is stable too)
float offBuf[AXIS_COUNT][AVG_SAMPLES];
uint8_t offIdx[AXIS_COUNT];
uint8_t offCount[AXIS_COUNT];
float offSum[AXIS_COUNT];

float shownDistances[AXIS_COUNT];   // Displayed distances = avg(raw) - avg(offset)

uint8_t axisEnabled[AXIS_COUNT];
const uint8_t UNIT_MM = 0, UNIT_INCH = 1;
uint8_t units[AXIS_COUNT];

const uint8_t SIGN_POSITIVE = 0, SIGN_NEGATIVE = 1;
int bit_array[AXIS_COUNT][25];

// Button state for debouncing + edge detection
uint8_t  btnLastStable[AXIS_COUNT];
uint8_t  btnLastRead[AXIS_COUNT];
uint32_t btnLastChangeMs[AXIS_COUNT];

void setup() {
  Serial.begin(9600);
  initDisplay();
  initPinsAndArrays();
  initButtons();

  putMessageOnDisplay("Initializing");
  invalidateDisplay();
}

void loop() {
  // Read calipers first (so button press uses most recent value)
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    readDistanceAndUnit(axis);
    if (axisEnabled[axis] == 1) {
      pushAvgSample(axis, distances[axis]);
    }
  }

  // Handle zero buttons
  pollButtons();

  // Compute displayed distances (avg(raw) - avg(offset))
  updateShownDistances();

  // Update display
  clearDisplay();
  putValuesOnDisplay();
  invalidateDisplay();

  delay(100);
}

void delaymks() {
  unsigned long tn = micros();
  while (micros() - tn < 30) {}
}

void readDistanceAndUnit(const uint8_t axis) {
  uint8_t clockPin = pins[axis][CLOCK_PIN];
  int readCount = 0;

  while (digitalRead(clockPin) == LOW) {
    delaymks();
    if (readCount++ > READ_MAX_CONT) {
      axisEnabled[axis] = 0;
      return;
    }
  }

  time_now = micros();
  readCount = 0;

  while (digitalRead(clockPin) == HIGH) {
    delaymks();
    if (readCount++ > READ_MAX_CONT) {
      axisEnabled[axis] = 0;
      return;
    }
  }

  axisEnabled[axis] = 1;

  // Long HIGH pulse indicates start of a new bit sequence
  if ((micros() - time_now) > 500) {
    decode(axis);
  }
}

void decode(const uint8_t axis) {
  populateBitArray(axis);
  if (axisEnabled[axis] == 1) {
    setDistanceAndUnitFromBitArray(axis);
  }
}

void populateBitArray(const uint8_t axis) {
  uint8_t dataPin = pins[axis][DATA_PIN], clockPin = pins[axis][CLOCK_PIN];
  bit_array[axis][0] = digitalRead(dataPin);

  int readCount = 0;
  while (digitalRead(clockPin) == HIGH) {
    delaymks();
    if (readCount++ > READ_MAX_CONT) {
      axisEnabled[axis] = 0;
      return;
    }
  }

  for (uint8_t i = 1; i <= 24; i++) {
    readCount = 0;
    while (digitalRead(clockPin) == LOW) {
      delaymks();
      if (readCount++ > READ_MAX_CONT) {
        axisEnabled[axis] = 0;
        return;
      }
    }

    bit_array[axis][i] = digitalRead(dataPin);

    readCount = 0;
    while (digitalRead(clockPin) == HIGH) {
      delaymks();
      if (readCount++ > READ_MAX_CONT) {
        axisEnabled[axis] = 0;
        return;
      }
    }
  }

  axisEnabled[axis] = 1;
}

void setDistanceAndUnitFromBitArray(const uint8_t axis) {
  units[axis] = bit_array[axis][24];
  distances[axis] = getSign(axis) * getValueFromBitArray(axis) /
                    (units[axis] == UNIT_INCH ? 2000.00 : 100.00);
}

float getValueFromBitArray(const uint8_t axis) {
  float value = 0.0;
  for (uint8_t i = 1; i <= 20; i++) {
    value = value + (pow(2, i - 1) * bit_array[axis][i]);
  }
  return value;
}

int getSign(const uint8_t axis) {
  return bit_array[axis][21] == SIGN_NEGATIVE ? -1 : 1;
}

// ---------------- Averaging (0.5 s moving average) ----------------

void resetAveragers(uint8_t axis) {
  avgIdx[axis] = 0;
  avgCount[axis] = 0;
  avgSum[axis] = 0.0f;
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) avgBuf[axis][i] = 0.0f;

  offIdx[axis] = 0;
  offCount[axis] = 0;
  offSum[axis] = 0.0f;
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) offBuf[axis][i] = 0.0f;
}

void pushAvgSample(uint8_t axis, float v) {
  if (avgCount[axis] < AVG_SAMPLES) {
    avgBuf[axis][avgIdx[axis]] = v;
    avgSum[axis] += v;
    avgCount[axis]++;
    avgIdx[axis] = (avgIdx[axis] + 1) % AVG_SAMPLES;
    return;
  }

  // Replace oldest sample
  float old = avgBuf[axis][avgIdx[axis]];
  avgSum[axis] -= old;
  avgBuf[axis][avgIdx[axis]] = v;
  avgSum[axis] += v;
  avgIdx[axis] = (avgIdx[axis] + 1) % AVG_SAMPLES;
}

float getAvgValue(uint8_t axis) {
  if (avgCount[axis] == 0) return 0.0f;
  return avgSum[axis] / (float)avgCount[axis];
}

void setOffsetAveragerToCurrent(uint8_t axis, float v) {
  // Fill the offset averager with the same value so it becomes stable immediately
  offIdx[axis] = 0;
  offCount[axis] = AVG_SAMPLES;
  offSum[axis] = v * (float)AVG_SAMPLES;
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) offBuf[axis][i] = v;
}

float getAvgOffset(uint8_t axis) {
  if (offCount[axis] == 0) return zeroOffsets[axis];
  return offSum[axis] / (float)offCount[axis];
}

// ---------------- Buttons (per-axis zero) ----------------

void initButtons() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    pinMode(BTN_PINS[axis], INPUT_PULLUP);

    uint8_t s = digitalRead(BTN_PINS[axis]);
    btnLastStable[axis] = s;
    btnLastRead[axis] = s;
    btnLastChangeMs[axis] = millis();
  }
}

void pollButtons() {
  uint32_t now = millis();

  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    uint8_t r = digitalRead(BTN_PINS[axis]);

    if (r != btnLastRead[axis]) {
      btnLastRead[axis] = r;
      btnLastChangeMs[axis] = now;
    }

    if ((now - btnLastChangeMs[axis]) >= DEBOUNCE_MS) {
      if (btnLastStable[axis] != btnLastRead[axis]) {
        btnLastStable[axis] = btnLastRead[axis];

        // Active LOW: pressed when becomes LOW
        if (btnLastStable[axis] == LOW) {
          zeroAxis(axis);
        }
      }
    }
  }
}

void zeroAxis(uint8_t axis) {
  // Only zero if this axis currently has valid data
  if (axisEnabled[axis] != 1) return;

  // Store raw distance as the new zero reference
  zeroOffsets[axis] = distances[axis];

  // Make the offset stable immediately by filling the offset averager
  setOffsetAveragerToCurrent(axis, zeroOffsets[axis]);

  Serial.print("Zero set for axis ");
  Serial.print(axis_names[axis]);
  Serial.print(" at ");
  Serial.println(zeroOffsets[axis], 4);
}

void updateShownDistances() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    if (axisEnabled[axis] == 1) {
      float rawAvg = getAvgValue(axis);
      float offAvg = getAvgOffset(axis);
      shownDistances[axis] = rawAvg - offAvg;
    } else {
      shownDistances[axis] = 0.0f;
    }
  }
}

// ---------------- Display / Init ----------------

void initPinsAndArrays() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    pinMode(pins[axis][CLOCK_PIN], INPUT);
    pinMode(pins[axis][DATA_PIN], INPUT);

    distances[axis] = 0.0f;
    zeroOffsets[axis] = 0.0f;
    shownDistances[axis] = 0.0f;

    axisEnabled[axis] = 0;
    units[axis] = UNIT_MM;

    resetAveragers(axis);

    for (uint8_t i = 0; i < 25; i++) {
      bit_array[axis][i] = 0;
    }
  }
}

void putValuesOnDisplay() {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    String line = String(axis_names[axis]) + ":";

    if (axisEnabled[axis] != 1) {
      line += "N/A";
    } else {
      // Keep formatting stable (mm: 2 decimals, inch: 3 decimals)
      if (units[axis] == UNIT_INCH) {
        line += String(shownDistances[axis], 3) + " in";
      } else {
        line += String(shownDistances[axis], 2) + " mm";
      }
    }

    putOnDisplay(line, 1, 0, axis * 10);
  }
}

void putMessageOnDisplay(String text) {
  putOnDisplay(text, 1, 0, 20);
}

void putOnDisplay(String text, int textSize, int posX, int posY) {
  display.setTextSize(textSize);
  display.setTextColor(BLACK);
  display.setCursor(posX, posY);
  display.print(text);
}

void invalidateDisplay() {
  display.display();
}

void clearDisplay() {
  display.clearDisplay();
}

void initDisplay() {
  display.begin();
  display.setContrast(60);
  delay(1000);
  clearDisplay();
}
