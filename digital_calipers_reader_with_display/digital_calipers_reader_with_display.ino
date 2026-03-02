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
*/

/* Extended code from https://curiousscientist.tech/blog/caliper-based-dro 
* similar and more advanced code https://github.com/Aggebitter/TouchDRO-Simulator-on-ESP32 
* 3 axises. To use only 1 or 2, change values of AXIS_COUNT, axis_names and pins
* When an axes is not connectedit ismarked "N/A" on the display
*/
#include <cstdint>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
Adafruit_PCD8544 display = Adafruit_PCD8544(14, 12, 13, 33, 2); //pins description are above

unsigned long time_now;   // For storing the time when the clock signal is changed from HIGH to LOW (falling edge trigger of data output).

#define AXIS_COUNT 3
#define READ_MAX_CONT 5000
char axis_names[AXIS_COUNT] = {'X','Y','Z'};
const uint8_t CLOCK_PIN = 0, DATA_PIN = 1;
uint8_t pins[AXIS_COUNT][2] = {
  {21,32},//X
  {17,16},//Y
  {26,5} //Z
};

float distances[AXIS_COUNT];
uint8_t axisEnabled[AXIS_COUNT];
const uint8_t UNIT_MM = 0, UNIT_INCH = 1; 
uint8_t units[AXIS_COUNT];
const uint8_t SIGN_POSITIVE = 0, SIGN_NEGATIVE = 1; 
int bit_array[AXIS_COUNT][25];        // For storing the data bit. bit_array[axis][0] = data bit 1 (LSB), bit_array[axis][23] = data bit 24 (MSB).

void setup() {
  Serial.begin(9600);
  initDisplay();
  initPinsAndArrays();
  putMessageOnDisplay("Initializing");
  invalidateDisplay();
}

void loop() {
  clearDisplay();
  putValuesOnDisplay();
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    readDistanceAndUnit(axis);
  }
  //printDistancesAndUnits();
  invalidateDisplay();
  delay(100);
}

void delaymks(){
   unsigned long tn;
   tn = micros();
   while (micros() - tn < 30) {}
}

void readDistanceAndUnit(const uint8_t axis) {
  uint8_t clockPin = pins[axis][CLOCK_PIN];
  int readCount = 0;
  while (digitalRead(clockPin) == LOW) {
    delaymks();
    if (readCount++ > READ_MAX_CONT) {
      axisEnabled[axis] = 0;
      return;//break
    }
  }
  time_now = micros();
  readCount = 0;
  while (digitalRead(clockPin) == HIGH) {
    delaymks();
    if (readCount++ > READ_MAX_CONT) {
      axisEnabled[axis] = 0;
      return;//break
    }
  }
  axisEnabled[axis] = 1;
  if ((micros() - time_now) > 500) {        // If the HIGH pulse was longer than 500 micros we are at the start of a new bit sequence
    decode(axis); //decode the bit sequence
  }
  //delay(10); //Optional delay, only needed for testing purpose, when pins[AXIS_COUNT][2] popilated with the same pins for different axises, like: pins[AXIS_COUNT][2]{21,32},{21,32},{21,32} //X,Y,Z
}

void decode(const uint8_t axis) {
  populateBitArray(axis);
  setDistanceAndUnitFromBitArray(axis);
  //printBitArray(axis);// only needed for testing purpose
}

void populateBitArray(const uint8_t axis) {
  uint8_t dataPin = pins[axis][DATA_PIN], clockPin = pins[axis][CLOCK_PIN];
  bit_array[axis][0] = digitalRead(dataPin); // Store the 1st bit (start bit) which is always 1.
  int readCount = 0;
  while (digitalRead(clockPin) == HIGH) {     
    delaymks();
    if (readCount++ > READ_MAX_CONT) {
        axisEnabled[axis] = 0;
        return;//break
    }
  };
  for (uint8_t i = 1; i <= 24; i++) {
    readCount = 0;
    while (digitalRead(clockPin) == LOW) {
      delaymks();
      if (readCount++ > READ_MAX_CONT) {
        axisEnabled[axis] = 0;
        return;//break
      }
    } // Wait until clock returns to HIGH
    bit_array[axis][i] = digitalRead(dataPin);
    readCount = 0;  
    while (digitalRead(clockPin) == HIGH) {
      delaymks();
      if (readCount++ > READ_MAX_CONT) {
        axisEnabled[axis] = 0;
        return;//break
      }
    } // Wait until clock returns to LOW
  }
  axisEnabled[axis] = 1;
}

void setDistanceAndUnitFromBitArray(const uint8_t  axis) {
  units[axis] = bit_array[axis][24];  // Bit 24 tells the measuring unit (1 -> in, 0 -> mm)
  distances[axis] = getSign(axis) * getValueFromBitArray(axis) / (units[axis] == UNIT_INCH? 2000.00: 100.00);
}

float getValueFromBitArray(const uint8_t axis) {
  float value = 0.0;
  for (uint8_t i = 1; i <= 20; i++) {
      value = value + (pow(2, i-1) * bit_array[axis][i]);// Turning the value in the bit array from binary to decimal.
  }
  return value;
}

// Bit 21 is the sign bit. 0 -> +, 1 => -
int getSign(const uint8_t axis) {
  return bit_array[axis][21] == SIGN_NEGATIVE ? -1: 1;
}

void printDistancesAndUnits() {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    Serial.print(axis_names[axis]);
    Serial.print(":");
    if (axisEnabled[axis] != 1) {
      Serial.print(" N/A");
    } else if (units[axis] == UNIT_INCH) {
      Serial.print(distances[axis], 3); // Print result with 3 decimals
      Serial.print(" in;");
    } else {
      Serial.print(distances[axis], 2); // Print result with 2 decimals
      Serial.print(" mm;");
    }
  }
  Serial.println();
}

void printBitArray(const uint8_t axis) {
  if (axis <= 0) {
    return;
  }
  Serial.print(axis_names[axis]);
  Serial.print(":");
  for (uint8_t i = 0; i <= 24; i++) {
    Serial.print(bit_array[axis][i]);
    Serial.print(" ");
  }
  Serial.println();
}

void initPinsAndArrays() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    pinMode(pins[axis][CLOCK_PIN], INPUT);
    pinMode(pins[axis][DATA_PIN], INPUT);
    distances[axis] = 0.0;
    axisEnabled[axis] = 0;
    units[axis] = UNIT_MM;
    for (uint8_t i = 0; i < 25; i++) {
      bit_array[axis][i] = 0;
    }
  }
}

void putValuesOnDisplay() {
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    putOnDisplay(String(axis_names[axis]) + ":" + (axisEnabled[axis] != 1 
                                                  ? "N/A" 
                                                  : String(distances[axis]) + ((units[axis] == UNIT_INCH)?" in":" mm")),
                1, 0, axis*10);
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
  display.clearDisplay();   // clears the screen and buffer
}

void initDisplay() {
  display.begin();  // init done
  display.setContrast(60); // change the contrast for the best viewing!
  delay(1000);
  clearDisplay();
}