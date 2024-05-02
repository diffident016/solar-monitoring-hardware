#include <Keypad.h>
#include <Wire.h>
#include <LedControl.h>

int DIN = 12;
int CS = 11;
int CLK = 10;
LedControl lc = LedControl(DIN, CLK, CS, 0);

#define I2C_SLAVE_ADDRESS 11
#define TILT1 A6
#define TILT2 A7
#define LED_R A1
#define LED_G A2
#define LED_B A0

const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 2, 3, 4, 5 };
byte colPins[COLS] = { 6, 7, 8, 9 };

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

int CMD = 0;
int LED = 0;

int smile1[8] = { B00111100, B01000010, B10010101, B10100001, B10100001, B10010101, B01000010, B00111100 };
int smile2[8] = { B00111100, B01000010, B10010101, B10010001, B10010001, B10010101, B01000010, B00111100 };
int smile3[8] = { B00111100, B01000010, B10100101, B10010001, B10010001, B10100101, B01000010, B00111100 };
int C[8] = { B00111110, B01000001, B01000001, B00100010, B00000000, B00000000, B00000000, B00000000 };
int A[8] = { B01111110, B00010001, B00010001, B01111110, B00000000, B00000000, B00000000, B00000000 };
int R[8] = { B01111111, B00001001, B00001001, B01110110, B00000000, B00000000, B00000000, B00000000 };
int L[8] = { B01111111, B01000000, B01000000, B01000000, B00000000, B00000000, B00000000, B00000000 };

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  Serial.begin(9600);
  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);

  pinMode(TILT1, INPUT);
  pinMode(TILT2, INPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  lc.shutdown(0, false);
  lc.setIntensity(0, 0);
  lc.clearDisplay(0);
}

void loop() {
  if (CMD == 32) {
    for (int i = 0; i < 8; i++) lc.setRow(0, i, smile1[i]);
    delay(2000);
    lc.clearDisplay(0);
    for (int i = 0; i < 8; i++) lc.setRow(0, i, smile2[i]);
    delay(2000);
    for (int i = 0; i < 8; i++) lc.setRow(0, i, smile3[i]);
    delay(2000);
    for (int i = 0; i < 8; i++) lc.setRow(0, i, C[i]);
    delay(2000);
    for (int i = 0; i < 8; i++) lc.setRow(0, i, A[i]);
    delay(2000);
    for (int i = 0; i < 8; i++) lc.setRow(0, i, R[i]);
    delay(2000);
    for (int i = 0; i < 8; i++) lc.setRow(0, i, L[i]);
    delay(2000);
    lc.clearDisplay(0);
  }
}

void requestEvents() {

  if (CMD == 30) {
    char customKey = customKeypad.getKey();

    if (customKey) {
      Serial.println(customKey);
      Wire.write(customKey);
    } else {
      Wire.write(-1);
    }
  } else if (CMD == 31) {
    int t1 = analogRead(TILT1);
    int t2 = analogRead(TILT2);
    int tilt = 0;

    if (t1 < 100) {
      tilt = 1;
    } else if (t2 < 100) {
      tilt = 2;
    } else {
      tilt = 0;
    }

    Wire.write(tilt);

    if (LED == 1) {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_B, LOW);
    } else if (LED == 2) {
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_B, HIGH);
    } else {
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_B, LOW);
    }
  } 
}

void receiveEvents(int numBytes) {
  int n = Wire.read();

  Serial.println(n);
  if (n == 30) {
    CMD = 30;
  } else if (n == 31) {
    CMD = 31;
  } else if (n == 32){
    CMD = 32;
  }else{
    CMD = 33;
  }

  if (CMD == 31) {
    LED = n;
  }
}
