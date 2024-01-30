#include <Arduino.h>
#include <SoftwareSerial.h>

#define RX        6 // RS485 Receive pin
#define TX        7 // RS485 Transmit pin
#define DE        8 // RS485 direction control pin

// RS485 direction control flags
#define RS485Transmit HIGH
#define RS485Receive LOW

SoftwareSerial RS485(RX, TX);

int analogPin = A3;

byte nothing[11] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x4A};
byte forward[11] = {0x01, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0x8A};
// ... (other byte arrays)

void setup() {
  Serial.begin(9600);
  pinMode(DE, OUTPUT);
  digitalWrite(DE, RS485Transmit);
  RS485.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
      while (1) {
      for (int i = 0; i < 11; i++) {
        RS485.write(forward[i]);
      }
      if (Serial.available() < 2) {
        break;
      } 
    }
  }
  else {
      for (int i = 0; i < 11; i++) {
        RS485.write(nothing[i]);
      }
    }
}
