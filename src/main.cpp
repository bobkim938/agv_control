#include <Arduino.h>
#include <SoftwareSerial.h>

#define RX		6 // RS485 Receive pin
#define TX		7 // RS485 Transmit pin
#define DE    8 // RS485 direction control pin

// RS485 direction control flags
#define RS485Transmit HIGH
#define RS485Receive LOW

SoftwareSerial RS485(RX, TX);

int analogPin = A3;

byte nothing[11] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x4A}; 
byte forward[11] = {0x01, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0x8A};
byte backward[11] = {0x01, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA4, 0x8A};
byte CCW[11] = {0x01, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0x8A};
byte CW[11] = {0x01, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x8A};
byte slower[11] = {0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x8B};
byte faster[11] = {0x01, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6, 0x88};
byte go_left[11] = {0x01, 0x06, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x76, 0x8A};
byte go_right[11] = {0x01, 0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8A};

void setup() {
  Serial.begin(9600);
  pinMode(DE, OUTPUT); 
  digitalWrite(DE, RS485Transmit);
  RS485.begin(9600);
}

void loop() {
  // int sensor_val = analogRead(analogPin); // 0 - 1023
  // Serial.println(sensor_val);
  // delay(100);

  Serial.println("Type 1 if you want to move the robot forward");
  int input = Serial.parseInt();
  if (input == 1) {
    Serial.println("Moving the robot forward");
    while(1) {
      for (int i = 0; i < 11; i++) {
        Serial.print("Sending: ");
        Serial.println(forward[i]);
        RS485.write(forward[i]);
        delay(1);
      }
      int cancel = Serial.parseInt();   
      if (cancel == 5) {
        break;
      }
    }
  }
  else {
    for (int i = 0; i < 11; i++) {
      Serial.print("Sending: ");
      Serial.println(nothing[i]);
      RS485.write(nothing[i]);
      delay(1);
    }
  }

  if (RS485.available()) {
    byte readByte = RS485.read();
    Serial.println(readByte);
  }

}
