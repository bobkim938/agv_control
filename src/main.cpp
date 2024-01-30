//    FILE: RS485_simple_master.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: simple master
//     URL: https://github.com/RobTillaart/RS485

//  this is the code of a simple master  (needs simple slave)
//  it send one of 3 (single char) commands to the slave
//  '0' == set LED LOW
//  '1' == set LED HIGH
//  '2' == request status.
//
//  print debug messages SEND and RECV with data.
//  Note that one needs a 2nd Serial port for nice debugging.
//  (or an LCD screen whatever).


#include <Arduino.h>
#include <RS485.h>
#include <TimerOne.h>

const uint8_t sendPin  = 8;
const uint8_t deviceID = 0;
RS485 rs485(&Serial1, sendPin);  //uses default deviceID

uint32_t lastCommand = 0; 
uint8_t commandState, group = 20;
byte idle[11]   = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x4A}; 
byte fwd[11]    = {0x01, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0x8A};
byte bkwd[11]   = {0x01, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA4, 0x8A};
byte ccw[11]    = {0x01, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0x8A};
byte cw[11]     = {0x01, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x8A};
byte slow[11]   = {0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x8B};
byte fast[11]   = {0x01, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6, 0x88};
byte left[11]   = {0x01, 0x06, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x76, 0x8A};
byte right[11]  = {0x01, 0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8A};

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial);
  Serial.println();
  //Serial.println(__FILE__);
  //Serial.print("RS485_LIB_VERSION: ");
  //Serial.println(RS485_LIB_VERSION);
  Timer1.initialize(50000); //50 millseconds
  Timer1.attachInterrupt(callbackCommand);
}


void loop() {
/*  if (millis() - lastCommand >= 120) {
    lastCommand = millis();
    for (int i = 0; i < 11; i++) {
        rs485.write(idle[i]);
    }
    for (int i = 0; i < 11; i++) {
        rs485.write(fwd[i]);
    }
  }*/

  if (Serial.available() <= 0) return;
  int incomingByte = Serial.read();
  setIdle(incomingByte);  // " " (space)
  setFwd(incomingByte);  // "W" or "w"
  setBkwd(incomingByte);  // "S" or "s"

}

void callbackCommand() {
  switch (commandState) {
    case 0: //idle
      for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      break;
    case 1: //fwd
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(fwd[i]);
      }
      break;
    case 2: //bkwd
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(bkwd[i]);
      }
      break;
    default:
      break;
  }
  commandState = 0;
}

void setIdle(int incomingByte) { // " ": 32 (space)
  if (incomingByte != 32) return;
  commandState = 0;
}

void setFwd(int incomingByte) { // W: 87, w:119
  if ((incomingByte != 87) and (incomingByte != 119)) return;
  commandState = 1;
}

void setBkwd(int incomingByte) { // S: 83, s: 115
  if ((incomingByte != 83) and (incomingByte != 115)) return;
  commandState = 2;
}