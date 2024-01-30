#include <Arduino.h>
#include <RS485.h>
#include <TimerOne.h>

const uint8_t sendPin  = 8;
const uint8_t deviceID = 0;
RS485 rs485(&Serial1, sendPin);  //uses default deviceID

uint32_t lastCommand = 0; 
uint8_t commandState, group = 5;

byte idle[11]   = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x4A}; 
byte fwd[11]    = {0x01, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0x8A};
byte bkwd[11]   = {0x01, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA4, 0x8A};
byte ccw[11]    = {0x01, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0x8A};
byte cw[11]     = {0x01, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x8A};
byte slow[11]   = {0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x8B};
byte fast[11]   = {0x01, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6, 0x88};
byte left[11]   = {0x01, 0x06, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x76, 0x8A};
byte right[11]  = {0x01, 0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8A};

void callbackCommand();
void setIdle(int incomingByte);
void setFwd(int incomingByte);
void setBkwd(int incomingByte);
void setCCW(int incomingByte);
void setCW(int incomingByte);
void setSlower(int incomingByte);
void setFaster(int incomingByte);
void setLeft(int incomingByte);
void setRight(int incomingByte);


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
  if (Serial.available() <= 0) return;
  int incomingByte = Serial.read();
  setIdle(incomingByte);  // " " (space)
  setFwd(incomingByte);  // "W" or "w"
  setBkwd(incomingByte);  // "S" or "s"
  setCCW(incomingByte);  // "Q" or "q"
  setCW(incomingByte);  // "E" or "e"
  setSlower(incomingByte);  // "-" (minus)
  setFaster(incomingByte);  // "+" (plus)
  setLeft(incomingByte);  // "A" or "a"
  setRight(incomingByte);  // "D" or "d"

}

void callbackCommand() {
  switch (commandState) {
    case 0: // idle
      for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      break;
    case 1: // fwd
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(fwd[i]);
      }
      break;
    case 2: // bkwd
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(bkwd[i]);
      }
      break;
    case 3: // ccw
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(ccw[i]);
      }
      break;
    case 4: // cw
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(cw[i]);
      }
      break;
    case 5: // slower
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(slow[i]);
      }
      break;
    case 6: // faster
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(fast[i]);
      }
      break;
    case 7: // left
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(left[i]);
      }
      break;
    case 8: // right
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(right[i]);
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
  if ((incomingByte != 87) && (incomingByte != 119)) return;
  commandState = 1;
}

void setBkwd(int incomingByte) { // S: 83, s: 115
  if ((incomingByte != 83) && (incomingByte != 115)) return;
  commandState = 2;
}

void setCCW(int incomingByte) { // Q: 81, q: 113
  if ((incomingByte != 81) && (incomingByte != 113)) return;
  commandState = 3;
}

void setCW(int incomingByte) { // E: 69, e: 101
  if ((incomingByte != 69) && (incomingByte != 101)) return;
  commandState = 4;
}

void setSlower(int incomingByte) { // -: 45
  if (incomingByte != 45) return;
  commandState = 5;
}

void setFaster(int incomingByte) { // +: 43
  if (incomingByte != 43) return;
  commandState = 6;
}

void setLeft(int incomingByte) { // A: 65, a: 97
  if ((incomingByte != 65) && (incomingByte != 97)) return;
  commandState = 7;
}

void setRight(int incomingByte) { // D: 68, d: 100
  if ((incomingByte != 68) && (incomingByte != 100)) return;
  commandState = 8;
}

