#include <RS485.h>
#include <TimerOne.h>
#include <Arduino.h>

#define sonic_0 A0
#define sonic_1 A1

const uint8_t sendPin  = 8;
const uint8_t deviceID = 0;
RS485 rs485(&Serial1, sendPin);  //uses default deviceID

uint32_t lastCommand = 0; 
uint8_t commandState, group = 5;
bool align_flg = false;

int sensor_0;
int sensor_1;

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
void setCommand(int incomingByte);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial);
  Serial.println();
  Timer1.initialize(50000); //50 millseconds
  Timer1.attachInterrupt(callbackCommand);
}


void loop() {
  sensor_0 = analogRead(sonic_0); // right
  sensor_1 = analogRead(sonic_1); // left 
  Serial.println(sensor_0);
  Serial.println(sensor_1);
  Serial.println("     ");

  if (Serial.available() <= 0) return;
  int incomingByte = Serial.read();
  set_command(incomingByte);
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

void set_command(int incomingByte) {
  if (incomingByte == 32) {
    commandState = 0;
  } else if ((incomingByte == 87) || (incomingByte == 119)) {
    commandState = 1;
  } else if ((incomingByte == 83) || (incomingByte == 115)) {
    commandState = 2;
  } else if ((incomingByte == 81) || (incomingByte == 113)) {
    commandState = 3;
  } else if ((incomingByte == 69) || (incomingByte == 101)) {
    commandState = 4;
  } else if (incomingByte == 45) {
    commandState = 5;
  } else if (incomingByte == 43) {
    commandState = 6;
  } else if ((incomingByte == 65) || (incomingByte == 97)) {
    commandState = 7;
  } else if ((incomingByte == 68) || (incomingByte == 100)) {
    commandState = 8;
  }
}


