#include <RS485.h>
#include <TimerOne.h>
#include <Arduino.h>

#define sonic_0 A0
#define sonic_1 A1

const uint8_t sendPin  = 8;
const uint8_t deviceID = 0;
RS485 rs485(&Serial1, sendPin);  //uses default deviceID

uint32_t lastCommand = 0; 
uint8_t commandState, group = 2;

bool alg = false;

int sensor_0; // right
int sensor_1; // left

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
  sensor_0 = analogRead(sonic_0);
  sensor_1 = analogRead(sonic_1);
  if (Serial.available() <= 0) return;
  int incomingByte = Serial.read(); // 'M' or 'm' for alignmnent
  if (incomingByte == 77 || incomingByte == 109) {
    alg = true;
  }
  else setCommand(incomingByte);
}


void callbackCommand() {
  if (alg) {
    if (abs(sensor_0 - sensor_1) > 5) {
      if (sensor_0 > sensor_1) {
          for (int j = 0; j < group; j++) {
            for (int i = 0; i < 11; i++) rs485.write(ccw[i]);
        }
        for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      }
      else if (sensor_0 < sensor_1) {
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(cw[i]);
        }
        for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      }
    }
    else {
      alg = false;
    }
  }
  else {
    switch (commandState) {
      case 0: // idle
        for (int i = 0; i < 11; i++) rs485.write(idle[i]);
        break;

      case 1: // fwd
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(fwd[i]);
        }
        commandState = 0;
        break;

      case 2: // bkwd
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(bkwd[i]);
        }
        commandState = 0;
        break;

      case 3: // ccw
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(ccw[i]);
        }
        commandState = 0;
        break;

      case 4: // cw
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(cw[i]);
        }
        commandState = 0;
        break;

      case 5: // slower
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(slow[i]);
        }
        commandState = 0;
        break;

      case 6: // faster
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(fast[i]);
        }
        commandState = 0;
        break;

      case 7: // left
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(left[i]);
        }
        commandState = 0;
        break;

      case 8: // right
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(right[i]);
        }
        commandState = 0;
        break;  

      default:
        break;
    }
  }
}

void setCommand(int incomingByte) {
  if (incomingByte == 32) { // space
    commandState = 0;
  } else if ((incomingByte == 87) || (incomingByte == 119)) { // W or w
    commandState = 1;
  } else if ((incomingByte == 83) || (incomingByte == 115)) { // S or s
    commandState = 2;
  } else if ((incomingByte == 81) || (incomingByte == 113)) { // Q or q
    commandState = 3;
  } else if ((incomingByte == 69) || (incomingByte == 101)) { // E or e
    commandState = 4;
  } else if (incomingByte == 45) { // -
    commandState = 5;
  } else if (incomingByte == 43) { // +
    commandState = 6;
  } else if ((incomingByte == 65) || (incomingByte == 97)) { // A or a
    commandState = 7;
  } else if ((incomingByte == 68) || (incomingByte == 100)) { // D or d
    commandState = 8;
  }
}
