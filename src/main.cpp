#include <RS485.h>
#include <TimerOne.h>
#include <Arduino.h>

#define sonic_0 A0
#define sonic_1 A1
#define tof A2

const uint8_t sendPin  = 8;
const uint8_t deviceID = 0;
RS485 rs485(&Serial1, sendPin);  //uses default deviceID

// PID parameters
const uint8_t Kp = 12;
const uint8_t Ki = 0;
const uint8_t Kd = 5;
int P, I, D, prev_e = 0;
float sample_t = 0.05;
int tau = 2;
int cnt_alg = 0;

// lateral motion
float current_pos_lat = 0;
float desired_pos_lat = 0;

// longitudinal motion
float desired_pos_long; // fixed y position for scanning (300 mm)
int current_pos_long = 0;
int desired_pos_adc = 0;

uint32_t lastCommand = 0; 
uint8_t commandState, group = 2;

unsigned long alg_timeout; // alignment timeout (10 seconds)

// internal flags for traffic control
bool alg = false;         // angular alignment 
bool mv_lat = false;      // lateral motion 500 mm
bool mv_long = false;     // longitudinal motion

int sensor_0; // right ultrasonic
int sensor_1; // left ultrasonic
int sensor_2; // TOF 
// longitudinal pos = (485/1023) * (sensor_0 + sensor_1) * 0.5 + 15
// lateral pos = sensor_2 * (2350/1023) + 150;

// RS485 commands
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
void setCommand(int incomingByte, char* num = nullptr);
void align(); // angular adjustment
void move_lat();  // lateral motion
void move_long(); // longitudinal motion
void manual(); // manual control
int PID();

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial);
  Serial.println();
  Timer1.initialize(50000); // 50 millseconds
  Timer1.attachInterrupt(callbackCommand);
}

void loop() {
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (incomingByte == 'N' || incomingByte == 'n' || incomingByte == 'C' || incomingByte == 'c') {
      char num[4]; 
      Serial.readBytes(num,4);
      num[3] = '\0';
      setCommand(incomingByte, num);
    } else {
      setCommand(incomingByte);
    }
  }
}

void callbackCommand() {
  sensor_0 = analogRead(sonic_0);
  sensor_1 = analogRead(sonic_1);
  sensor_2 = analogRead(tof);
  current_pos_lat = sensor_2 * 2.297 + 150;
  current_pos_long = ((sensor_0 + sensor_1) * 0.5);
  // Serial.println(sensor_0);
  // Serial.println(sensor_1);
  // Serial.println(sensor_2);
  // Serial.println("");

  if (alg) align();
  else if (mv_lat) move_lat();
  else if (mv_long) move_long();
  else {
    manual();
  }
}

void setCommand(int incomingByte, char* num = nullptr) {
  if (incomingByte == 32) { // space
    commandState = 0;
  } else if ((incomingByte == 87) || (incomingByte == 119)) { // W or w(forward)
    commandState = 1;
  } else if ((incomingByte == 83) || (incomingByte == 115)) { // S or s(backward)
    commandState = 2;
  } else if ((incomingByte == 81) || (incomingByte == 113)) { // Q or q(ccw)
    commandState = 3;
  } else if ((incomingByte == 69) || (incomingByte == 101)) { // E or e (cw)
    commandState = 4;
  } else if (incomingByte == 45) { // -(slower)
    commandState = 5;
  } else if (incomingByte == 43) { // +(faster)
    commandState = 6;
  } else if ((incomingByte == 65) || (incomingByte == 97)) { // A or a (move left)
    commandState = 7;
  } else if ((incomingByte == 68) || (incomingByte == 100)) { // D or d (move right)
    commandState = 8;
  } else if (incomingByte == 77 || incomingByte == 109) { // 'M' or 'm' for alignmnent
    alg = true;
  } else if (incomingByte == 67 || incomingByte == 99) {  // 'C(number)' or 'c' for move side TOF
    for(int i = 0; i < 3; i++) {
      // check if the num is valid
      if (num[i] < '0' || num[i] > '9') {
          return;
        }
    }
    desired_pos_lat = current_pos_lat + atoi(num);
    mv_lat = true;
  } else if (incomingByte == 78 || incomingByte == 110) { // 'N(number)' or 'n' for adjusting longitudinal position ULTRASONIC
    for(int i = 0; i < 3; i++) {
      // check if the num is valid
      if (num[i] < '0' || num[i] > '9') {
          return;
        }
    }
    desired_pos_long = atoi(num);
    desired_pos_adc = (desired_pos_long - 15) / 0.474;
    mv_long = true;
  } else if (incomingByte == 80 || incomingByte == 112) { // 'P' or 'p' for sending sensor data
    Serial.print(0.4741 * (sensor_0 + sensor_1) * 0.5 + 15);
    Serial.print(" ");
    Serial.print(sensor_2 * 2.2971 + 150);
    Serial.print(" ");
  }
}

void align() {
  alg_timeout = millis();
  if (abs(sensor_0 - sensor_1) >= 6) {
      group = PID();
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
    ++cnt_alg;
  }

  if (cnt_alg >= 20 && abs(sensor_0 - sensor_1) <= 6) {
    alg = false;
    cnt_alg = 0;
    group = 2;
  }
  else if (millis() - alg_timeout > 10000 && abs(sensor_0 - sensor_1) <= 10) {
    alg = false;
    cnt_alg = 0;
    group = 2;
  }
}

void move_lat() {
  if (abs(current_pos_lat - desired_pos_lat) <= 5) {
    mv_lat = false;
  }
  else if (current_pos_lat > desired_pos_lat) {
    for (int j = 0; j < group; j++) {
      for (int i = 0; i < 11; i++) rs485.write(left[i]);
    }
  }
  else if (current_pos_lat < desired_pos_lat) {
    for (int j = 0; j < group; j++) {
      for (int i = 0; i < 11; i++) rs485.write(right[i]);
    }
  }
}

void move_long() {
  int desired;
  int dif = desired_pos_adc - current_pos_long;
  if(dif > 55) { // AGV pos < desired
    desired = desired_pos_adc - 20;
  }
  else if(dif > 30 && dif <= 55) { // AGV pos < desired
    desired = desired_pos_adc - 15;
    group = 1;
  }
  else if (dif < -30 && dif >= -55) { // AGV pos > desired
    desired = desired_pos_adc + 15;
    group = 1;
  }
  else if (dif < -55) { // AGV pos < desired
    desired = desired_pos_adc + 20;
  }
  else {
    desired = current_pos_long;
  }


  if(abs(current_pos_long - desired) > 5) {
    if (current_pos_long > desired) {
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(fwd[i]);
      }
    }
    else if (current_pos_long < desired) {
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(bkwd[i]);
      }
    }
  }
  else {
    mv_long = false;
    group = 2;
  }
}

void manual() {
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

int PID() {
    int error = abs(sensor_0 - sensor_1);
    P = Kp * error;
    I += Ki * sample_t * (error + prev_e) * 0.5;
    D = ((2 * Kd) / (sample_t + 2 * tau)) * (error - prev_e) - (
          (sample_t - 2 * tau) / (sample_t + 2 * tau)) * D;

    prev_e = error;
    int PID = P + I + D;

    // anti-windup
    if (PID > 500) PID = 500;
    else if (PID < 0) PID = abs(PID);

    return PID/100;
}