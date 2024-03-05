#include <RS485.h>
#include <TimerOne.h>
#include <Arduino.h>
 
#define R_usonic A0
#define L_usonic A1
#define L_TOF A2
#define group 2
 
const uint8_t sendPin  = 8;
const uint8_t deviceID = 0;
RS485 rs485(&Serial1, sendPin);  //uses default deviceID
 
byte idle[11]   = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x4A}; 
byte fwd[11]    = {0x01, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0x8A};
byte bkwd[11]   = {0x01, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA4, 0x8A};
byte ccw[11]    = {0x01, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0x8A};
byte cw[11]     = {0x01, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x8A};
byte slow[11]   = {0x01, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6, 0x88};
byte fast[11]   = {0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x8B};
byte left[11]   = {0x01, 0x06, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x76, 0x8A};
byte right[11]  = {0x01, 0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8A};
 
int cnt = 0; // interrupt counter to control speed
int cmd_state = 0;
unsigned long last_cmd_time = 0;
 
float current_lat_speed = 0.1; // max: 0.1, min = 0.02
float speed_table[10] = {0.1, 0.091, 0.0822, 0.0733, 0.0644, 0.0555, 0.0466, 0.0377, 0.0288, 0.0199};
float lat_pos = 0;
float desired_lat_pos = 0;
 
bool lateral_mode = false;
 
// PID parameters
float P, I, D;
float Kp = 5.0;
float Ki = 1.0;
float Kd = 2.0;
float tau = 0.5;
float sample_t = 0.05;
float prev_e = 0;;
 
void cntrl();
void set_cmd(int incomingByte);
void manual();
int search_index(float val, float arr[], int n);
 
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial);
  Timer1.initialize(50000); // 50 millseconds
  Timer1.attachInterrupt(cntrl); 
}
 
void loop() {
  if(Serial.available() > 0){
    int incomingByte = Serial.read();
    if(incomingByte == 'C' || incomingByte == 'c') {
      lateral_mode = true;
      desired_lat_pos = lat_pos + 500; // target set to move 500 mm to the right 
      Serial.println(desired_lat_pos);
    }
    else set_cmd(incomingByte);
  }
}
 
void cntrl(){
  int R_usonic_val = analogRead(R_usonic);
  int L_usonic_val = analogRead(L_usonic);
  int L_TOF_val = analogRead(L_TOF);
  lat_pos = L_TOF_val * (2350.0/1023) + 150.0; // current lateral pos from the left wall in mm
  if(lateral_mode) {
    Serial.println(lat_pos);
    float error = desired_lat_pos - lat_pos;
    P = Kp * error;
    I += Ki * sample_t * error;
    D = (2 * Kd / (sample_t + 2 * tau)) * (error - prev_e) - ((sample_t - 2 * tau) / (sample_t + 2 * tau)) * D;
    prev_e = error;
    float output = P + I + D;
    Serial.println(output);
 
    if (I > 300) I = 300;
    else if (I < -300) I = -300;
 
    if (output > 1500) output = 1500;
    else if (output < -1500) output = -1500;
 
    current_lat_speed = 0.1 * output / 1500;
    Serial.println(current_lat_speed);
    int speed_index = search_index(current_lat_speed, speed_table, 10);
    Serial.println(speed_index);
    for(int i = 0; i < speed_index * 2; i++) {
      for (int j = 0; j < 11; j++) rs485.write(slow[j]);
    }
 
    if (output <= 150 && output >= 0 && abs(error) <= 5) { // stop condition
      lateral_mode = false;
      I = 0;
      D = 0;
      for(int i = 0; i < 20; i++) {
        for (int j = 0; j < 11; j++) rs485.write(fast[j]);
    }
    }
 
    else if(lat_pos > desired_lat_pos) {
      for (int j = 0; j < 1; j++) {
        for (int i = 0; i < 11; i++) rs485.write(left[i]);
      }
    }
    else if (lat_pos < desired_lat_pos) {
      for (int j = 0; j < 1; j++) {
        for (int i = 0; i < 11; i++) rs485.write(right[i]);
      }
    }
  } 
  else manual();
}
 
void set_cmd(int incomingByte) {
  if (incomingByte == 32) { // space
    cmd_state = 0;
  } else if ((incomingByte == 87) || (incomingByte == 119)) { // W or w(forward)
    cmd_state = 1;
  } else if ((incomingByte == 83) || (incomingByte == 115)) { // S or s(backward)
    cmd_state = 2;
  } else if ((incomingByte == 81) || (incomingByte == 113)) { // Q or q(ccw)
    cmd_state = 3;
  } else if ((incomingByte == 69) || (incomingByte == 101)) { // E or e (cw)
    cmd_state = 4;
  } else if (incomingByte == 45) { // -(slower)
    cmd_state = 5;
  } else if (incomingByte == 61) { // =(faster)
    cmd_state = 6;
  } else if ((incomingByte == 65) || (incomingByte == 97)) { // A or a (move left)
    cmd_state = 7;
  } else if ((incomingByte == 68) || (incomingByte == 100)) { // D or d (move right)
    cmd_state = 8;
  } else if(incomingByte == 'P' || incomingByte == 'p') {
     Serial.println(lat_pos);
     cmd_state = 0;
  }
}
 
void manual() {
  switch (cmd_state) {
    case 0: // idle
      for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      break;
 
    case 1: // fwd
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(fwd[i]);
      }
      cmd_state = 0;
      break;
 
    case 2: // bkwd
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(bkwd[i]);
      }
      cmd_state = 0;
      break;
 
    case 3: // ccw
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(ccw[i]);
      }
      cmd_state = 0;
      break;
 
    case 4: // cw
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(cw[i]);
      }
      cmd_state = 0;
      break;
 
    case 5: // slower
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(slow[i]);
      }
      cmd_state = 0;
      break;
 
    case 6: // faster
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(fast[i]);
      }
      cmd_state = 0;
      break;
 
    case 7: // left
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(left[i]);
      }
      cmd_state = 0;
      break;
 
    case 8: // right
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(right[i]);
      }
      cmd_state = 0;
      break;  
 
    default:
      break;
  }
}
 
int search_index(float val, float arr[], int n) {
  // searching for val with the closest value in the speed_table
  int index = 0;
  float min_diff = abs(val - arr[0]);
  for (int i = 1; i < n; i++) {
    if (abs(val - arr[i]) < min_diff) {
      min_diff = abs(val - arr[i]);
      index = i;
    }
  }
  return index;
}