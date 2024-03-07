#include <RS485.h>
#include <TimerOne.h>
#include <Arduino.h>
 
#define R_usonic A0
#define L_usonic A1
#define L_TOF A2
 
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
 
int cmd_state = 0;
int group = 2;
static float lat_pos_avg[5] = {0, 0, 0, 0, 0};
 
 // lateral motion control parameters
int L_TOF_val = 0;
float current_lat_speed = 0.1; // max: 0.1, min = 0.02
float speed_table[10] = {0.1, 0.091, 0.0822, 0.0733, 0.0644, 0.0555, 0.0466, 0.0377, 0.0288, 0.0199}; // speed table for lateral control
float lat_pos = 0;
float filtered_lat_pos = 0;
float desired_lat_pos = 0;
// PID parameters for lateral control
float P, I, D;
float Kp = 5.0;
float Ki = 1.0;
float Kd = 2.0;
float tau = 0.5;
float sample_t = 0.05;
float prev_e = 0;
bool lateral_mode = false;
 
// alignment parameters
int R_usonic_val = 0;
int L_usonic_val = 0;
bool alg = false;
unsigned long alg_timeout; // alignment timeout (10 seconds)
int cnt_alg = 0; // alignment counter
int P_alg, I_alg, D_alg;
int Kp_alg = 12;
int Ki_alg = 0;
int Kd_alg = 5;
int prev_e_alg = 0;
 
// longitudinal motion control parameters
bool longi_mode = false; // setting distance to 245 mm
bool longi_mode_bkwd = false; // setting distance to 345 mm
int desired_long_pos_adc = 0;
int current_long_pos_adc = 0;
float current_long_pos;
 
void cntrl();
void set_cmd(int incomingByte);
void manual();
void lateral_500(); // lateral control for moving 500 mm to the right
void longi_245(); // longitudinal control for moving to 245 mm from the glass
void align(); // angular adjustment
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
    else if(incomingByte == 'N' || incomingByte == 'n') {
      longi_mode = true;
      desired_long_pos_adc = (245 - 15) / (0.474); // target set to move 245 mm forward
    }
    else if(incomingByte == 'B' || incomingByte == 'b') {
      longi_mode_bkwd = true;
      desired_long_pos_adc = (345 - 15) / (0.474); // target set to move 345 mm backward
    }
    else set_cmd(incomingByte);
  }
}
 
void cntrl(){
  R_usonic_val = analogRead(R_usonic);
  L_usonic_val = analogRead(L_usonic);
  current_long_pos_adc = (R_usonic_val + L_usonic_val) * 0.5; // in ADC value
  current_long_pos = current_long_pos_adc * (485.0/1023) + 15.0; // current distance from the wall in mm
 
  L_TOF_val = analogRead(L_TOF);
  lat_pos = L_TOF_val * (2350.0/1023) + 150.0; // current lateral pos from the left wall in mm
 
  // moving average with 5 samples of lat_pos
  for(int i = 0; i < 4; i++) lat_pos_avg[i] = lat_pos_avg[i+1];
  lat_pos_avg[4] = lat_pos;
  filtered_lat_pos = (lat_pos_avg[0] + lat_pos_avg[1] + lat_pos_avg[2] + lat_pos_avg[3] + lat_pos_avg[4]) / 5;
 
  if(lateral_mode) lateral_500();
  else if(alg) align();
  else if(longi_mode || longi_mode_bkwd) longi_245();
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
  } else if (incomingByte == 77 || incomingByte == 109) { // 'M' or 'm' for alignmnent
    alg = true;
  } else if(incomingByte == 'P' || incomingByte == 'p') {
     Serial.println(filtered_lat_pos);
     Serial.println(current_long_pos);
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
 
void lateral_500() {
  Serial.println(filtered_lat_pos);
    float error = desired_lat_pos - filtered_lat_pos;
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
      for (int j = 0; j < 1; j++) {
        for (int i = 0; i < 11; i++) rs485.write(left[i]);
      }
      lateral_mode = false;
      I = 0;
      D = 0;
    }
 
    else if(filtered_lat_pos > desired_lat_pos) {
      for (int j = 0; j < 1; j++) {
        for (int i = 0; i < 11; i++) rs485.write(left[i]);
      }
    }
    else if (filtered_lat_pos < desired_lat_pos) {
      for (int j = 0; j < 1; j++) {
        for (int i = 0; i < 11; i++) rs485.write(right[i]);
      }
    }
}
 
void longi_245() {
  int desired;
  int dif = desired_long_pos_adc - current_long_pos_adc;
  if(dif > 55) { // AGV pos < desired
    desired = desired_long_pos_adc - 20;
  }
  else if(dif > 10 && dif <= 55) { // AGV pos < desired
    desired = desired_long_pos_adc - 15;
    group = 1;
  }
  else if (dif < -10 && dif >= -55) { // AGV pos > desired
    desired = desired_long_pos_adc + 15;
    group = 1;
  }
  else if (dif < -55) { // AGV pos < desired
    desired = desired_long_pos_adc + 20;
  }
  else {
    desired = desired_long_pos_adc;
  }
 
  if(abs(current_long_pos_adc - desired) > 5) {
    if (current_long_pos_adc > desired) {
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(fwd[i]);
      }
    }
    else if (current_long_pos_adc < desired) {
      for (int j = 0; j < group; j++) {
        for (int i = 0; i < 11; i++) rs485.write(bkwd[i]);
      }
    }
  }
  else {
    longi_mode = false;
    longi_mode_bkwd = false;
    group = 2;
  }
}
 
void align() {
  alg_timeout = millis();
  if (abs(R_usonic_val - L_usonic_val) >= 6) {
      int err = abs(R_usonic_val - L_usonic_val);
      P_alg = Kp_alg * err;
      I_alg += Ki_alg * sample_t * (err + prev_e_alg) * 0.5;
      D_alg = ((2 * Kd_alg) / (sample_t + 2 * tau)) * (err - prev_e_alg) - (
            (sample_t - 2 * tau) / (sample_t + 2 * tau)) * D_alg;
 
      prev_e_alg = err;
      int PID = P_alg + I_alg + D_alg;
 
      // anti-windup
      if (PID > 500) PID = 500;
      else if (PID < 0) PID = abs(PID);
 
      group = PID/70;
      if (R_usonic_val > L_usonic_val) {
          for (int j = 0; j < group; j++) {
            for (int i = 0; i < 11; i++) rs485.write(ccw[i]);
        }
        for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      }
      else if (R_usonic_val < L_usonic_val) {
        for (int j = 0; j < group; j++) {
          for (int i = 0; i < 11; i++) rs485.write(cw[i]);
        }
        for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      }
    }
  else {
    ++cnt_alg;
  }
 
  if (cnt_alg >= 20 && abs(R_usonic_val - L_usonic_val) <= 6) {
    alg = false;
    cnt_alg = 0;
    group = 2;
    I_alg = 0;
    D_alg = 0;
  }
  else if (millis() - alg_timeout > 10000 && abs(R_usonic_val - L_usonic_val) <= 10) {
    alg = false;
    cnt_alg = 0;
    group = 2;
    I_alg = 0;
    D_alg = 0;
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