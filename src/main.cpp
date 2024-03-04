#include <RS485.h>
#include <TimerOne.h>
#include <TimerTwo.h>
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
byte left[11]   = {0x01, 0x06, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x76, 0x8A};
byte right[11]  = {0x01, 0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8A};
  // 0.002 m/s increase or decrease speed for sending every 20 ms
byte slow[11]   = {0x01, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6, 0x88};
byte fast[11]   = {0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x8B};

int cnt = 0; // interrupt counter to control speed
int cmd_state = 0;
unsigned long last_cmd_time = 0;

float current_lat_speed = 0.1; // max: 0.1, min = 0.02
float lat_pos = 0;
float desired_lat_pos = 0;

bool lateral_mode = false;

// PID parameters
float P, I, D;
float Kp = 0.5;
float Ki = 0.5;
float Kd = 0.5;
float tau = 0.5;
float sample_t = 0.05;
float prev_e = 0;;

void cntrl();
void cntrl_speed();
void set_cmd(int incomingByte);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial);
  Timer1.initialize(50000); // 50 millseconds
  Timer1.attachInterrupt(cntrl); 
  Timer2.init(20000, cntrl_speed); // 20 millseconds
}

void loop() {
  if(Serial.available() > 0){
    int incomingByte = Serial.read();
    if(incomingByte == 'c') {
      lateral_mode = true;
      desired_lat_pos = lat_pos + 500; // target set to move 500 mm to the right
    }
    set_cmd(incomingByte);
  }
}

void cntrl(){
  int R_usonic_val = analogRead(R_usonic);
  int L_usonic_val = analogRead(L_usonic);
  int L_TOF_val = analogRead(L_TOF);
  lat_pos = L_TOF_val * (2350/1023) + 150; // current lateral pos from the left wall in mm
  if(lateral_mode) {
    if (abs(desired_lat_pos - lat_pos) <= 5) {
      lateral_mode = false;
    }
    if(lat_pos > desired_lat_pos) {
      for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 11; i++) rs485.write(left[i]);
      }
    }
    else if (lat_pos < desired_lat_pos) {
      for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 11; i++) rs485.write(right[i]);
      }
    }
  } 
}

void set_cmd(int incomingByte) {
  switch (incomingByte) {
    case 119: // w, fwd
      cmd_state = 1;
      break;
    
    case 115: // s, bkwd
      cmd_state = 2;
      break;

    case 97: // a, left
      cmd_state = 3;
      break;

    case 100: // d, right
      cmd_state = 4;
      break;

    case 113: // q, ccw
      cmd_state = 5;
      break;

    case 101: // e, cw  
      cmd_state = 6;
      break;
  }
}

void cntrl_speed() {
  static unsigned long last_cmd_time = 0;  // Time of last command sent

  // Calculate PID output and desired speed change
  float error = desired_lat_pos - lat_pos;
  P = Kp * error;
  I += Ki * sample_t * error;
  D = (2 * Kd / (sample_t + 2 * tau)) * (error - prev_e) - ((sample_t - 2 * tau) / (sample_t + 2 * tau)) * D;
  prev_e = error;

  // Normalize desired speed change to range [-1, 1] based on min and max speeds
  float normalized_speed_change = (P + I + D) / (0.1 - 0.02);  // Adjust the denominator based on your min and max speed values

  // Constrain normalized speed change to [-1, 1]
  normalized_speed_change = constrain(normalized_speed_change, -1.0, 1.0);

  // Convert normalized speed change to speed steps (0 to 10)
  int speed_steps = abs(round(normalized_speed_change * 10));  // Scale by 10 for 0-10 steps

  // Send commands based on desired speed change and timing
  if (millis() - last_cmd_time >= 20) {  // Check if 20ms have passed since last command
    if (normalized_speed_change > 0) {
      rs485.write(fast, sizeof(fast));  // Send "fast" for 20ms
      last_cmd_time = millis();  // Update last command time
    } else if (normalized_speed_change < 0) {
      rs485.write(slow, sizeof(slow));  // Send "slow" for 20ms
      last_cmd_time = millis();  // Update last command time
    }
  }
}
