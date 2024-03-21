#include <Arduino.h>
#include <TimerOne.h>
// #include <TimerThree.h>
#include <RS485.h>
#include <MovingAverage.h>
// #include <PID_v1_bc.h>
#include <ADS1X15.h>

#define R_usonic A0
#define L_usonic A1
// #define L_TOF A2

const uint8_t sendPin  = 8;
const uint8_t deviceID = 0;
RS485 rs485(&Serial1, sendPin);  // uses default deviceID
ADS1115 ADS(0x48); // initialize on I2C bus 1 with default address 0x48

const byte idle[11]   = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x4A}; 
const byte fwd[11]    = {0x01, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0x8A};
const byte bkwd[11]   = {0x01, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA4, 0x8A};
const byte ccw[11]    = {0x01, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0x8A};
const byte cw[11]     = {0x01, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x8A};
const byte slow[11]   = {0x01, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6, 0x88};
const byte fast[11]   = {0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x8B};
const byte left[11]   = {0x01, 0x06, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x76, 0x8A};
const byte right[11]  = {0x01, 0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8A};

bool align_flag, stride_flag, adjust_flag, print_flag, machine_state;
int cmd_state;
int adjustTarget;
double strideTarget, strideTarget_mm, current_long_pos;
double difference;
int lUsonicRead, rUsonicRead, lTofRead;
double lUsonic, rUsonic, lTof;
// float current_long_pos, filtered_lat_pos;
MovingAverage <int, 4> lUsonicFilter;
MovingAverage <int, 4> rUsonicFilter;
MovingAverage <int, 4> lTofFilter;

// double strideOutput, Kp=2, Ki=5, Kd=1;
// PID stridePID(&lTof, &strideOutput, &strideTarget, Kp, Ki, Kd, DIRECT);
// function prototypes
void send_485();
void read_sensor();
void stride_control();
void set_speed(bool speed);
void process_terminal(int incomingByte);
void adjust_long();

void setup() { // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial); 
  Timer1.initialize(50000); // 50 milliseconds
  Timer1.attachInterrupt(send_485); 
  // stridePID.SetMode(AUTOMATIC);
  Wire.begin();
  if (!ADS.begin()) Serial.println("Invalid I2C address");
  if (!ADS.isConnected()) Serial.println("ADS1115 is not connected");
}

void loop() { // put your main code here, to run repeatedly:
  read_sensor();
  if (machine_state) { // true means under auto control
    if (align_flag) {
      align_flag = false; machine_state = false;
    }
    if (stride_flag) {
      stride_control();
    }
    if (adjust_flag) {
      adjust_long();
    }
    if (print_flag) {
      Serial.print("L: "); Serial.print(lUsonic); Serial.print('\t'); 
      Serial.print("R: "); Serial.print(rUsonic); Serial.print('\t'); 
      Serial.print("ToF: "); Serial.println(lTof);
      print_flag = false; machine_state = false;
    }
  }
  else { // false means under manual control
    if (Serial.available() <= 0) return;
    int incomingByte = Serial.read();
    process_terminal(incomingByte);
  }
  delayMicroseconds(50000); // TODO Very important 
}

void send_485() { // This function to send out 485 com to the AGV
  switch (cmd_state) {
    case 0: // idle
      for (int i = 0; i < 11; i++) rs485.write(idle[i]);
      break;
    case 1: // forward
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(fwd[i]);
      cmd_state = 0;
      break;
    case 2: // backward
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(bkwd[i]);
      cmd_state = 0;
      break;
    case 3: // ccw
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(ccw[i]);
      cmd_state = 0;
      break;
    case 4: // cw
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(cw[i]);
      cmd_state = 0;
      break;
    case 5: // slower
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(slow[i]);
      cmd_state = 0;
      break;
    case 6: // faster
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(fast[i]);
      cmd_state = 0;
      break;
    case 7: // left
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(left[i]);
      cmd_state = 0;
      break;
    case 8: // right
      for (int j = 0; j < 1; j++) for (int i = 0; i < 11; i++) rs485.write(right[i]);
      cmd_state = 0;
      break;  
    default:
      break;
  }
}

void read_sensor() { // This function to read sensor data and average them
  lUsonicRead = lUsonicFilter.add(analogRead(L_usonic)); lUsonic = lUsonicFilter.get();
  rUsonicRead = rUsonicFilter.add(analogRead(R_usonic)); rUsonic = rUsonicFilter.get();
  lTofRead = lTofFilter.add(ADS.readADC(0)); lTof = lTofFilter.get();   
  current_long_pos = (lUsonic + rUsonic) * 0.5;
}

// double strideAllowance = 5; // equivalent to 1 mm
double strideAllowance = 70; // equivalent to 1 mm
double magicNumber = 1850;
int speed_i;
bool speed_flag;

void stride_control() {
  difference = strideTarget - lTof;
  if (difference > (strideAllowance*1.0)) { // should move right
    if (difference > (magicNumber*1.0)) { // high speed
      if (!speed_flag) set_speed(true);
      else cmd_state = 8; 
    }
    else { // low speed
      if (speed_flag) set_speed(false);
      else cmd_state = 8; 
    }
  }
  else if (difference < (strideAllowance*-1.0)) { // should move left
    if (difference < (magicNumber*-1.0)) { // high speed
      if (!speed_flag) set_speed(true);
      else cmd_state = 7;
    }
    else { // low speed
      if (speed_flag) set_speed(false);
      else cmd_state = 7;
    }
  }
  else { //should not move
    cmd_state = 0; stride_flag = false; machine_state = false;
  }
  Serial.print("ToF: "); Serial.print(lTof); Serial.print('\t'); 
  Serial.print("Target: "); Serial.println(strideTarget); 
}

void set_speed(bool speed) {
  if (speed) { // to set highest speed
    if (speed_i<20) { speed_i++; cmd_state = 6; }
    else { speed_i = 0; cmd_state = 0; speed_flag = true; }
  }
  else { // to set lowest speed
    if (speed_i<20) { speed_i++; cmd_state = 5; }
    else { speed_i = 0; cmd_state = 0; speed_flag = false; }
  }
}

void process_terminal(int incomingByte) { // This function to process the incoming terminal command
  if (incomingByte == 32) cmd_state = 0; // space (idle)
  else if ((incomingByte == 87) || (incomingByte == 119)) cmd_state = 1; // W or w (forward)
  else if ((incomingByte == 83) || (incomingByte == 115)) cmd_state = 2; // S or s (backward)
  else if ((incomingByte == 81) || (incomingByte == 113)) cmd_state = 3; // Q or q (ccw)
  else if ((incomingByte == 69) || (incomingByte == 101)) cmd_state = 4; // E or e (cw)
  else if (incomingByte == 45) cmd_state = 5; // - (slower)
  else if (incomingByte == 61) cmd_state = 6; // = (faster)
  else if ((incomingByte == 65) || (incomingByte == 97)) cmd_state = 7; // A or a (left)
  else if ((incomingByte == 68) || (incomingByte == 100)) cmd_state = 8; // D or d (right)
  else if ((incomingByte == 77) || (incomingByte == 109)) { // M or m (align)
    align_flag = true; machine_state = true;
  }
  else if ((incomingByte == 67) || (incomingByte == 99)) { // C or c (stride)
    // strideTarget_mm = 500; //strideTarget_mm = Serial.parseInt();
    // strideTarget = map(strideTarget_mm, 150, 2500, 1, 1023) + lTof; // Target is relative to current ToF value
    strideTarget = Serial.parseInt();
    Serial.print("ToF: "); Serial.print(lTof); Serial.print('\t'); 
    Serial.print("Target: "); Serial.println(strideTarget); 
    stride_flag = true; machine_state = true;
  }
  else if ((incomingByte == 78) || (incomingByte == 110)) { // N or n (adjust)
    adjustTarget = Serial.parseInt();
    adjust_flag = true; machine_state = true;
    Serial.print("Sonic: "); Serial.print(current_long_pos); Serial.print('\t'); 
    Serial.print("Target: "); Serial.println(adjustTarget); 
  }
  else if ((incomingByte == 80) || (incomingByte == 112)) { // P or p (print)
    print_flag = true; machine_state = true;
  }
}

void adjust_long() {
    int desired;
    int dif = adjustTarget - current_long_pos;
    if(dif > 55) { // AGV pos < desired
        desired = adjustTarget - 20;
    }
    else if(dif > 10 && dif <= 55) { // AGV pos < desired
        desired = adjustTarget - 15;
    }
    else if (dif < -10 && dif >= -55) { // AGV pos > desired
        desired = adjustTarget + 15;
    }
    else if (dif < -55) { // AGV pos < desired
        desired = adjustTarget + 20;
    }
    else {
        desired = adjustTarget;
    }

    if(abs(current_long_pos - desired) > 5) {
        if (current_long_pos > desired) {
            cmd_state = 1;
        }
        else if (current_long_pos < desired) {
            cmd_state = 2;
        }
    }
    else {
        adjust_flag = false;
        machine_state = false;
    }
}