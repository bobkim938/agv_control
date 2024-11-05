#include <Arduino.h>
#include <TimerOne.h>
#include <RS485.h>
#include <MovingAverage.h>
#include <ADS1X15.h>
#include <PS2X_lib.h>  //for v1.6

enum SPEED {SLOW, FAST};

#define R_usonic A0 // 1st
#define L_usonic A1 // 2nd
#define R_tof A3

#define PS2_DAT        13    
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        12
#define pressures   true
#define rumble      true

#define FLidarZone1 22 // front zone
#define FLidarZone2 26 // right zone
#define FLidarZone3 30
#define BLidarZone1 34 // back zone
#define BLidarZone2 38 // left zone
#define BLidarZone3 42 // back zone for SLOW DOWN

// Emergency Stop Interrupt
const uint8_t interruptPin = 3;
volatile bool estopFlag;

SPEED current_speed;

PS2X ps2x; // create PS2 Controller Class
int ps2_error = 0;
byte type = 0;
byte vibrate = 0;

// function prototypes
void send_485();
void read_sensor();
void align_control();
void stride_control();
void adjust_control();
void strideSpeed(bool speed);
void process_terminal(int incomingByte, int32_t target = 0);
void set_speed(SPEED speed);
void estop(); // interrupt for estop pressed, check for debounce
void unstop(); // interrupt for estop released, check for debounce
void controller_setup(); // PS2 controller setup

// Reset func 
void (*resetFunc) (void) = 0;
void process_controller();

const uint8_t sendPin  = 8;
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
const uint8_t magicLabAlign = 1; 
const uint8_t magicLabStride = 5; // equivalent to 1 mm
const uint8_t magicLabAdjust = 2;

bool align_flag, stride_flag, adjust_flag, printTOF_flag, printSONIC_flag, strideSpeed_flag, print_state_flag, false_alarm = false;
bool onStart_speed = true; // reduce speed when starting
bool printlTofSub_flag = false;
bool printCeilTof_flag = false;
uint8_t align_i, stride_i, adjust_i, speed_i, cmd_state = 0;
uint8_t adjusting_cnt = 0;
int32_t adjustTarget;
int32_t lUsonicRead, rUsonicRead, lTofRead, rTofRead, CeilTofRead, CeilTof;
int32_t lUsonic, rUsonic, Usonic, UsonicDiff, rTof;
int32_t lTof, strideTarget, prev_ltof;
int32_t lTofSubRead, lTofSub;
int32_t lTofDiff;

MovingAverage <int, 16> lUsonicFilter;
MovingAverage <int, 16> rUsonicFilter;
MovingAverage <int, 4> lTofFilter;
MovingAverage <int, 4> rTofFilter;
MovingAverage <int, 4> lTofSubFilter;
MovingAverage <int, 4> CeilTofFilter;

void estop() { // interrupt for estop pressed, check for debounce
  delayMicroseconds(5);
  while (digitalRead(interruptPin) == 0) { estopFlag = true; break; }
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  attachInterrupt(digitalPinToInterrupt(interruptPin), unstop, HIGH);
}

void unstop() { // interrupt for estop released, check for debounce
  delayMicroseconds(50);
  while (digitalRead(interruptPin) == 1) { estopFlag = false; break; }
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  attachInterrupt(digitalPinToInterrupt(interruptPin), estop, LOW);
}

void setup() { // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial1.begin(9600);
  Serial.setTimeout(50);
  pinMode(interruptPin, INPUT_PULLUP); //Inverts the behavior of the INPUT mode, HIGH means off, LOW means on

  pinMode(FLidarZone1, INPUT_PULLUP);
  pinMode(FLidarZone2, INPUT_PULLUP);
  pinMode(FLidarZone3, INPUT_PULLUP);
  pinMode(BLidarZone1, INPUT_PULLUP);
  pinMode(BLidarZone2, INPUT_PULLUP);
  pinMode(BLidarZone3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(interruptPin), estop, LOW);
  Timer1.initialize(55000); // 50 milliseconds
  Timer1.attachInterrupt(send_485); 
  Wire.begin();
  if (!ADS.begin()) Serial.println("Invalid I2C address");
  if (!ADS.isConnected()) Serial.println("ADS1115 is not connected");

  controller_setup();
}


void loop() { // put your main code here, to run repeatedly:
  read_sensor();
  if ((abs(prev_ltof - lTofSub) > 200) && stride_flag) {
    false_alarm = true;
  }
  prev_ltof = lTofSub;
  if (estopFlag || false_alarm) {
    cmd_state = 0;
    align_flag = false;
    stride_flag = false;
    adjust_flag = false;
    strideSpeed_flag = false;
  }

  if(onStart_speed) {
    set_speed(SLOW);
    onStart_speed = false;
  }
  
  if (align_flag) align_control(); 
  else if (stride_flag) stride_control();
  else if (adjust_flag) {
    adjust_control();
  }
  else {
    // dualshock controller
    if(ps2_error == 1){ // reset board
      resetFunc();
    }
    ps2x.read_gamepad(false, vibrate); // read controller and set large motor to spin at 'vibrate' speed
    process_controller();
  }

  if (printTOF_flag) {
    Serial.print('u'); Serial.print(lTof); Serial.print(',');
    printTOF_flag = false; 
  }
  if (printSONIC_flag) {
    Serial.print(lUsonic); Serial.print(' '); Serial.print(rUsonic); Serial.print(',');
    printSONIC_flag = false;
  }
  if (printlTofSub_flag) {
    Serial.print('p'); Serial.print(lTofSub); Serial.print(' '); Serial.print(rTof); Serial.print(',');
    printlTofSub_flag = false;
  }
  if (printCeilTof_flag) {
    Serial.print('y'); Serial.print(CeilTof); Serial.print(',');
    printCeilTof_flag = false;
  }
  if (print_state_flag) {
    if(estopFlag || false_alarm) Serial.print("s"); // Emergency Stop state
    else if(!align_flag && !stride_flag && !adjust_flag && !estopFlag && !false_alarm && !onStart_speed) Serial.print("okla"); // Normal state
    else if (align_flag || stride_flag || adjust_flag || onStart_speed) Serial.print("move"); // Moving state
    else Serial.print("NO"); // Invalid State
    print_state_flag = false; 
  }


  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (incomingByte == 'C' || incomingByte == 'c' || incomingByte == 'N' || incomingByte == 'n') {
      int32_t target = Serial.parseInt();
      if(target - lTofSub - 129 >= (int32_t)(rTof * 26559.0/1019.0)) { // width of the AGV: 670 mm
        target = lTofSub + (int32_t)((rTof - 18) * 26559.0/1019.0);
      }
      process_terminal(incomingByte, target);
    }
    else process_terminal(incomingByte);
    Serial.flush();
  }

  delayMicroseconds(50000); // TODO Very important 
}



void send_485() { // This function to send out 485 com to the AGV. Don't touch this part!
  if(!estopFlag && !false_alarm) {
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
}

void read_sensor() { // This function to read sensor data and average them
  lUsonicRead = lUsonicFilter.add(analogRead(L_usonic)); lUsonic = lUsonicFilter.get();
  rUsonicRead = rUsonicFilter.add(analogRead(R_usonic)); rUsonic = rUsonicFilter.get();
  lTofRead = lTofFilter.add(ADS.readADC(0)); lTof = lTofFilter.get();  
  lTofSubRead = lTofSubFilter.add(ADS.readADC(2)); lTofSub = lTofSubFilter.get(); // front LTOF
  CeilTofRead = CeilTofFilter.add(ADS.readADC(3)); CeilTof = CeilTofFilter.get();
  // Serial.println(ADS.readADC(0));
  // Serial.println(lTofRead);
  // Serial.println(lTof);
  Usonic = (lUsonic + rUsonic) * 0.5;
  rTofRead = rTofFilter.add(analogRead(R_tof)); rTof = rTofFilter.get();
}

void align_control() {
  UsonicDiff = lUsonic - rUsonic;
  // if(UsonicDiff <= 32) {
    if (align_i< 2) { // only enter the align_control after the count is reached
      align_i++; 
      cmd_state = 0; 
    }
    else { 
      // if(digitalRead(FLidarZone1) < 1 && digitalRead(BLidarZone1) < 1 && digitalRead(FLidarZone2) < 1 && digitalRead(BLidarZone2) < 1) {
        align_i = 0; 
        if (UsonicDiff > (magicLabAlign*1.0)) {
          cmd_state = 4; // should rotate cw
          // Serial.println("CW");
        } 
        else if (UsonicDiff < (magicLabAlign*-1.0)) {
          cmd_state = 3; // should rotate ccw
          // Serial.println("CCW");
        }
        else { cmd_state = 0; align_flag = false; } // should not move
      // }
      // else {
      //   cmd_state = 0;
      // }
    }
  // }
  // else {
  //   cmd_state = 0;
  //   align_flag = false;
  // }
}

void stride_control() { // stride control based on FRONT LTOF (lTofSub)
  lTofDiff = strideTarget - lTofSub;
  if (lTofDiff > (magicLabStride*1.0)) { // should move right
    if(digitalRead(FLidarZone2) < 1) {
      if (rTof > 18) { // check right clearance (25 ADC value)
        if (lTofDiff < (magicLabStride*120*1.0) || rTof < 50) { //crawling speed
          if (strideSpeed_flag) strideSpeed(false);
          else {
            if (stride_i<1) { stride_i++; cmd_state = 0; }
            else { stride_i = 0; cmd_state = 8; } 
          }
        }
        else if (lTofDiff < (magicLabStride*400*1.0) && lTofDiff >= (magicLabStride*120*1.0)) { // low speed
          if (strideSpeed_flag) strideSpeed(false);
          else cmd_state = 8;       
        }
        else if (lTofDiff >= (magicLabStride*400*1.0)) { // high speed
        if (!strideSpeed_flag) strideSpeed(true);
        else cmd_state = 8; 
        }
      }
      else { // right clearance is not enough. Stop
        cmd_state = 0; stride_flag = false;
      }
    }
    else { // something on the right side of the AGV
      cmd_state = 0;
    }
  }
  else if (lTofDiff < (magicLabStride*-1.0)) { // should move left
    // if(digitalRead(BLidarZone2) < 1) {
      if (lTofSub > 520) { // check left clearance
        if(lTofDiff > (magicLabStride*120*-1.0) || lTofSub < 1040) { //crawling speed
          if (strideSpeed_flag) strideSpeed(false);
          else {
            if (stride_i<1) { stride_i++; cmd_state = 0; }
            else { stride_i = 0; cmd_state = 7; } 
          }
        }
        else if (lTofDiff <= (magicLabStride*120*-1.0) && lTofDiff > (magicLabStride*400*-1.0)) { // low speed
          if (strideSpeed_flag) strideSpeed(false);
          else cmd_state = 7;       
        }
        else if (lTofDiff <= (magicLabStride*400*-1.0)) { // high speed
          if (!strideSpeed_flag) strideSpeed(true);
          else cmd_state = 7; 
        }
      }
      else { // left clearance is not enough. Stop
        cmd_state = 0; stride_flag = false;
      }
    // }
    // else { // something on the left side of the AGV
    //   cmd_state = 0;
    // }
  }
  else { //should not move
    cmd_state = 0; stride_flag = false;
  }
}

void strideSpeed(bool speed) {
  if (speed) { // to set highest speed
    if (speed_i<30) { speed_i++; cmd_state = 6; }
    else { speed_i = 0; cmd_state = 0; strideSpeed_flag = true; }
  }
  else { // to set lowest speed
    if (speed_i<30) { speed_i++; cmd_state = 5;}
    else { speed_i = 0; cmd_state = 0; strideSpeed_flag = false; }
  }
}

void adjust_control() {
  int UsonicDiff = abs(adjustTarget - Usonic);
    if (abs(Usonic - adjustTarget) > magicLabAdjust) {
      if (Usonic > adjustTarget) { // shall move forward
        // if(digitalRead(FLidarZone1) < 1) {
          if(lUsonic > 270 && rUsonic > 270) { // only move when both of them is higher than 270 ADC (avoid vision head crashing)
            if (UsonicDiff < 50) { // crawling speed
              if(adjusting_cnt == 0) {
                if (adjust_i<2) { adjust_i++; cmd_state = 0;}
                else { adjust_i = 0; cmd_state = 1; }
              }
              else {                                      
                cmd_state = 0;
                delay(500);
                adjusting_cnt = 0;
              }
            } 
            else {
              cmd_state = 1; // low speed
              adjusting_cnt++;
            }
          }
          else { //should not move
            cmd_state = 0; adjust_flag = false; adjusting_cnt = 0;
          } 
        // }
        // else { // something on the front side of the AGV
        //   cmd_state = 0;
        // }                                                                                          
      }
      
      else if (Usonic < adjustTarget) { // shall move backward
        // if(digitalRead(BLidarZone1) < 1) {
          if(Usonic < 1022) {
            if(UsonicDiff < 50) { // crawling speed
              if(adjusting_cnt == 0) {
                if (adjust_i<2) { adjust_i++; cmd_state = 0;}
                else { adjust_i = 0; cmd_state = 2; }
              }
              else {
                cmd_state = 0;
                delay(500);
                adjusting_cnt = 0;
              }
            } 
            else {
              cmd_state = 2; // low speed
              adjusting_cnt++;
            }
          }
          else { //should not move
            cmd_state = 0; adjust_flag = false; adjusting_cnt = 0;
          }
        // }
        // else { // something on the back side of the AGV
        //   cmd_state = 0;
        // }
      }

    }
    else { //should not move
      cmd_state = 0; adjust_flag = false; adjusting_cnt = 0;
    }
}

void process_terminal(int incomingByte, int32_t target) { // This function to process the incoming terminal command
  if (incomingByte == 32) { // space (idle)
    cmd_state = 0; 
    align_flag = false;
    stride_flag = false;
    adjust_flag = false;
    strideSpeed_flag = false;
    false_alarm = false;
    Serial.println("Reset");
  }
  else if ((incomingByte == 87) || (incomingByte == 119)) { // W or w (forward)
    // if(lUsonic > 180 && rUsonic > 180)  // if both sensors are not blocked (100 mm == 180 ADC)
    if(digitalRead(FLidarZone1) < 1) {  // if front sensor is not blocked
      cmd_state = 1;
    }
    else cmd_state = 0;
    } 
  else if ((incomingByte == 83) || (incomingByte == 115)) { // S or s (backward)
    if(digitalRead(BLidarZone1) < 1) {  // if front sensor is not blocked
      cmd_state = 2;
    }
    else cmd_state = 0;
  } 
  else if ((incomingByte == 81) || (incomingByte == 113)) { // Q or q (ccw)
    // if(lTof > 3448 && rTof > 20 && lUsonic > 390 && rUsonic > 390) // L, RTOF > 650 mm , L, RUSONIC > 200 mm
    // if(digitalRead(FLidarZone2) < 1 && digitalRead(BLidarZone2) < 1) { 
      cmd_state = 3;
    // }
    // else cmd_state = 0;
  } 
  else if ((incomingByte == 69) || (incomingByte == 101)) { // E or e (cw)
    // if(lTof > 3448 && rTof > 20 && lUsonic > 390 && rUsonic > 390) // L, RTOF > 650 mm , L, RUSONIC > 200 mm
    // if(digitalRead(FLidarZone2) < 1 && digitalRead(BLidarZone2) < 1) {
      cmd_state = 4; 
    // }
    // else cmd_state = 0;
  } 
  else if (incomingByte == 45) {
    set_speed(SLOW); // - (slower)
  }
  else if (incomingByte == 61) {
    set_speed(FAST); // = (faster)
  }
  else if ((incomingByte == 65) || (incomingByte == 97)) { // A or a (left)
    // if(lTof > 521) // if left sensor is not blocked (100 mm == 521 ADC)
    // if(digitalRead(BLidarZone2) < 1) {
      cmd_state = 7; // A or a (left)
    // }
    // else cmd_state = 0;
  }
  else if ((incomingByte == 68) || (incomingByte == 100)) { // D or d (right)
    // if(rTof > 20) // if right sensor is not blocked (100 mm == 20 ADC)
    if(digitalRead(FLidarZone2) < 1) {
      cmd_state = 8;
    }
    else cmd_state = 0;
  }
  else if ((incomingByte == 77) || (incomingByte == 109)) { // M or m (align)
    align_flag = true; 
  }
  else if ((incomingByte == 67) || (incomingByte == 99)) { // C or c (stride)
    if(target <= 24037 && target >= 533) { // receiving range of 100 - 4500 mm only (distance = 0.1872*ADC + 0.2184) for lTofSub
      strideTarget = target;
      stride_flag = true;
      Serial.print('c');
      Serial.print(target);
      Serial.print(',');
    }
    else stride_flag = false;
  }
  else if ((incomingByte == 78) || (incomingByte == 110)) { // N or n (adjust)
    if(target <= 1002 && target >= 179) { // receiving range of 100 - 490 mm only
      adjustTarget = target;
      adjust_flag = true; 
      Serial.print('n');
      Serial.print(target);
      Serial.print(',');
    }
    else adjust_flag = false;
  }
  else if ((incomingByte == 80) || (incomingByte == 112)) { // P or p (print left FRONT TOF)
    printlTofSub_flag = true;
  }
  else if(incomingByte == '[') { // [ (print current state)
    print_state_flag = true;
  }
  else if(incomingByte == 'O' || incomingByte == 'o') { // O or o (print sonic)
    printSONIC_flag = true;
  }
  else if(incomingByte == 'U' || incomingByte == 'u') { // U or u (print left BACK and right TOF)
    printTOF_flag = true; 
  }
  else if(incomingByte == 'Y' || incomingByte == 'y') { // Y or y (print ceiling Tof)
    printCeilTof_flag = true;
  }
}

void process_controller() {     // Function to receive PS2 input
  if(ps2x.Button(PSB_START)) {         // start (idle)
    cmd_state = 0; 
    align_flag = false;
    stride_flag = false;
    adjust_flag = false;
    strideSpeed_flag = false;
    false_alarm = false;
    Serial.println("Reset");
  }
  else if (ps2x.Button(PSB_CROSS) && ps2x.Button(PSB_R1)) {
    set_speed(SLOW);    // - (slower)
  }
  else if (ps2x.Button(PSB_TRIANGLE) && ps2x.Button(PSB_R1)) {
    set_speed(FAST); // = (faster)
  }
  else if((ps2x.Button(PSB_PAD_UP) && ps2x.Button(PSB_R1)) && (!ps2x.Button(PSB_PAD_DOWN) || !ps2x.Button(PSB_SQUARE) || !ps2x.Button(PSB_CIRCLE) || !ps2x.Button(PSB_PAD_LEFT) || !ps2x.Button(PSB_PAD_RIGHT))) { // Up pad (forward)
    if((!ps2x.Button(PSB_L1) && ((current_speed == FAST && digitalRead(FLidarZone3) == 1) || digitalRead(FLidarZone1) == 1))) {
      cmd_state = 0;
    }
    else {
      cmd_state = 1;
    }
  }
  else if ((ps2x.Button(PSB_PAD_DOWN) && ps2x.Button(PSB_R1)) && (!ps2x.Button(PSB_PAD_UP) || !ps2x.Button(PSB_SQUARE) || !ps2x.Button(PSB_CIRCLE) || !ps2x.Button(PSB_PAD_LEFT) || !ps2x.Button(PSB_PAD_RIGHT))) {// Down pad (backward)
    if((!ps2x.Button(PSB_L1) && ((current_speed == FAST && digitalRead(BLidarZone3) == 1) || digitalRead(BLidarZone1) == 1))) {
      cmd_state = 0;
    }
    else {
      cmd_state = 2;
    }
  }
  else if ((ps2x.Button(PSB_SQUARE) && ps2x.Button(PSB_R1)) && (!ps2x.Button(PSB_PAD_UP) || !ps2x.Button(PSB_PAD_DOWN) || !ps2x.Button(PSB_CIRCLE) || !ps2x.Button(PSB_PAD_LEFT) || !ps2x.Button(PSB_PAD_RIGHT))) { // L1 (ccw)
    // if(!ps2x.Button(PSB_L1) && (digitalRead(FLidarZone2) == 1 || digitalRead(BLidarZone2) == 1)) { 
    //   cmd_state = 0;
    // }
    /*else*/ cmd_state = 3;
  } 
  else if ((ps2x.Button(PSB_CIRCLE) && ps2x.Button(PSB_R1)) && (!ps2x.Button(PSB_PAD_UP) || !ps2x.Button(PSB_PAD_DOWN) || !ps2x.Button(PSB_SQUARE) || !ps2x.Button(PSB_PAD_LEFT) || !ps2x.Button(PSB_PAD_RIGHT))) { // R1 (cw)
    // if(!ps2x.Button(PSB_L1) && (digitalRead(FLidarZone2) == 1 || digitalRead(BLidarZone2) == 1)) {
    //   cmd_state = 0; 
    // }
    /*else*/ cmd_state = 4;
  } 
  else if ((ps2x.Button(PSB_PAD_LEFT) && ps2x.Button(PSB_R1)) && (!ps2x.Button(PSB_PAD_UP) || !ps2x.Button(PSB_PAD_DOWN) || !ps2x.Button(PSB_SQUARE) || !ps2x.Button(PSB_CIRCLE) || !ps2x.Button(PSB_PAD_RIGHT))) { // Left pad (left)
    // if(lTof > 521) // if left sensor is not blocked (100 mm == 521 ADC)
    // if((!ps2x.Button(PSB_L1) && (digitalRead(BLidarZone2) == 1))) {
    //   cmd_state = 0; // A or a (left)
    // }
    /*else*/ cmd_state = 7;
  }
  else if ((ps2x.Button(PSB_PAD_RIGHT) && ps2x.Button(PSB_R1)) && (!ps2x.Button(PSB_PAD_UP) || !ps2x.Button(PSB_PAD_DOWN) || !ps2x.Button(PSB_SQUARE) || !ps2x.Button(PSB_CIRCLE) || !ps2x.Button(PSB_PAD_LEFT))) { // Right pad (right)
    // if(rTof > 20) // if right sensor is not blocked (100 mm == 20 ADC)
    if((!ps2x.Button(PSB_L1) && (digitalRead(FLidarZone2) == 1))) {
      cmd_state = 0;
    }
    else cmd_state = 8;
  }
}

void set_speed(SPEED speed) {
  unsigned long first_trigger = millis();
  if (speed == FAST) {
    while (millis() - first_trigger < 3500) {
      cmd_state = 6;
    }
    current_speed = FAST;
    strideSpeed_flag = true;
  } else {
    while (millis() - first_trigger < 3500) {
      cmd_state = 5;
    }
    current_speed = SLOW;
    strideSpeed_flag = false;
  }
}

void controller_setup() {
  delay(500);  //added delay to give wireless ps2 module some time to startup, before configuring it
     
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(ps2_error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures) {
      Serial.println("true ");
    }
    else {
      Serial.println("false");
    }
    Serial.print("rumble = ");
    if (rumble) {
      Serial.println("true)");
    }
    else {
      Serial.println("false");
    }
  }  
  else if(ps2_error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug");
   
  else if(ps2_error == 2)
    Serial.println("Controller found but not accepting commands");

  else if(ps2_error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
   }
}