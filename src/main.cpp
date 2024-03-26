#include <Arduino.h>
#include <TimerOne.h>
#include <RS485.h>
#include <MovingAverage.h>
#include <ADS1X15.h>

#define R_usonic A0
#define L_usonic A1
#define R_tof A3

// Emergency Stop Interrupt
const uint8_t interruptPin = 3;
volatile bool estopFlag;

// function prototypes
void send_485();
void read_sensor();
void align_control();
void stride_control();
void adjust_control();
void set_speed(bool speed);
void process_terminal(int incomingByte, int target = 0);
void speed_to_lowest();
void estop(); // interrupt for estop pressed, check for debounce
void unstop(); 

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

bool align_flag, stride_flag, adjust_flag, printTOF_flag, printSONIC_flag, speed_flag, print_state_flag, false_alarm = false;
uint8_t align_i, stride_i, adjust_i, speed_i, cmd_state, adjust_speed_i = 0;
int adjustTarget;
int lUsonicRead, rUsonicRead, lTofRead, rTofRead;
int lUsonic, rUsonic, Usonic, UsonicDiff, rTof;
int lTof, strideTarget, prev_ltof;
long lTofDiff;
bool adjust_lowest = false;
unsigned long prev_time;
MovingAverage <int, 4> lUsonicFilter;
MovingAverage <int, 4> rUsonicFilter;
MovingAverage <int, 4> lTofFilter;
MovingAverage<int, 4> rTofFilter;

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
  Serial1.begin(9600);
  while (!Serial); 
  pinMode(interruptPin, INPUT_PULLUP); //Inverts the behavior of the INPUT mode, HIGH means off, LOW means on
  attachInterrupt(digitalPinToInterrupt(interruptPin), estop, LOW);
  Timer1.initialize(50000); // 50 milliseconds
  Timer1.attachInterrupt(send_485); 
  Wire.begin();
  if (!ADS.begin()) Serial.println("Invalid I2C address");
  if (!ADS.isConnected()) Serial.println("ADS1115 is not connected");
}


void loop() { // put your main code here, to run repeatedly:
  read_sensor();
  if((abs(prev_ltof - lTof) > 200 || lTof < 520) && stride_flag) false_alarm = true;
  prev_ltof = lTof;
  if(estopFlag || false_alarm) {
    cmd_state = 0;
    align_flag = false;
    stride_flag = false;
    adjust_flag = false;
    speed_flag = false;
  }
  if (align_flag) align_control(); 
  // if (align_i<2) { align_i++; cmd_state = 0; }
  // else { align_i = 0; align_control();
  if (stride_flag) stride_control();
  if (adjust_flag) {
    if(!adjust_lowest) speed_to_lowest();
    else adjust_control();
  }
  if (printTOF_flag) {
    //Serial.print("L: "); Serial.print(lUsonic); Serial.print('\t'); 
    //Serial.print("R: "); Serial.print(rUsonic); Serial.print('\t'); 
    //Serial.print("ToF: "); Serial.println(lTof);
    Serial.print('p');
    Serial.print(lTof);
    Serial.print(',');
    printTOF_flag = false; 
  }
  if (printSONIC_flag) {
    Serial.print(lUsonic);
    Serial.print(' ');
    Serial.print(rUsonic);
    Serial.print(',');
    printSONIC_flag = false;
  }
  if (print_state_flag) {
    if(estopFlag || false_alarm) Serial.print("s"); // Emergency Stop state
    else if(!align_flag && !stride_flag && !adjust_flag && !estopFlag && !false_alarm) Serial.print("okla"); // Normal state
    else if (align_flag || stride_flag || adjust_flag) Serial.print("move"); // Moving state
    else Serial.print("NO"); // Invalid State
    print_state_flag = false; 
  }

  int SerialAvailable = Serial.available();
  if(SerialAvailable <= 0) return;
  int incomingByte = Serial.read();
  if (incomingByte == 'C' || incomingByte == 'c' || incomingByte == 'N' || incomingByte == 'n') {
    char buffer[SerialAvailable - 1];
    Serial.readBytes(buffer, SerialAvailable - 1);
    int i = 0;
    for(i; i < SerialAvailable - 1; i++) {
      if(buffer[i] == ',') break;
    }
    if(i == 7) {
      delay(100);
      Serial.print(buffer);
      return;
    }
    *(buffer + i) = '\0';
    int target = atoi(buffer);
    process_terminal(incomingByte, target);
  }
  // if (Serial.available() <= 0) return;
  // int incomingByte = Serial.read();
  // if (incomingByte == 'C' || incomingByte == 'c' || incomingByte == 'N' || incomingByte == 'n') {
  //   char buffer[7];
  //   Serial.readBytes(buffer, 7);
  //   int i = 0;
  //   for(i; i < 7; i++) {
  //     if(buffer[i] == ',') break;
  //   }
  //   if(i == 7) {
  //     delay(100);
  //     Serial.print(buffer);
  //     return;
  //   }
  //   *(buffer + i) = '\0';
  //   int target = atoi(buffer);
  //   process_terminal(incomingByte, target);
  // }
  else process_terminal(incomingByte);
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
  Usonic = (lUsonic + rUsonic) * 0.5;
  rTofRead = rTofFilter.add(analogRead(R_tof)); rTof = rTofFilter.get();
}

void align_control() {
  UsonicDiff = lUsonic - rUsonic;
  if (align_i<1) { // only enter the align_control after the count is reached
    align_i++; 
    cmd_state = 0; 
  }
  else { 
    align_i = 0; 
    if (UsonicDiff > (magicLabAlign*1.0)) { // should rotate cw
      cmd_state = 4;
    }
    else if (UsonicDiff < (magicLabAlign*-1.0)) { // should rotate ccw
      cmd_state = 3; 
    }
    else { // should not move
    //  Serial.println("Align done"); // reply back to GUI
      cmd_state = 0; align_flag = false;
    }
    // Serial.print("L: "); Serial.print(lUsonic); Serial.print('\t'); 
    // Serial.print("R: "); Serial.println(rUsonic); 
  }
}

void stride_control() {
  lTofDiff = strideTarget - lTof;
  if (lTofDiff > (magicLabStride*1.0)) { // should move right
    if (lTofDiff < (magicLabStride*40*1.0)) { //crawling speed
      if (speed_flag) set_speed(false);
      else {
        if (stride_i<1) { stride_i++; cmd_state = 0; }
        else { stride_i = 0; cmd_state = 8; } 
      }
    }
    else if (lTofDiff < (magicLabStride*400*1.0) && lTofDiff >= (magicLabStride*40*1.0)) { // low speed
      if (speed_flag) set_speed(false);
      else cmd_state = 8;       
    }
    else { // high speed
      if (!speed_flag) set_speed(true);
      else cmd_state = 8; 
    }
  }
  else if (lTofDiff < (magicLabStride*-1.0)) { // should move left
    if (lTofDiff < (magicLabStride*400*-1.0)) { // high speed
      if (!speed_flag) set_speed(true);
      else cmd_state = 7; 
    }
    else if (lTofDiff < (magicLabStride*40*-1.0) && lTofDiff >= (magicLabStride*400*-1.0)) { // low speed
      if (speed_flag) set_speed(false);
      else cmd_state = 7;       
    }
    else { //crawling speed
      if (speed_flag) set_speed(false);
      else {
        if (stride_i<1) { stride_i++; cmd_state = 0; }
        else { stride_i = 0; cmd_state = 7; } 
      }
    }
  }
  else { //should not move
  //  Serial.println("Stride done"); // reply back to GUI
    cmd_state = 0; stride_flag = false;
  }
  // Serial.print("ToF: "); Serial.print(lTof); Serial.print('\t'); 
  // Serial.print("Target: "); Serial.println(strideTarget); 
}

void adjust_control() {
  int UsonicDiff = abs(adjustTarget - Usonic);
  if(Usonic > 270 && Usonic < 1022)
  {
    if(abs(Usonic - adjustTarget) > magicLabAdjust){
      if(Usonic > adjustTarget) { // shall move forward
        if(UsonicDiff < 75) 
        { // crawling speed
          if (adjust_i<1) 
          { 
            adjust_i++; cmd_state = 0; 
          }
          else 
          {
            adjust_i = 0; cmd_state = 1;
          }
        } 
        else 
        { // low speed
            cmd_state = 1;
        }
      }
      else if(Usonic < adjustTarget) { // shall move backward
        if(UsonicDiff < 75) 
        { // crawling speed
          if (adjust_i<1) 
          { 
            adjust_i++; cmd_state = 0; 
            }
          else 
          {
            adjust_i = 0; cmd_state = 2;
          }
        } else 
        { // low speed
          cmd_state = 2;
        }
      }
    } 
    else { //should not move
    //  Serial.println("Adjust done"); // reply back to GUI
      cmd_state = 0; adjust_flag = false; adjust_speed_i = 0;
      adjust_lowest = false;
    }
  }
  else
  { //should not move
  //  Serial.println("Adjust done"); // reply back to GUI
    cmd_state = 0; adjust_flag = false; adjust_speed_i = 0;
    adjust_lowest = false;
  }
  // Serial.print("L: "); Serial.print(lUsonic); Serial.print('\t'); 
  // Serial.print("R: "); Serial.println(rUsonic); 
}

void set_speed(bool speed) {
  if (speed) { // to set highest speed
    if (speed_i<20) { speed_i++; cmd_state = 6; }
    else { speed_i = 0; cmd_state = 0; speed_flag = true; }
  }
  else { // to set lowest speed
    if (speed_i<20) { speed_i++; cmd_state = 5;}
    else { speed_i = 0; cmd_state = 0; speed_flag = false; }
  }
}

void process_terminal(int incomingByte, int target = 0) { // This function to process the incoming terminal command
  if (incomingByte == 32) { // space (idle)
    cmd_state = 0; 
    align_flag = false;
    stride_flag = false;
    adjust_flag = false;
    speed_flag = false;
    false_alarm = false;
    Serial.println("Reset");
  }
  else if ((incomingByte == 87) || (incomingByte == 119)) cmd_state = 1; // W or w (forward)
  else if ((incomingByte == 83) || (incomingByte == 115)) cmd_state = 2; // S or s (backward)
  else if ((incomingByte == 81) || (incomingByte == 113)) cmd_state = 3; // Q or q (ccw)
  else if ((incomingByte == 69) || (incomingByte == 101)) cmd_state = 4; // E or e (cw)
  else if (incomingByte == 45) cmd_state = 5; // - (slower)
  else if (incomingByte == 61) cmd_state = 6; // = (faster)
  else if ((incomingByte == 65) || (incomingByte == 97)) cmd_state = 7; // A or a (left)
  else if ((incomingByte == 68) || (incomingByte == 100)) cmd_state = 8; // D or d (right)
  else if ((incomingByte == 77) || (incomingByte == 109)) { // M or m (align)
    align_flag = true; 
  }
  else if ((incomingByte == 67) || (incomingByte == 99)) { // C or c (stride)
    if(target <= 18655 && target >= 521) { // receiving range of 100 - 3500 mm only
      strideTarget = target;
      // valid if 15
      stride_flag = true;
      Serial.print('c');
      Serial.print(target);
      // Serial.print("ToF: "); Serial.print(lTof); Serial.print('\t'); 
      // Serial.print("Target: "); Serial.println(strideTarget); 
    }
    else {
      stride_flag = false;
    }
  }
  else if ((incomingByte == 78) || (incomingByte == 110)) { // N or n (adjust)
    if(target <= 1002 && target >= 179) { // receiving range of 100 - 490 mm only
      adjustTarget = target;
      adjust_flag = true; 
      Serial.print('n');
      Serial.print(target);
      // Serial.print("Sonic: "); Serial.print(Usonic); Serial.print('\t'); 
      // Serial.print("Target: "); Serial.println(adjustTarget); 
    }
    else {
      adjust_flag = false;
    }
  }
  else if ((incomingByte == 80) || (incomingByte == 112)) { // P or p (printTOF)
    printTOF_flag = true; 
  }
  else if(incomingByte == '[') { // [ (print current state)
    print_state_flag = true;
  }
  else if(incomingByte == 'O' || incomingByte == 'o') { // O or o (print sonic)
    printSONIC_flag = true;
  }
}

void speed_to_lowest() {
  if(adjust_speed_i == 0) {
    prev_time = millis();
    adjust_speed_i++;
  }
  if(millis() - prev_time < 3000) {
    cmd_state = 5;
  }
  else {
    adjust_lowest = true;
  }
}