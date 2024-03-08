#include "calibration.h"

calibration::calibration() {
    offset = 0;
    tof = 0;
}

float calibration::setOffset() {
    while (!Serial);
    Serial.println("Place the AGV at the starting position and press the C or c for calibration");
    for(int i = 0; i < 10; i++) {
        tof += analogRead(A2); // read current tof value
    }
    float avg_tof = tof / 10;
    offset = avg_tof;
    
    return offset;
}
