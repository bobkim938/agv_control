#include <RS485.h>
#include <TimerOne.h>
#include <Arduino.h>

class calibration {
    public:
        calibration();
        float setOffset();
        ~calibration();
    private:
        float offset;
        int tof;
};