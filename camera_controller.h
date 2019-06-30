#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include "config.h"

class CameraController {
  
    public:

        // set PIN and default values, call only from setup procedure!
        void initialize() {         
            pinMode(TRIGGER_PIN, OUTPUT);
            delay(1);
            reset();
        }

        // triggers single shooting, camera dependent, should have specific implementation
        // cannot be blocking because of long exposures, timers are overkill as we do not
        // need to be very precise in this scenario
        // returns true while is shooting, false otherwise
        virtual boolean shoot(int duration) = 0;

        // immediately stop all shutter releasing activity and reset counter
        inline void reset() {
            digitalWrite(TRIGGER_PIN, HIGH);
            _last_invoked = 0;
            _last_duration = 0;
            _repeated = 0;
        }

        inline int get_repetition_num() { return _repeated; }

    protected:

        long _last_invoked;
        long _last_duration;
        int _repeated;
};

#endif

