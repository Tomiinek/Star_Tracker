#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include "../config.h"

class CameraController {
  
    public:

        // set PIN and default values, call only from setup procedure!
        void initialize() {         
            pinMode(TRIGGER_PIN, OUTPUT);
            delay(1);
            reset();
        }

        // triggers shooting, camera dependent, should have specific implementation
        // cannot be blocking because of long exposures, timers are overkill as we do not
        // need to be very precise in this scenario
        // should take into account repeating snaps (like burst mode) with delay between them
        virtual void shoot(int duration, int delay) = 0;
        // returns true while is shooting, false otherwise
        virtual boolean update() = 0; 

        // immediately stop all shutter releasing activity and reset counter
        inline void reset() {
            stop();
            _last_invoked = 0;
            _last_duration = 0;
            _last_delay = 0;
        }

        inline void set_repeating(boolean repeating) { _repeating = repeating; }
        inline bool get_repeating() { return _repeating; }

    protected:

        // stop shooting
        virtual void stop() = 0;

        unsigned long _last_invoked = 0;
        unsigned long _last_duration = 0;
        unsigned long _last_delay = 0;
        boolean _repeating = false;
};

#endif

