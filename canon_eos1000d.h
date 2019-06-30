#ifndef CAMERAEOS1000D_H
#define CAMERAEOS1000D_H

#include "config.h"
#include "camera_controller.h"

class CanonEOS1000D : public CameraController {

    public:
    
        boolean shoot(int duration) override {
            long from_last_snap = millis() - _last_invoked;
            if (from_last_snap < _last_duration) return true;
            if (from_last_snap < _last_duration + SNAP_DELAY_MS) {
                int curr_state = digitalRead(TRIGGER_PIN);
                if (curr_state == LOW) {
                    digitalWrite(TRIGGER_PIN, HIGH);            
                    ++_repeated;
                    return false;
                }
                return false;
            }
            digitalWrite(TRIGGER_PIN, LOW);
            _last_duration = duration * 1000;
            _last_invoked = millis();
            return true;
        }
};

#endif

