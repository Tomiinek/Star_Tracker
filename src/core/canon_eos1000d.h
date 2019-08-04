#ifndef CAMERAEOS1000D_H
#define CAMERAEOS1000D_H

#include "../config.h"
#include "camera_controller.h"

class CanonEOS1000D : public CameraController {

    public:
    
        void shoot(int duration_seconds, int delay_seconds) override {
            if (update()) return;
            if (_repeating) _repeating_invoked = true;
            shoot_internal(duration_seconds * 1000, delay_seconds * 1000);
        }

        void shoot_internal(int duration_ms, int delay_ms) {
            digitalWrite(TRIGGER_PIN, HIGH);
            _last_delay = delay_ms ;
            _last_duration = duration_ms ;
            _last_invoked = millis();
        }

        boolean update() override {
            long from_last_snap = millis() - _last_invoked;

            if (from_last_snap < _last_duration) return true;

            if (from_last_snap < _last_duration + SNAP_DELAY_MS) {
                int curr_state = digitalRead(TRIGGER_PIN);
                if (curr_state == HIGH) digitalWrite(TRIGGER_PIN, LOW);  
                return true;
            }

            if (_repeating_invoked && from_last_snap > _last_duration + _last_delay) shoot_internal(_last_duration, _last_delay);

            return false;
        }

        void stop() override {
            _repeating_invoked = false;
            digitalWrite(TRIGGER_PIN, LOW);
        }

    private: 

        bool _repeating_invoked = false;
};

#endif

