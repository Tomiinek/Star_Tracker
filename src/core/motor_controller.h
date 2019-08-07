#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "../config.h"
#include "queue.h"

#define TMR_RESOLUTION  64
#define TIMER_TOP (F_CPU / (1000000.0 / TMR_RESOLUTION))

class MountController;
class MotorController {
    
    public:

        // singleton class
        MotorController(MotorController const&) = delete;
        void operator=(MotorController const&)  = delete;
        static MotorController& instance() {
            static MotorController instance;
            return instance;
        }
        
        // set PINs and default values
        void initialize();

        // returns true if motors have absolutely no job
        inline bool is_ready() { return _dec.pulses_remaining == 0 && _ra.pulses_remaining == 0; }

        // interrupts all motor movements and clear command queue
        void stop();

        // estimates time (millis) of the complete fast_turn duration
        float estimate_fast_turn_time(float revs_dec, float revs_ra);
        
        // make a fast turn with subsequent slow turn for compensate the coarse resolution of full step
        void fast_turn(float revs_dec, float revs_ra, boolean queueing);

        // make a turn with given motor revolutions per second and with microstepping enabled (implies low speed)
        void slow_turn(float revs_dec, float revs_ra, float speed_dec, float speed_ra, boolean queueing);

        // interrupt service rutine
        void trigger();

        // returns the number of revolutions relative to the starting position
        void get_made_revolutions(float& dec, float& ra) {
            #ifdef DEBUG
                Serial.println(F("Revolutions"));
                Serial.print(F(" DEC: ")); Serial.println(_dec_balance); 
                Serial.print(F("  RA: ")); Serial.println(_ra_balance); 
            #endif
            dec = (float) _dec_balance / 2.0f / STEPS_PER_REV_DEC / MICROSTEPPING_MUL;
            ra = (float) _ra_balance / 2.0f / STEPS_PER_REV_RA / MICROSTEPPING_MUL;
        }

    private:
        MotorController() {}

        // structure holding state of motors and movement while executing a command
        struct motor_data {
            volatile uint32_t steps_total = 0;  // steps to be done during this particular movement
            volatile uint32_t pulses_remaining = 0;  // pulses to be done until the end of this movement
            volatile uint32_t pulses_until_correction = 0;  // pulses to be done until inserting an extra pulse
            volatile uint32_t pulses_to_correct = 0;  // number of pulses after which is done a correction
            volatile uint32_t mcu_ticks_per_pulse = 0;  // number of ticks after which is done a pulse
            volatile uint32_t pulses_to_accel = 0;  // number of pulses after which is done an ac/deceleration
            volatile uint32_t start_steps_delay = 0;  // delay between steps at the start of fast movement
            volatile uint32_t target_steps_delay = 0;  // minimal delay between steps during fast movement
            volatile uint32_t current_steps_delay = 0;  // current delay between steps
            volatile uint32_t ticks_passed = 0;  // counter of ticks for triggering pulse (see mcu_ticks_per_pulse)
            volatile bool correction = false;  // flag to state whether do or do not do correction 
        };

        // structre holding a command for motors
        struct command_t {
            float revs_dec;  // desired number of revolutions of DEC
            float revs_ra;  // desired number of revolutions of RA
            unsigned long delay_start_dec;  // starting delay between steps - DEC
            unsigned long delay_start_ra;  // starting delay between steps - RA
            unsigned long delay_end_dec;  // minimal delay between steps - DEC
            unsigned long delay_end_ra;  // minimal delay between steps - RA
            bool microstepping;  // whether enable microstepping
        };

        // estimates time (millis) of the complete fast_turn duration of a single motor
        float estimate_motor_fast_turn_time(float steps, int accel_each, int accel_amount, int dalay_start, int dalay_end);

        // make a turn of specified angles, speed (starting, ending) and command queueing
        void turn_internal(command_t cmd, bool queueing);

        // set job to move specified number of steps with delays between them
        void step_micros(motor_data& data, long steps, unsigned long micros_between_steps);

        // performs acceleration or decceleration 'amount' if 'change_steps' passed
        inline void change_motor_speed(motor_data& data, int change_steps, int amount);

        // subrutine of the interrupt service rutine, returns microsteps which were done
        inline int motor_trigger(motor_data& data, byte pin, byte dir, bool dir_swap, byte ms);

        // returns true if 'value' is defferent from current value (1 or 0) and changes pin appropriately 
        inline bool change_pin(byte pin, byte value);

        inline void revs_to_steps(float &steps_dec, float &steps_ra, float revs_dec, float revs_ra, bool microstepping) {
            steps_dec = abs(revs_dec) * STEPS_PER_REV_DEC * (microstepping ? MICROSTEPPING_MUL : 1);
            steps_ra  = abs(revs_ra)  * STEPS_PER_REV_RA  * (microstepping ? MICROSTEPPING_MUL : 1);
        }

        inline void steps_to_revs(float &revs_dec, float &revs_ra, float steps_dec, float steps_ra, bool microstepping) {
            revs_dec = steps_dec / STEPS_PER_REV_DEC / (microstepping ? MICROSTEPPING_MUL : 1);
            revs_ra  = steps_ra  / STEPS_PER_REV_RA  / (microstepping ? MICROSTEPPING_MUL : 1);
        }

        // some motor state variables
        motor_data _dec;
        motor_data _ra;
        queue<command_t> _commands;

        long _dec_balance;
        long _ra_balance;
};

#ifndef FROM_LIB
ISR(TIMER5_COMPA_vect) { MotorController::instance().trigger(); }
#endif

#endif

