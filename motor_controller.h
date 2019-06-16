#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "config.h"

#define TMR1_PRESCLR 64
#define TMR1_PRESCLR_BITS (1 << CS10) | (1 << CS11)
#define TMR1_RESOLUTION  25

#define OCR1A_TIMER1_TOP (F_CPU / TMR1_PRESCLR / (1000000.0 / TMR1_RESOLUTION))

class MotorController {
  
  public:

    // singleton class
    static MotorController& instance() {
      static MotorController instance;
      return instance;
    }
    MotorController(MotorController const&) = delete;
    void operator=(MotorController const&)  = delete;

    // set PINs and default values
    void initialize();

    // returns true if motors have absolutely no job
    inline bool is_ready() { return _dec.pulses_remaining == 0 && _ra.pulses_remaining == 0; }

    // interrupts all motor movements
    void stop();
    
    // make a turn of specified angles, speed and 
    void turn(float angle_dec, float angle_ra, bool fast = true, bool over_ride = false);

    // interrupt service rutine
    void trigger();

  private:
    MotorController() {}

    struct motor_data {
      volatile uint32_t pulses_remaining = 0;
      volatile uint32_t pulses_remaining_correction = 0;
      volatile uint32_t pulses_to_correction = 0;
      volatile uint32_t mcu_ticks_per_pulse = 0; 
      volatile uint32_t pulses_remaining_accel = 0;
      volatile uint16_t target_steps_delay = 0;
      volatile uint16_t current_steps_delay = 0;
      volatile uint32_t ticks_passed = 0;
    };

    // set job to move specified number of steps with delays between them
    void step_micros(motor_data& data, long steps, long micros_between_steps);

    // subrutine of the interrupt service rutine
    void motor_trigger(motor_data& data, byte pin);

    // returns true if 'value' is defferent from current value (1 or 0) and changes pin appropriately 
    inline bool change_pin(byte pin, byte value);

    // some motor state variables
    motor_data _dec;
    motor_data _ra;
};

#ifndef FROM_LIB
ISR(TIMER1_COMPA_vect)
{
  MotorController::instance().trigger();
}
#endif

#endif

