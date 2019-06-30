#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "config.h"
#include "queue.h"

#define TMR1_PRESCLR 64
#define TMR1_PRESCLR_BITS (1 << CS10) | (1 << CS11)
#define TMR1_RESOLUTION  50

#define OCR1A_TIMER1_TOP (F_CPU / TMR1_PRESCLR / (1000000.0 / TMR1_RESOLUTION))

class MountController;
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

    // interrupts all motor movements and clear command queue
    void stop();
    
    // make a fast turn
    void fast_turn(float revs_dec, float revs_ra, boolean queueing);

    // make a turn with given motor revolutions per second and with microstepping enabled (implies low speed)
    void slow_turn(float revs_dec, float revs_ra, float speed_dec, float speed_ra, boolean queueing);

    // interrupt service rutine
    void trigger();

    // returns the number of revolutions relative to the last call of this method
    void pop_made_revolutions(float& dec, float& ra) {
      dec = (float) _dec_balance / 2.0f / STEPS_PER_REV_DEC / MICROSTEPPING_MUL;
      ra = (float) _ra_balance / 2.0f / STEPS_PER_REV_RA / MICROSTEPPING_MUL;
      _dec_balance = 0;
      _ra_balance = 0;
    }

  private:
    MotorController() {}

    struct motor_data {
      volatile uint32_t pulses_remaining = 0;
      volatile uint32_t pulses_remaining_correction = 0;
      volatile uint32_t pulses_to_correction = 0;
      volatile uint32_t mcu_ticks_per_pulse = 0; 
      volatile uint32_t pulses_remaining_accel = 0;
      volatile uint32_t target_steps_delay = 0;
      volatile uint32_t current_steps_delay = 0;
      volatile uint32_t ticks_passed = 0;
    };

    struct command_t {
      float revs_dec;
      float revs_ra;
      unsigned long delay_start_dec;
      unsigned long delay_start_ra;
      unsigned long delay_end_dec;
      unsigned long delay_end_ra; 
      bool microstepping;
    };

    // make a turn of specified angles, speed (starting, ending) and command queueing
    void turn_internal(command_t cmd, bool queueing);

    // set job to move specified number of steps with delays between them
    void step_micros(motor_data& data, long steps, long micros_between_steps);

    // subrutine of the interrupt service rutine, returns microsteps which were done
    inline int motor_trigger(motor_data& data, byte pin, byte dir, byte dir_swap, byte ms);

    // returns true if 'value' is defferent from current value (1 or 0) and changes pin appropriately 
    inline bool change_pin(byte pin, byte value);

    // some motor state variables
    motor_data _dec;
    motor_data _ra;
    queue<command_t> _commands;

    long _dec_balance;
    long _ra_balance;
};

#ifndef FROM_LIB
ISR(TIMER1_COMPA_vect)
{
  MotorController::instance().trigger();
}
#endif

#endif

