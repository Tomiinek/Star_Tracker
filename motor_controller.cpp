#define FROM_LIB

#include <Arduino.h>
#include "motor_controller.h"

void MotorController::initialize() {
    
    uint8_t pin_mask = (1 << STEP_PIN_DEC) | (1 << DIR_PIN_DEC) | (1 << MS_PIN_DEC) |
                       (1 << STEP_PIN_RA)  | (1 << DIR_PIN_RA)  | (1 << MS_PIN_RA);

    MOTORS_DDR |= pin_mask;
    MOTORS_PORT &= ~pin_mask;

    #ifdef DEBUG
        Serial.println("Stepper motors pins initialized:");
        Serial.print("  Pinout: "); Serial.println(pin_mask, BIN);
        Serial.print("  DDR:    "); Serial.println(MOTORS_DDR, BIN);
        Serial.print("  PORT:   "); Serial.println(MOTORS_PORT, BIN);
    #endif

    // set Fast PWM mode
    TCCR1A |= (1 << WGM10) | (1 << WGM11); 
    TCCR1B |= (1 << WGM12) | (1 << WGM13); 

    // set interrupt frequency
    OCR1A = OCR1A_TIMER1_TOP;
      
    // set interrupt TIMER1_COMPA_vect
    TIMSK1 |= (1 << OCIE1A);

    #ifdef DEBUG
        Serial.println("Timer1 initialized.");
        Serial.print("  TCCR1A: "); Serial.println(TCCR1A, BIN);
        Serial.print("  TCCR1B: "); Serial.println(TCCR1B, BIN);
        Serial.print("  TIMSK1: "); Serial.println(TIMSK1, BIN);
    #endif

    _commands = queue<command_t>(8);

    _dec_balance = 0;
    _ra_balance = 0;
}

void MotorController::stop() {

    #ifdef DEBUG
        Serial.println("Stopping both motors:");
    #endif
    
    cli();
    _dec.pulses_remaining = 0;
    _ra.pulses_remaining = 0;
    sei();

    #ifdef DEBUG
        Serial.print("  DEC: "); Serial.println(_dec.pulses_remaining);
        Serial.print("  RA:  "); Serial.println(_ra.pulses_remaining);
    #endif
      
    MOTORS_PORT &= ~((1 << STEP_PIN_DEC) | (1 << STEP_PIN_RA)); // step pins to LOW

    #ifdef DEBUG
        Serial.print("  PORT:   "); Serial.println(MOTORS_PORT, BIN);
    #endif

    _commands.clear();
}

void MotorController::fast_turn(float revs_dec, float revs_ra, boolean queueing) {
    turn_internal({revs_dec, revs_ra, FAST_DELAY_START_DEC, FAST_DELAY_START_RA, FAST_DELAY_END_DEC, FAST_DELAY_END_RA, false}, queueing);
}

void MotorController::slow_turn(float revs_dec, float revs_ra, float speed_dec, float speed_ra, boolean queueing) {
    long delay_dec = 1000000.0 / (speed_dec * STEPS_PER_REV_DEC * MICROSTEPPING_MUL);
    long delay_ra  = 1000000.0 / (speed_ra  * STEPS_PER_REV_RA  * MICROSTEPPING_MUL);
    turn_internal({revs_dec, revs_ra, delay_dec, delay_ra, delay_dec, delay_ra, true}, queueing);
}

void MotorController::turn_internal(command_t cmd, bool queueing) {

    if (queueing && !is_ready()) {
        _commands.push(cmd);
        return;
    }

    #ifdef DEBUG
        Serial.println("Initializing new movement.");
        Serial.print("  revs DEC:       "); Serial.println(cmd.revs_dec);
        Serial.print("  resv RA:        "); Serial.println(cmd.revs_ra);
        Serial.print("  micro s. (t/f): "); Serial.println(cmd.microstepping ? "enabled" : "disabled");
    #endif

    #ifdef DEBUG
        Serial.println("Setting DIR and MS pins:");
        Serial.print("  PORT:  "); Serial.print(MOTORS_PORT, BIN);
    #endif

    cli();

    // wait 1ms for pins to stabilize if needed
    if (change_pin(DIR_PIN_DEC, (cmd.revs_dec > 0 && DIRECTION_DEC) || (cmd.revs_dec < 0 && !DIRECTION_DEC)) | 
        change_pin(DIR_PIN_RA,  (cmd.revs_ra  > 0 && DIRECTION_RA)  || (cmd.revs_ra  < 0 && !DIRECTION_RA))  |
        change_pin(MS_PIN_DEC, cmd.microstepping) |
        change_pin(MS_PIN_RA,  cmd.microstepping)) delay(1);

    #ifdef DEBUG
        Serial.print(" ---> "); Serial.println(MOTORS_PORT, BIN);
    #endif

    unsigned long steps_dec = abs(cmd.revs_dec) * STEPS_PER_REV_DEC * (cmd.microstepping ? MICROSTEPPING_MUL : 1);
    unsigned long steps_ra  = abs(cmd.revs_ra)  * STEPS_PER_REV_RA  * (cmd.microstepping ? MICROSTEPPING_MUL : 1);

    _dec.pulses_remaining_accel = 0;
    _ra.pulses_remaining_accel  = 0;

    _dec.target_steps_delay = cmd.delay_end_dec;
    _ra.target_steps_delay  = cmd.delay_end_ra;
    
    step_micros(_dec, steps_dec, cmd.delay_start_dec);
    step_micros(_ra,  steps_ra, cmd.delay_start_ra);

    TCNT1 = 0; // reset Timer1 counter

    sei();
}

bool MotorController::change_pin(byte pin, byte value) {
    if (((MOTORS_PORT >> pin) & 1) == value) return false;
    MOTORS_PORT ^= (-value ^ MOTORS_PORT) & (1 << pin);
    return true;
}

void MotorController::step_micros(motor_data& data, long steps, long micros_between_steps) {

    data.current_steps_delay = micros_between_steps;

    #ifdef DEBUG
        Serial.println("Steps to be done:");
        Serial.print("  "); Serial.print(steps); Serial.print(", delay (us): "); Serial.println(micros_between_steps);
    #endif

    data.pulses_remaining = 2 * steps;

    float mcu_ticks_per_pulse = micros_between_steps / 2.0 / TMR1_RESOLUTION;

    #ifdef DEBUG
        Serial.println("MCU ticks per one pulse:");
        Serial.print("  "); Serial.println(mcu_ticks_per_pulse); 
    #endif

    data.mcu_ticks_per_pulse = mcu_ticks_per_pulse;

    data.pulses_to_correction = 1.0 / (mcu_ticks_per_pulse - data.mcu_ticks_per_pulse);

    #ifdef DEBUG
        Serial.println("Number of pulses after which is added a step:");
        Serial.print("  "); Serial.println(data.pulses_to_correction); 
    #endif

    data.ticks_passed = 0;
    data.pulses_remaining_correction = 0;
}

void MotorController::trigger() {

    if (_ra.pulses_remaining == 0 && _dec.pulses_remaining == 0 && _commands.count() > 0) {
        turn_internal(_commands.pop(), false);
    }

    // DEC motor pulse should be done
    _dec_balance += motor_trigger(_dec, STEP_PIN_DEC, DIR_PIN_DEC, DIRECTION_DEC, MS_PIN_DEC);

    // RA motor pulse should be done
    _ra_balance += motor_trigger(_ra, STEP_PIN_RA, DIR_PIN_RA, DIRECTION_RA, MS_PIN_RA);
    
    bool accel_desired_dec = (_dec.current_steps_delay > _dec.target_steps_delay && _dec.pulses_remaining_accel == ACCEL_STEPS_DEC / 2);
    bool accel_desired_ra  = (_ra.current_steps_delay  > _ra.target_steps_delay  && _ra.pulses_remaining_accel  == ACCEL_STEPS_RA  / 2);

    if (accel_desired_dec || accel_desired_ra) {
        uint16_t new_steps_delay_dec = _dec.current_steps_delay - (accel_desired_dec ? ACCEL_DELAY_DEC : 0);
        uint16_t new_steps_delay_ra  = _ra.current_steps_delay  - (accel_desired_ra  ? ACCEL_DELAY_RA  : 0);
        step_micros(_dec, _dec.pulses_remaining / 2, new_steps_delay_dec);
        step_micros(_ra,  _ra.pulses_remaining / 2,  new_steps_delay_ra);
        if (accel_desired_dec) _dec.pulses_remaining_accel = 0;
        if (accel_desired_ra)  _ra.pulses_remaining_accel  = 0;
    }
}

int MotorController::motor_trigger(motor_data& data, byte pin, byte dir, byte dir_swap, byte ms) {

    if (data.pulses_remaining == 0 || ++data.ticks_passed < data.mcu_ticks_per_pulse) return 0;

    if (data.pulses_to_correction != 0 && ++data.pulses_remaining_correction == data.pulses_to_correction) {
        data.pulses_to_correction = 0;
    } else {
        --data.pulses_remaining;
    }

    ++data.pulses_remaining_accel;
    data.ticks_passed = 0;
    MOTORS_PORT ^= (1 << pin);

    return (MOTORS_PORT & (1 << ms) ? 1 : MICROSTEPPING_MUL) * ((MOTORS_PORT & (1 << dir)) ^ !dir_swap ? 1 : -1);
}