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
}

void MotorController::turn(float angle_dec, float angle_ra, bool fast = true, bool over_ride = false) {

    #ifdef DEBUG
        Serial.println("Checking if steppers are ready:");
        Serial.print("  DEC: "); Serial.println(_dec.pulses_remaining);
        Serial.print("  RA:  "); Serial.println(_ra.pulses_remaining);
    #endif

    if (!over_ride && !is_ready()) return;

    #ifdef DEBUG
        Serial.println("Initializing new movement.");
        Serial.print("  angle DEC:      "); Serial.println(angle_dec);
        Serial.print("  angle RA:       "); Serial.println(angle_ra);
        Serial.print("  micro s. (t/f): "); Serial.println(fast ? "disabled" : "enabled");
    #endif

    #ifdef DEBUG
        Serial.println("Setting DIR and MS pins:");
        Serial.print("  PORT:  "); Serial.print(MOTORS_PORT, BIN);
    #endif

    cli();

    // wait 1ms for pins to stabilize if needed
    if (change_pin(DIR_PIN_DEC, angle_dec > 0) | 
        change_pin(DIR_PIN_RA,  angle_ra > 0) |
        change_pin(MS_PIN_DEC, !fast) |
        change_pin(MS_PIN_RA,  !fast)) delay(1);

    #ifdef DEBUG
        Serial.print(" ---> "); Serial.println(MOTORS_PORT, BIN);
    #endif

    unsigned long steps_dec = abs(angle_dec) / DEG_PER_MOUNT_REV_DEC * REDUCTION_RATIO_DEC * STEPS_PER_REV_DEC;
    unsigned long steps_ra  = abs(angle_ra)  / DEG_PER_MOUNT_REV_RA  * REDUCTION_RATIO_RA  * STEPS_PER_REV_RA;

    _dec.pulses_remaining_accel = 0;
    _ra.pulses_remaining_accel  = 0;

    if (fast) {
        _dec.target_steps_delay = FAST_DELAY_END_DEC;
        _ra.target_steps_delay  = FAST_DELAY_END_RA;
        step_micros(_dec, steps_dec, FAST_DELAY_START_DEC);
        step_micros(_ra,  steps_ra, FAST_DELAY_START_RA);
    }
    else {

        // TODO: compute delay for slow movement properly. Based on the real orientation of mount.
        // TODO: 1000000/((15/60/60)*(200*8*4/2.68656716418))

        _dec.target_steps_delay = SLOW_DELAY_DEC;
        _ra.target_steps_delay  = SLOW_DELAY_RA;
        step_micros(_dec, steps_dec * MICROSTEPPING_MUL, _dec.target_steps_delay);
        step_micros(_ra,  steps_ra * MICROSTEPPING_MUL, _ra.target_steps_delay);
    }

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
        Serial.println("MCU ticks per one pulse (us):");
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

    // DEC motor pulse should be done
    motor_trigger(_dec, STEP_PIN_DEC);

    // RA motor pulse should be done
    motor_trigger(_ra, STEP_PIN_RA);
    
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

void MotorController::motor_trigger(motor_data& data, byte pin) {

    if (data.pulses_remaining > 0 && data.ticks_passed++ >= data.mcu_ticks_per_pulse) {
        if (data.pulses_to_correction != 0 && 
            data.pulses_remaining_correction == data.pulses_to_correction) {
            data.pulses_remaining_correction = 0;
        } else {                
            --data.pulses_remaining;  
            ++data.pulses_remaining_correction;
        }
        ++data.pulses_remaining_accel;
        data.ticks_passed = 0;
        MOTORS_PORT ^= (1 << pin);
    }
}