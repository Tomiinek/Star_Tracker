#define FROM_LIB

#include <Arduino.h>
#include "motor_controller.h"

void MotorController::initialize() {
    
    uint8_t pin_mask = (1 << STEP_PIN_DEC) | (1 << DIR_PIN_DEC) | (1 << MS_PIN_DEC) |
                       (1 << STEP_PIN_RA)  | (1 << DIR_PIN_RA)  | (1 << MS_PIN_RA);

    MOTORS_DDR |= pin_mask;
    MOTORS_PORT &= ~pin_mask;

    #ifdef DEBUG
        Serial.println(F("Stepper motors pins initialized:"));
        Serial.print(F("  Pinout: ")); Serial.println(pin_mask, BIN);
        Serial.print(F("  DDR:    ")); Serial.println(MOTORS_DDR, BIN);
        Serial.print(F("  PORT:   ")); Serial.println(MOTORS_PORT, BIN);
    #endif

    // Timer/Counter Control Register: set Fast PWM mode
    TCCR5A = 0x23 ; // || set mode 7 (Fast PWM) with
    TCCR5B = 0x09 ; // || prescaler 1 (no prescaling)

    // Output Compare Register: set interrupt frequency
    OCR5A = TIMER_TOP - 1;
    OCR5B = 0;
      
    // Timer/Counter Interrupt Mask Register: set interrupt TIMERx_COMPA_vect
    TIMSK5 |= (1 << OCIE5A);

    #ifdef DEBUG
        Serial.println(F("TimerX initialized."));
        Serial.print(F("  TCCRxA: ")); Serial.println(TCCR5A, BIN);
        Serial.print(F("  TCCRxB: ")); Serial.println(TCCR5B, BIN);
        Serial.print(F("  TIMSKx: ")); Serial.println(TIMSK5, BIN);
    #endif

    _commands = queue<command_t>(8);

    _dec_balance = 0;
    _ra_balance = 0;
}

void MotorController::stop() {

    #ifdef DEBUG
        Serial.println(F("Stopping both motors:"));
    #endif
    
    cli();
    _dec.pulses_remaining = 0;
    _ra.pulses_remaining = 0;
    sei();

    #ifdef DEBUG
        Serial.print(F("  DEC: ")); Serial.println(_dec.pulses_remaining);
        Serial.print(F("  RA:  ")); Serial.println(_ra.pulses_remaining);
    #endif
      
    MOTORS_PORT &= ~((1 << STEP_PIN_DEC) | (1 << STEP_PIN_RA)); // step pins to LOW

    #ifdef DEBUG
        Serial.print(F("  PORT:   ")); Serial.println(MOTORS_PORT, BIN);
    #endif

    _commands.clear();
}

float MotorController::estimate_fast_turn_time(float revs_dec, float revs_ra) {

    float sd, sr;
    revs_to_steps(sd, sr, revs_dec, revs_ra, false);
    
    auto time_dec = estimate_motor_fast_turn_time(sd, ACCEL_STEPS_DEC, ACCEL_DELAY_DEC, FAST_DELAY_START_DEC, FAST_DELAY_END_DEC);
    auto time_ra  = estimate_motor_fast_turn_time(sr, ACCEL_STEPS_RA,  ACCEL_DELAY_RA,  FAST_DELAY_START_RA,  FAST_DELAY_END_RA);

    return max(time_dec, time_ra);
} 

float MotorController::estimate_motor_fast_turn_time(float steps, int accel_each, int accel_amount, int delay_start, int delay_end) {

    float time = 0;

    float total_steps = floor(steps);
    float accel_steps = floor(steps);
    int delay_curr = delay_start; 

    for (accel_steps -= accel_each; (accel_steps > steps / 2.0f) && (delay_curr > delay_end);) {
        time += (float)delay_curr * accel_each;
        delay_curr -= accel_amount;
        accel_steps -= accel_each;
    }
    accel_steps += accel_each;

    return (2 * time + (2 * accel_steps - total_steps) * delay_curr) / 1000.0f; 
}

void MotorController::fast_turn(float revs_dec, float revs_ra, boolean queueing) {
    turn_internal({revs_dec, revs_ra, FAST_DELAY_START_DEC, FAST_DELAY_START_RA, FAST_DELAY_END_DEC, FAST_DELAY_END_RA, false}, queueing);
}

void MotorController::slow_turn(float revs_dec, float revs_ra, float speed_dec, float speed_ra, boolean queueing) {
    // revolutions per second convert to delay in micros
    // there might be some overflows, but nobody cares ... (hopefully)
    float delay_dec = 1000000.0 / (speed_dec * STEPS_PER_REV_DEC * MICROSTEPPING_MUL);
    float delay_ra  = 1000000.0 / (speed_ra  * STEPS_PER_REV_RA  * MICROSTEPPING_MUL);
    turn_internal({revs_dec, revs_ra, delay_dec, delay_ra, delay_dec, delay_ra, true}, queueing);
}

void MotorController::turn_internal(command_t cmd, bool queueing) {

    if (queueing && !is_ready()) {
        _commands.push(cmd);
        return;
    }

    #ifdef DEBUG
        Serial.println(F("Initializing new movement."));
        Serial.print(F("  revs DEC:       ")); Serial.println(cmd.revs_dec);
        Serial.print(F("  resv RA:        ")); Serial.println(cmd.revs_ra);
        Serial.print(F("  micro s. (t/f): ")); Serial.println(cmd.microstepping ? "enabled" : "disabled");
    #endif

    #ifdef DEBUG
        Serial.println(F("Setting DIR and MS pins:"));
        Serial.print(F("  PORT:  ")); Serial.print(MOTORS_PORT, BIN);
    #endif

    cli();

    // wait 1ms for pins to stabilize if needed
    if (change_pin(DIR_PIN_DEC, (cmd.revs_dec > 0 && DIRECTION_DEC) || (cmd.revs_dec < 0 && !DIRECTION_DEC)) | 
        change_pin(DIR_PIN_RA,  (cmd.revs_ra  > 0 && DIRECTION_RA)  || (cmd.revs_ra  < 0 && !DIRECTION_RA))  |
        change_pin(MS_PIN_DEC, cmd.microstepping) |
        change_pin(MS_PIN_RA,  cmd.microstepping)) delay(1);

    #ifdef DEBUG
        Serial.print(F(" ---> ")); Serial.println(MOTORS_PORT, BIN);
    #endif

    float steps_dec, steps_ra;
    revs_to_steps(steps_dec, steps_ra, cmd.revs_dec, cmd.revs_ra, cmd.microstepping);

    uint32_t effective_steps_dec = steps_dec;
    uint32_t effective_steps_ra = steps_ra;

    _dec.pulses_to_accel = 0;
    _ra.pulses_to_accel  = 0;

    _dec.steps_total = effective_steps_dec;
    _ra.steps_total = effective_steps_ra;
    
    _dec.target_steps_delay = cmd.delay_end_dec;
    _ra.target_steps_delay  = cmd.delay_end_ra;

    _dec.start_steps_delay = cmd.delay_start_dec;
    _ra.start_steps_delay  = cmd.delay_start_ra;

    step_micros(_dec, effective_steps_dec * 2, _dec.start_steps_delay);
    step_micros(_ra,  effective_steps_ra  * 2, _ra.start_steps_delay);

    // compensate coarse resolution of the full-step movement
    if (!cmd.microstepping) {
        float revs_dec, revs_ra;
        steps_to_revs(revs_dec, revs_ra, steps_dec - effective_steps_dec, steps_ra - effective_steps_ra, false);
        slow_turn(revs_dec, revs_ra, FAST_REVS_PER_SEC_DEC / MICROSTEPPING_MUL, FAST_REVS_PER_SEC_RA / MICROSTEPPING_MUL, true);
    }

    TCNT1 = 0; // reset Timer1 counter

    sei();
}

bool MotorController::change_pin(byte pin, byte value) {
    if (((MOTORS_PORT >> pin) & 1) == value) return false;
    MOTORS_PORT ^= (-value ^ MOTORS_PORT) & (1 << pin);
    return true;
}

void MotorController::step_micros(motor_data& data, long pulses, unsigned long micros_between_steps) {

    data.current_steps_delay = micros_between_steps;

    #ifdef DEBUG
        Serial.println(F("Steps to be done:"));
        Serial.print(F("  ")); Serial.print(pulses / 2); Serial.print(F(", delay (us): ")); Serial.println(micros_between_steps);
    #endif

    data.pulses_remaining = pulses;

    float mcu_ticks_per_pulse = micros_between_steps / 2.0 / TMR_RESOLUTION;

    #ifdef DEBUG
        Serial.println(F("MCU ticks per one pulse:"));
        Serial.print(F("  ")); Serial.println(mcu_ticks_per_pulse); 
    #endif

    data.mcu_ticks_per_pulse = mcu_ticks_per_pulse;

    data.pulses_to_correct = 0;
    float err = mcu_ticks_per_pulse - data.mcu_ticks_per_pulse;
    if (err == 0.0) data.pulses_to_correct = 0;
    else data.pulses_to_correct = 1.0 / err;

    #ifdef DEBUG
        Serial.println(F("Number of pulses after which is added an empty tick:"));
        Serial.print(F("  ")); Serial.println(data.pulses_to_correct); 
    #endif

    data.ticks_passed = 0;
    data.pulses_until_correction = 0;
    data.correction = false;
}

void MotorController::trigger() {

    if (_ra.pulses_remaining == 0 && _dec.pulses_remaining == 0 && _commands.count() > 0) {
        turn_internal(_commands.pop(), false);
    }

    // DEC motor pulse should be done
    _dec_balance += motor_trigger(_dec, STEP_PIN_DEC, DIR_PIN_DEC, DIRECTION_DEC, MS_PIN_DEC);

    // RA motor pulse should be done
    _ra_balance += motor_trigger(_ra, STEP_PIN_RA, DIR_PIN_RA, DIRECTION_RA, MS_PIN_RA);

    // these calls will take some time so we will probaly miss some next 
    // interrupts but we do not really care because we are changing speed
    // and this does not happen during tracking so everything should be ok
    change_motor_speed(_dec, ACCEL_STEPS_DEC * 2, ACCEL_DELAY_DEC);
    change_motor_speed(_ra, ACCEL_STEPS_RA * 2, ACCEL_DELAY_RA);
}

void MotorController::change_motor_speed(motor_data& data, int change_pulses, int amount) {

    bool accel_desired = false;
    bool decel_desired = false;

    if (data.pulses_to_accel >= change_pulses) {

        // are we at the middle of the motor movement?
        if (data.pulses_remaining > data.steps_total) {
            accel_desired = (data.current_steps_delay > data.target_steps_delay);
        }
        else if ((data.start_steps_delay - data.current_steps_delay) / amount >= data.pulses_remaining / change_pulses) {
            decel_desired = (data.current_steps_delay < data.start_steps_delay);
        }
 
        if (accel_desired || decel_desired) {
            float new_steps_delay = data.current_steps_delay - (accel_desired ? 1 : -1) * amount;
            step_micros(data, data.pulses_remaining,  new_steps_delay); 
            // this procedure take some time (like 100 us) so we may compensate missed intrrupts somehow
            data.ticks_passed += 2; 
        }

        data.pulses_to_accel = 0;
    }
}

int MotorController::motor_trigger(motor_data& data, byte pin, byte dir, bool dir_swap, byte ms) {

    if (data.pulses_remaining == 0) return 0;
    if (!data.correction) ++data.ticks_passed;

    if (data.ticks_passed < data.mcu_ticks_per_pulse) return 0;

    if (data.pulses_to_correct != 0 && ++data.pulses_until_correction == data.pulses_to_correct) {
        data.pulses_until_correction = 0;
        data.correction = true;
    }
    else data.correction = false;

    ++data.pulses_to_accel;
    --data.pulses_remaining;
    data.ticks_passed = 0;
    MOTORS_PORT ^= (1 << pin);

    return (MOTORS_PORT & (1 << ms) ? 1 : MICROSTEPPING_MUL) * (((MOTORS_PORT >> dir) & 1) != dir_swap ? -1 : 1);
}