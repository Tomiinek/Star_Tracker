#ifndef CONFIG_H
#define CONFIG_H

// #define DEBUG
// #define DEBUG_MOUNT

/************************ GENERAL SETTING ***********************/

#define LONGITUDE               16.2607719 
#define LATITUDE                49.8225003

/************************* MOUNT SETTING ************************/

// gears reduction ratios
#define REDUCTION_RATIO_DEC     8
#define REDUCTION_RATIO_RA      8   
#define DEG_PER_MOUNT_REV_DEC   5.37313432836f  // 1:67
#define DEG_PER_MOUNT_REV_RA    2.68656716418f  // 1:134

// motors direction
#define DIRECTION_DEC           1       // 1 or 0 to swap movement direction
#define DIRECTION_RA            0       // 1 or 0 to swap movement direction

/*********************** STEPPERS SETTING ***********************/

// we suppose that we have both steppers on the same port
#define MOTORS_DDR              DDRD
#define MOTORS_PORT             PORTD 
#define STEP_PIN_DEC            PD5
#define DIR_PIN_DEC             PD6
#define MS_PIN_DEC              PD7
#define STEP_PIN_RA             PD2
#define DIR_PIN_RA              PD3
#define MS_PIN_RA               PD4

// steppers microstepping level
#define MICROSTEPPING_MUL       8

// steppers resolution
#define STEPS_PER_REV_DEC       200
#define STEPS_PER_REV_RA        200

// steppers speed settings
#define ACCEL_STEPS_DEC         300     // every ACCEL_STEPS_DEC steps is the delay 
#define ACCEL_DELAY_DEC         200     // decreased by ACCEL_DELAY_DEC (should be even)
#define FAST_DELAY_START_DEC    1400    // 1400 us, ~714 Hz
#define FAST_DELAY_END_DEC      800     //  800 us, 1250 Hz

#define ACCEL_STEPS_RA          300     // every ACCEL_STEPS_RA steps is the delay 
#define ACCEL_DELAY_RA          200     // decreased by ACCEL_DELAY_RA (should be even)
#define FAST_DELAY_START_RA     1400    // 1400 us, ~714 Hz
#define FAST_DELAY_END_RA       800     //  800 us, 1250 Hz

/******************** CAMERA TRIGGER SETTINGS ********************/

#define TRIGGER_PIN             12
#define SNAP_DELAY_MS           250          


#endif