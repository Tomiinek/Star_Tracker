#ifndef CONFIG_H
#define CONFIG_H

// #define DEBUG
// #define DEBUG_MOUNT
// #define DEBUG_TIME
// #define DEBUG_CONTROL
// #define DEBUG_KEYS

#define SERIAL_BAUD_RATE       115200
#define EEPROM_ADDR            0
#define VERSION                1.0

/************************ GENERAL SETTING ***********************/

#define LONGITUDE               16.2607719 
#define LATITUDE                49.8225003

#define EEPROM_ADDR             0       // starting EEPROM offset, 18 bytes needed 

/************************* MOUNT SETTING ************************/

// gears reduction ratios
#define REDUCTION_RATIO_DEC     8
#define REDUCTION_RATIO_RA      8   
#define DEG_PER_MOUNT_REV_DEC   5.37313432836  // 1:67
#define DEG_PER_MOUNT_REV_RA    2.68656716418  // 1:134

// motors direction
#define DIRECTION_DEC           0       // 1 or 0 to swap movement direction
#define DIRECTION_RA            0       // 1 or 0 to swap movement direction

// default orientation
#define DEFAULT_POLE_RA         0
#define DEFAULT_POLE_DEC        90
#define DEFUALT_RA_OFFSET       0

// calibration
#define CAL_BUFFER_SIZE         12
#define OPT_PRECISION           5000000    // if objective is less then 1/OPT_PRECISION, opt. stops
#define OPT_POPULATION_SIZE     4
#define OPT_GENERAITONS         1250
#define OPT_SIGMA               1.0        // initial sigma value
#define OPT_SIGMA_DECAY         0.997      // every generation is sigma multiplied by this    

/*********************** STEPPERS SETTING ***********************/

// we suppose that we have both steppers on the same port
#define MOTORS_DDR              DDRK
#define MOTORS_PORT             PORTK 
#define STEP_PIN_DEC            PK5
#define DIR_PIN_DEC             PK4
#define MS_PIN_DEC              PK3
#define STEP_PIN_RA             PK2
#define DIR_PIN_RA              PK1
#define MS_PIN_RA               PK0

// steppers microstepping level
#define MICROSTEPPING_MUL       8

// steppers resolution
#define STEPS_PER_REV_DEC       200
#define STEPS_PER_REV_RA        200

// steppers speed settings
#define ACCEL_STEPS_DEC         256     // every ACCEL_STEPS_DEC steps is the delay 
#define ACCEL_DELAY_DEC         50      // in/decreased by ACCEL_DELAY_DEC (should be even)
#define FAST_DELAY_START_DEC    2048    // 1400 us, ~714 Hz
#define FAST_DELAY_END_DEC      1024    // 800 us, 1250 Hz

#define ACCEL_STEPS_RA          256     // every ACCEL_STEPS_RA steps is the delay 
#define ACCEL_DELAY_RA          50      // in/decreased by ACCEL_DELAY_RA (should be even)
#define FAST_DELAY_START_RA     2048    // 1400 us, ~714 Hz
#define FAST_DELAY_END_RA       1024    //  800 us, 1250 Hz

#define FAST_REVS_PER_SEC_DEC   1000000.0 / FAST_DELAY_START_DEC / STEPS_PER_REV_DEC 
#define FAST_REVS_PER_SEC_RA    1000000.0 / FAST_DELAY_START_RA  / STEPS_PER_REV_RA

/******************** CAMERA TRIGGER SETTINGS ********************/

#define TRIGGER_PIN             40
#define SNAP_DELAY_MS           2000   

/************************** SD SETTINGS **************************/

#define SD_CS                   53

/************************ CONTROL SETTINGS ***********************/

#define KEYPAD_IR_PIN           7   

#define SHORT_HOLD_TIME_MS      200
#define LONG_HOLD_TIME_MS       800

/************************ DISPLAY SETTINGS ***********************/

#define DSP_DATA_PIN4           2
#define DSP_DATA_PIN5           3
#define DSP_DATA_PIN6           4
#define DSP_DATA_PIN7           5
#define DSP_ANODE_PIN           8       // this pin control brightness
#define DSP_ENABLE_PIN          11
#define DSP_REGISTER_SEL_PIN    12

#endif