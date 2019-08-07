#ifndef CONFIG_H
#define CONFIG_H

/* ================================== GENERAL SETTINGS ================================== */

#define SERIAL_BAUD_RATE       115200
#define EEPROM_ADDR            0            // starting EEPROM offset, 18 bytes needed 
#define VERSION                1.0

#define LONGITUDE              16.2607719   // CHANGE THIS !!!!!
#define LATITUDE               49.8225003   // CHANGE THIS !!!!!


/* ======================================== MOUNT ======================================= */

#define REDUCTION_RATIO_DEC     8              // reduction ratio of DEC motor gearbox (1:8)
#define REDUCTION_RATIO_RA      8              // reduction ratio of RA motor gearbox (1:8)
#define DEG_PER_MOUNT_REV_DEC   5.37313432836  // reduction ratio of DEC mount gears (1:67)
#define DEG_PER_MOUNT_REV_RA    2.68656716418  // reduction ratio of DEC mount gears (1:134)

#define DIRECTION_DEC           0    // 1 or 0 to swap DEC movement direction
#define DIRECTION_RA            0    // 1 or 0 to swap RA movement direction

#define DEFAULT_POLE_RA         0    // default equatorial coordinates of the mount pole 
#define DEFAULT_POLE_DEC        90   // these values are changed during alignment
#define DEFUALT_RA_OFFSET       0    // offset of RA axis (defines where mount's local RA is 0)


// Alignement is done by optimization of rotation matrix parameters (three), this is done 
// by a simple evolutionary strategy. Exact numeric solutions can be unstable due to Arduino
// floating point precision and I haven't found a better soluiton than this :( 

#define CAL_BUFFER_SIZE         12         // maximal number of point pairs used for alignmenet
#define OPT_PRECISION           5000000    // if objective is less then 1/OPT_PRECISION, opt. stops
#define OPT_POPULATION_SIZE     4          // population size of the ES
#define OPT_GENERAITONS         1250       // # gen., reduce for faster but less precise solutions
                                           // higher number of generations probably does not imply
                                           // better solutions
#define OPT_SIGMA               1.0        // initial sigma value
#define OPT_SIGMA_DECAY         0.997      // every generation is sigma multiplied by this value 


/* ==================================== STEPPER MOTORS ================================== */

#define MOTORS_PORT             PORTK  // port of pins of stepper motors (serach for A. Mega pinout)
#define MOTORS_DDR              DDRK   // DDR of pins of stepper motors
#define STEP_PIN_DEC            PK2    // = A8 
#define DIR_PIN_DEC             PK1    // = A9
#define MS_PIN_DEC              PK0    // = A10
#define STEP_PIN_RA             PK5    // = A13
#define DIR_PIN_RA              PK4    // = A12
#define MS_PIN_RA               PK3    // = A11

#define MICROSTEPPING_MUL       8   // level of microstepping (depends of your wiring of A4988 pins)

#define STEPS_PER_REV_DEC       200     // number of steps per DEC motor revolution (200 for NEMA 17)
#define STEPS_PER_REV_RA        200     // number of steps per RA motor revolution (200 for NEMA 17)

#define ACCEL_STEPS_DEC         256     // every ACCEL_STEPS_DEC steps is the delay in/decreased by
#define ACCEL_DELAY_DEC         64      // ACCEL_DELAY_DEC (should be even, multiple of 2)
#define FAST_DELAY_START_DEC    2048    // DEC delay at the start of fast movement (2048 us, ~488 Hz)
#define FAST_DELAY_END_DEC      1024    // DEC delay at the end of fast movement (1024 us, ~976 Hz)

#define ACCEL_STEPS_RA          256     // every ACCEL_STEPS_RA steps is the delay in/decreased by
#define ACCEL_DELAY_RA          64      // ACCEL_DELAY_RA (should be even, multiple of 2)
#define FAST_DELAY_START_RA     2048    // RA delay at the start of fast movement (2048 us, ~488 Hz)
#define FAST_DELAY_END_RA       1024    // RA delay at the end of fast movement (1024 us, ~976 Hz)

#define FAST_REVS_PER_SEC_DEC   1000000.0 / FAST_DELAY_START_DEC / STEPS_PER_REV_DEC 
#define FAST_REVS_PER_SEC_RA    1000000.0 / FAST_DELAY_START_RA  / STEPS_PER_REV_RA


/* ==================================== OTHER SETTINGS ================================== */

#define TRIGGER_PIN             40      // pin which controls camera trigger
#define SNAP_DELAY_MS           2000    // minimal delay (ms) between two snaps (camera protection)

#define SD_CS                   53      // SD card chip select pin

#define KEYPAD_IR_PIN           7       // IR receiver signal pin 
#define SHORT_HOLD_TIME_MS      200     // minimal duration (ms) of a fast remote control key press
#define LONG_HOLD_TIME_MS       800     // minimal duration (ms) of a slow remote control key press

#define DSP_DATA_PIN4           2       // LCD data pins
#define DSP_DATA_PIN5           3
#define DSP_DATA_PIN6           4
#define DSP_DATA_PIN7           5
#define DSP_ANODE_PIN           8       // LCD pin for controling brightness
#define DSP_ENABLE_PIN          11      // LCD enable pin
#define DSP_REGISTER_SEL_PIN    12      // LCD register select pin

/* ======================================== DEBUG ======================================= */

// #define DEBUG
// #define DEBUG_MOUNT
// #define DEBUG_TIME
// #define DEBUG_CONTROL
// #define DEBUG_KEYS

#endif