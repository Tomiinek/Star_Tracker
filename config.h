
// #define DEBUG

// we suppose that we have both steppers on the same port
#define MOTORS_DDR              DDRD
#define MOTORS_PORT             PORTD 
#define STEP_PIN_DEC            PD2
#define DIR_PIN_DEC             PD3
#define MS_PIN_DEC              PD4
#define STEP_PIN_RA             PD5
#define DIR_PIN_RA              PD6
#define MS_PIN_RA               PD7

// gears reduction ratios
#define REDUCTION_RATIO_DEC     8
#define REDUCTION_RATIO_RA      8   
#define DEG_PER_MOUNT_REV_DEC   2.68656716418f  // 1:134
#define DEG_PER_MOUNT_REV_RA    5.37313432836f  // 1:67

// steppers microstepping level
#define MICROSTEPPING_MUL       8

// steppers resolution
#define STEPS_PER_REV_DEC       200
#define STEPS_PER_REV_RA        200

// steppers speed settings
#define ACCEL_STEPS_DEC         200     // every ACCEL_STEPS_DEC steps is the delay 
#define ACCEL_DELAY_DEC         100     // decreased by ACCEL_DELAY_DEC (should be even)
#define FAST_DELAY_START_DEC    1400    // 1400 us, ~714 Hz
#define FAST_DELAY_END_DEC      800     //  800 us, 1250 Hz
#define SLOW_DELAY_DEC          40000   // 100746  // 15 degrees per hour, exactly 9.9259259259 Hz for MS 4

#define ACCEL_STEPS_RA          200     // every ACCEL_STEPS_RA steps is the delay 
#define ACCEL_DELAY_RA          100     // decreased by ACCEL_DELAY_RA (should be even)
#define FAST_DELAY_START_RA     1400    // 1400 us, ~714 Hz
#define FAST_DELAY_END_RA       800     //  800 us, 1250 Hz
#define SLOW_DELAY_RA           50373   // 50373   // 15 degrees per hour, exactly 9.9259259259/2 Hz for MS 4



