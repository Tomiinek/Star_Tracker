#ifndef MOUNTCONTROLLER_H
#define MOUNTCONTROLLER_H

#include <math.h>

#include "../config.h"
#include "motor_controller.h"
#include "clock.h"

class MountController {
  
  public:

    using deg_t = float;

    struct coord_t {
        float dec;
        float ra;
    };

    struct cartesian_t {
        float x;
        float y;
        float z;
    };

    MountController(MotorController& mc) : _motors(mc) {}

    // initialize stepper motors, default values, call from setup!
    void initialize();

    inline void get_mount_pole(coord_t& pole, deg_t& ra_offset) {
        pole = _mount_pole;
        ra_offset = _mount_ra_offset;
    }  
    inline void set_mount_pole(coord_t pole, deg_t ra_offset) {
        _transition = make_transition_matrix(pole, ra_offset);
        _transition_inverse = make_inverse_transition_matrix(pole, ra_offset);
        _mount_pole = pole;
        _mount_ra_offset = ra_offset;
    }

    // orientation of mount in the global equatorial coordinates (DEC, RA)
    coord_t get_global_mount_orientation();

    // orientation of mount in its coordinate system (does not take into account LST)
    coord_t get_local_mount_orientation();

    // calibration of mount pole
    void all_star_alignment(coord_t kernel[], coord_t image[], uint8_t points_num);

    // same as move_absolute method but with JToDate correction of J2000 cordinates
    void move_absolute_J2000(deg_t angle_dec, deg_t angle_ra);

    // moves the mount in order to point at the target in absolute coordinates (at max speed)
    void move_absolute(deg_t angle_dec, deg_t angle_ra);

    // moves a bit relatively to the current mount orientation (at max speed in mount coord. sys.)
    void move_relative_local(deg_t angle_dec, deg_t angle_ra);

    // moves a bit relatively to the current mount orientation (at max speed in equatorial coord. sys.)
    void move_relative_global(deg_t angle_dec, deg_t angle_ra);

    // starts tracking the object for an hour given the current mount orientation, also adapts
    // the speed of motors in RA and DEC according to starting position (so it should  a little but 
    // compensate improperly calibrated mount), however this SPEED MIGHT BE IRRELEVANT ONCE THE TRACKING
    // IS RUNNING FOR A LONG TIME AND/OR THE MOUNT IS VERY POORLY CALIBRATED (i.e. its pole is at 45 DEC)
    void set_tracking();

    // stops all motors immediately
    void stop_all() { _motors.stop(); _is_tracking = false; }

    // stops motors just is tracking
    void stop_tracking();

    // check whether motors do move
    inline boolean is_moving() { return !_motors.is_ready(); }

    // check whether we are tracking something
    inline boolean is_tracking() { return _is_tracking; }

    static float to_time_global_ra(float ra) {
        // see _mount_pole comments in for the explanation of 180-...
        return fmod(180 - ra +  15 * Clock::get_decimal_LST(), 360);
    }

    static float to_future_global_ra(float ra, float decimal_future_hours) {
        // see _mount_pole comments in for the explanation of 180-...
        return fmod(180 - ra +  15 * (Clock::get_decimal_LST() + decimal_future_hours), 360);
    }

  private:

    struct matrix_t {

        double data[3][3];
        
        matrix_t& operator*= (matrix_t const & b){
            matrix_t product = {};
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) 
                    for (int k = 0; k < 3; k++) {
                        product.data[i][j] += data[i][k] * b.data[k][j];
                    }
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++){
                    data[i][j] = product.data[i][j];
                }
            return *this;
        }

        friend matrix_t operator*(matrix_t left, matrix_t const & right) {
            left *= right;
            return left;
        }

        friend cartesian_t operator*(matrix_t const & left, cartesian_t const & right) {
            return cartesian_t {
                left.data[0][0] * right.x + left.data[0][1] * right.y + left.data[0][2] * right.z,
                left.data[1][0] * right.x + left.data[1][1] * right.y + left.data[1][2] * right.z,
                left.data[2][0] * right.x + left.data[2][1] * right.y + left.data[2][2] * right.z,
            };
        }
    };

    inline float to_deg(float rad) { return rad / M_PI * 180; }
    inline float to_rad(float deg) { return deg / 180 * M_PI; }

    coord_t angle_to_revolutions(coord_t angles) {
        return { angles.dec * REDUCTION_RATIO_DEC / DEG_PER_MOUNT_REV_DEC,
                 angles.ra  * REDUCTION_RATIO_RA  / DEG_PER_MOUNT_REV_RA };
    }

    coord_t revolutions_to_angle(coord_t revolutions) {
        return { revolutions.dec * DEG_PER_MOUNT_REV_DEC / REDUCTION_RATIO_DEC,
                 revolutions.ra  * DEG_PER_MOUNT_REV_RA  / REDUCTION_RATIO_RA };
    }

    inline float to_180_range(float angle) {
        if (angle >  180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    // returns coordinates of 'point' (defined in equatorial coord. sys.) w. r. to 
    // coordinate system given by the transition matrix 'transition'
    inline coord_t polar_to_polar(coord_t point, const matrix_t& transition) {
        return cartesian_to_polar(transition * polar_to_cartesian(point));
    }

    // converts spherical coordinates with unit radius to cartesian
    cartesian_t polar_to_cartesian(coord_t polar);

    // converts cartesian to spherical coordinates with unit radius
    coord_t cartesian_to_polar(cartesian_t cartesian);

    matrix_t make_transition_matrix(coord_t pole, float ra_offset) {
        return get_ra_transition(ra_offset) * 
               get_dec_transition(pole.dec) * 
               get_ra_transition(pole.ra);
    }

    matrix_t make_inverse_transition_matrix(coord_t pole, float ra_offset) {
        return get_ra_transition_inverse(pole.ra) * 
               get_dec_transition_inverse(pole.dec) * 
               get_ra_transition_inverse(ra_offset);
    }

    matrix_t get_dec_transition(deg_t dec);

    matrix_t get_dec_transition_inverse(deg_t dec);

    matrix_t get_ra_transition(deg_t ra);

    matrix_t get_ra_transition_inverse(deg_t ra);

    // Returns angular speed (DEC, RA) of a point with coordinates 'dec', 'ra' w. r. to the 
    // coordinate system defined by the 'pole' and 'offset' at a particular time 't'. The point has 
    // angular speed in the global coordinates 0 deg/s DEC and 'ra_speed' deg/s RA. USE ONLY FOR SMALL 
    // TRANSFORMS NEARBY THE REAL GLOBAL POLE.
    coord_t get_ra_speed_transform(deg_t ra_speed, float t, coord_t point, coord_t pole, deg_t ra_offset);

    // returns a number from standard normal distribution using transform from uniform distribution
    float random_normal();

    boolean _is_tracking;

    // DEC and RA of the real mount pole, BUT! RA is 0 for points
    // on the meridian which is opposite to the local one 
    // (i.e. pointing to north) and DEC is 90 for the celestial 
    // pole as is usual (so properly aligned mount is {90, ..} for 
    // equatorial coords. and {LATITUDE, 0} for azimuthal coords.)
    coord_t _mount_pole;

    // Offset of the RA coordinate, it is dependent on the initial RA 
    // position and should ideally be chosen in order to have the mount 
    // local 0 RA in the opposite side than is the observed location
    // because we cannot move mount for example from 355 RA --> 5 RA
    deg_t _mount_ra_offset;

    // DEC and RA in the local coordinate system of the mount.
    coord_t _mount_orientation;
    
    matrix_t _transition;
    matrix_t _transition_inverse;

    MotorController& _motors;
};

#endif