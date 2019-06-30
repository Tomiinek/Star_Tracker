#define FROM_LIB

#include <Arduino.h>
#include "mount_controller.h"

void MountController::initialize() {

    _motors.initialize();

    #ifdef DEBUG_MOUNT
        Serial.println("Stepper motors initialized.");
    #endif

    _is_tracking = false;
    
    _mount_orientation = {0, 0};
    _mount_pole = {90, 0};
    _mount_ra_offset = 0;

    #ifdef DEBUG_MOUNT
        Serial.println("Initializing transition matrices:");
    #endif
    
    _transition = make_transition_matrix(_mount_pole, _mount_ra_offset);
    _transition_inverse = make_inverse_transition_matrix(_mount_pole, _mount_ra_offset);
}

MountController::coord_t MountController::get_global_mount_orientation() {

    coord_t local = get_local_mount_orientation();
    coord_t global = polar_to_polar(local, _transition_inverse);

    // see _mount_pole comments in header file for the explanation of 180-...
    global.ra = 180 - global.ra + get_LST() * 15.0f;
    if (global.ra < 0) global.ra += 360;

    #ifdef DEBUG_MOUNT
        Serial.println("Global orientation:");
        Serial.print("  DEC:  "); Serial.println(global.dec);
        Serial.print("  RA:   "); Serial.println(global.ra);
    #endif

    return global;
}

MountController::coord_t MountController::get_local_mount_orientation() {

    float dec_revs_done, ra_revs_done;
    _motors.pop_made_revolutions(dec_revs_done, ra_revs_done);

    _mount_orientation.dec += dec_revs_done * DEG_PER_MOUNT_REV_DEC / REDUCTION_RATIO_DEC;
    _mount_orientation.ra  += ra_revs_done  * DEG_PER_MOUNT_REV_RA  / REDUCTION_RATIO_RA;

    // DEC and RA must be in bounds and this should never happen! exception would be wonderful 
    if (_mount_orientation.dec > 90.0f || _mount_orientation.dec < -90.0f ||
        _mount_orientation.ra > 360.0f || _mount_orientation.ra < -360.0f){
        Serial.println("Weird things happed! DEC out of bounds in get_local_mount_orientation");
    }   
   
    #ifdef DEBUG_MOUNT
        Serial.println("Local orientation:");
        Serial.print("  DEC:  "); Serial.println(_mount_orientation.dec);
        Serial.print("  RA:   "); Serial.println(_mount_orientation.ra);
    #endif

    return _mount_orientation;
}

void MountController::all_star_alignment() {

    // TODO: should work similarly to Celestron All-star alignment
}

void MountController::move_absolute(deg_t angle_dec, deg_t angle_ra) {

    if (angle_dec < -90 || angle_dec > 90 || angle_ra < 0 || angle_ra > 360) return;

    _motors.stop(); 

    // see _mount_pole comments in header file for the explanation of 180-...
    coord_t target = polar_to_polar({angle_dec, fmod(180 - angle_ra + 15 * get_LST(), 360)}, _transition);
    coord_t o = get_local_mount_orientation();

    float revs_dec = (target.dec - o.dec) / DEG_PER_MOUNT_REV_DEC * REDUCTION_RATIO_DEC;
    float revs_ra  = (target.ra  - o.ra)  / DEG_PER_MOUNT_REV_RA  * REDUCTION_RATIO_RA;

    #ifdef DEBUG_MOUNT
        Serial.println("Turning at high speed by:");
        Serial.print("       DEC:  "); Serial.println(target.dec - o.dec);
        Serial.print("       RA:   "); Serial.println(target.ra  - o.ra);
        Serial.print("  tran DEC:  "); Serial.print(angle_dec); Serial.print(" --> "); Serial.println(target.dec);
        Serial.print("  tran RA:   "); Serial.print(angle_ra);  Serial.print(" --> "); Serial.println(target.ra);
        Serial.print("  revs DEC:  "); Serial.println(revs_dec);
        Serial.print("  revs RA:   "); Serial.println(revs_ra);
    #endif
        
    _motors.fast_turn(revs_dec, revs_ra, false);
}

void MountController::move_relative_local(deg_t angle_dec, deg_t angle_ra) {

    coord_t curr_pos = get_local_mount_orientation();

    angle_dec = fmod(angle_dec, 360);
    angle_ra  = fmod(angle_ra,  360);

    // DEC cannot exceed -90..90 degrees
    if (curr_pos.dec + angle_dec < -90) angle_dec = -90 - curr_pos.dec;
    else if (curr_pos.dec + angle_dec > 90) angle_dec = 90 - curr_pos.dec;

    // RA can, but should not exceed 0..360 because of wires etc.
    if (curr_pos.ra + angle_ra < 0) angle_ra = -curr_pos.ra;
    else if (curr_pos.ra + angle_ra > 360) angle_ra = 360 - curr_pos.ra;

    float revs_dec = angle_dec / DEG_PER_MOUNT_REV_DEC * REDUCTION_RATIO_DEC;
    float revs_ra  = angle_ra  / DEG_PER_MOUNT_REV_RA  * REDUCTION_RATIO_RA;

    #ifdef DEBUG_MOUNT
        Serial.println("Turning at high speed by:");
        Serial.print("  DEC:  "); Serial.println(angle_dec);
        Serial.print("  RA:   "); Serial.println(angle_ra);
        Serial.print("  revs DEC:  "); Serial.println(revs_dec);
        Serial.print("  revs RA:   "); Serial.println(revs_ra);
    #endif
        
    _motors.fast_turn(revs_dec, revs_ra, false);
}

void MountController::move_relative_global(deg_t angle_dec, deg_t angle_ra) {

    angle_dec = to_180_range(fmod(angle_dec, 360));
    angle_ra  = to_180_range(fmod(angle_ra,  360));

    coord_t curr_pos = get_local_mount_orientation();  
    coord_t curr_pos_global = get_global_mount_orientation();

    // new desired global pos DEC can also change RA if exceeds bounds

    curr_pos_global.dec += angle_dec;

    if (curr_pos_global.dec > 90) {
        angle_ra += 180;
        curr_pos_global.dec = 180 - curr_pos_global.dec;
    } else
    if (curr_pos_global.dec < -90) {
        angle_ra += 180;
        curr_pos_global.dec = -180 - curr_pos_global.dec;
    }

    curr_pos_global.ra = fmod(curr_pos_global.ra + angle_ra, 360);
    if (curr_pos_global.ra < 0) curr_pos_global.ra += 360;

    // almost the same code as in move_absolute, creation of another method could be considered

    coord_t new_pos = polar_to_polar(curr_pos_global, _transition);

    float revs_dec = (new_pos.dec - curr_pos.dec) / DEG_PER_MOUNT_REV_DEC * REDUCTION_RATIO_DEC;
    float revs_ra  = (new_pos.ra  - curr_pos.ra)  / DEG_PER_MOUNT_REV_RA  * REDUCTION_RATIO_RA;

    #ifdef DEBUG_MOUNT
        Serial.println("Turning at high speed by:");
        Serial.print("  DEC:  "); Serial.println(new_pos.dec - curr_pos.dec);
        Serial.print("  RA:   "); Serial.println(new_pos.ra  - curr_pos.ra);
        Serial.print("  revs DEC:  "); Serial.println(revs_dec);
        Serial.print("  revs RA:   "); Serial.println(revs_ra);
    #endif
        
    _motors.fast_turn(revs_dec, revs_ra, false);
}

void MountController::set_tracking() {

    // TODO: it would be nice to change the speed dynamically after some time, based on the time
    // ellapsed from the start of the movement. Unfortunately it would require a schedule 
    // of steppers speed and a clever correction (we are doing a conversion of a complex
    // sin/cos/sqrt function into a simple stair function) and it is hard.

    float w = 15.0f;
    
    coord_t target = get_global_mount_orientation();
    coord_t speed = get_ra_speed_transform(w, 0.0f, target, _mount_pole, _mount_ra_offset);
    
    speed.dec *= REDUCTION_RATIO_DEC / DEG_PER_MOUNT_REV_DEC;
    speed.ra  *= REDUCTION_RATIO_RA  / DEG_PER_MOUNT_REV_RA;

    #ifdef DEBUG_MOUNT
        Serial.println("Tracking:");
        Serial.print("  target DEC: "); Serial.println(target.dec);
        Serial.print("  target RA:  "); Serial.println(target.ra);
        Serial.print("  speed DEC:   "); Serial.println(speed.dec / 3600.0f, 4);
        Serial.print("  speed RA:    "); Serial.println(speed.ra  / 3600.0f, 4);
    #endif

    _motors.slow_turn(speed.dec, speed.ra, speed.dec / 3600.0f, speed.ra / 3600.0f, true);

    _is_tracking = true;
}

MountController::coord_t MountController::get_ra_speed_transform(deg_t ra_speed, float t, coord_t point, coord_t pole, deg_t ra_offset) {

    // Ugly and hardcoded derivative :-(

    //  1) take x, y, z of a point based on start point, time and angle velocity
    //  2) transform into z' (like using _transform), but matrices must be hardcoded because of the next step
    //  3) take derivative dz'/dt
    //  4) compute derivatives of dec angle velocity: w_dec = (dz'/dt) / sqrt(1 - z'^2)
    //  5) ra angle velocity w_ra^2 = 1 - w_dec^2

    double coscos = cos(pole.dec) * cos(point.dec);
    double sinsin = sin(pole.ra)  * sin(point.dec);
    double real_ra = point.ra - ra_offset + ra_speed * t;

    double z_transformed = sinsin - coscos * cos(real_ra);
    double z_derivative  = coscos * sin(real_ra);

    double w_dec = z_derivative / sqrt(1.0 - z_transformed * z_transformed); // arcsin derivative
    float w_ra = sqrt(1.0 - w_dec);

    return coord_t { ra_speed * w_dec, ra_speed * w_ra };
}

void MountController::stop_tracking() {

    if (!_is_tracking) return;

    _motors.stop();
    _is_tracking = false;
}

double MountController::get_LST() {

    // Arduino cannot handle 64 bit floats so this
    // https://aa.usno.navy.mil/faq/docs/GAST.php
    // algorithm must be a little bit tweaked to 
    // reach a precision of 4 decimal places ~ 1s
   
    // LST gives me an angle [0..24) between local meridian and 0 RA

    double D1 = (367L * year()) - 730531.5;
    double D2 = D1 - (long)((7.0 * (year() + (long)((month() + 9.0) / 12.0f))) / 4.0);
    
    double D3 = (long)(30.55555555 * month()) + day();
    double D4 = D3 + (hour() + minute() / 60.0 + second() / 3600.0) / 24.0;
    
    long LD2 = D2;
    long LD4 = D4;

    double RD2 = D2 - LD2;
    double RD4 = D4 - LD4;

    // nutation, precession omitted ...
    double GMST = 0.06570982441908 * (LD2 + LD4) + 24.0 * (RD2 + RD4) + 0.06570982441908 * (RD2 + RD4);
    GMST += 18.697374558 + LONGITUDE / 15.0;

    return fmod(GMST, 24.0f);
}

MountController::cartesian_t MountController::polar_to_cartesian(coord_t polar) {

    double rad_dec = to_rad(polar.dec);
    double rad_ra  = to_rad(polar.ra);
    double cos_dec = cos(rad_dec);

    return cartesian_t { cos_dec * cos(rad_ra),
                         cos_dec * sin(rad_ra),
                         sin(rad_dec) };
}

MountController::coord_t MountController::cartesian_to_polar(cartesian_t cartesian) {

    double ra = to_deg(atan2(cartesian.y, cartesian.x));
    if (ra < 0) ra += 360;
       
    return coord_t { to_deg(asin(cartesian.z)), ra};
}

MountController::matrix_t MountController::get_dec_transition(deg_t dec) {

    double cos_dec = cos(to_rad(dec));
    double sin_dec = sin(to_rad(dec));

    return matrix_t {
        {{ sin_dec, 0, -cos_dec },
         { 0      , 1,  0       },
         { cos_dec, 0,  sin_dec }}
    };
}

MountController::matrix_t MountController::get_dec_transition_inverse(deg_t dec) {

    double cos_dec = cos(to_rad(dec));
    double sin_dec = sin(to_rad(dec));
    
    return matrix_t {
        {{ sin_dec, 0, cos_dec},
         { 0      , 1, 0      },
         {-cos_dec, 0, sin_dec}}
    };
}

MountController::matrix_t MountController::get_ra_transition(deg_t ra) {

    double cos_ra = cos(to_rad(ra));
    double sin_ra = sin(to_rad(ra));

    return matrix_t {
        {{  cos_ra, sin_ra, 0},
         { -sin_ra, cos_ra, 0},
         {  0,      0,      1}}
    };
}

MountController::matrix_t MountController::get_ra_transition_inverse(deg_t ra) {

    double cos_ra = cos(to_rad(ra));
    double sin_ra = sin(to_rad(ra));

    return matrix_t {
        {{ cos_ra, -sin_ra, 0},
         { sin_ra,  cos_ra, 0},
         { 0,       0,      1}}
    };
}