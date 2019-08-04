#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal.h>

#include "../core/clock.h"
#include "../config.h"

#define DSP_ROWS            2
#define DSP_COLS            16
#define DSP_REFRESH_MS      250
#define DPS_BLINKING_MS     300

class Control;
enum  ControlSubState : short;
class Display {
    
    public:

        Display() : _lcd(LiquidCrystal(DSP_REGISTER_SEL_PIN, DSP_ENABLE_PIN, DSP_DATA_PIN7, 
                                       DSP_DATA_PIN6, DSP_DATA_PIN5, DSP_DATA_PIN4)) {}

        // initialize LCD and display intro screen
        void initialize(int brightness);

        // sets LCD brightness
        void set_brightness(int brightness);

        // help screen with few hints
        void render_help(bool refresh, ControlSubState phase);

        // screen just with the given global position
        void render_position(bool refresh, float ra, float dec);

        // main screen with some indicators and time
        void render_main(bool refresh, ControlSubState phase, const DateTime& lst, bool tracking, bool moving, bool shooting);

        // screen which announces next goto step
        void render_goto_info(bool refresh);

        // GoTo screen with new global positon input 
        void render_goto(bool refresh, ControlSubState phase, int ra[3], int dec[3]);

        // camera menu, leads to exposure and delay time, busrt mode on/off
        void render_camera(bool refresh, bool repeating);

        // camera time settings
        void render_camera_settings(bool refresh, ControlSubState phase, int time, int delay);

        // time adjustment
        void render_time(bool refresh, ControlSubState phase, int y, int m, int d, int h, int i, int s);

        // simple "enter UTC datetime" screen
        void render_time_info(bool refresh);

        // calibration menu, leads to next target point definition and alignment computation
        void render_calibration(bool refresh, bool can_submit, int num_pairs);

        // screen which announces next calibration steps
        void render_calibration_info(bool refresh);

        // similar to main screen, manual control to align the target point with the real position
        void render_calibration_alignment(bool refresh, ControlSubState phase);

        // similar to GoTo screen, choose next target point coordinates
        void render_calibration_selection(bool refresh, ControlSubState phase, int ra[3], int dec[3]);

        // screen which confirms loading of the mount calibration, also displays the cal. values
        void render_calibration_loaded(bool refresh, float pole_ra, float pole_dec, float ra_offset);
        
        // catalogue menu, leads to object selection in three defined catalogues - Messier, NGC and Caldwell
        void render_catalogue(bool refresh, ControlSubState phase, int object_number);
        
        // simple "please wait" screen
        void render_wait(bool refresh);

        // simple "entry not found" screen
        void render_not_found(bool refresh);

        // catalogue search results, display the object info including magnitude, size and type
        void render_catalogue_results(bool refresh, ControlSubState phase, int object_number, float magnitude, float size_a, float size_b, char type[5]);

        // brightness setting
        void render_brightness(bool refresh, int brightness, bool changed);

    private:

        // print icon indicating manual control at the top right corner
        void print_manual(ControlSubState phase);

        // print coordinates, dec should be in dms and ra in his format
        void print_coords(int dec[3], int ra[3], int start_col);

        void print_blinking(int col, int row, int value, int characters);

        // print signed 'number' padded by spaces from left to reach width of 'characters'
        void print_padded(int number, int characters);

        // print signed floating point 'number' padded by spaces from left to reach width of 'characters'
        void print_padded_float(float number, int characters);

        // converts decimal number to degrees, arc minutes, arc seconds format
        void dec_to_dms(float decimal, int dms[3]);

        // converts decimal number to hours, minutes, seconds format
        void dec_to_his(float decimal, int his[3]);

        // changes state of _blink according to last blink time, true if changed, false otherwise
        bool should_blink();

        LiquidCrystal _lcd;

        bool _blink = false;
        unsigned long _last_blink = 0;
        unsigned long _last_refresh = 0;

        byte arc_minute[8] = { 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00 };
        byte arc_second[8] = { 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
        byte arr_up[8] = { 0x04, 0x0A, 0x15, 0x04, 0x04, 0x04, 0x04, 0x00 };
        byte arr_down[8] = { 0x04, 0x04, 0x04, 0x04, 0x15, 0x0A, 0x04, 0x00 };
        byte cross[8] = { 0x00, 0x00, 0x0A, 0x04, 0x0A, 0x00, 0x00, 0x00 };
        byte shadow_two[8] = { 0x0A, 0x11, 0x00, 0x02, 0x00, 0x08, 0x15, 0x00 };
};

#endif