#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>

#include "../config.h"
#include "../core/mount_controller.h"
#include "../core/camera_controller.h"
#include "../core/clock.h"

#include "keypad.h"
#include "display.h"

//	/=======================================================\
// 	|  MAPPING OF KEYPAD ONTO KEYS EXPECTED BY CONTROLLER : |
// 	\=======================================================/

#define C_ARROW_UP 				KP_KEY_UP_ARROW
#define C_ARROW_LEFT            KP_KEY_LEFT_ARROW
#define C_ARROW_RIGHT           KP_KEY_RIGHT_ARROW
#define C_ARROW_DOWN            KP_KEY_DOWN_ARROW     
#define C_ENTER					KP_KEY_OK
#define C_EXIT					KP_KEY_E
#define C_SHOOT					KP_KEY_F
#define C_TRACKING				KP_KEY_OK		// just pressed
#define C_GOTO					KP_KEY_D
#define C_POSITION				KP_KEY_A
#define C_PARKING				KP_KEY_0
#define C_CALIBRATION           KP_KEY_C    	
#define C_CAMERA				KP_KEY_6
#define C_TIME					KP_KEY_5
#define C_BRIGHTNESS            KP_KEY_B
#define C_MESSIER				KP_KEY_3
#define C_CALDWELL				KP_KEY_2
#define C_NGC					KP_KEY_1
#define C_N1					KP_KEY_1
#define C_N2					KP_KEY_2
#define C_N3					KP_KEY_3        
#define C_N4					KP_KEY_4
#define C_N5					KP_KEY_5
#define C_N6					KP_KEY_6
#define C_N7					KP_KEY_7
#define C_N8					KP_KEY_8
#define C_N9					KP_KEY_9
#define C_N0					KP_KEY_0

//	=================================
//	=================================

#define INFO_SCREEN_MS       1500 	// how long will be an intermediate (informative) screen displayed

enum ControlSubState : short { S0 = 0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11 };

class Control {

      public:

        enum State { MAIN, HELP, GOTO, CALIB, CATALOG, SHOOT, TIME, POSITION, BRIGHT };

        Control(MountController& mount, CameraController& camera, Clock& clock)
            : _mount(mount), _camera(camera), _clock(clock) {}

        // initialize display, camera, mount, clock and SD card
        void initialize();

        // updates state of the overall controller, proper menus are displayed and actions 
        // based on done keypad actions are performed
        void update();

    private:

        // save a variable to EEPROM
        template <class T> 
          void save(T value, uint16_t adress) {
            byte* p = (byte*)(void*)&value;
            for (int i = 0; i < sizeof(value); i++) EEPROM.write(adress++, *p++);
        }

        template <class T>
        void load(T& target, uint16_t adress, T lower, T upper, T default_value) {
            load(target, adress);
            if (target >= lower && target <= upper) return; // NaN returns false for all comp.
            target = default_value;
        }

        template <class T>
        void load(T& target, uint16_t adress) {
            byte* p = (byte*)(void*)&target;
            for (int i = 0; i < sizeof(target); i++) *p++ = EEPROM.read(adress++);
        }

        // display global position, facilitate manual controll and motors stopping
        void main_menu();

        // display current position
        void position_menu();

        // brightness settings
        void brightness_menu();

        // selection of a target postiion
        void goto_menu();

        // handling camera settings, stopping
        void camera_menu();

        // time adjustment
        void time_menu();

        // overall control of the calibration procedure
        void calibration_menu();

        // catalogue search
        void catalogue_menu();

        void help_menu();

        void manual_control(ControlSubState nothing, ControlSubState degrees, ControlSubState minutes, ControlSubState seconds);

        void clear_position_buffers();

        void add_digit(int& number, int digit, int min, int max);

        // returns pushed digit or -1 if no key was pushed or 10 is zero was pressed for longer a
        // longer time (which is used in this code to alter sign of the number being specified)
        int get_pushed_digit();

        // search in a catalogue file on the SD card for the specified object with number 'object'
        bool find_in_catalogue(ControlSubState catalogue, int object, MountController::coord_t& coords, 
                               float& magnitude, float& size_a, float& size_b, char type[5]);

        inline MountController::coord_t position_buffers_to_coords() {
            return MountController::coord_t {
                 _ra_buffer[0] +  _ra_buffer[1] / 60.0f +  _ra_buffer[2] / 3600.0f,
                _dec_buffer[0] + _dec_buffer[1] / 60.0f + _dec_buffer[2] / 3600.0f,
            };
        }

        inline void change_state(State new_state) {
            _state = new_state;
            _substate = ControlSubState::S0;
            _substate_changed = false;
            _state_changed = true;
            _last_substate_change_time = millis();
        }

        inline void change_substate(ControlSubState new_state) {
            _substate = new_state;
            _substate_changed = true;
        }

        inline ControlSubState increment_substate() {
            return static_cast<ControlSubState>(static_cast<uint8_t>(_substate)+1);
        }

        State _state;
        ControlSubState _substate;
        bool _state_changed = false;
        bool _substate_changed = false;
        bool _last_state_changed = false;
        bool _last_substate_changed = false;
        unsigned long _last_substate_change_time = 0;

        int _ra_buffer[3];
        int _dec_buffer[3];
        int _time[6];

        int _shooting_time_buffer = 123;
        int _shooting_delay_buffer = 123;

        int _catalogue_buffer = 0;

        int _brightness_buffer = 128;

        SDClass* _sd;

        Display _display;
        Keypad _keypad;
        
        Clock& _clock;
        MountController& _mount;
        CameraController& _camera;

        uint8_t _calibration_buffer_size = 0;
        MountController::coord_t _kernel;
        MountController::coord_t _kernel_buffer[CAL_BUFFER_SIZE];
        MountController::coord_t _image_buffer[CAL_BUFFER_SIZE];
};

#endif
