#define FROM_LIB

#include "control.h"

void Control::initialize() {

    load(_shooting_time_buffer, EEPROM_ADDR, 1, 9999, 30);
    load(_shooting_delay_buffer, EEPROM_ADDR + 2, 1, 9999, 30);
    load(_brightness_buffer, EEPROM_ADDR + 4, 0, 255, 128);

    _mount.initialize();
    _display.initialize(_brightness_buffer);

    SD.begin(SD_CS);
    _sd = &SD;

    _keypad.initialize();
    _camera.initialize();

    _clock.obtain_time();

    delay(100);

    _state = MAIN;
    _state_changed = true;
}

void Control::update() {

    _keypad.update();
    _camera.update();

    _last_state_changed = _state_changed;
    _last_substate_changed = _substate_changed;
    _state_changed = false;
    _substate_changed = false;

    if (_keypad.pressed(C_EXIT)) change_state(HELP);

    switch (_state) {
        case MAIN: 	  main_menu(); break;
        case HELP: 	  help_menu(); break;
        case GOTO: 	  goto_menu(); break;
        case CALIB:   calibration_menu(); break;
        case SHOOT:   camera_menu(); break;
        case CATALOG: catalogue_menu(); break;
        case TIME:    time_menu(); break;
        case BRIGHT:    brightness_menu(); break;
        case POSITION:  position_menu(); break;
    }
}

void Control::help_menu() {

    if (_last_state_changed) _last_substate_change_time = millis();

    if (_keypad.pushed(C_EXIT)) change_state(MAIN);

    if ((millis() - _last_substate_change_time) > INFO_SCREEN_MS) {
        _last_substate_change_time = millis();
        change_substate(increment_substate());
        if (_substate > S9) change_substate(S0);
    }

    _display.render_help(_last_state_changed || _last_substate_changed, _substate);			
}

void Control::position_menu() {

    if (_keypad.pushed(C_EXIT)) change_state(MAIN);

    auto orientation = _mount.get_global_mount_orientation();
    _display.render_position(_last_state_changed || _last_substate_changed, orientation.ra, orientation.dec);			
}

void Control::brightness_menu() {

    if (_keypad.pushed(C_EXIT)) {
        change_state(MAIN);
        save(_brightness_buffer, EEPROM_ADDR + 4);
    }

    int change = 0;
    if (_keypad.pressed(C_ARROW_UP))    	change = 32;
    else if (_keypad.pressed(C_ARROW_DOWN)) change = -32;	
    else if (_keypad.pushed(C_ARROW_UP))    change = 8;
    else if (_keypad.pushed(C_ARROW_DOWN))  change = -8;

    _brightness_buffer += change;
    if (_brightness_buffer > 255) _brightness_buffer = 255;
    if (_brightness_buffer < 0) _brightness_buffer = 0;

    _display.render_brightness(_last_state_changed, _brightness_buffer, change != 0);
    _display.set_brightness(_brightness_buffer);
}

void Control::main_menu() {

    if (_keypad.pressed(C_TRACKING))     	_mount.set_tracking();
    else if (_keypad.pushed(C_GOTO))     	change_state(GOTO);
    else if (_keypad.pushed(C_POSITION)) 	change_state(POSITION);
    else if (_keypad.pushed(C_BRIGHTNESS))  change_state(BRIGHT);	
    else if (_keypad.pushed(C_SHOOT))       _camera.shoot(_shooting_time_buffer, _shooting_delay_buffer);
    else if (_keypad.pressed(C_CALIBRATION)) {		
        float ra_offset;
        load(ra_offset, EEPROM_ADDR + 6, 0.0f, 360.0f, (float)DEFUALT_RA_OFFSET);

        MountController::coord_t pole;
        load(pole, EEPROM_ADDR + 10);
        if (isnan(pole.ra) || pole.ra < 0 || pole.ra >= 360) pole.ra = DEFAULT_POLE_RA;
        if (isnan(pole.dec) || pole.dec < -90 || pole.dec > 90) pole.dec = DEFAULT_POLE_DEC;

        _display.render_calibration_loaded(true, pole.ra, pole.dec, ra_offset);
        _mount.set_mount_pole(pole, ra_offset);

        delay(INFO_SCREEN_MS);
        _state_changed = true;
    }
    else if (_keypad.pushed(C_CALIBRATION)) change_state(CALIB);
    else if (_keypad.pushed(C_CAMERA)) 		change_state(SHOOT);
    else if (_keypad.pushed(C_TIME))   		change_state(TIME);
    else if (_keypad.pushed(C_MESSIER)) {
        change_state(CATALOG);
        change_substate(S0);
    }
    else if (_keypad.pushed(C_CALDWELL)) {
        change_state(CATALOG);
        change_substate(S1);
    } 
    else if (_keypad.pushed(C_NGC)) {
        change_state(CATALOG);
        change_substate(S2);
    }

    manual_control(S0, S1, S2, S3);

    _display.render_main(_last_state_changed || _last_substate_changed, _substate, Clock::get_LST(), 
                         _mount.is_tracking(), _mount.is_moving(), _camera.update());
            
    if (_keypad.pushed(C_EXIT)) {
        if (_camera.update()) _camera.reset();
        else if (_mount.is_tracking()) _mount.stop_tracking();
        else if (_mount.is_moving()) _mount.stop_all();
    } 
}

void Control::goto_menu() {

    if (_last_state_changed) {
        clear_position_buffers();
        _display.render_goto_info(true);
        delay(INFO_SCREEN_MS);
    }
    _display.render_goto(_last_substate_changed || _last_state_changed, _substate, _ra_buffer, _dec_buffer);
                
    if (_substate == S5 && _keypad.pushed(C_ENTER)) {

        change_state(MAIN);
        auto coords = position_buffers_to_coords();
        _mount.stop_all();
        _camera.reset();
        _mount.move_absolute(coords.ra, coords.dec);

        return;
    }

    if (_keypad.pushed(C_EXIT)) change_state(MAIN);
    if (_keypad.pushed(C_ENTER)) change_substate(increment_substate());

    int pushed_digit = get_pushed_digit();
    if (pushed_digit == -1) return;

    switch (_substate) {
        case S0: add_digit(_ra_buffer[0], pushed_digit, 0, 23); break;
        case S1: add_digit(_ra_buffer[1], pushed_digit, 0, 59); break;
        case S2: add_digit(_ra_buffer[2], pushed_digit, 0, 59); break;
        case S3: add_digit(_dec_buffer[0], pushed_digit, -89, 89); break;
        case S4: add_digit(_dec_buffer[1], pushed_digit, 0, 59); break;
        case S5: add_digit(_dec_buffer[2], pushed_digit, 0, 59); break;
    }
}

void Control::camera_menu() {	

    _camera.reset();

    if (_substate == S0) {
                
        if (_keypad.pushed(C_EXIT)) change_state(MAIN);

        bool repeating = _camera.get_repeating();
        _display.render_camera(_last_substate_changed || _last_state_changed, repeating);
                        
        if (_keypad.pushed(C_N1)) change_substate(S1);
        else if (_keypad.pushed(C_N2)) {		
            _camera.set_repeating(!repeating);
            change_state(MAIN);
        }

        return;
    }
    
    if ((_substate == S2 || _substate == S4) && _keypad.pushed(C_ENTER)) {
        change_state(MAIN);
        save(_shooting_time_buffer, EEPROM_ADDR);
        save(_shooting_delay_buffer, EEPROM_ADDR + 2);
        return;
    }

    _display.render_camera_settings(_last_substate_changed, _substate, _shooting_time_buffer, _shooting_delay_buffer);
    
    if (_keypad.pushed(C_EXIT)) {
        change_substate(S0);
        save(_shooting_time_buffer, EEPROM_ADDR);
        save(_shooting_delay_buffer, EEPROM_ADDR + 2);
    }

    if (_keypad.pushed(C_ENTER)) change_substate(S2);

    int pushed_digit = get_pushed_digit();
    if (pushed_digit == -1) return;

    switch (_substate) {
        case S1: 
            _shooting_time_buffer = 0;
            change_substate(S3);
        case S3: add_digit(_shooting_time_buffer,  pushed_digit, 1, 9999); break;
        case S2: 
            _shooting_delay_buffer = 0;
            change_substate(S4);
        case S4: add_digit(_shooting_delay_buffer, pushed_digit, 1, 9999); break;
    }
}

void Control::time_menu() {

    if (_last_state_changed) {
        auto dt = Clock::get_time();
        _time[0] = dt.year();
        _time[1] = dt.month();
        _time[2] = dt.day();
        _time[3] = dt.hour();
        _time[4] = dt.minute();
        _time[5] = dt.second();
        _display.render_time_info(true);
        delay(INFO_SCREEN_MS);
    }

    if ((_substate == S5 || _substate == S11) && _keypad.pushed(C_ENTER)) {
        _clock.sync(DateTime(_time[0], _time[1], _time[2], _time[3], _time[4], _time[5]));
        change_state(MAIN);
        return;
    }

    _display.render_time(_last_state_changed || _last_substate_changed, _substate, _time[0], _time[1], _time[2], _time[3], _time[4], _time[5]);

    if (_keypad.pushed(C_EXIT)) change_state(MAIN);
    if (_keypad.pushed(C_ENTER)) {
        if (_substate == S0 || _substate == S6) change_substate(S1);
        else if (_substate == S1 || _substate == S7) change_substate(S2);
        else if (_substate == S2 || _substate == S8) change_substate(S3);
        else if (_substate == S3 || _substate == S9) change_substate(S4);
        else if (_substate == S4 || _substate == S10) change_substate(S5);
    }

    int pushed_digit = get_pushed_digit();
    if (pushed_digit == -1) return;

    switch (_substate) {
        case S0: _time[0] = 0; change_substate(S6);
        case S6: add_digit(_time[0], pushed_digit, 1, 2100); break;

        case S1: _time[1] = 0; change_substate(S7);
        case S7: add_digit(_time[1], pushed_digit, 1, 12); break;

        case S2: _time[2] = 0; change_substate(S8);
        case S8: add_digit(_time[2], pushed_digit, 1, 31); break;

        case S3: _time[3] = 0; change_substate(S9);
        case S9: add_digit(_time[3], pushed_digit, 0, 23); break;

        case S4: _time[4] = 0; change_substate(S10);
        case S10: add_digit(_time[4], pushed_digit, 0, 59); break;

        case S5: _time[5] = 0; change_substate(S11);
        case S11: add_digit(_time[5], pushed_digit, 0, 59); break;
    }
}

void Control::calibration_menu() {

    if (_last_state_changed) _calibration_buffer_size = 0;
    if (_last_state_changed || (_last_substate_changed && _substate == S1)) clear_position_buffers();

    if (_substate == S0) {

        _display.render_calibration(_last_substate_changed || _last_state_changed, _calibration_buffer_size >= 3, _calibration_buffer_size);
        
        if (_keypad.pushed(C_EXIT)) change_state(MAIN);
        else if (_keypad.pushed(C_N1)) {
            change_substate(S1);
            _last_substate_change_time = millis();
        }
        else if (_keypad.pushed(C_N2) && _calibration_buffer_size >= 3) {
            
            _display.render_wait(true);
            
            _mount.all_star_alignment(_kernel_buffer, _image_buffer, _calibration_buffer_size);

            change_state(MAIN);
            _calibration_buffer_size = 0;

            MountController::coord_t pole;
            float offset; 
            _mount.get_mount_pole(pole, offset);
            save(offset, EEPROM_ADDR + 6);
            save(pole,   EEPROM_ADDR + 10);
        }
        return;
    }

    if (_substate == S1) {
        if (_keypad.pushed(C_EXIT)) change_substate(S0);
        _display.render_calibration_info(_last_substate_changed);
        if (millis() - _last_substate_change_time > INFO_SCREEN_MS) change_substate(S2);
        return;
    }

    if (_substate > S7) {
      
        _display.render_calibration_alignment(_last_substate_changed, _substate);	

        if (_keypad.pushed(C_CALIBRATION_CONFIRM)) {

            _mount.stop_all();

            _kernel_buffer[_calibration_buffer_size] = {_kernel.dec, MountController::to_time_global_ra(_kernel.ra)};
            _image_buffer[_calibration_buffer_size] = _mount.get_local_mount_orientation();        
            ++_calibration_buffer_size;

            change_substate(S0);
        }
        if (_keypad.pushed(C_EXIT)) {
            if (_mount.is_moving()) _mount.stop_all();
            else change_substate(S0);       
        } 

        manual_control(S8, S9, S10, S11);

        return;
    }

    _display.render_calibration_selection(_last_substate_changed, _substate, _ra_buffer, _dec_buffer);	
                
    if (_substate == S7 && _keypad.pushed(C_ENTER)) {

        if (_calibration_buffer_size < CAL_BUFFER_SIZE) {
            _kernel = position_buffers_to_coords();							
            _mount.stop_all();
            _camera.reset();
            _mount.move_absolute(_kernel.ra, _kernel.dec);
        }

        change_substate(S8);
        return;
    }

    if (_keypad.pushed(C_EXIT)) change_substate(S0);
    if (_keypad.pushed(C_ENTER)) change_substate(increment_substate());

    int pushed_digit = get_pushed_digit();
    if (pushed_digit == -1) return;
    _substate_changed = true;

    switch (_substate) {
        case S2: add_digit(_ra_buffer[0], pushed_digit, 0, 23); break;
        case S3: add_digit(_ra_buffer[1], pushed_digit, 0, 59); break;
        case S4: add_digit(_ra_buffer[2], pushed_digit, 0, 59); break;
        case S5: add_digit(_dec_buffer[0], pushed_digit, -89, 89); break;
        case S6: add_digit(_dec_buffer[1], pushed_digit, 0, 59); break;
        case S7: add_digit(_dec_buffer[2], pushed_digit, 0, 59); break;
    }
}

void Control::catalogue_menu() {

    if (_last_state_changed) _catalogue_buffer = 0;

    // handle search results
    if (_substate == S3) {

        if (_keypad.pushed(C_EXIT)) change_state(MAIN);
        if (_keypad.pushed(C_ENTER)) {
            change_state(MAIN);
            _mount.stop_all();
            _camera.reset();
            _mount.move_absolute_J2000(_kernel.dec, _kernel.ra);
        }
        return;
    }
    
    _display.render_catalogue(_last_substate_changed, _substate, _catalogue_buffer);

    if (_keypad.pushed(C_EXIT)) change_state(MAIN);
    if (_keypad.pushed(C_ENTER)) {
        
        _display.render_wait(true);
        
        float magnitude;
        float size_a, size_b; 
        char type[6];

        if (find_in_catalogue(_substate, _catalogue_buffer, _kernel, magnitude, size_a, size_b, type)) {
            _display.render_catalogue_results(true, _substate, _catalogue_buffer, magnitude, size_a, size_b, type);
            change_substate(S3);	
        }
        else {
            _display.render_not_found(true);
            delay(INFO_SCREEN_MS);
            _substate_changed = true;
            _catalogue_buffer = 0;
        }
    }

    int pushed_digit = get_pushed_digit();
    if (pushed_digit == -1) return;
    add_digit(_catalogue_buffer, pushed_digit, 0, 9999); 
    
    return;
}

void Control::manual_control(ControlSubState nothing, ControlSubState degrees, ControlSubState minutes, ControlSubState seconds) {

    if (_keypad.pushed(C_ENTER)) {
        if (_substate == seconds) change_substate(nothing);
        else change_substate(increment_substate());
    }

    if (_mount.is_moving() || _substate == nothing) return;

    float conversion_ratio = 1.0f;
    if (_substate == minutes) conversion_ratio = 60.0f/5.0f;
    else if (_substate == seconds) conversion_ratio = 3600.0f/5.0f;

    if (_keypad.pressed(C_ARROW_UP)) 		 _mount.move_relative_local(5 / conversion_ratio, 0);
    else if (_keypad.pressed(C_ARROW_LEFT))  _mount.move_relative_local(0, -5 / conversion_ratio);
    else if (_keypad.pressed(C_ARROW_RIGHT)) _mount.move_relative_local(0, 5 / conversion_ratio);
    else if (_keypad.pressed(C_ARROW_DOWN))  _mount.move_relative_local(-5 / conversion_ratio, 0);
    else if (_keypad.pushed(C_ARROW_UP))     _mount.move_relative_local(1 / conversion_ratio, 0);
    else if (_keypad.pushed(C_ARROW_LEFT))   _mount.move_relative_local(0, -1 / conversion_ratio);
    else if (_keypad.pushed(C_ARROW_RIGHT))  _mount.move_relative_local(0, 1 / conversion_ratio);
    else if (_keypad.pushed(C_ARROW_DOWN))   _mount.move_relative_local(-1 / conversion_ratio, 0);
}

int Control::get_pushed_digit() {

         if (_keypad.pushed(C_N1)) { return 1; }
    else if (_keypad.pushed(C_N2)) { return 2; }
    else if (_keypad.pushed(C_N3)) { return 3; }
    else if (_keypad.pushed(C_N4)) { return 4; }
    else if (_keypad.pushed(C_N5)) { return 5; }
    else if (_keypad.pushed(C_N6)) { return 6; }
    else if (_keypad.pushed(C_N7)) { return 7; }
    else if (_keypad.pushed(C_N8)) { return 8; }
    else if (_keypad.pushed(C_N9)) { return 9; }
    else if (_keypad.pressed(C_N0)) { return 10; }
    else if (_keypad.pushed(C_N0)) { return 0; }

    return -1;
}

bool Control::find_in_catalogue(ControlSubState catalogue, int object, MountController::coord_t& coords, 
                                float& magnitude, float& size_a, float& size_b, char type[6]) {
    #ifdef DEBUG_CONTROL
        Serial.println(F("=============="));
        File root = SD.open("/");
        while (true) {
            File entry =  root.openNextFile();
            if (!entry) break;
            Serial.print('\t');
            Serial.print(entry.name());
            Serial.print("\t\t");
            Serial.println(entry.size(), DEC);
            entry.close();
        }
        Serial.println(F("===================="));
    #endif

    const char* path = "/catalog.csv";

    File file = _sd->open(path);
            
    if (!file) {
        #ifdef DEBUG_CONTROL
            Serial.println("Catalogue file not found!");
        #endif
        return false;
    }

    int index = 0;
    int index_col = 0;
    bool row_found = false;
    bool skip_line = false;
    char buffer[24];

    int next;
    while ((next = file.read()) != -1) {
        char c = (char)next;

        if (index == 0 && c == '#') {
            skip_line = true;
            continue;
        }
        if (skip_line) {
            if (c == '\n') {
                skip_line = false;
                index = 0;
                index_col = 0;
            }
            continue;
        }

        if (c == ';' || c == '\n') {
            buffer[index] = '\0';
            index = 0;
            
            if (!row_found) {
                if ((catalogue == ControlSubState::S0 && index_col == 1) ||
                    (catalogue == ControlSubState::S1 && index_col == 2) ||
                    (catalogue == ControlSubState::S2 && index_col == 0)) {
                    int row_object = atoi(buffer);
                    if (row_object == object) row_found = true;
                    else skip_line = true;
                }
            } 
            else {
                switch (index_col) {
                case 3: coords.ra = atof(buffer);  break;
                case 4: coords.dec = atof(buffer); break;
                case 5: magnitude = atof(buffer); break;
                case 6: strncpy(type, buffer, 6); break;
                case 7: size_a = atof(buffer); break;
                case 8: size_b = atof(buffer); break;
                }
            }

            if (c == '\n') {
                if (row_found) break;
                index_col = 0;
            }
            else ++index_col;
        }
        else {
            buffer[index] = c;
            ++index;
        }
    }

    #ifdef DEBUG_CONTROL
        if (!row_found) {
            Serial.print(F("Object "));
            Serial.print(object);
            Serial.println(" not found in the catalogue!");
        }
    #endif	 

    file.close();
    return row_found;
}

void Control::clear_position_buffers() {
    for (uint8_t i = 0; i < 3; ++i) {
        _ra_buffer[i] = 0;
        _dec_buffer[i] = 0;
    }
}

void Control::add_digit(int& number, int digit, int min, int max) {
    if ((number == 0 && digit == 0) || digit < 0) return;
    int tmp = (digit == 10 ? -number : number * 10 + digit);
    if (tmp > max || tmp < min) return;
    number = tmp;
}