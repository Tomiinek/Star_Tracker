#define FROM_LIB

#include "display.h"
#include "control.h"

void Display::initialize(int brightness) {

    _lcd.begin(DSP_COLS, DSP_ROWS);

    _lcd.createChar(0, arc_minute);
    _lcd.createChar(1, arc_second);
    _lcd.createChar(2, arr_up);
    _lcd.createChar(3, arr_down);
    _lcd.createChar(4, cross);
    _lcd.createChar(5, shadow_two);
    
    _lcd.noCursor();
    _lcd.clear();
    
    pinMode(DSP_ANODE_PIN, OUTPUT);
    set_brightness(brightness);

    delay(100);
  
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Hello!")); 

    _lcd.setCursor(0, 1); 
    _lcd.print(F("StarTracker v")); 
    _lcd.print(VERSION);

    delay(2000);

    render_wait(true);

    _lcd.setCursor(0, 1); 
    _lcd.print(F("Press # for help")); 

    delay(2000);
}

void Display::set_brightness(int brightness) {
    if (brightness > 255) brightness = 255;
    if (brightness < 0)   brightness = 0;
    analogWrite(DSP_ANODE_PIN, brightness);
}

void Display::render_help(bool refresh, ControlSubState phase) {

    if (!refresh) return;

    _lcd.clear();
    _lcd.setCursor(0, 0); 

    if (phase == S0) {
        _lcd.print(F("Instructions    "));
        _lcd.setCursor(0, 1);
        _lcd.print(F("follow:         "));
    }
    else if(phase == S1) {
        _lcd.print(F("Arrows .... Move"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("Ent .. Precision"));
    }
    else if(phase == S2) {
        _lcd.print(F("* .... Cam.shoot"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("7 . Cam.settings"));
    }
    else if(phase == S3) {
        _lcd.print(F("#  Stop Sh/Tr/Mv"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("# long .... Help"));
    }
    else if(phase == S4) {
        _lcd.print(F("1 ......... GoTo"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("2 .... Show pos."));
    }
    else if(phase == S5) {
        _lcd.print(F("3 .. Calibration"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("3 long Load cal."));
    }
    else if(phase == S6) {
        _lcd.print(F("4 ...... Messier"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("5 ..... Caldwell"));
    }
    else if(phase == S7) {
        _lcd.print(F("6 .......... NGC")); 
        _lcd.setCursor(0, 1);
        _lcd.print(F("                "));
    }
    else if(phase == S8) {
        _lcd.print(F("8 ... Brightness"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("9 .. Time config"));
    }
    else if(phase == S9) {
        _lcd.print(F("Use long 0 press"));
        _lcd.setCursor(0, 1);
        _lcd.print(F("for negative n. "));
    }
}

void Display::render_position(bool refresh, float ra, float dec) {

    if (refresh) {
        _lcd.clear();
        _lcd.setCursor(0, 0); _lcd.print(F("R:"));
        _lcd.setCursor(0, 1); _lcd.print(F("D:"));
    }

    if (millis() - _last_refresh < DSP_REFRESH_MS && !refresh) return;
    _last_refresh = millis();

    int his_ra[3];  dec_to_his(ra, his_ra);
    int dms_dec[3]; dec_to_dms(dec, dms_dec);

    print_coords(dms_dec, his_ra, 3);
}

void Display::render_main(bool refresh, ControlSubState phase, const DateTime& lst, bool tracking, bool moving, bool shooting) {

    if (refresh) {
        _lcd.clear();
        _lcd.setCursor(0, 0); _lcd.print(F("LST:"));      
    }

    if (millis() - _last_refresh < DSP_REFRESH_MS && !refresh) return;
    _last_refresh = millis();

    _lcd.setCursor(5, 0);  print_padded(lst.hour(), 2); _lcd.print(F("h "));
    _lcd.setCursor(9, 0); print_padded(lst.minute(), 2); _lcd.print(F("m "));
    _lcd.setCursor(13, 0); print_padded(lst.second(), 2); _lcd.print(F("s "));

    _lcd.setCursor(0, 1); 
    print_manual(phase);

    _lcd.setCursor(12, 1); 
    if ((tracking || moving) && shooting) _lcd.print(F("/"));
    else _lcd.print(F(" "));

    _lcd.setCursor(DSP_COLS - 1 - 2, 1); 
    if (tracking) _lcd.print(F("Trk"));
    else if (moving) _lcd.print(F("Mov"));
    else _lcd.print(F("   "));

    _lcd.setCursor(7, 1); 
    if (shooting) _lcd.print(F("Photo"));
    else _lcd.print(F("     "));
}

void Display::render_goto_info(bool refresh) {
    
    if (!refresh) return;

    _lcd.clear(); 
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Enter target")); 

    _lcd.setCursor(0, 1); 
    _lcd.print(F("coord (to date):")); 
}

void Display::render_goto(bool refresh, ControlSubState phase, int ra[3], int dec[3]) {

    if (refresh) {
        _lcd.clear();
        _lcd.setCursor(0, 0); _lcd.print(F("R:"));
        _lcd.setCursor(0, 1); _lcd.print(F("D:"));

        print_coords(dec, ra, 3);
    }

    if (!should_blink()) return;

    if (phase == ControlSubState::S0) print_blinking(3, 0, ra[0], 3);      
    if (phase == ControlSubState::S1) print_blinking(9, 0, ra[1], 2);        
    if (phase == ControlSubState::S2) print_blinking(13, 0, ra[2], 2);
    if (phase == ControlSubState::S3) print_blinking(3, 1, dec[0], 3);      
    if (phase == ControlSubState::S4) print_blinking(9, 1, dec[1], 2);
    if (phase == ControlSubState::S5) print_blinking(13, 1, dec[2], 2);
}

void Display::render_camera(bool refresh, bool repeating) {

    if (!refresh) return;

    _lcd.clear();
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Time conf.   (1)")); 

    _lcd.setCursor(0, 1); 
    _lcd.print(F("Burst        (2)")); 

    _lcd.setCursor(6, 1); 
    if (repeating) _lcd.print(F("ON")); 
    else _lcd.print(F("0FF")); 
}

void Display::render_camera_settings(bool refresh, ControlSubState phase, int time, int delay) {

    if (refresh) {
        _lcd.clear();

        _lcd.setCursor(0, 0); 
        _lcd.print(F("Exposure:"));

        _lcd.setCursor(DSP_COLS -1 - 4, 0);
        print_padded(time, 4);
        _lcd.print(F("s"));

        _lcd.setCursor(0, 1);
        _lcd.print(F("Snap delay:")); 
        
        _lcd.setCursor(DSP_COLS - 1 - 4, 1);
        print_padded(delay, 4);
        _lcd.print(F("s"));
    }

    if (!should_blink()) return;
            
    if (phase == ControlSubState::S1 || phase == ControlSubState::S3) print_blinking(DSP_COLS - 1 - 4, 0, time, 4);
    if (phase == ControlSubState::S2 || phase == ControlSubState::S4) print_blinking(DSP_COLS - 1 - 4, 1, delay, 4);
}

void Display::render_time(bool refresh, ControlSubState phase, int y, int m, int d, int h, int i, int s) {

    if (refresh) {
        _lcd.clear();

        _lcd.setCursor(0, 0);  
        _lcd.print(F("Y:"));
        print_padded(y, 4);
        _lcd.print(F(" M:"));
        print_padded(m, 2);
        _lcd.print(F(" D:"));
        print_padded(d, 2);
        
        _lcd.setCursor(0, 1);
        _lcd.print(F("H:"));
        print_padded(h, 2);
        _lcd.print(F(" Mi:"));
        print_padded(i, 2);
        _lcd.print(F(" Sc:"));
        print_padded(s, 2);
    }

    if (!should_blink()) return;
            
    if (phase == ControlSubState::S0 || phase == ControlSubState::S6)       print_blinking(2, 0, y, 4);
    else if (phase == ControlSubState::S1 || phase == ControlSubState::S7)  print_blinking(9, 0, m, 2);
    else if (phase == ControlSubState::S2 || phase == ControlSubState::S8)  print_blinking(14, 0, d, 2);
    else if (phase == ControlSubState::S3 || phase == ControlSubState::S9)  print_blinking(2, 1, h, 2);
    else if (phase == ControlSubState::S4 || phase == ControlSubState::S10) print_blinking(8, 1, i, 2);
    else if (phase == ControlSubState::S5 || phase == ControlSubState::S11) print_blinking(14, 1, s, 2);
}

void Display::render_time_info(bool refresh) {
    if (!refresh) return;
    _lcd.clear(); 
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Enter UTC")); 
    _lcd.setCursor(0, 1); 
    _lcd.print(F("date and time:")); 
}

void Display::render_calibration(bool refresh, bool can_submit, int num_pairs) {
            
    if (!refresh) return;

    _lcd.clear();
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Add pair:"));

    _lcd.setCursor(DSP_COLS - 1 - 2, 0); 
    _lcd.print(F("(1)"));

    _lcd.setCursor(0, 1); 
    _lcd.print(F("Align ("));
    char digits[4];
    itoa(num_pairs, digits, 10);
    _lcd.print(digits);
    _lcd.print(F("/3):"));

    _lcd.setCursor(DSP_COLS - 1 - 2, 1);
    if (can_submit) _lcd.print(F("(2)"));
    else {
        _lcd.print(F("("));
        _lcd.write((uint8_t)5);
        _lcd.print(F(")"));
    }
}

void Display::render_calibration_info(bool refresh) {
            
    if (!refresh) return;

    _lcd.clear();
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Enter new target"));

    _lcd.setCursor(0, 1); 
    _lcd.print(F("coord (to date):"));
}

void Display::render_calibration_selection(bool refresh, ControlSubState phase, int ra[3], int dec[3]) {
            
    if (refresh) {
        _lcd.clear();
        _lcd.setCursor(0, 0); _lcd.print(F("R:"));
        _lcd.setCursor(0, 1); _lcd.print(F("D:"));

        print_coords(dec, ra, 3);
    }

    if (!should_blink()) return;

    if (phase == ControlSubState::S2) print_blinking(3, 0, ra[0], 3);      
    if (phase == ControlSubState::S3) print_blinking(9, 0, ra[1], 2);        
    if (phase == ControlSubState::S4) print_blinking(13, 0, ra[2], 2);
    if (phase == ControlSubState::S5) print_blinking(3, 1, dec[0], 3);      
    if (phase == ControlSubState::S6) print_blinking(9, 1, dec[1], 2);
    if (phase == ControlSubState::S7) print_blinking(13, 1, dec[2], 2);
}

void Display::render_calibration_alignment(bool refresh, ControlSubState phase) {

    if (refresh) {
        _lcd.clear();

        _lcd.setCursor(0, 0); 
        _lcd.print(F("Aim! Use arrows")); 

        _lcd.setCursor(0, 1); 
        _lcd.print(F("Confirm (*)"));
    }

    if (millis() - _last_refresh > DSP_REFRESH_MS || refresh) {
        _last_refresh = millis();
        _lcd.setCursor(DSP_COLS - 1 - 3, 1); 
        print_manual(phase);
    }
}

void Display::render_calibration_loaded(bool refresh, float pole_ra, float pole_dec, float ra_offset) {
    
    if (!refresh) return;

    char digits[8];

    _lcd.clear();
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Calib. loaded:"));

    _lcd.setCursor(0, 1); 
    _lcd.print(F("P: "));
    itoa((int)pole_ra, digits, 10);
    _lcd.print(digits);

    _lcd.print(F("/"));
    itoa((int)pole_dec, digits, 10);
    _lcd.print(digits);
    
    _lcd.setCursor(DSP_COLS - 1 - 6, 1); 
    _lcd.print(F(" OF:"));
    print_padded((int)ra_offset, 3);
}

void Display::render_catalogue(bool refresh, ControlSubState phase, int object_number) {

    if (refresh) {
        _lcd.clear();
        _lcd.setCursor(0, 0); 
        _lcd.print(F("Object number:")); 

        _lcd.setCursor(0, 1); 
        if (phase == ControlSubState::S0) _lcd.print(F("(Messier)"));
        else if (phase == ControlSubState::S1) _lcd.print(F("(Caldwell)"));
        else if (phase == ControlSubState::S2) _lcd.print(F("(NGC)"));

        _lcd.setCursor(DSP_COLS - 1 - 3, 1);
        print_padded(object_number, 4);
    }

    if (!should_blink()) return;
    print_blinking(DSP_COLS - 1 - 3, 1, object_number, 4);
}

void Display::render_catalogue_results(bool refresh, ControlSubState phase, int object_number, 
                                       float magnitude, float size_a, float size_b, char type[5]) {
    if (!refresh) return;

    _lcd.clear();
    _lcd.setCursor(0, 0); 

    if (phase == ControlSubState::S0) _lcd.print(F("Mes "));
    else if (phase == ControlSubState::S1) _lcd.print(F("Cal "));
    else if (phase == ControlSubState::S2) _lcd.print(F("NGC "));

    char digits[8];
    itoa(object_number, digits, 10);
    _lcd.print(digits);

    if (magnitude != 0 && magnitude != 99) {
        _lcd.setCursor(DSP_COLS - 1 - 7, 0); 
        _lcd.print(F(" ("));
        print_padded_float(magnitude, 4);
        _lcd.print(F("m)"));
    }
    
    _lcd.setCursor(0, 1);
    _lcd.print(type);

    _lcd.setCursor(DSP_COLS - 1 - 9, 1);
    print_padded_float(size_a, 4);
    _lcd.write((uint8_t)4);
    print_padded_float(size_b, 4);
    _lcd.write((uint8_t)0);
}

void Display::render_wait(bool refresh) {

    if (!refresh) return;

    _lcd.clear();
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Please wait ...")); 
    _lcd.setCursor(0, 1); 
    _lcd.print(F("Calculate ")); 
    _lcd.print(random(100, 999));
    _lcd.print(F("/")); 
    _lcd.print(random(20, 99));
}

void Display::render_not_found(bool refresh) {

    if (!refresh) return;

    _lcd.clear(); 
    _lcd.setCursor(0, 0); 
    _lcd.print(F("Not found ...")); 
    _lcd.setCursor(0, 1); 
    _lcd.print(F("It is a pity :(")); 
}

void Display::render_brightness(bool refresh, int brightness, bool changed) {

    if (refresh) {
        _lcd.clear(); 

        _lcd.setCursor(0, 0); 
        _lcd.print(F("Brightness:")); 

        _lcd.setCursor(0, 1); 
        _lcd.print(F("(use "));
        _lcd.write((uint8_t)2);
        _lcd.write((uint8_t)3);
        _lcd.print(F(")")); 
        
        _lcd.setCursor(DSP_COLS - 1 - 3, 1);
        _lcd.print(F("/255")); 
    }

    if (changed || refresh) {
        _lcd.setCursor(DSP_COLS - 1 - 6, 1);
        print_padded(brightness, 3);
    }
}

void Display::print_manual(ControlSubState phase) {
    
    if (phase != ControlSubState::S0 && phase != ControlSubState::S8) {
        _lcd.print(F("Man"));
        if (phase == ControlSubState::S1 || phase == ControlSubState::S9)       _lcd.print((char)223);
        else if (phase == ControlSubState::S2 || phase == ControlSubState::S10) _lcd.write((uint8_t)0);
        else if (phase == ControlSubState::S3 || phase == ControlSubState::S11) _lcd.write((uint8_t)1);
    }
    else _lcd.print(F("    "));
}

void Display::print_coords(int dec[3], int ra[3], int start_col) {

    _lcd.setCursor(start_col, 0);       print_padded(ra[0], 3); _lcd.print(F("h"));
    _lcd.setCursor(start_col + 6, 0);   print_padded(ra[1], 2); _lcd.print(F("m"));
    _lcd.setCursor(start_col + 10, 0);  print_padded(ra[2], 2); _lcd.print(F("s"));

    _lcd.setCursor(start_col, 1);       print_padded(dec[0], 3); _lcd.print((char)223);
    _lcd.setCursor(start_col + 6, 1);   print_padded(dec[1], 2); _lcd.write((uint8_t)0);
    _lcd.setCursor(start_col + 10, 1);  print_padded(dec[2], 2); _lcd.write((uint8_t)1);
}

void Display::print_blinking(int col, int row, int value, int characters) {
    
    _lcd.setCursor(col, row);
    
    if (_blink) print_padded(value, characters);
    else {
        int i = 0;
        char space_buffer[9];
        for (; i < min(8, characters); ++i) space_buffer[i] = ' '; 
        space_buffer[i] = '\0';
        _lcd.print(space_buffer); 
    }
}

void Display::print_padded(int number, int characters) {

    if (characters > 7) return;

    char digits[8];
    itoa(number, digits, 10);
    int n = 0;
    for (; n < 8; ++n) if (digits[n] == '\0') break;

    char str[8];
    for (int i = 0; i < characters - n; ++i) str[i] = ' ';
    for (int i = 0; i < n; ++i) str[i + characters - n] = digits[i];
    str[characters] = '\0';

    _lcd.print(str);
}

void Display::print_padded_float(float number, int characters) {

    if (characters > 7) return;

    char str[8];

    int precision = characters - 1;
    float tmp = number;
    if (tmp < 0) {
        --precision;
        tmp *= -1;
    }
    while (tmp >= 1) {
        precision--;
        tmp /= 10.0f;
    }

    dtostrf(number, characters, precision, str);
    str[characters] = '\0';

    _lcd.print(str);
}

void Display::dec_to_dms(float decimal, int dms[3]) {
    dms[0] = (int)decimal;
    dms[1] = (int)((decimal - dms[0]) * 60.0f);
    dms[2] = (int)((decimal - dms[0] - dms[1] / 60.0f) * 3600.0f);
}

void Display::dec_to_his(float decimal, int his[3]) {
    decimal /= 15.0f;
    dec_to_dms(decimal, his);
}

bool Display::should_blink() {

    if (millis() - _last_blink < DPS_BLINKING_MS) return false; 

    _last_blink = millis();
    _blink = !_blink;
    return true;
}