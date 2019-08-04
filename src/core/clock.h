#ifndef CLOCK_H
#define CLOCK_H

#include <Arduino.h>
#include <RTClib.h>

#include "../config.h"

class SubSecondRTC : public RTC_Millis {
    public:
        static void adjust(const DateTime& dt) {
            RTC_Millis::adjust(dt);
        }
        static void adjust(uint32_t t = SECONDS_FROM_1970_TO_2000) {
            RTC_Millis::adjust(DateTime(t));
        }
        static void adjust(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0, uint8_t min = 0, uint8_t sec = 0) {
            RTC_Millis::adjust(DateTime(year, month, day, hour, min, sec));
        }
        
        static uint32_t sub_second_millis() { return millis() - lastMillis; } 
};

class Clock  {

    public:

        // acquire current time and call adjust(the_time)
        virtual void obtain_time() = 0;

        // adjust internal clocks and synchronize RTC module if needed
        virtual void sync(const DateTime& dt) = 0;

        static DateTime get_time() { return _time.now(); }

        static double get_decimal_time() { 
            auto dt = _time.now();
            return dt.hour() + dt.minute() / 60.0 + ((float)dt.second() + _time.sub_second_millis() / 1000.0) / 3600.0;
        }

        static DateTime get_LST() { return _time.now() + _local_siderial_time_offset; }

        static double get_decimal_LST() { 
            auto dt = _time.now() + _local_siderial_time_offset;
            return dt.hour() + dt.minute() / 60.0 + ((float)dt.second() + _time.sub_second_millis() / 1000.0f) / 3600.0f;   
        }

    protected:

        // compute local siderial time, precision of few seconds
        static TimeSpan compute_LST_offset() {

            // Arduino cannot handle 64 bit floats so this
            // https://aa.usno.navy.mil/faq/docs/GAST.php
            // algorithm must be a little bit tweaked to 
            // reach a precision of 4 decimal places ~ 1s
        
            // LST gives me an angle [0..24) between local meridian and 0 RA

            DateTime dt = get_time();
            double dt_d = get_decimal_time();

            double D1 = (367L * dt.year()) - 730531.5;
            double D2 = D1 - (long)((7.0 * (dt.year() + (long)((dt.month() + 9.0) / 12.0f))) / 4.0);
            
            double D3 = (long)(30.55555555 * dt.month()) + dt.day();
            double D4 = D3 + dt_d / 24.0;
            
            long LD2 = D2;
            long LD4 = D4;

            double RD2 = D2 - LD2;
            double RD4 = D4 - LD4;

            // nutation, precession omitted ...
            double GMST = 0.06570982441908 * (LD2 + LD4) + 24.0 * (RD2 + RD4) + 0.06570982441908 * (RD2 + RD4);
            GMST += 18.697374558 + LONGITUDE / 15.0;

            #ifdef DEBUG_TIME
                Serial.print(F("Local siderial time: ")); Serial.println(fmod(GMST, 24.0f), 5);
            #endif

            double GMST_diff = fmod(GMST, 24.0f) - dt_d;
            if (GMST_diff < 0) GMST_diff += 24.0f;

            return TimeSpan(GMST_diff * 3600.0);
        }

        static SubSecondRTC _time;
        static TimeSpan     _local_siderial_time_offset;
};

#endif REALTIMECLOCK_H