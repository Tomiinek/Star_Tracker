#ifndef RTC_DS3231_H
#define RTC_DS3231_H

#include <RTClib.h>
#include <Time.h>

#include "../config.h"
#include "clock.h"

#define DS3231_ADDRESS    RTC_ADRESS

class RtcDS3231 : public Clock {

   public:

		void obtain_time() override { 
      		if (!_rtc.begin()) {
        		#ifdef DEBUG_TIME
          			Serial.println("Cannot find DS3231");
        		#endif
				while(true);
      		}
			set_time(_rtc.now());
    	}

    	void sync(const DateTime& dt) override { 
      		_rtc.adjust(dt);
      		set_time(dt);
    	};

  	private: 

	  	inline void set_time(const DateTime& dt) {
			Clock::_time.adjust(dt);
			Clock::_local_siderial_time_offset = Clock::compute_LST_offset();
		}

    	RTC_DS3231 _rtc;

};

#endif RTC_DS3231_H