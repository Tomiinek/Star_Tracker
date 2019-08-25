# Arduino Star Tracker

This project aims an **automatic control for telescopes**, is **based on Arduino** and is **low-cost**!

The control utilizes a calibration method and various settings which enable you to control **any type of mount** (not just equatorial and altazimuthal) pointed at any direction.

The **total cost** of upgrading your mount should be around **75 USD** (*SkyWatcher SynScan GOTO Upgrade for EQ3-2* costs around *460 USD*).

## Features

See [future work](#future-work) for missing features (like communication via USB or Bluetooth). Some implemented and working freatures:

* **Manual control** with various precision (degrees, arc minutes and arc seconds).
* **Automatic pointing** at predefined coordinates.
* Onboard **catalogue of deep sky objects** (the one which is present in Stellarium), so you can search thousands of Messier, Caldwell and NGC objects.
* Precise **tracking** (for any type of mount).
* **Parking** to default position.
* **Calibration** of **mount pole** which works similarly to All-Star polar alignement.
* **Camera control** which alows you to take photos with predefined exposure time and with a predefined period.
* **Wireless control** via IR remote control.
* Real **asynchronous** control of **stepper motors** (any other code can be run in parallel). 

## Hardware setup

#### 0. What you will need:
* **Arduino MEGA2560**, unfortunately Arduino Uno cannot be used due to the lack of memory
* **Power supply** or battery, e.g. *12V* and around *2.5A* (depends on your stepper motors)
* *2×* **NEMA 17** stepper motor (forget about 28BYJ!)
* *2×* stepper motor driver like **A4988**
* *2×* stepper driver **extender for A4988** (we will modify this part, so you should consider custom design with 100μF capacitor)
* *4×* **4k7** or **10k resistor** (or any other suitable resistor you hold in your hands)
* *(optionally)* **Gearboxes** for the motors (battery screwdriver is typically much cheaper than profi gearboxes and has some planetary gearboxes inside!)
* *2×* **flexible shaft couplers** for joing the motors with mount
* **Real Time Clock**, note that *DS3231* or more precise clocks are preffered over DS1307 
* **microSD Card module**
* **16×2 LCD** (without I2C module)
* **10k Potentiometer**
* **IR receiver**
* **Remote control**, you can use your favourite one you use for TV or you can use any other for makers! This repo settings match with a [remote control](https://www.makeblock.com/project/me-ir-remote-controller) by Makeblock. 
* Some kind of **transistor**, e. g. *BC337* and suitable resistor, e. g. *1k ohms* 
* **Bunch of wires and connectors**

#### 1. Parts modification and tuning

You should **add** the four **resistors** between `STEP`-`GND` and `DIR`-`GND` pins **to avoid a jitter** of stepper motors at startup. 

A typical extender for A4988 has an `ENABLE` pin, however, we do not need it. Instead, we **need a pin for controlling microstepping**. That is why you should hardwire the three microstepping pins and you should add a single pin to control them. Search for *microstepping resolution truth table* of A4988.

A **cheap IR remote control** might have a **weak diode**. In that case, you won't be able to control the mount from a distant places. So I recommend you to **replace the diode with a better one.**  

You also need to **tune potentiometers of stepper drivers**. Search for *How to set output current limit on A4988 stepper driver*. This can reduce noise produced by the motors or you can remove **problems with skipping of steps**. 

Use the **potentiometer** connected to the LCD to **adjust contrast**.

#### 2. Wiring

![Wiring diagram](https://github.com/Tomiinek/Star_Tracker/blob/master/_img/wiring.png)

## Software Settings & Uploding to Arduino

#### 0. Prepearing libraries

The following list contains libraries you will need to sucessfuly compile the Star Tracker project (most of them is included in the Arduino IDE by default):

* [RTCLib](https://github.com/adafruit/RTClib)
* [LiquidCrystal](https://github.com/arduino-libraries/LiquidCrystal)
* [SD](https://github.com/arduino-libraries/SD)
* [SPI](https://www.arduino.cc/en/Reference/SPI)
* [Wire](https://www.arduino.cc/en/Reference/Wire)
* [EEPROM](https://www.arduino.cc/en/Reference/EEPROM)

At first, **make sure you have already installed all the libraries**.

#### 1. General configuration

Next, you should **modify the `src/config.h` file to fit your needs**. Familiarize yourself with all the definitions in the file. **You should definitely change `LONGITUDE`, `LATITUDE`, `REDUCTION_RATIO_xx` and `DEG_PER_MOUNT_REV_xx` variables!** You also need to change `DEFAULT_POLE_xx` if you use an altazimuthal mount.

You may encounter a problem with stucked motors. In that case, I **advice you to adjust `ACCEL_STEPS_xx`, `ACCEL_DELAY_xx`, `FAST_DELAY_START_xx` and `FAST_DELAY_END_xx`**. Change these definitions in order to change motors speed or ac/deceleration.

#### 2. Remote control

You need to **change definitions of key codes** to fit the protocol used by your favourite remote control. Take **look at the beginning of the `src/keypad.h` file**. There are few definitions of `KP_KEY_xx` where `xx` describes the particular key. To figure out **codes used by your remote control**, upload the following sketch to your Arduino:

```
#include <Arduino.h>
#include <IRremote.h>
#include "src/config.h"

IRrecv ir(KEYPAD_IR_PIN);
decode_results results;

void setup(){
    Serial.begin(9600);
    ir.enableIRIn();
}

void loop(){
    if (!ir.decode(&results)) {
      delay(5);
      return;
    }
    Serial.println(results.value, HEX);
    ir.resume();
    delay(500);
}
```

Then **open the Serial monitor** and **observe the key codes** of pressed keys. Note that you will receive `0xFFFFFFFF` when you hold a key continuously.

You can also **change mapping of these keys to particular actions** in the `src/control.h` file (but be careful as these keys and actions should not cause conflicts and ambiguity).

#### 3. SD card and catalogue

Take an **empty and formatted microSD** card and **copy there the content** of the `SD` directory. It should be a single file `catalog.csv` which is a limited version of **Stellarium catalogue of deep space objects**. You can delete or add object to this file, however the structure and meaning of columns should be preserved.

#### 4. Real Time Clock

The `src/rtc_ds3231.h` file contains implementation of `Clock` class for `DS3231` module. In case you use **other module** or you want to obtain time from NTP servers, **implement the `Clock` interface** and change some lines in `Star_Tracker.ino`.

#### 5. Camera trigger

Similarly, you may need to change the implementation of the camera trigger control. The `src/CanonEOS1000D.h` file contains implementation of `CameraController` class for *Canon EOS1000D*. If you have other camera with **other trigger logic**, you should **create a new implementation of `CameraController`** and change some lines at `Star_Tracker.ino`. Note that in this case, you may also need a different wiring!

## Notes on precision

The only loss of precision is caused by Arduino's floating point unit which cannot work with 64-bit floating point numbers. Especially while computing extremal values of some goniometric funcions (tangens and other functions which are reduced to computing tangens). These problems can occur while pointing to stars near celestial pole. The GoTo feature can miss few arc minutes and alignement can be imprecise in that case. However points with lower DEC values should be handled properly.  

The tracking with very badly aligned mount can also be imprecise after some time because the speeds of DEC and RA motors are computed just once at the start of the movement. Unfortunately these speeds does not have to be constant necessarily.

## Future work

- [ ] enable **serial communication** with *USB* and *Bluetooth*
- [ ] **LX200** support
- [ ]  **double** floating point **precision** 
- [ ]  better **speed** computation **while tracking**
- [ ]  compute **alignment analytically**
