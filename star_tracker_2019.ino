
#include "config.h"
#include "motor_controller.h"

MotorController* mc;

void setup() {
  delay(1); 
  Serial.begin(115200);
  delay(100); 
  mc = &MotorController::instance();
  mc->initialize();
  delay(100); 
}

void loop() { 

  static float angle = 5.0f;
  static long last = 0;

  if (mc->is_ready() && (long)millis() - last > 1000) {
    //Serial.println((long)millis() - last);
    delay(5000);
    last = millis();    
    mc->turn(angle, angle, false, false);
    angle *= -1;
    //Serial.println("New command invoked!");

    // TODO: test stop()
  }

  delay(10);

}
