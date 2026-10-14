#include <Arduino.h>
#include "config.h"
#include "PWM.h"
#include "stepper.h"

unsigned long long time_loop;
int sp = 1600;

void setup() {
  Serial.begin(926100);
  pwmInit();
  stepInit();
  enableStep(true);
  runPwm(1500, 1500, 1500, 1500);
  time_loop = micros();
}

void loop() {
  if(micros() - time_loop >= 1000000 / LOOP_FREQ){
    time_loop = micros();
    releaseStep();

    delayMicroseconds(1000);

    if(sp > 0){
      sp += runStep(true, false);
    }
  }
}
