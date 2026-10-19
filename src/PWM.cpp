#include "PWM.h"


void pwmInit(){
  ledcSetup(0, 50, 11); ledcAttachPin(FIRST_ARM , 0);  //PWM Hardware
  ledcSetup(1, 50, 11); ledcAttachPin(SECOND_ARM, 1);
  ledcSetup(2, 50, 11); ledcAttachPin(WRIST     , 2);
  ledcSetup(3, 50, 11); ledcAttachPin(GRIPPER   , 3);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline uint16_t duty(uint16_t pulse) {
    return (uint32_t)pulse * 2048 / 20000;
}

void runPwm(float pos1, float pos2, float pos3, float pos4){
  uint16_t ch1 = mapFloat(pos1, 0.0f, 90.0f, 2480.0f, 1550.0f);
  uint16_t ch2 = mapFloat(pos2, 90.0f, 180.0f, 960.0f, 1950.0f);
  uint16_t ch3 = mapFloat(pos3, 0.0f, 180.0f, 600.0f, 2400.0f);;
  uint16_t ch4 = mapFloat(pos4, 0.0f, 180.0f, 600.0f, 2400.0f);;
  ledcWrite(0, duty(ch1)); 
  ledcWrite(1, duty(ch2));   
  ledcWrite(2, duty(ch3)); 
  ledcWrite(3, duty(ch4));
}
