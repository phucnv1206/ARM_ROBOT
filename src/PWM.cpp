#include "PWM.h"


void pwmInit(){
  ledcSetup(0, 50, 11); ledcAttachPin(FIRST_ARM , 0);  //PWM Hardware
  ledcSetup(1, 50, 11); ledcAttachPin(SECOND_ARM, 1);
  ledcSetup(2, 50, 11); ledcAttachPin(WRIST     , 2);
  ledcSetup(3, 50, 11); ledcAttachPin(GRIPPER   , 3);
}

inline uint16_t duty(uint16_t pulse) {
    return (uint32_t)pulse * 2048 / 20000;
}

void runPwm(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4){
  ledcWrite(0, duty(ch1)); 
  ledcWrite(1, duty(ch2));   
  ledcWrite(2, duty(ch3)); 
  ledcWrite(3, duty(ch4));
}
