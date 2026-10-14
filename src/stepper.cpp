#include "stepper.h"

void stepInit(void){
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(ENA, OUTPUT);
    digitalWrite(STEP, LOW);
}

void enableStep(bool en){
    if(en) digitalWrite(ENA, LOW);
    else digitalWrite(ENA, HIGH);
}

int16_t runStep(bool en, bool direct){
    digitalWrite(DIR, direct);
    if(en){
        digitalWrite(STEP, HIGH);
        if(direct) return 1;
        else return -1;
    }else{
        return 0;
    }
}

void releaseStep(void){
    digitalWrite(STEP, LOW);
}
