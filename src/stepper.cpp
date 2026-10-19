#include "stepper.h"
#include <driver/gpio.h>
#include <math.h>
#include <limits.h>

namespace {
constexpr uint32_t halfPeriodUs = 1000000UL / (2UL * STEP_RATE_HZ);
constexpr float stepsPerDegree =
    (360.0f / MOTOR_STEP_ANGLE) * MICROSTEP * RATIO / 360.0f;

hw_timer_t *stepTimer = nullptr;
portMUX_TYPE stepMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t currentSteps = 0;
volatile int32_t targetSteps = 0;
volatile bool moving = false;
volatile bool pulseHigh = false;
volatile bool activeDirection = false;
volatile bool positionInitialized = false;
bool feedbackControlEnabled = false;
float feedbackTargetAngle = 0.0f;
volatile float requestedStepRateHz = (float)STEP_RATE_HZ;
volatile float stepRateAccumulator = 0.0f;

void IRAM_ATTR onStepTimer() {
    portENTER_CRITICAL_ISR(&stepMux);

    if (pulseHigh) {
        gpio_set_level((gpio_num_t)STEP, 0);
        pulseHigh = false;
        if (currentSteps == targetSteps) {
            moving = false;
        }
        portEXIT_CRITICAL_ISR(&stepMux);
        return;
    }

    if (!moving || currentSteps == targetSteps) {
        moving = false;
        portEXIT_CRITICAL_ISR(&stepMux);
        return;
    }

    stepRateAccumulator += requestedStepRateHz;
    if (stepRateAccumulator < STEP_RATE_HZ) {
        portEXIT_CRITICAL_ISR(&stepMux);
        return;
    }
    stepRateAccumulator -= STEP_RATE_HZ;

    const bool direction = targetSteps > currentSteps;
    if (direction != activeDirection) {
        gpio_set_level((gpio_num_t)DIR, direction);
        activeDirection = direction;
        portEXIT_CRITICAL_ISR(&stepMux);
        return;
    }

    gpio_set_level((gpio_num_t)STEP, 1);
    pulseHigh = true;
    currentSteps += direction ? 1 : -1;

    portEXIT_CRITICAL_ISR(&stepMux);
}
}

bool stepInit(void){
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(ENA, OUTPUT);
    digitalWrite(STEP, LOW);
    digitalWrite(DIR, LOW);

    stepTimer = timerBegin(0, 80, true);
    if (stepTimer == nullptr) {
        return false;
    }
    timerAttachInterrupt(stepTimer, &onStepTimer, true);
    timerAlarmWrite(stepTimer, halfPeriodUs, true);
    timerAlarmEnable(stepTimer);
    positionInitialized = false;
    return true;
}

bool setStepperInitialAngle(float angleDeg) {
    if (stepTimer == nullptr || !isfinite(angleDeg)) {
        return false;
    }

    const double initialSteps = (double)angleDeg * stepsPerDegree;
    if (initialSteps < (double)INT32_MIN || initialSteps > (double)INT32_MAX) {
        return false;
    }

    portENTER_CRITICAL(&stepMux);
    if (moving || pulseHigh) {
        portEXIT_CRITICAL(&stepMux);
        return false;
    }
    currentSteps = (int32_t)lround(initialSteps);
    targetSteps = currentSteps;
    positionInitialized = true;
    feedbackControlEnabled = false;
    portEXIT_CRITICAL(&stepMux);
    return true;
}

void enableStep(bool en){
    if(en) digitalWrite(ENA, LOW);
    else digitalWrite(ENA, HIGH);
}

int16_t runStep(bool en, bool direct){ // true là CCW, góc tăng; false là CW, góc giảm
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

bool moveToAngle(float angleDeg) {
    if (stepTimer == nullptr || !isfinite(angleDeg)) {
        return false;
    }

    const double requestedSteps = (double)angleDeg * stepsPerDegree;
    if (requestedSteps < (double)INT32_MIN || requestedSteps > (double)INT32_MAX) {
        return false;
    }
    const int32_t newTarget = (int32_t)lround(requestedSteps);

    portENTER_CRITICAL(&stepMux);
    if (!positionInitialized) {
        portEXIT_CRITICAL(&stepMux);
        return false;
    }
    targetSteps = newTarget;
    moving = currentSteps != targetSteps;
    feedbackControlEnabled = false;
    portEXIT_CRITICAL(&stepMux);
    return true;
}

bool moveToAngleFeedback(float angleDeg) {
    if (stepTimer == nullptr || !isfinite(angleDeg) ||
        angleDeg < STEPPER_MIN_DEG || angleDeg > STEPPER_MAX_DEG) {
        return false;
    }

    portENTER_CRITICAL(&stepMux);
    if (!positionInitialized) {
        portEXIT_CRITICAL(&stepMux);
        return false;
    }
    feedbackTargetAngle = angleDeg;
    feedbackControlEnabled = true;
    portEXIT_CRITICAL(&stepMux);
    return true;
}

void setStepperRate(float rateHz) {
    if (!isfinite(rateHz)) {
        return;
    }
    rateHz = fmaxf(0.0f, fminf((float)STEP_RATE_HZ, rateHz));
    portENTER_CRITICAL(&stepMux);
    requestedStepRateHz = rateHz;
    if (rateHz >= (float)STEP_RATE_HZ) {
        stepRateAccumulator = 0.0f;
    }
    portEXIT_CRITICAL(&stepMux);
}

bool updateStepperFeedback(float measuredAngleDeg) {
    if (!isfinite(measuredAngleDeg) || measuredAngleDeg < 0.0f ||
        measuredAngleDeg >= 360.0f || measuredAngleDeg < STEPPER_MIN_DEG ||
        measuredAngleDeg > STEPPER_MAX_DEG) {
        stopStepper();
        return false;
    }

    portENTER_CRITICAL(&stepMux);
    if (!positionInitialized || !feedbackControlEnabled) {
        portEXIT_CRITICAL(&stepMux);
        return false;
    }

    const float errorDeg = feedbackTargetAngle - measuredAngleDeg;

    if (fabsf(errorDeg) <= STEPPER_FEEDBACK_DEADBAND_DEG) {
        targetSteps = currentSteps;
        moving = false;
        portEXIT_CRITICAL(&stepMux);
        return true;
    }

    const int64_t correctionSteps = lround((double)errorDeg * stepsPerDegree);
    const int64_t newTargetSteps = (int64_t)currentSteps + correctionSteps;
    if (newTargetSteps < INT32_MIN || newTargetSteps > INT32_MAX) {
        targetSteps = currentSteps;
        moving = false;
        feedbackControlEnabled = false;
        portEXIT_CRITICAL(&stepMux);
        return false;
    }

    targetSteps = (int32_t)newTargetSteps;
    moving = currentSteps != targetSteps;
    portEXIT_CRITICAL(&stepMux);
    return true;
}

void stopStepper(void) {
    portENTER_CRITICAL(&stepMux);
    targetSteps = currentSteps;
    moving = false;
    feedbackControlEnabled = false;
    requestedStepRateHz = (float)STEP_RATE_HZ;
    stepRateAccumulator = 0.0f;
    portEXIT_CRITICAL(&stepMux);
}

bool isStepperMoving(void) {
    portENTER_CRITICAL(&stepMux);
    const bool result = moving || pulseHigh;
    portEXIT_CRITICAL(&stepMux);
    return result;
}

float getStepperAngle(void) {
    portENTER_CRITICAL(&stepMux);
    const int32_t steps = currentSteps;
    const bool initialized = positionInitialized;
    portEXIT_CRITICAL(&stepMux);
    if (!initialized) {
        return NAN;
    }
    return steps / stepsPerDegree;
}
