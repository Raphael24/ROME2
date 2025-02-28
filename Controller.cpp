/*
 * Controller.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "Controller.h"

using namespace std;

const float Controller::PERIOD = 0.001f;                    // period of control task, given in [s]
const float Controller::COUNTS_PER_TURN = 1200.0f;          // encoder resolution (pololu motors: 1200.0f, maxon motors: 86016.0f)
const float Controller::LOWPASS_FILTER_FREQUENCY = 300.0f;  // given in [rad/s]
const float Controller::KN = 40.0f;                         // speed constant in [rpm/V] (pololu motors: 40.0f, maxon motors: 45.0f)
const float Controller::KP = 0.01f;                         // speed control parameter
const float Controller::MAX_VOLTAGE = 12.0f;                // battery voltage in [V]
const float Controller::MIN_DUTY_CYCLE = 0.02f;             // minimum duty-cycle
const float Controller::MAX_DUTY_CYCLE = 0.98f;             // maximum duty-cycle

/**
 * Creates and initialises the robot controller.
 * @param pwmLeft a reference to the pwm output for the left motor.
 * @param pwmRight a reference to the pwm output for the right motor.
 * @param counterLeft a reference to the encoder counter of the left motor.
 * @param counterRight a reference to the encoder counter of the right motor.
 */
Controller::Controller(PwmOut& pwmLeft, PwmOut& pwmRight, EncoderCounter& counterLeft, EncoderCounter& counterRight) : pwmLeft(pwmLeft), pwmRight(pwmRight), counterLeft(counterLeft), counterRight(counterRight), thread(osPriorityHigh, STACK_SIZE) {
    
    // initialise pwm outputs

    pwmLeft.period(0.00005f);  // pwm period of 50 us
    pwmLeft = 0.5f;            // duty-cycle of 50%

    pwmRight.period(0.00005f); // pwm period of 50 us
    pwmRight = 0.5f;           // duty-cycle of 50%

    // initialise local variables

    previousValueCounterLeft = counterLeft.read();
    previousValueCounterRight = counterRight.read();

    speedLeftFilter.setPeriod(PERIOD);
    speedLeftFilter.setFrequency(LOWPASS_FILTER_FREQUENCY);

    speedRightFilter.setPeriod(PERIOD);
    speedRightFilter.setFrequency(LOWPASS_FILTER_FREQUENCY);

    desiredSpeedLeft = 0.0f;
    desiredSpeedRight = 0.0f;

    actualSpeedLeft = 0.0f;
    actualSpeedRight = 0.0f;
    
    // start thread and timer interrupt
    
    thread.start(callback(this, &Controller::run));
    ticker.attach(callback(this, &Controller::sendThreadFlag), PERIOD);
}

/**
 * Deletes this Controller object.
 */
Controller::~Controller() {
    
    ticker.detach(); // stop the timer interrupt
}

/**
 * Sets the desired speed of the left motor.
 * @param desiredSpeedLeft desired speed given in [rpm].
 */
void Controller::setDesiredSpeedLeft(float desiredSpeedLeft) {

    this->desiredSpeedLeft = desiredSpeedLeft;
}

/**
 * Sets the desired speed of the right motor.
 * @param desiredSpeedRight desired speed given in [rpm].
 */
void Controller::setDesiredSpeedRight(float desiredSpeedRight) {

    this->desiredSpeedRight = desiredSpeedRight;
}

/**
 * This method is called by the ticker timer interrupt service routine.
 * It sends a flag to the thread to make it run again.
 */
void Controller::sendThreadFlag() {
    
    thread.flags_set(threadFlag);
}

/**
 * This is an internal method of the controller that is running periodically.
 */
void Controller::run() {

    while (true) {
        
        // wait for the periodic thread flag
        
        ThisThread::flags_wait_any(threadFlag);
        
        // calculate the actual speed of the motors in [rpm]

        short valueCounterLeft = counterLeft.read();
        short valueCounterRight = counterRight.read();

        short countsInPastPeriodLeft = valueCounterLeft-previousValueCounterLeft;
        short countsInPastPeriodRight = valueCounterRight-previousValueCounterRight;

        previousValueCounterLeft = valueCounterLeft;
        previousValueCounterRight = valueCounterRight;

        actualSpeedLeft = speedLeftFilter.filter((float)countsInPastPeriodLeft/COUNTS_PER_TURN/PERIOD*60.0f);
        actualSpeedRight = speedRightFilter.filter((float)countsInPastPeriodRight/COUNTS_PER_TURN/PERIOD*60.0f);

        // calculate desired motor voltages Uout

        // bitte implementieren!
        
        //float voltageLeft = ...
        //float voltageRight = ...

        // calculate, limit and set the duty-cycle

        float dutyCycleLeft = 0.5f+0.5f*voltageLeft/MAX_VOLTAGE;
        if (dutyCycleLeft < MIN_DUTY_CYCLE) dutyCycleLeft = MIN_DUTY_CYCLE;
        else if (dutyCycleLeft > MAX_DUTY_CYCLE) dutyCycleLeft = MAX_DUTY_CYCLE;
        pwmLeft = dutyCycleLeft;

        float dutyCycleRight = 0.5f+0.5f*voltageRight/MAX_VOLTAGE;
        if (dutyCycleRight < MIN_DUTY_CYCLE) dutyCycleRight = MIN_DUTY_CYCLE;
        else if (dutyCycleRight > MAX_DUTY_CYCLE) dutyCycleRight = MAX_DUTY_CYCLE;
        pwmRight = dutyCycleRight;
    }
}
