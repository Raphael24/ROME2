/*
 * Controller.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "Controller.h"

using namespace std;

const float Controller::PERIOD = 0.001f;                    // period of control task, given in [s]
const float Controller::M_PI = 3.14159265f;                 // the mathematical constant PI
const float Controller::WHEEL_DISTANCE = 0.190f;            // distance between wheels, given in [m]
const float Controller::WHEEL_RADIUS = 0.0375f;             // radius of wheels, given in [m]
const float Controller::MAXIMUM_VELOCITY = 500.0;           // maximum wheel velocity, given in [rpm]
const float Controller::MAXIMUM_ACCELERATION = 200.0;       // maximum wheel acceleration, given in [rpm/s]
const float Controller::COUNTS_PER_TURN = 1200.0f;          // encoder resolution (pololu motors: 1200.0f, maxon motors: 86016.0f)
const float Controller::LOWPASS_FILTER_FREQUENCY = 300.0f;  // given in [rad/s]
const float Controller::KN = 40.0f;                         // speed constant in [rpm/V] (pololu motors: 40.0f, maxon motors: 45.0f)
const float Controller::KP = 0.15f;                         // speed control parameter
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

    translationalVelocity = 0.0f;
    rotationalVelocity = 0.0f;

    actualTranslationalVelocity = 0.0f;
    actualRotationalVelocity = 0.0f;

    desiredSpeedLeft = 0.0f;
    desiredSpeedRight = 0.0f;

    actualSpeedLeft = 0.0f;
    actualSpeedRight = 0.0f;

    motionLeft.setProfileVelocity(MAXIMUM_VELOCITY);
    motionLeft.setProfileAcceleration(MAXIMUM_ACCELERATION);
    motionLeft.setProfileDeceleration(MAXIMUM_ACCELERATION);

    motionRight.setProfileVelocity(MAXIMUM_VELOCITY);
    motionRight.setProfileAcceleration(MAXIMUM_ACCELERATION);
    motionRight.setProfileDeceleration(MAXIMUM_ACCELERATION);

    previousValueCounterLeft = counterLeft.read();
    previousValueCounterRight = counterRight.read();

    speedLeftFilter.setPeriod(PERIOD);
    speedLeftFilter.setFrequency(LOWPASS_FILTER_FREQUENCY);

    speedRightFilter.setPeriod(PERIOD);
    speedRightFilter.setFrequency(LOWPASS_FILTER_FREQUENCY);

    x = 0.0f;
    y = 0.0f;
    alpha = 0.0f;

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
 * Sets the desired translational velocity of the robot.
 * @param velocity the desired translational velocity, given in [m/s].
 */
void Controller::setTranslationalVelocity(float velocity) {
    
    this->translationalVelocity = velocity;
}

/**
 * Sets the desired rotational velocity of the robot.
 * @param velocity the desired rotational velocity, given in [rad/s].
 */
void Controller::setRotationalVelocity(float velocity) {
    
    this->rotationalVelocity = velocity;
}

/**
 * Gets the actual translational velocity of the robot.
 * @return the actual translational velocity, given in [m/s].
 */
float Controller::getActualTranslationalVelocity() {
    
    return actualTranslationalVelocity;
}

/**
 * Gets the actual rotational velocity of the robot.
 * @return the actual rotational velocity, given in [rad/s].
 */
float Controller::getActualRotationalVelocity() {
    
    return actualRotationalVelocity;
}

/**
 * Sets the actual x coordinate of the robots position.
 * @param x the x coordinate of the position, given in [m].
 */
void Controller::setX(float x) {
    
    this->x = x;
}

/**
 * Gets the actual x coordinate of the robots position.
 * @return the x coordinate of the position, given in [m].
 */
float Controller::getX() {
    
    return x;
}

/**
 * Sets the actual y coordinate of the robots position.
 * @param y the y coordinate of the position, given in [m].
 */
void Controller::setY(float y) {
    
    this->y = y;
}

/**
 * Gets the actual y coordinate of the robots position.
 * @return the y coordinate of the position, given in [m].
 */
float Controller::getY() {
    
    return y;
}

/**
 * Sets the actual orientation of the robot.
 * @param alpha the orientation, given in [rad].
 */
void Controller::setAlpha(float alpha) {
    
    this->alpha = alpha;
}

/**
 * Gets the actual orientation of the robot.
 * @return the orientation, given in [rad].
 */
float Controller::getAlpha() {
    
    return alpha;
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
        
        // calculate the values 'desiredSpeedLeft' and 'desiredSpeedRight' using the kinematic model
        
        desiredSpeedLeft = (translationalVelocity-WHEEL_DISTANCE/2.0f*rotationalVelocity)/WHEEL_RADIUS*60.0f/2.0f/M_PI;
        desiredSpeedRight = -(translationalVelocity+WHEEL_DISTANCE/2.0f*rotationalVelocity)/WHEEL_RADIUS*60.0f/2.0f/M_PI;
        
        // calculate planned speedLeft and speedRight values using the motion planner
        
        motionLeft.incrementToVelocity(desiredSpeedLeft, PERIOD);
        motionRight.incrementToVelocity(desiredSpeedRight, PERIOD);
        
        desiredSpeedLeft = motionLeft.getVelocity();
        desiredSpeedRight = motionRight.getVelocity();
        
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

        float voltageLeft = KP*(desiredSpeedLeft-actualSpeedLeft)+desiredSpeedLeft/KN;
        float voltageRight = KP*(desiredSpeedRight-actualSpeedRight)+desiredSpeedRight/KN;

        // calculate, limit and set the duty-cycle

        float dutyCycleLeft = 0.5f+0.5f*voltageLeft/MAX_VOLTAGE;
        if (dutyCycleLeft < MIN_DUTY_CYCLE) dutyCycleLeft = MIN_DUTY_CYCLE;
        else if (dutyCycleLeft > MAX_DUTY_CYCLE) dutyCycleLeft = MAX_DUTY_CYCLE;
        pwmLeft = dutyCycleLeft;

        float dutyCycleRight = 0.5f+0.5f*voltageRight/MAX_VOLTAGE;
        if (dutyCycleRight < MIN_DUTY_CYCLE) dutyCycleRight = MIN_DUTY_CYCLE;
        else if (dutyCycleRight > MAX_DUTY_CYCLE) dutyCycleRight = MAX_DUTY_CYCLE;
        pwmRight = dutyCycleRight;

        // calculate the values 'actualTranslationalVelocity' and 'actualRotationalVelocity' using the kinematic model

        actualTranslationalVelocity = (actualSpeedLeft-actualSpeedRight)*2.0f*M_PI/60.0f*WHEEL_RADIUS/2.0f;
        actualRotationalVelocity = (-actualSpeedRight-actualSpeedLeft)*2.0f*M_PI/60.0f*WHEEL_RADIUS/WHEEL_DISTANCE;
        
        // calculate the actual robot pose
        
        float deltaTranslation = actualTranslationalVelocity*PERIOD;
        float deltaOrientation = actualRotationalVelocity*PERIOD;
        
        x += cos(alpha+deltaOrientation)*deltaTranslation;
        y += sin(alpha+deltaOrientation)*deltaTranslation;
        
        float alpha = this->alpha+deltaOrientation;
        
        while (alpha > M_PI) alpha -= 2.0f*M_PI;
        while (alpha < -M_PI) alpha += 2.0f*M_PI;
        
        this->alpha = alpha;
    }
}
