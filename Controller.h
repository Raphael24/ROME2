/*
 * Controller.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <cstdlib>
#include <mbed.h>
#include "EncoderCounter.h"
#include "Motion.h"
#include "LowpassFilter.h"
#include "ThreadFlag.h"

/**
 * This class implements a controller that regulates the
 * speed of the two motors of the ROME2 mobile robot.
 */
class Controller {
    
    public:
        
                Controller(PwmOut& pwmLeft, PwmOut& pwmRight, EncoderCounter& counterLeft, EncoderCounter& counterRight);
        virtual ~Controller();
        void    setTranslationalVelocity(float velocity);
        void    setRotationalVelocity(float velocity);
        float   getActualTranslationalVelocity();
        float   getActualRotationalVelocity();
        
    private:
        
        static const unsigned int   STACK_SIZE = 4096;  // stack size of thread, given in [bytes]
        static const float          PERIOD;             // period of control task, given in [s]
        
        static const float  M_PI;                       // the mathematical constant PI
        static const float  WHEEL_DISTANCE;             // distance between wheels, given in [m]
        static const float  WHEEL_RADIUS;               // radius of wheels, given in [m]
        static const float  MAXIMUM_VELOCITY;           // maximum wheel velocity, given in [rpm]
        static const float  MAXIMUM_ACCELERATION;       // maximum wheel acceleration, given in [rpm/s]
        static const float  COUNTS_PER_TURN;
        static const float  LOWPASS_FILTER_FREQUENCY;
        static const float  KN;
        static const float  KP;
        static const float  MAX_VOLTAGE;
        static const float  MIN_DUTY_CYCLE;
        static const float  MAX_DUTY_CYCLE;

        PwmOut&             pwmLeft;
        PwmOut&             pwmRight;
        EncoderCounter&     counterLeft;
        EncoderCounter&     counterRight;
        float               translationalVelocity;
        float               rotationalVelocity;
        float               actualTranslationalVelocity;
        float               actualRotationalVelocity;
        float               desiredSpeedLeft;
        float               desiredSpeedRight;
        float               actualSpeedLeft;
        float               actualSpeedRight;
        Motion              motionLeft;
        Motion              motionRight;
        short               previousValueCounterLeft;
        short               previousValueCounterRight;
        LowpassFilter       speedLeftFilter;
        LowpassFilter       speedRightFilter;
        ThreadFlag          threadFlag;
        Thread              thread;
        Ticker              ticker;
        
        void    sendThreadFlag();
        void    run();
};

#endif /* CONTROLLER_H_ */
