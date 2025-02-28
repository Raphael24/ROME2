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
        void    setDesiredSpeedLeft(float desiredSpeedLeft);
        void    setDesiredSpeedRight(float desiredSpeedRight);
        
    private:
        
        static const unsigned int   STACK_SIZE = 4096;  // stack size of thread, given in [bytes]
        static const float          PERIOD;             // period of control task, given in [s]
        
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
        short               previousValueCounterLeft;
        short               previousValueCounterRight;
        LowpassFilter       speedLeftFilter;
        LowpassFilter       speedRightFilter;
        float               desiredSpeedLeft;
        float               desiredSpeedRight;
        float               actualSpeedLeft;
        float               actualSpeedRight;
        ThreadFlag          threadFlag;
        Thread              thread;
        Ticker              ticker;
        
        void    sendThreadFlag();
        void    run();
};

#endif /* CONTROLLER_H_ */
