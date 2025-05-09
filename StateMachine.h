/*
 * StateMachine.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <cstdlib>
#include <deque>
#include <mbed.h>
#include "Controller.h"
#include "IRSensor.h"
#include "Task.h"
#include "ThreadFlag.h"

/**
 * This class implements a simple state machine for a mobile robot.
 * It allows to move the robot forward, and to turn left or right,
 * depending on distance measurements, to avoid collisions with
 * obstacles.
 */
class StateMachine {
    
    public:
        
        static const int    ROBOT_OFF = 0;      // discrete states of this state machine
        static const int    MOVE_FORWARD = 1;
        static const int    TURN_LEFT = 2;
        static const int    TURN_RIGHT = 3;
        static const int    SLOWING_DOWN = 4;
        
                    StateMachine(Controller& controller, DigitalOut& enableMotorDriver, DigitalOut& led0, DigitalOut& led1, DigitalOut& led2, DigitalOut& led3, DigitalOut& led4, DigitalOut& led5, DigitalIn& button, IRSensor& irSensor0, IRSensor& irSensor1, IRSensor& irSensor2, IRSensor& irSensor3, IRSensor& irSensor4, IRSensor& irSensor5);
        virtual     ~StateMachine();
        int         getState();
        
    private:
        
        static const unsigned int   STACK_SIZE = 4096;  // stack size of thread, given in [bytes]
        static const float          PERIOD;             // period of task, given in [s]
        
        static const float  DISTANCE_THRESHOLD;         // minimum allowed distance to obstacle in [m]
        static const float  TRANSLATIONAL_VELOCITY;     // translational velocity in [m/s]
        static const float  ROTATIONAL_VELOCITY;        // rotational velocity in [rad/s]
        static const float  VELOCITY_THRESHOLD;         // velocity threshold before switching off, in [m/s] and [rad/s]
        
        Controller&     controller;
        DigitalOut&     enableMotorDriver;
        DigitalOut&     led0;
        DigitalOut&     led1;
        DigitalOut&     led2;
        DigitalOut&     led3;
        DigitalOut&     led4;
        DigitalOut&     led5;
        DigitalIn&      button;
        IRSensor&       irSensor0;
        IRSensor&       irSensor1;
        IRSensor&       irSensor2;
        IRSensor&       irSensor3;
        IRSensor&       irSensor4;
        IRSensor&       irSensor5;
        int             state;
        int             buttonNow;
        int             buttonBefore;
        deque<Task*>    taskList;
        ThreadFlag      threadFlag;
        Thread          thread;
        Ticker          ticker;
        
        void    sendThreadFlag();
        void    run();
};

#endif /* STATE_MACHINE_H_ */
