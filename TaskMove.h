/*
 * TaskMove.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef TASK_MOVE_H_
#define TASK_MOVE_H_

#include <cstdlib>
#include "Controller.h"
#include "Task.h"

/**
 * This is a specific implementation of a task class that moves the robot with a given speed.
 */
class TaskMove : public Task {
    
    public:
        
        static const float  DEFAULT_DURATION;       /**< Default duration value, given in [s]. */
        
                    TaskMove(Controller& controller, float translationalVelocity, float rotationalVelocity);
                    TaskMove(Controller& controller, float translationalVelocity, float rotationalVelocity, float duration);
        virtual     ~TaskMove();
        virtual int run(float period);
        
    private:
        
        Controller& controller;             // reference to the controller object to use
        float       translationalVelocity;
        float       rotationalVelocity;
        float       duration;
        float       time;
};

#endif /* TASK_MOVE_H_ */
