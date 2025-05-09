/*
 * TaskMoveTo.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef TASK_MOVE_TO_H_
#define TASK_MOVE_TO_H_

#include <cstdlib>
#include "Controller.h"
#include "Task.h"

/**
 * This is a specific implementation of a task class that moves the robot to a given pose.
 */
class TaskMoveTo : public Task {
    
    public:
        
        static const float  DEFAULT_VELOCITY;   /**< Default velocity value, given in [m/s]. */
        static const float  DEFAULT_ZONE;       /**< Default zone value, given in [m]. */
        
                    TaskMoveTo(Controller& controller, float x, float y, float alpha);
                    TaskMoveTo(Controller& controller, float x, float y, float alpha, float velocity);
                    TaskMoveTo(Controller& controller, float x, float y, float alpha, float velocity, float zone);
        virtual     ~TaskMoveTo();
        virtual int run(float period);
        
    private:
        
        static const float  M_PI;
        static const float  K1;
        static const float  K2;
        static const float  K3;
        
        Controller& controller;     // reference to the controller object to use
        float       x;              // x coordinate of target position, given in [m]
        float       y;              // y coordinate of target position, given in [m]
        float       alpha;          // target orientation, given in [rad]
        float       velocity;       // maximum translational velocity, given in [m/s]
        float       zone;           // zone threshold around target position, given in [m]
};

#endif /* TASK_MOVE_TO_H_ */
