/*
 * TaskMove.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "TaskMove.h"

using namespace std;

const float TaskMove::DEFAULT_DURATION = 24.0f*60.0f*60.0f; // default duration value, given in [s]

/**
 * Creates a task object that moves the robot with a given speed.
 * @param conroller a reference to the controller object of the robot.
 * @param translationalVelocity the translational velocity to move with, given in [m/s].
 * @param rotationalVelocity the rotational velocity to move with, given in [rad/s].
 */
TaskMove::TaskMove(Controller& controller, float translationalVelocity, float rotationalVelocity) : controller(controller) {
    
    this->translationalVelocity = translationalVelocity;
    this->rotationalVelocity = rotationalVelocity;
    this->duration = DEFAULT_DURATION;
    this->time = 0.0f;
}

/**
 * Creates a task object that moves the robot with a given speed.
 * @param conroller a reference to the controller object of the robot.
 * @param translationalVelocity the translational velocity to move with, given in [m/s].
 * @param rotationalVelocity the rotational velocity to move with, given in [rad/s].
 * @param duration the duration of this task.
 */
TaskMove::TaskMove(Controller& controller, float translationalVelocity, float rotationalVelocity, float duration) : controller(controller) {
    
    this->translationalVelocity = translationalVelocity;
    this->rotationalVelocity = rotationalVelocity;
    this->duration = duration;
    this->time = 0.0f;
}

/**
 * Deletes the task object.
 */
TaskMove::~TaskMove() {}

/**
 * This method is called periodically by a task sequencer.
 * @param period the period of the task sequencer, given in [s].
 * @return the status of this task, i.e. RUNNING or DONE.
 */
int TaskMove::run(float period) {
    
    time += period;
    
    if (time < duration) {
        
        controller.setTranslationalVelocity(translationalVelocity);
        controller.setRotationalVelocity(rotationalVelocity);
        
        return RUNNING;
        
    } else {
        
        controller.setTranslationalVelocity(0.0f);
        controller.setRotationalVelocity(0.0f);
        
        return DONE;
    }
}
