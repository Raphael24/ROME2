/*
 * TaskWait.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "TaskWait.h"

using namespace std;

/**
 * Creates a task object that waits for a given duration.
 */
TaskWait::TaskWait(Controller& controller, float duration) : controller(controller) {
    
    this->duration = duration;
    
    time = 0.0f;
}

/**
 * Deletes the task object.
 */
TaskWait::~TaskWait() {}

/**
 * This method is called periodically by a task sequencer.
 * @param period the period of the task sequencer, given in [s].
 * @return the status of this task, i.e. RUNNING or DONE.
 */
int TaskWait::run(float period) {
    
    controller.setTranslationalVelocity(0.0f);
    controller.setRotationalVelocity(0.0f);
    
    time += period;
    
    if (time < duration) {
        
        return RUNNING;
        
    } else {
        
        return DONE;
    }
}
