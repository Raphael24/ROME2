/*
 * TaskWait.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef TASK_WAIT_H_
#define TASK_WAIT_H_

#include <cstdlib>
#include "Controller.h"
#include "Task.h"

/**
 * This is a specific implementation of a task class that waits for a given duration.
 */
class TaskWait : public Task {
    
    public:
        
                        TaskWait(Controller& controller, float duration);
        virtual         ~TaskWait();
        virtual int     run(float period);
        
    private:
        
        Controller&     controller;
        float           duration;
        float           time;
};

#endif /* TASK_WAIT_H_ */
