/*
 * Task.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef TASK_H_
#define TASK_H_

#include <cstdlib>

/**
 * This is an abstract task class with a method that
 * is called periodically by a task sequencer.
 */
class Task {
    
    public:
        
        static const int    FAULT = -1;     /**< Task return value. */
        static const int    RUNNING = 0;    /**< Task return value. */
        static const int    DONE = 1;       /**< Task return value. */
        
                        Task();
        virtual         ~Task();
        virtual int     run(float period);
};

#endif /* TASK_H_ */
