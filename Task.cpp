/*
 * Task.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "Task.h"

using namespace std;

/**
 * Creates an abstract task object.
 */
Task::Task() {}

/**
 * Deletes the task object.
 */
Task::~Task() {}

/**
 * This method is called periodically by a task sequencer.
 * It contains the code this task has to work on.
 * @param period the period of the task sequencer, given in [s].
 * @return the status of this task, i.e. RUNNING or DONE.
 */
int Task::run(float period) {
    
    return DONE;
}
