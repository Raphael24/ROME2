/*
 * IRSensor.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include <cstdlib>
#include <mbed.h>

/**
 * This class implements a driver to read the distance sensors
 * of the ROME2 mobile robot.
 */
class IRSensor {

    public:
        
                IRSensor(AnalogIn& distance, DigitalOut& bit0, DigitalOut& bit1, DigitalOut& bit2, int number);
        virtual ~IRSensor();
        float   read();
                operator float();
        
    private:
        
        AnalogIn&   distance;
        DigitalOut& bit0;
        DigitalOut& bit1;
        DigitalOut& bit2;
        int         number;
};

#endif /* IR_SENSOR_H_ */
