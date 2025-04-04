/*
 * IRSensor.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "IRSensor.h"

using namespace std;

/**
 * Creates and initialises the driver to read the distance sensors.
 * @param distance the analog input to read a distance value from.
 * @param bit0 a digital output to control the multiplexer.
 * @param bit1 a digital output to control the multiplexer.
 * @param bit2 a digital output to control the multiplexer.
 * @param number the number of the sensor. This value must be between 0 and 5.
 */
IRSensor::IRSensor(AnalogIn& distance, DigitalOut& bit0, DigitalOut& bit1, DigitalOut& bit2, int number) : distance(distance), bit0(bit0), bit1(bit1), bit2(bit2) {
    
    this->number = number;
}

/**
 * Deletes this IRSensor object and releases all allocated resources.
 */
IRSensor::~IRSensor() {}

/**
 * This method reads from the distance sensor.
 * @return a distance value, given in [m].
 */
float IRSensor::read() {
    
    bit0 = (number >> 0) & 1;
    bit1 = (number >> 1) & 1;
    bit2 = (number >> 2) & 1;
    
    float d = 0.09f/(distance+0.001f)-0.03f;  // calculate the distance in [m]
    
    return d;
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
IRSensor::operator float() {
    
    return read();
}
