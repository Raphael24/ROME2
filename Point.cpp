/*
 * Point.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "Point.h"

using namespace std;

/**
 * Creates a Point object.
 */
Point::Point() {
    
    x = 0.0f;
    y = 0.0f;
    r = 0.0f;
    alpha = 0.0f;
}

/**
 * Creates a Point object from given polar coordinates.
 * @param r the radius of a point.
 * @param alpha the angle of a point.
 */
Point::Point(float r, float alpha) {
    
    x = r*cos(alpha);
    y = r*sin(alpha);
    
    this->r = r;
    this->alpha = alpha;
}

/**
 * Deletes this object.
 */
Point::~Point() {}

/**
 * Calculates the distance of this point from the origin.
 */
float Point::distance() {
    
    return sqrt(x*x+y*y);
}

/**
 * Calculates the distance between this point and a given point.
 * @param point another point to calculate the distance to.
 */
float Point::distance(Point& point) {
    
    return sqrt((x-point.x)*(x-point.x)+(y-point.y)*(y-point.y));
}

/**
 * Calculates the manhattan distance of this point from the origin.
 * This calculation is only an approximation, but a lot faster to compute.
 */
float Point::manhattanDistance() {
    
    return fabs(x)+fabs(y);
}

/**
 * Calculates the manhattan distance between this point and a given point.
 * This calculation is only an approximation, but a lot faster to compute.
 * @param point another point to calculate the distance to.
 */
float Point::manhattanDistance(Point& point) {
    
    return fabs(x-point.x)+fabs(y-point.y);
}
