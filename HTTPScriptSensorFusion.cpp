/*
 * HTTPScriptSensorFusion.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "HTTPScriptSensorFusion.h"

using namespace std;

inline string float2String(float f) {
    
    char buffer[32];
    sprintf(buffer, "%.3f", f);
    
    return string(buffer);
}

/**
 * Create and initialize this http script.
 * @param sensorFusion a reference to the sensorFusion object to read data from.
 */
HTTPScriptSensorFusion::HTTPScriptSensorFusion(SensorFusion& sensorFusion) : sensorFusion(sensorFusion) {}

HTTPScriptSensorFusion::~HTTPScriptSensorFusion() {}

/**
 * This method gets called by the http server, when an object of this class is
 * registered with the server, and the corresponding script is called
 * by an http client.
 */
string HTTPScriptSensorFusion::call(vector<string> names, vector<string> values) {
    
    string response;
    
    response += "  <tiltAngle>\r\n";
    response += "    <a><float>"+float2String(sensorFusion.readTiltAngleA())+"</float></a>\r\n";
    response += "    <g><float>"+float2String(sensorFusion.readTiltAngleG())+"</float></g>\r\n";
    response += "    <k><float>"+float2String(sensorFusion.readTiltAngleK())+"</float></k>\r\n";
    response += "    <c><float>"+float2String(sensorFusion.readTiltAngleC())+"</float></c>\r\n";
    response += "  </tiltAngle>\r\n";
    
    return response;
}
