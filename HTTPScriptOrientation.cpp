/*
 * HTTPScriptOrientation.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "HTTPScriptOrientation.h"

using namespace std;

inline string float2String(float f) {
    
    char buffer[32];
    sprintf(buffer, "%.3f", f);
    
    return string(buffer);
}

/**
 * Create and initialize this http script.
 * @param controller a reference to the controller to read data from.
 * @param imu a reference to the imu to read data from.
 */
HTTPScriptOrientation::HTTPScriptOrientation(Controller& controller, IMU& imu) : controller(controller), imu(imu) {}

HTTPScriptOrientation::~HTTPScriptOrientation() {}

/**
 * This method gets called by the http server, when an object of this class is
 * registered with the server, and the corresponding script is called
 * by an http client.
 */
string HTTPScriptOrientation::call(vector<string> names, vector<string> values) {
    
    string response;
    
    response += "  <controller>\r\n";
    response += "    <alpha><float>"+float2String(controller.getAlpha())+"</float></alpha>\r\n";
    response += "  </controller>\r\n";
    response += "  <imu>\r\n";
    response += "    <heading><float>"+float2String(imu.readHeading())+"</float></heading>\r\n";
    response += "  </imu>\r\n";
    
    return response;
}
