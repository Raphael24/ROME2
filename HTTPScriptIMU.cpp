/*
 * HTTPScriptIMU.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "HTTPScriptIMU.h"

using namespace std;

inline string float2String(float f) {
    
    char buffer[32];
    sprintf(buffer, "%.3f", f);
    
    return string(buffer);
}

/**
 * Create and initialize this http script.
 * @param imu a reference to the imu to read data from.
 */
HTTPScriptIMU::HTTPScriptIMU(IMU& imu) : imu(imu) {}

HTTPScriptIMU::~HTTPScriptIMU() {}

/**
 * This method gets called by the http server, when an object of this class is
 * registered with the server, and the corresponding script is called
 * by an http client.
 */
string HTTPScriptIMU::call(vector<string> names, vector<string> values) {
    
    string response;
    
    response += "  <imu>\r\n";
    response += "    <acceleration>\r\n";
    response += "      <x><float>"+float2String(imu.readAccelerationX())+"</float></x>\r\n";
    response += "      <y><float>"+float2String(imu.readAccelerationY())+"</float></y>\r\n";
    response += "      <z><float>"+float2String(imu.readAccelerationZ())+"</float></z>\r\n";
    response += "    </acceleration>\r\n";
    response += "    <gyro>\r\n";
    response += "      <x><float>"+float2String(imu.readGyroX())+"</float></x>\r\n";
    response += "      <y><float>"+float2String(imu.readGyroY())+"</float></y>\r\n";
    response += "      <z><float>"+float2String(imu.readGyroZ())+"</float></z>\r\n";
    response += "    </gyro>\r\n";
    response += "    <magnetometer>\r\n";
    response += "      <x><float>"+float2String(imu.readMagnetometerX())+"</float></x>\r\n";
    response += "      <y><float>"+float2String(imu.readMagnetometerY())+"</float></y>\r\n";
    response += "      <z><float>"+float2String(imu.readMagnetometerZ())+"</float></z>\r\n";
    response += "    </magnetometer>\r\n";
    response += "  </imu>\r\n";
    
    return response;
}
