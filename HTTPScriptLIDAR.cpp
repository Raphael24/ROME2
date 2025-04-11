/*
 * HTTPScriptLIDAR.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <deque>
#include "HTTPScriptLIDAR.h"

using namespace std;

inline string int2String(int i) {
    
    char buffer[32];
    sprintf(buffer, "%d", i);
    
    return string(buffer);
}

inline string float2String(float f) {
    
    char buffer[32];
    sprintf(buffer, "%.3f", f);
    
    return string(buffer);
}

/**
 * Create and initialize this http script.
 * @param lidar a reference to the lidar to read scans from.
 */
HTTPScriptLIDAR::HTTPScriptLIDAR(LIDAR& lidar) : lidar(lidar) {}

HTTPScriptLIDAR::~HTTPScriptLIDAR() {}

/**
 * This method gets called by the http server, when an object of this class is
 * registered with the server, and the corresponding script is called
 * by an http client.
 */
string HTTPScriptLIDAR::call(vector<string> names, vector<string> values) {
    
    string response;
    
    deque<Point> scan = lidar.getScan();
    deque<Point> beacons = lidar.getBeacons();
    
    response += "  <lidar>\r\n";
    response += "    <scan>\r\n";
    response += "      <size><int>"+int2String(scan.size())+"</int></size>\r\n";
    for (unsigned short i = 0; i < scan.size(); i++) {
        response += "      <point><x><float>"+float2String(scan[i].x)+"</float></x><y><float>"+float2String(scan[i].y)+"</float></y></point>\r\n";
    }
    response += "    </scan>\r\n";
    response += "    <beacons>\r\n";
    response += "      <size><int>"+int2String(beacons.size())+"</int></size>\r\n";
    for (unsigned short i = 0; i < beacons.size(); i++) {
        response += "      <point><x><float>"+float2String(beacons[i].x)+"</float></x><y><float>"+float2String(beacons[i].y)+"</float></y></point>\r\n";
    }
    response += "    </beacons>\r\n";
    response += "  </lidar>\r\n";
    
    return response;
}
