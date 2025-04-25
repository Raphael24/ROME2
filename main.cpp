/*
 * Main.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <stdio.h>
#include <mbed.h>
#include "IMU.h"
#include "SensorFusion.h"
#include "HTTPServer.h"
#include "HTTPScriptSensorFusion.h"

int main() {
    
    // create miscellaneous periphery objects
    
    DigitalIn button(BUTTON1);
    DigitalOut led(LED1);
    
    DigitalOut led0(PD_4);
    DigitalOut led1(PD_3);
    DigitalOut led2(PD_6);
    DigitalOut led3(PD_2);
    DigitalOut led4(PD_7);
    DigitalOut led5(PD_5);
    
    // create inertial measurement unit object
    
    SPI spi(PC_12, PC_11, PC_10);
    DigitalOut csAG(PC_8);
    DigitalOut csM(PC_9);
    
    IMU imu(spi, csAG, csM);
    
    SensorFusion sensorFusion(imu);
    
    // create ethernet interface and webserver
    
    DigitalOut enableRouter(PB_15);
    enableRouter = 1;
    
    EthernetInterface* ethernet = new EthernetInterface();
    ethernet->set_network("192.168.0.10", "255.255.255.0", "192.168.0.1"); // configure IP address, netmask and gateway address
    ethernet->connect();
    
    HTTPServer* httpServer = new HTTPServer(*ethernet);
    httpServer->add("sensorfusion", new HTTPScriptSensorFusion(sensorFusion));
    
    while (true) {
        
        led = !led;
        
        ThisThread::sleep_for(250ms);
    }
}
