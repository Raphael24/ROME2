/*
 * Main.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

 // Test Commit

#include <stdio.h>
#include <mbed.h>
#include "HTTPServer.h"
#include "HTTPScriptLIDAR.h"

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
    
    // create LIDAR device driver
    
    PwmOut pwm(PE_9);
    pwm.period(0.00005f);
    pwm.write(0.5f);
    
    ThisThread::sleep_for(500ms);
    
    UnbufferedSerial* serial = new UnbufferedSerial(PG_14, PG_9);
    LIDAR* lidar = new LIDAR(*serial);
    
    // create ethernet interface and webserver
    
    DigitalOut enableRouter(PB_15);
    enableRouter = 1;
    
    EthernetInterface* ethernet = new EthernetInterface();
    ethernet->set_network("192.168.0.10", "255.255.255.0", "192.168.0.1"); // configure IP address, netmask and gateway address
    ethernet->connect();
    
    HTTPServer* httpServer = new HTTPServer(*ethernet);
    httpServer->add("lidar", new HTTPScriptLIDAR(*lidar));
    
    while (true) {
        
        led = !led;
        
        ThisThread::sleep_for(250ms);
    }
}
