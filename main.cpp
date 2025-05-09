/*
 * Main.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "Controller.h"
#include "EncoderCounter.h"
#include "HTTPScriptLIDAR.h"
#include "HTTPServer.h"
#include "IMU.h"
#include "IRSensor.h"
#include "LIDAR.h"
#include "Point.h"
#include "StateMachine.h"
#include <mbed.h>
#include <stdio.h>

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

  // create IR sensor objects

  AnalogIn distance(PA_0);
  DigitalOut enable(PG_1);
  DigitalOut bit0(PF_0);
  DigitalOut bit1(PF_1);
  DigitalOut bit2(PF_2);

  IRSensor irSensor0(distance, bit0, bit1, bit2, 0);
  IRSensor irSensor1(distance, bit0, bit1, bit2, 1);
  IRSensor irSensor2(distance, bit0, bit1, bit2, 2);
  IRSensor irSensor3(distance, bit0, bit1, bit2, 3);
  IRSensor irSensor4(distance, bit0, bit1, bit2, 4);
  IRSensor irSensor5(distance, bit0, bit1, bit2, 5);

  enable = 1;

  // create motor control objects

  DigitalOut enableMotorDriver(PG_0);
  DigitalIn motorDriverFault(PD_1);
  DigitalIn motorDriverWarning(PD_0);

  PwmOut pwmLeft(PF_9);
  PwmOut pwmRight(PF_8);

  EncoderCounter counterLeft(PD_12, PD_13);
  EncoderCounter counterRight(PB_4, PC_7);

  // create inertial measurement unit object

  SPI spi(PC_12, PC_11, PC_10);
  DigitalOut csAG(PC_8);
  DigitalOut csM(PC_9);

  IMU imu(spi, csAG, csM);

  // create LIDAR device driver

  PwmOut pwm(PE_9);
  pwm.period(0.00005f);
  pwm.write(0.5f);

  ThisThread::sleep_for(500ms);

  UnbufferedSerial *serial = new UnbufferedSerial(PG_14, PG_9);
  LIDAR *lidar = new LIDAR(*serial);

  // create robot controller objects

  Controller controller(pwmLeft, pwmRight, counterLeft, counterRight);
  StateMachine stateMachine(controller, enableMotorDriver, led0, led1, led2,
                            led3, led4, led5, button, irSensor0, irSensor1,
                            irSensor2, irSensor3, irSensor4, irSensor5);

  // create ethernet interface and webserver

  DigitalOut enableRouter(PB_15);
  enableRouter = 1;

  EthernetInterface *ethernet = new EthernetInterface();
  ethernet->set_network(
      "192.168.0.10", "255.255.255.0",
      "192.168.0.1"); // configure IP address, netmask and gateway address
  ethernet->connect();

  HTTPServer *httpServer = new HTTPServer(*ethernet);
  httpServer->add("lidar", new HTTPScriptLIDAR(*lidar));

  // Definiere feste Landmarken
  Point rohre[] = {Point(0.0f, 0.5f), Point(2.0f, 0.5f), Point(4.0f, 0.5f),
                   Point(6.0f, 0.5f)};

  // List of points
  deque<Point> points_mesuared;

  while (true) {

    led = !led;

    ThisThread::sleep_for(100ms);

    points_mesuared = lidar->getBeacons();
    float min_dist = 0;

    for (int i = 0; i < points_mesuared.size(); i++) {
      // Aktuelle, geschätzte Lagekoordinaten des Roboters
      float x = controller.getX();
      float y = controller.getY();
      float alpha = controller.getAlpha();

      // Globale koordinaten berechnen
      float x_glob = cos(alpha) * points_mesuared[i].x -
                     sin(alpha) * points_mesuared[i].y + x;
      float y_glob = sin(alpha) * points_mesuared[i].x +
                     cos(alpha) * points_mesuared[i].y + y;
      Point act_point(x_glob, y_glob);


      if (rohre[0].distance(act_point) < 0.4f) {
        controller.correctPoseWithBeacon(rohre[0], act_point);

      } else if (rohre[1].distance(act_point) < 0.4f) {
        controller.correctPoseWithBeacon(rohre[1], act_point);

      } else if (rohre[2].distance(act_point) < 0.4f) {
          controller.correctPoseWithBeacon(rohre[2], act_point);

      } else if (rohre[3].distance(act_point) < 0.4f) {
        controller.correctPoseWithBeacon(rohre[3], act_point);

      } else{
          // no match
      }
    }
  }
}
