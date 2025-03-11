/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

 //test Raphael V1
 //test Raphael V2

#include "mbed.h"
#include "IRSensor.h"
#include "EncoderCounter.h"
#include "LowpassFilter.h"
#include "Controller.h"

// Blinking rate in milliseconds
#define BLINKING_RATE     100ms
float distance_old = 0;
long int counter = 0;
float irgendwas = 1.0f;

int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led0(PD_4); 
    DigitalOut led1(PD_3); 
    DigitalOut led2(PD_6); 
    DigitalOut led3(PD_2); 
    DigitalOut led4(PD_7); 
    DigitalOut led5(PD_5); 

    // initialise IR-Sensors
    AnalogIn distance(PA_0); // Kreieren der Ein- und Ausgangsobjekte
    DigitalOut enable(PG_1);
    DigitalOut bit0(PF_0);
    DigitalOut bit1(PF_1);
    DigitalOut bit2(PF_2);


    IRSensor hinten(distance, bit0, bit1, bit2, 0);
    IRSensor hinten_l(distance, bit0, bit1, bit2, 1);
    IRSensor vorne_l(distance, bit0, bit1, bit2, 2);
    IRSensor vorne(distance, bit0, bit1, bit2, 3);
    IRSensor vorne_r(distance, bit0, bit1, bit2, 4);
    IRSensor hinten_r(distance, bit0, bit1, bit2, 5);

    enable = 1; // schaltet die Sensoren ein


    // Motoren Initialisieren
    EncoderCounter counterLeft(PD_12, PD_13);
    EncoderCounter counterRight(PB_4, PC_7);

    DigitalOut enableMotorDriver(PG_0);  
    DigitalIn motorDriverFault(PD_1); 
    DigitalIn motorDriverWarning(PD_0); 

    PwmOut pwmLeft(PF_9); 
    PwmOut pwmRight(PF_8); 
    enableMotorDriver = 1;     // Schaltet den Leistungstreiber ein 

    Controller controller(pwmLeft, pwmRight, counterLeft, counterRight);

    //controller.setDesiredSpeedLeft(100.0);  // Drehzahl in [rpm] 
    //controller.setDesiredSpeedRight(100.0); 

    controller.setTranslationalVelocity(0.0f);
    controller.setRotationalVelocity(1.0f);


    DigitalIn Button(BUTTON1);
    while (true) {

        if(counter >= 20){

            printf("8==================================================D");
            irgendwas = irgendwas+0.5;
            controller.setRotationalVelocity(irgendwas);
            counter=0;
        }


        counter++;


        led0 = hinten < 0.2f; 
        led1 = hinten_l < 0.2f; 
        led2 = vorne_l < 0.2f; 
        led3 = vorne < 0.2f; 
        led4 = vorne_r < 0.2f; 
        led5 = hinten_r < 0.2f; 
    
        float distance0 = hinten.read();
        float distance1 = hinten_l.read();
        float distance2 = vorne_l.read();
        float distance3 = vorne_r.read();
        float distance4 = vorne.read();
        float distance5 = hinten_r.read();

        //printf("distance = %d [mm]\r\n", (int)(1000.0f*distance0));
        //printf("distance = %d [mm]\r\n", (int)(1000.0f*distance1));
        //printf("distance = %d [mm]\r\n", (int)(1000.0f*distance2));
        //printf("distance = %d [mm]\r\n", (int)(1000.0f*distance3));
        //printf("distance = %d [mm]\r\n", (int)(1000.0f*distance4));
        //printf("distance = %d [mm]\r\n", (int)(1000.0f*distance5));

        printf("actual speed (left/right): %.3f / %.3f [rpm]\r\n", controller.getActualRotationalVelocity(), controller.getActualTranslationalVelocity());
        printf("counter, irgendwas %d , %.3f [rpm]\r\n",counter,irgendwas);

        ThisThread::sleep_for(100ms);


    }
}