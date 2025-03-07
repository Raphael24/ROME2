/*
 * EncoderCounter.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "EncoderCounter.h"

using namespace std;

/**
 * Creates and initialises the driver to read the quadrature
 * encoder counter of the STM32 microcontroller.
 * @param a the input pin for the channel A.
 * @param b the input pin for the channel B.
 */
EncoderCounter::EncoderCounter(PinName a, PinName b) {
    
    // check pins
    
    if ((a == PA_15) && (b == PB_3)) {
        
        // pinmap OK for TIM2 CH1 and CH2
        
        TIM = TIM2;
        
        // configure reset and clock control registers
        
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // manually enable port B (port A enabled by mbed library)
        
        // configure general purpose I/O registers
        
        GPIOA->MODER &= ~GPIO_MODER_MODER15;    // reset port A15
        GPIOA->MODER |= GPIO_MODER_MODER15_1;   // set alternate mode of port A15
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR15;    // reset pull-up/pull-down on port A15
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_1;   // set input as pull-down
        GPIOA->AFR[1] &= ~0xF0000000;           // reset alternate function of port A15
        GPIOA->AFR[1] |= 1 << 4*7;              // set alternate funtion 1 of port A15
        
        GPIOB->MODER &= ~GPIO_MODER_MODER3;     // reset port B3
        GPIOB->MODER |= GPIO_MODER_MODER3_1;    // set alternate mode of port B3
        GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR3;     // reset pull-up/pull-down on port B3
        GPIOB->PUPDR |= GPIO_PUPDR_PUPDR3_1;    // set input as pull-down
        GPIOB->AFR[0] &= ~(0xF << 4*3);         // reset alternate function of port B3
        GPIOB->AFR[0] |= 1 << 4*3;              // set alternate funtion 1 of port B3
        
        // configure reset and clock control registers
        
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;  //reset TIM2 controller
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
        
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // TIM2 clock enable
        
    } else if ((a == PB_4) && (b == PC_7)) {
        
        // pinmap OK for TIM3 CH1 and CH2
        
        TIM = TIM3;
        
        // configure reset and clock control registers
        
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // manually enable port B
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    // manually enable port C
        
        // configure general purpose I/O registers
        
        GPIOB->MODER &= ~GPIO_MODER_MODER4;     // reset port B4
        GPIOB->MODER |= GPIO_MODER_MODER4_1;    // set alternate mode of port B4
        GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4;     // reset pull-up/pull-down on port B4
        GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_1;    // set input as pull-down
        GPIOB->AFR[0] &= ~(0xF << 4*4);         // reset alternate function of port B4
        GPIOB->AFR[0] |= 2 << 4*4;              // set alternate funtion 2 of port B4
        
        GPIOC->MODER &= ~GPIO_MODER_MODER7;     // reset port C7
        GPIOC->MODER |= GPIO_MODER_MODER7_1;    // set alternate mode of port C7
        GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR7;     // reset pull-up/pull-down on port C7
        GPIOC->PUPDR |= GPIO_PUPDR_PUPDR7_1;    // set input as pull-down
        GPIOC->AFR[0] &= ~0xF0000000;           // reset alternate function of port C7
        GPIOC->AFR[0] |= 2 << 4*7;              // set alternate funtion 2 of port C7
        
        // configure reset and clock control registers
        
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;  //reset TIM3 controller
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
        
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     // TIM3 clock enable
        
    } else if ((a == PD_12) && (b == PD_13)) {
        
        // pinmap OK for TIM4 CH1 and CH2
        
        TIM = TIM4;
        
        // configure reset and clock control registers
        
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    // manually enable port D
        
        // configure general purpose I/O registers
        
        GPIOD->MODER &= ~GPIO_MODER_MODER12;    // reset port D12
        GPIOD->MODER |= GPIO_MODER_MODER12_1;   // set alternate mode of port D12
        GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR12;    // reset pull-up/pull-down on port D12
        GPIOD->PUPDR |= GPIO_PUPDR_PUPDR12_1;   // set input as pull-down
        GPIOD->AFR[1] &= ~(0xF << 4*4);         // reset alternate function of port D12
        GPIOD->AFR[1] |= 2 << 4*4;              // set alternate funtion 2 of port D12
        
        GPIOD->MODER &= ~GPIO_MODER_MODER13;    // reset port D13
        GPIOD->MODER |= GPIO_MODER_MODER13_1;   // set alternate mode of port D13
        GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR13;    // reset pull-up/pull-down on port D13
        GPIOD->PUPDR |= GPIO_PUPDR_PUPDR13_1;   // set input as pull-down
        GPIOD->AFR[1] &= ~(0xF << 4*5);         // reset alternate function of port D13
        GPIOD->AFR[1] |= 2 << 4*5;              // set alternate funtion 2 of port D13
        
        // configure reset and clock control registers
        
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;  //reset TIM4 controller
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;
        
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;     // TIM4 clock enable
        
    } else {
        
        printf("pinmap not found for peripheral\n");
        
        TIM = NULL;
    }
    
    // disable deep sleep for timer clocks
    
    sleep_manager_lock_deep_sleep();
    
    // configure general purpose timer 2, 3 or 4
    
    if (TIM != NULL) {
        
        TIM->CR1 = 0x0000;          // counter disable
        TIM->CR2 = 0x0000;          // reset master mode selection
        TIM->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; // counting on both TI1 & TI2 edges
        TIM->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0;
        TIM->CCMR2 = 0x0000;        // reset capture mode register 2
        TIM->CCER = TIM_CCER_CC2E | TIM_CCER_CC1E;
        TIM->CNT = 0x0000;          // reset counter value
        TIM->ARR = 0xFFFF;          // auto reload register
        TIM->CR1 = TIM_CR1_CEN;     // counter enable
    }
}

/**
 * Deletes this EncoderCounter object.
 */
EncoderCounter::~EncoderCounter() {}

/**
 * Resets the counter value to zero.
 */
void EncoderCounter::reset() {
    
    TIM->CNT = 0x0000;
}

/**
 * Resets the counter value to a given offset value.
 * @param offset the offset value to reset the counter to.
 */
void EncoderCounter::reset(short offset) {
    
    TIM->CNT = -offset;
}

/**
 * Reads the quadrature encoder counter value.
 * @return the quadrature encoder counter as a signed 16-bit integer value.
 */
short EncoderCounter::read() {
    
    return (short)(-TIM->CNT);
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
EncoderCounter::operator short() {
    
    return read();
}
