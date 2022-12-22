/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//***************************************************************************************
  Periodically reads and prints temperature/humidity readings from si7021 sensor
  Sensor is connected using I2C to pins 7.0 and 7.1.
//***************************************************************************************

#include <driverlib.h>
#include <stdio.h>
#include "serial.h"
#include "si7021.h"
og_up_time = 0;

/*
 * Clock System Initialization
 */
//Configure to 8 MHz
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768, 0);
    //Set ACLK=LFXT
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //Start XT1 with no time out
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
}

int Init_GPIO()
{
    // Set all pins to output and low, to save energy
    // Configure GPIO
    P1OUT = 0;
    P1DIR = 0xFF;
    P1OUT = 0;
    P1DIR = 0xFF;
    P2OUT = 0;
    P2DIR = 0xFF;
    P3OUT = 0;
    P3DIR = 0xFF;
    P4OUT = 0;
    P4DIR = 0xFF;
    P5OUT = 0;
    P5DIR = 0xFF;
    P6OUT = 0;
    P6DIR = 0xFF;
    //P7OUT = 0;
    //P7DIR = 0xFF;
    P8OUT = 0;
    P8DIR = 0xFF;

    PJOUT = 0;
    PJDIR = 0xFF;


    // Set P1.0 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);


    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    
    // Set P7.0 and P7.1 as Primary Module Function Input for I2C.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P7, GPIO_PIN1 + GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);
}

int main(void) {

    uint32_t i;
    float hum, temp = 0;

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    Init_GPIO();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    Init_Clock();

    uartinit(SERIAL_CLK_1MHZ);
    print2uart("Start");


    init_Si7021();
   
   // print2uart("FW Rev: %d\n", r_firmware_rev_Si7021());

    while(1)
    {
        // Toggle P1.0 output
        GPIO_toggleOutputOnPin(
            GPIO_PORT_P1,
			GPIO_PIN0
			);

    // Toggle P1.0 output
        GPIO_toggleOutputOnPin(
            GPIO_PORT_P1,
			GPIO_PIN1
			);

        r_both_Si7021(&hum, &temp);

        print2uart("Humidity: %d; Temp: %d\n", (int)hum, (int)temp);

        // Delay
        for(i=50000; i>0; i--);
    }
}
