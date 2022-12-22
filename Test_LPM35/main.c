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
//  Test application using LPM3.5 to perform lifetime estimation of MSP430 powered by a supercapacitor.
//  ---
//  Upon reset, and if MODE=0 (PIN1.3), the device remains in LPM3.5 and if LOG=0 (PIN1.4),
//  it periodically increases its timestamp and measures its operating voltage (both stored in FRAM).
//  In addition, if LOG_TEMP is set, the temperature is read from SHT21 sensor using I2C with an interval
//  of LOG_INTERVAL=30 minutes. The retrieved data is also stored in FRAM.
//  Upon reset, and if MODE=1 (PIN1.4), the device does not log anything, but waits until
//  Button 1 is pressed or PRINT is set to high (PIN5.2), to print the timestamp, temperature measurements 
//  and the last measured voltage value, respectively.
//  Note: Voltage is measured only if USE_ADC is set. This increases power consumption to ~20uA (~1uA otherwise).
//***************************************************************************************

#include <driverlib.h>
#include "serial.h"
#include "si7021.h"

#define USE_ADC 0
#define LOG_TEMP 1

#define LOG_INTERVAL  1
#define NUM_TEMP_VALS 800

#define MODCOUNT    LOG_INTERVAL * 60 * 32 - 1
float dummy;
//Data stored in FRAM
uint32_t __attribute__((section(".persistent"))) data = 0;
uint32_t __attribute__((section(".persistent"))) log_cnt = 0;
uint16_t __attribute__((section(".persistent"))) voltage = 0;
float __attribute__((section(".persistent"))) temperature[NUM_TEMP_VALS] = {0.0};
uint32_t __attribute__((section(".persistent"))) times[NUM_TEMP_VALS] = {0};
uint16_t __attribute__((section(".persistent"))) temperature_cnt = 0;

/*
 * Clock System Initialization
 */
//Configure to 8 MHz
void Init_Clock()
{
    // Set DCO frequency to 1 MHz
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
   // P7OUT = 0;
   // P7DIR = 0xFF;
    P8OUT = 0;
    P8DIR = 0xFF;

    PJOUT = 0;
    PJDIR = 0xFF;


    // Set P1.0 to output direction
  // GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
 //  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);

    //Enable P5.6 internal resistance as pull-Up resistance
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN3);  
  //  GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN4);  
  //  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3);
  //  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN4);

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    // I2C for temp sensor
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P7, GPIO_PIN1 + GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC()
{
       // Initialize the shared reference module
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
    while(REFCTL0 & REFGENBUSY);            // If ref generator busy, WAIT
    REFCTL0 |= REFVSEL_0 + REFON;           // Enable internal 1.2V reference

    // Initialize ADC12_A
    ADC12CTL0 &= ~ADC12ENC;                 // Disable ADC12
    ADC12CTL0 = ADC12SHT0_8 + ADC12ON;      // Set sample time
    ADC12CTL1 = ADC12SHP;                   // Enable sample timer
    ADC12CTL3 = ADC12BATMAP;                 
    ADC12MCTL0 = ADC12VRSEL_1 + ADC12INCH_31 ; // ADC input ch A30 => temp sense
    ADC12IER0 = 0x001;                      // ADC_IFG upon conv result-ADCMEMO

    while(!(REFCTL0 & REFGENRDY));          // Wait for reference generator
                                            // to settle
    ADC12CTL0 |= ADC12ENC;

}

void Init_RTC(char initial)
{
    // Configure RTC_C
    RTCCTL0_H = RTCKEY_H;                   // Unlock RTC
    RTCCTL0_L = RTCTEVIE_L | RTCRDYIE_L;    // enable RTC read ready interrupt
                                            // enable RTC time event interrupt

    RTCCTL13 = RTCBCD | RTCHOLD | RTCMODE;  // RTC enable, BCD mode, RTC hold
    if (initial)
    {
        RTCYEAR = 0x2010;                       // Year = 0x2010
        RTCMON = 0x4;                           // Month = 0x04 = April
        RTCDAY = 0x05;                          // Day = 0x05 = 5th
        RTCDOW = 0x01;                          // Day of week = 0x01 = Monday
        RTCHOUR = 0x10;                         // Hour = 0x10
        RTCMIN = 0x32;                          // Minute = 0x32
        RTCSEC = 0x45;                          // Seconds = 0x45
    }


    RTCCTL13 &= ~(RTCHOLD);                 // Start RTC

    __bis_SR_register(LPM3_bits | GIE);     // Enter LPM3 mode w/ interrupts enabled
    __no_operation();


}


int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    Init_GPIO();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    Init_Clock();

    if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3)) //check if PIN high
    {
        uartinit(SERIAL_CLK_1MHZ);

        //Enable P5.6 internal resistance as pull-Up resistance
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN5);
        
        //GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN2);
        GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN2);

        //P1.1 Hi/Lo edge
        GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
        GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
        
        GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);

        //P5.6 (Button S1) IFG cleared and enabled
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN5);
        GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN5);
        
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN2);
        GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN2);

        __enable_interrupt();

        //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        while(1) { };   //we could go to sleep as well
    }    
    else
    {                   
        temperature_cnt = 0;
        data = 0;
        log_cnt = 0;

        Init_RTC(true);
        while(1)
        {
            //P1OUT^=2;
            //we come here after we've been waking up from LPM3
            init_Si7021();
            r_single_Si7021(&temperature[temperature_cnt], Temperature);
            
            // P1OUT^=2;
            
            times[temperature_cnt] = data;
            temperature_cnt++;
            
            __bis_SR_register(LPM3_bits | GIE);     // Enter LPM3 mode w/ interrupts enabled
        }
        //should never get here - we are in LPM3 from now on
    }    
}



//******************************************************************************
//
//This is the PORT1_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT5_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT5_VECTOR)))
#endif
void Port_5 (void)
{
    
    if ( P5IFG & GPIO_PIN6 || P5IFG & GPIO_PIN2)
    {

        print2uart("{\"MSP430_Lifetime\":%l,\"MSP430_MinV\":%d,\"MSP430_Temperature\":", data, voltage);
        if(temperature_cnt > 0)
        {
            for(int i = 0; i < temperature_cnt - 1; i++)
                print2uart("%l:%d.%d,", times[i], (int)(temperature[i]), ((int)(temperature[i] * 100.0 + 0.5))%100);
            print2uart("%l:%d.%d}\n", times[temperature_cnt-1], (int)(temperature[temperature_cnt-1]), ((int)(temperature[temperature_cnt-1] * 100.0 + 0.5))%100);
        }
        else
            print2uart("999}\n");

    
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN2);
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_C_VECTOR
__interrupt void RTC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_C_VECTOR))) RTC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(RTCIV, RTCIV__RT1PSIFG))
    {
        case RTCIV__NONE:      break;       // No interrupts
        case RTCIV__RTCOFIFG:  break;       // RTCOFIFG
        case RTCIV__RTCRDYIFG:              // RTCRDYIFG //fires every second
            //if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4)==0)
            //{
                data++;
                #if USE_ADC
                Init_ADC(); 
                ADC12CTL0 |= ADC12SC;               // Sampling and conversion start
                #endif
           // }
            //else
            //{
           //     temperature_cnt = 0;
            ///    data = 0;
            //    log_cnt = 0;
           // }
                
            break;
        case RTCIV__RTCTEVIFG:        // RTCEVIFG // Interrupts every minute
            if(log_cnt++ == LOG_INTERVAL)
            {
                
                log_cnt = 0;
                __bic_SR_register_on_exit(LPM3_bits); /* Exit Low Power Mode 3 to get I2C data from sensor */
            }          
            break;
        case RTCIV__RTCAIFG:   break;       // RTCAIFG
        case RTCIV__RT0PSIFG:  break;       // RT0PSIFG
        case RTCIV__RT1PSIFG:  break;       // RT1PSIFG
        default: break;
    }
}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_B_VECTOR
__interrupt void ADC12ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_B_VECTOR))) ADC12ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADC12IV, ADC12IV__ADC12RDYIFG))
    {
        case ADC12IV__NONE:        break;   // Vector  0:  No interrupt
        case ADC12IV__ADC12OVIFG:  break;   // Vector  2:  ADC12MEMx Overflow
        case ADC12IV__ADC12TOVIFG: break;   // Vector  4:  Conversion time overflow
        case ADC12IV__ADC12HIIFG:  break;   // Vector  6:  ADC12BHI
        case ADC12IV__ADC12LOIFG:  break;   // Vector  8:  ADC12BLO
        case ADC12IV__ADC12INIFG:  break;   // Vector 10:  ADC12BIN
        case ADC12IV__ADC12IFG0:            // Vector 12:  ADC12MEM0 Interrupt
            voltage = ADC12MEM0;               // Move results, IFG is cleared
            // Enter LPM3.5 mode with interrupts enabled. Note that this
            //operation does not return. The LPM3.5 will exit through a
            //RESETevent, resulting in a re-start of the code.
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == 0)
            {
                __bis_SR_register_on_exit(LPM3_bits | GIE);
                __no_operation();
            }
            break;
        case ADC12IV__ADC12IFG1:   break;   // Vector 14:  ADC12MEM1
        case ADC12IV__ADC12IFG2:   break;   // Vector 16:  ADC12MEM2
        case ADC12IV__ADC12IFG3:   break;   // Vector 18:  ADC12MEM3
        case ADC12IV__ADC12IFG4:   break;   // Vector 20:  ADC12MEM4
        case ADC12IV__ADC12IFG5:   break;   // Vector 22:  ADC12MEM5
        case ADC12IV__ADC12IFG6:   break;   // Vector 24:  ADC12MEM6
        case ADC12IV__ADC12IFG7:   break;   // Vector 26:  ADC12MEM7
        case ADC12IV__ADC12IFG8:   break;   // Vector 28:  ADC12MEM8
        case ADC12IV__ADC12IFG9:   break;   // Vector 30:  ADC12MEM9
        case ADC12IV__ADC12IFG10:  break;   // Vector 32:  ADC12MEM10
        case ADC12IV__ADC12IFG11:  break;   // Vector 34:  ADC12MEM11
        case ADC12IV__ADC12IFG12:  break;   // Vector 36:  ADC12MEM12
        case ADC12IV__ADC12IFG13:  break;   // Vector 38:  ADC12MEM13
        case ADC12IV__ADC12IFG14:  break;   // Vector 40:  ADC12MEM14
        case ADC12IV__ADC12IFG15:  break;   // Vector 42:  ADC12MEM15
        case ADC12IV__ADC12IFG16:  break;   // Vector 44:  ADC12MEM16
        case ADC12IV__ADC12IFG17:  break;   // Vector 46:  ADC12MEM17
        case ADC12IV__ADC12IFG18:  break;   // Vector 48:  ADC12MEM18
        case ADC12IV__ADC12IFG19:  break;   // Vector 50:  ADC12MEM19
        case ADC12IV__ADC12IFG20:  break;   // Vector 52:  ADC12MEM20
        case ADC12IV__ADC12IFG21:  break;   // Vector 54:  ADC12MEM21
        case ADC12IV__ADC12IFG22:  break;   // Vector 56:  ADC12MEM22
        case ADC12IV__ADC12IFG23:  break;   // Vector 58:  ADC12MEM23
        case ADC12IV__ADC12IFG24:  break;   // Vector 60:  ADC12MEM24
        case ADC12IV__ADC12IFG25:  break;   // Vector 62:  ADC12MEM25
        case ADC12IV__ADC12IFG26:  break;   // Vector 64:  ADC12MEM26
        case ADC12IV__ADC12IFG27:  break;   // Vector 66:  ADC12MEM27
        case ADC12IV__ADC12IFG28:  break;   // Vector 68:  ADC12MEM28
        case ADC12IV__ADC12IFG29:  break;   // Vector 70:  ADC12MEM29
        case ADC12IV__ADC12IFG30:  break;   // Vector 72:  ADC12MEM30
        case ADC12IV__ADC12IFG31:  break;   // Vector 74:  ADC12MEM31
        case ADC12IV__ADC12RDYIFG: break;   // Vector 76:  ADC12RDY
        default: break;
    }
}