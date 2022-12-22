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
//  Test application to perform lifetime estimation of MSP430 powered by a supercapacitor.
//  ---
//  Upon reset, and if MODE=0 (PIN1.3), the device remains in LPM0 and periodically
//  increases its timestamp and measures its operating voltage (both stored in FRAM).
//  Upon reset, and if MODE=1 (PIN1.3), the device does not log anything, but waits until
//  Button 1 is pressed or PRINT is set to high (PIN5.2), to print the timestamp and 
//  the last measured voltage value, respectively.
//  If MONITOR_TEMP is set, the temperature is measured every TEMP_INTERVAL seconds
//  using the SHT21 temperature sensor.
//***************************************************************************************

#include <driverlib.h>
#include "serial.h"
#include "si7021.h"

#define TIMER_COMPARE_VALUE 10000 //Timer IRQ freq: 100Hz (1MHz/10000))

#define USE_ADC         1
#define MONITOR_TEMP    1
#define TEMP_INTERVAL   30
#define NUM_TEMP_VALS    2000
#define TEMP_NON_PERIODIC   0


//Data stored in FRAM
uint32_t __attribute__((section(".persistent"))) time_cnt = 0;
uint16_t __attribute__((section(".persistent"))) voltage = 0;
float __attribute__((section(".persistent"))) temperature[NUM_TEMP_VALS] = {0.0};
uint32_t __attribute__((section(".persistent"))) times[NUM_TEMP_VALS] = {0};
uint16_t __attribute__((section(".persistent"))) temperature_cnt = 0;

uint8_t log_up_time = 0;
uint8_t interval_cnt = 0;
uint32_t TEMP_INTERVALS[] = {30, 10, 5, 2, 5, 50, 10}; 

/*
 * Clock System Initialization
 */
//Configure to 8 MHz
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
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
    P7OUT = 0;
    P7DIR = 0xFF;
    P8OUT = 0;
    P8DIR = 0xFF;

    PJOUT = 0;
    PJDIR = 0xFF;


    // Set P1.0 to output direction
    //GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    //Enable P1.3/5.2/5.6 internal resistance as pull-Up resistance
    //1.3 for mode selection
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN3);  
    //GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3);

    // I2C for temp sensor
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P7, GPIO_PIN1 + GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

}

Init_Timer()
{
    //Start timer in continuous mode sourced by SMCLK
    Timer_B_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_8; // Timer freq: 1 MHz
    initContParam.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    initContParam.timerClear = TIMER_B_DO_CLEAR;
    initContParam.startTimer = false;
    Timer_B_initContinuousMode(TIMER_B0_BASE, &initContParam);

	 //Initiaze compare mode
	Timer_B_clearCaptureCompareInterrupt(TIMER_B0_BASE,
		TIMER_B_CAPTURECOMPARE_REGISTER_0);

	Timer_B_initCompareModeParam initCompParam = {0};
    initCompParam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_0;
    initCompParam.compareInterruptEnable = TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
    initCompParam.compareOutputMode = TIMER_B_OUTPUTMODE_OUTBITVALUE;
    initCompParam.compareValue = TIMER_COMPARE_VALUE;
    Timer_B_initCompareMode(TIMER_B0_BASE, &initCompParam);

	Timer_B_startCounter( TIMER_B0_BASE, TIMER_B_CONTINUOUS_MODE);

    __bis_SR_register(LPM3_bits + GIE);       // Enter LPM0, enable interrupts
    __no_operation();                         // For debugger
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

void printLog()
{
    print2uart("{\"MSP430_Lifetime\":%l0,\"MSP430_MinV\":%d,\"MSP430_Temperature\":", time_cnt, voltage);
    if(temperature_cnt > 0)
    {
        for(int i = 0; i < temperature_cnt - 1; i++)
            print2uart("%l:%d.%d,", times[i], (int)(temperature[i]), ((int)(temperature[i] * 100.0 + 0.5))%100);
        print2uart("%l:%d.%d}\n", times[temperature_cnt-1], (int)(temperature[temperature_cnt-1]), ((int)(temperature[temperature_cnt-1] * 100.0 + 0.5))%100);
    }
    else
        print2uart("999}\n");
}


int main(void) {

    volatile uint32_t i;

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    Init_GPIO();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    Init_Clock();


    if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3)) //check if PIN high
    {
        uartinit(SERIAL_CLK_8MHZ);
        log_up_time = 0;
        printLog();

        //Enable P5.6 (Button) internal resistance as pull-Up resistance and P5.2 as input
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN2);

        //P1.1 Hi/Lo edge
        GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
        GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);

        //P5.6 (Button S1) IFG cleared and enabled
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN6);
        
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN2);
        GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN2);

        //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        while(1) { };   //we could go to sleep as well
    }    
    else
    { 
        log_up_time = 1;
        time_cnt = 0;
        temperature_cnt = 0;
        Init_ADC();
        // Init temp_sensor
        Init_Timer();

        while(1)
        {
            //P1OUT^=1;
            //we come here after we've been waking up from LPM3
            init_Si7021();
            r_single_Si7021(&temperature[temperature_cnt], Temperature);
            
            // // P1OUT^=2;
            
            times[temperature_cnt] = time_cnt*10; //in ms
            temperature_cnt++;
            
            __bis_SR_register(LPM3_bits | GIE);     // Enter LPM3 mode w/ interrupts enabled
        }
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
        printLog();
    
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN2);
    }
}


// Timer B0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMERB0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMERB0_VECTOR)))
#endif
void TIMERB0_ISR (void)
{
    uint16_t compVal = Timer_B_getCaptureCompareCount(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_0) + TIMER_COMPARE_VALUE;

    time_cnt += 1;

    if(USE_ADC && time_cnt > 0 && time_cnt%100 == 0)
    {
        ADC12CTL0 |= ADC12SC;               // Sampling and conversion start
    }

    //if TEMP_INTERVAL: exit lpm and read temp.
    if(MONITOR_TEMP )
    {
        #if TEMP_NON_PERIODIC
        if(time_cnt%(100*TEMP_INTERVALS[interval_cnt]) == 0){
            interval_cnt = (interval_cnt + 1) % 7;
        #else
        if(time_cnt%(100*TEMP_INTERVAL) == 0){
        #endif
        __bic_SR_register_on_exit(LPM3_bits); /* Exit Low Power Mode 3 to get I2C data from sensor */
        }
    }

	// Add Offset to CCR0 [Cont mode]
	Timer_B_setCompareValue(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_0, compVal);
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