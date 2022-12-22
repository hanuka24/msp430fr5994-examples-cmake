/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
//******************************************************************************
//  Timer_B, Toggle P1.0, CCR0 Cont. Mode ISR, DCO SMCLK
//
//  Description: Toggle P1.0 using software and TB_0 ISR. Toggles every
//  50000 SMCLK cycles. SMCLK provides clock source for TBCLK.
//  During the TB_0 ISR, P1.0 is toggled and 50000 clock cycles are added to
//  CCR0. TB_0 ISR is triggered every 50000 cycles. CPU is normally off and
//  used only during TB_ISR.
//  ACLK = n/a, MCLK = SMCLK = TBCLK = default DCO ~1.045MHz
//
//  Tested On: MSP430FR5969
//         ---------------
//     /|\|               |
//      | |               |
//      --|RST            |
//        |               |
//        |           P1.0|-->LED
//
//******************************************************************************
#include "driverlib.h"
#include "serial.h"

#define SMCLK_115200
#define SMCLK_9600
#define ACLK_9600

#define COMPARE_VALUE 50000

//TODOOOO: Adjust TIMER to new clock settings!!!

//Configure to 16 MHz
void CLOCK_Init()
{
     // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz
    // Set SMCLK = MCLK = DCO, ACLK = LFXTCLK (VLOCLK if unavailable)
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;   // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz
    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1 for 16MHz operation

    CSCTL4 &= ~LFXTOFF;
    do
    {
    CSCTL5 &= ~LFXTOFFG;                      // Clear XT1 fault flag
    SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

    CSCTL0_H = 0;                             // Lock CS registerss
}

void GPIO_Init()
{
    /* Terminating all remaining pins to minimize power consumption. This is
        done by register accesses for simplicity and to minimize branching API
        calls */
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN_ALL16);

    /* Configuring LFXTOUT and LFXTIN for XTAL operation*/
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 | GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring UART pins */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1,  GPIO_SECONDARY_MODULE_FUNCTION);
}

void main(void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    GPIO_Init();

    PMM_unlockLPM5();

    CLOCK_Init();

    uartinit();

    GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);

    print2uart("Init done");

	//Start timer in continuous mode sourced by SMCLK
    Timer_B_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
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
    initCompParam.compareValue = COMPARE_VALUE;
    Timer_B_initCompareMode(TIMER_B0_BASE, &initCompParam);

	Timer_B_startCounter( TIMER_B0_BASE,
		TIMER_B_CONTINUOUS_MODE
		);

    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts
    __no_operation();                         // For debugger
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
    uint16_t compVal = Timer_B_getCaptureCompareCount(TIMER_B0_BASE,
    		TIMER_B_CAPTURECOMPARE_REGISTER_0)
    		+ COMPARE_VALUE;

    GPIO_toggleOutputOnPin(
            GPIO_PORT_P1,
            GPIO_PIN0
            );

	// Add Offset to CCR0 [Cont mode]
	Timer_B_setCompareValue(TIMER_B0_BASE,
	        TIMER_B_CAPTURECOMPARE_REGISTER_0,
	        compVal
	        );
}

