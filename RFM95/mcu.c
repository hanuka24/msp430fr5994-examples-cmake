#include <msp430.h>
#include <stdint.h>
#include "mcu.h"
#include <driverlib.h>

#define MSP430_CPU_CLK_HZ 1000000
/*
 * Clock System Initialization
 */
//Configure to 1 MHz
void clock_init()
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

int gpio_init()
{
    // Set all pins to output and low, to save energy
    // Configure GPIO
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

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN3);  

    //Enable P5.6 internal resistance as pull-Up resistance (Button)
    //GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN6);  

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

      // Init irq interrupts
    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN2);

    GPIO_selectInterruptEdge(GPIO_PORT_P8, GPIO_PIN3, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_enableInterrupt(GPIO_PORT_P8, GPIO_PIN3);

}


void mcu_init() {
  WDTCTL = WDTPW + WDTHOLD;   // Disable watchdog

  // Configure one FRAM waitstate as required by the device datasheet for MCLK
  // operation beyond 8MHz _before_ configuring the clock system.
  FRCTL0 = FRCTLPW | NWAITS_1;

  gpio_init();
  PMM_unlockLPM5();
  clock_init();

}

void mcu_delayms(uint32_t ms) {
  while (ms) {
    __delay_cycles(MSP430_CPU_CLK_HZ / 1000);
  	ms--;
  }
}

void mcu_delayus(uint32_t us) {
	while (us) {
    __delay_cycles(MSP430_CPU_CLK_HZ / 1000000);
		//__delay_cycles(14); //for 16MHz
		us--;
  }
}

void mcu_memcpy1(uint8_t *dst, const uint8_t *src, uint16_t size) {
    while(size--) *dst++ = *src++;
}
