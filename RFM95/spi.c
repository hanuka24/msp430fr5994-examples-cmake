#include <msp430.h>
#include <stdint.h>
#include "spi.h"
#include "driverlib.h"

volatile uint8_t spi_buf = 0;

#define SCLK    BIT2  //PORT5
#define SDI     BIT0  //PORT5
#define SDO     BIT1  //PORT5
#define CS      BIT3  //PORT5

//#define INT1    BIT4
//#define INT2    BIT3

void spi_sleep()
{
  UCB1CTLW0 = UCSWRST;
  GPIO_setAsOutputPin(GPIO_PORT_P1, SDI);
  GPIO_setAsOutputPin(GPIO_PORT_P1, SCLK);

  GPIO_setOutputLowOnPin(GPIO_PORT_P1, SDI);
  GPIO_setOutputLowOnPin(GPIO_PORT_P1, SCLK);

  GPIO_setAsInputPin(GPIO_PORT_P1, SDO);

  P5OUT &= ~CS;// | CS2;                           // P2.0 CS (chip select) to low
}

void spi_init() {
  UCB1CTLW0 = UCSWRST;
  UCB1CTLW0 |= UCSYNC  + UCMSB + UCMST + UCCKPL;   //UCMSB + UCMST + UCSYNC; // 3-pin, 8-bit SPI master
  UCB1CTLW0 |= UCSSEL_2;                         // SMCLK
  UCB1BR0 = 0x01;                               // Frequency CPU / 2 (16Mhz / 2 = 8 Mhz SPI)
  UCB1BR1 = 0;


  /* USCI_B1 SCLK, MOSI, and MISO pin */
  P5SEL1 &= ~(SDI | SDO | SCLK);
  P5SEL0 |= (SDI | SDO | SCLK);

  P5DIR |= CS;// | CS2;                           // P2.0 CS (chip select)
  P5OUT |= CS;// | CS2;

  //P1DIR &= ~(INT1 | INT2);                      // P1.4 and P1.3 as INT (INTERRUPT, not used yet)

  UCB1CTLW0 &= ~UCSWRST;                         // Initialize USCI state machine
}

void spi_txready() {
  while (!(UCB1IFG & UCTXIFG)); // TX buffer ready?
}

void spi_rxready() {
  while (!(UCB1IFG & UCRXIFG)); // RX Received?
}

void spi_send(uint8_t data) {
  spi_txready();
  UCB1TXBUF = data;            // Send data over SPI to Slave
}

void spi_recv() {
  spi_rxready();
  spi_buf = UCB1RXBUF;         // Store received data
}

void spi_transfer(uint8_t data) {
  spi_send(data);
  spi_recv();
}

void spi_chipEnable() {
  P5OUT &= ~CS;
}

void spi_chipDisable() {
   P5OUT |= CS;
}
