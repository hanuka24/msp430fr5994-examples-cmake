/*

Transmit 'Ping' Lora packet periodically using RFM95 tranceiver.
RFM95 module is connected using SPI (see spi.c)

*/



#include <msp430.h>
#include <stdint.h>
#include "mcu.h"
#include "serial.h"
#include "spi.h"
#include "sx1276.h"
#include "sx1276regs-fsk.h"
#include "base64.h"
#include "driverlib.h"
        
#define LOG_INTERVAL 5


#define RF_FREQUENCY   868000000 // Hz

#define FSK_FDEV                          25e3      // Hz
#define FSK_DATARATE                      50e3      // bps
#define FSK_BANDWIDTH                     50e3      // Hz
#define FSK_AFC_BANDWIDTH                 83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH               5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON         false

#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR                       7        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         3         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                  1000
#define TX_OUTPUT_POWER                   0        // dBm
#define BUFFER_SIZE                       32 // Define the payload size here


uint32_t __attribute__((section(".persistent"))) log_cnt = 0;
uint16_t __attribute__((section(".persistent"))) packets = 0;
uint8_t buffer[BUFFER_SIZE];

static radio_events_t radio_events;

int state = 0;
int tx_done = 0;

void SendPing() {
  buffer[0] = 1;
  buffer[1] = 0x10;
  buffer[2] = 8;
  buffer[3] = 1;
  buffer[4] = 'P';
  buffer[5] = 'I';
  buffer[6] = 'N';
  buffer[7] = 'G';

  sx1276_send(buffer, 4);
}

void OnTxDone() {
 // P1OUT ^= 0x2;
  sx1276_set_opmode(RF_OPMODE_SLEEP);
  spi_sleep();
  tx_done=true;
  packets++;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {

  print2uart("$RXS,");

  base64_encode(payload, size);
  print2uartc('\n');

  if(state == 1) SendPing();
}

void OnRxError() {
  print2uart("$RXE\n");
}

void rf_init_lora() {
  radio_events.TxDone = OnTxDone;
  radio_events.RxDone = OnRxDone;
  //radio_events.TxTimeout = OnTxTimeout;
  //radio_events.RxTimeout = OnRxTimeout;
  radio_events.RxError = OnRxError;

  sx1276_init(radio_events);
  sx1276_set_channel(RF_FREQUENCY);

  sx1276_set_txconfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

   sx1276_set_rxconfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

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
  mcu_init();


  if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3)) //check if PIN high
    {
        uartinit(SERIAL_CLK_1MHZ);

        //Enable P5.6 internal resistance as pull-Up resistance
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN6);
        
        //GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN2);
        GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN2);

        //P1.1 Hi/Lo edge
        GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);

        //P5.6 (Button S1) IFG cleared and enabled
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN6);
        GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN6);

        __enable_interrupt();

        //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        while(1) { };   //we could go to sleep as well
    }  
else
{

  log_cnt=0;
  packets=0;
  uartinit(SERIAL_CLK_1MHZ);
  print2uart("=========\n");
  
  spi_init();
  rf_init_lora();

  // print2uart("DEV VERSION: %d\n", sx1276_read(REG_VERSION));
  // print2uart("FREQUENCY: %lu\n", RF_FREQUENCY);
  // print2uart("TX POWER: %d\n", TX_OUTPUT_POWER);
  // print2uart("BW: %d\n", LORA_BANDWIDTH);
  // print2uart("SF: %d\n", LORA_SPREADING_FACTOR);
  // print2uart("CR: %d\n", LORA_CODINGRATE);
  // print2uart("==========\n");

  SendPing();
  // mcu_delayms(1000);
  Init_RTC(true);
  while(1) {
   // P1OUT ^= 0x01;
    //P1OUT^=0x01;
    spi_init();
    SendPing();
    mcu_delayms(100);
   // while(!tx_done);
   // tx_done=false;
    __bis_SR_register(LPM3_bits | GIE);     // Enter LPM3 mode w/ interrupts enabled
  }
}
  return 0;
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
            if(log_cnt++%LOG_INTERVAL == 0)
            {
                __bic_SR_register_on_exit(LPM3_bits | GIE); /* Exit Low Power Mode 3 to get send packet */
            }                
            break;
        case RTCIV__RTCTEVIFG:        // RTCEVIFG // Interrupts every minute

       
            break;
        case RTCIV__RTCAIFG:   break;       // RTCAIFG
        case RTCIV__RT0PSIFG:  break;       // RT0PSIFG
        case RTCIV__RT1PSIFG:  break;       // RT1PSIFG
        default: break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT5_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT5_VECTOR)))
#endif
void Port_5 (void)
{
    
    if ( P5IFG & GPIO_PIN6)
    {
        print2uart("{\"MSP430_Lifetime\":%l, \"Packets\":%d}\n", log_cnt, packets);
        GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN6);
    }
}


