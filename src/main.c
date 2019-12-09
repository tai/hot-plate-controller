
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "xc.h"
#include "libpic30.h"
#include "uart.h"

// FOSC - Oscillator Configurations (DS 19.2)
#pragma config FOS = FRC
#pragma config FCKSMEN = CSW_FSCM_OFF // clock switching disable, monitoring disabled

// FWDT
#pragma config WDT = WDT_OFF          // watchdog disabled

// FBORPOR
#pragma config FPWRT = PWRT_OFF // Power-on-Reset timer disabled
#pragma config MCLRE = MCLR_EN // MCLR enabled

// FICD
#pragma config ICS = ICS_PGD

#define BAUDRATE 9600

int
main(void) {
  static int c = 'a';

  TRISD = 0x0000;

  //
  // Setup UART (DS 17.2, FRM 19.2, FRM 4.6.1)
  //
  U1BRG = FCY / BAUDRATE / 16 - 1; // DS 17.8, Equation 17-1
  U1MODE = 0x8000; // Enable UART in 8N1, normal mode
  U1STAbits.UTXEN = 1; // TX enable

  for (;;) {
    __delay_ms(100);
    LATD ^= 0xFFFF;

    if (c > 'z') c = 'a';
    while (U1STAbits.UTXBF);
    U1TXREG = c++;
  }

  return 0;
}
