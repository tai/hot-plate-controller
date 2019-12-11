
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "xc.h"
#include "libpic30.h"
#include "uart.h"

#include "hd44780.h"

// FOSC - Oscillator Configurations (DS 19.2)
#pragma config FOS = FRC
#pragma config FCKSMEN = CSW_FSCM_OFF // clock switching disable, monitoring disabled
#pragma config FPR = ERCIO // Use OSC2/RC15 as GPIO (clock is set by FOS=FRC)

// FWDT
#pragma config WDT = WDT_OFF          // watchdog disabled

// FBORPOR
#pragma config FPWRT = PWRT_OFF // Power-on-Reset timer disabled
#pragma config MCLRE = MCLR_EN // MCLR enabled
#pragma config PWMPIN = RST_IOPIN // Switch pinmode between PWM/GPIO

// FICD
#pragma config ICS = ICS_PGD

#define BAUDRATE 9600

static inline void
uart_init(void) {
  //
  // Setup UART (DS 17.2, FRM 19.2, FRM 4.6.1)
  //
  U1BRG = FCY / BAUDRATE / 16 - 1; // DS 17.8, Equation 17-1
  U1MODEbits.ALTIO = 1;
  U1MODEbits.UARTEN = 1;
  U1STAbits.UTXEN = 1;
}

static inline void
uart_putc(uint8_t ch) {
  while (U1STAbits.UTXBF);
  U1TXREG = ch;
}

void
uart_puts(const char *s) {
  while (*s) {
    uart_putc((uint8_t)*s++);
  }
}

static inline void
led_init(void) {
  TRISDbits.TRISD0 = 0;
}

static inline void
led_blink(void) {
  LATDbits.LATD0 ^= 1;
}

static void
spi_init(void) {
  // Use RD1 as nSS
  TRISDbits.TRISD1 = 0;

  SPI1CON = 0b0000111000100000; /*
              ^-
               ^FRMEM:0=Framed SPI disabled
                ^SPIFSD:Frame sync pulse direction is ouput
                 ^-
                  ^DISSDO:1=SDO pin is free as GPIO
                   ^MODE16:1=16bit data IO
                    ^SMP:1=input sampled at end of data output time
                     ^CKE:0=Output change on active->idle level change of clock
                      ^SSEN:1=Enable as chip select input in slave mode
                       ^CKP:0=clock is low on idle
                        ^MSTEN:1=Master
                         ^^^SPRE:000=1/8 secondary prescale
                            ^^PPRE:00=1/64 primary prescale
  */
  SPI1STATbits.SPIROV = 0; // clear overflow flag
  SPI1STATbits.SPIEN = 1;  // enable SPI
}

uint16_t
spi_send(uint16_t val) {
  LATDbits.LATD1 = 0;
  __delay_us(1);
  SPI1BUF = val;
  __delay_ms(6);
  LATDbits.LATD1 = 1;
  return SPI1BUF;
}

void
ssr_init(void) {
  TRISCbits.TRISC15 = 0;
}

void
ssr_control(void) {
  LATCbits.LATC15 ^= 1;
}

void
timer_init(void) {
  // Make timer1 overflow ~30times/s
  T1CON = 0b1000000000000000; /*
            ^TON:1=start timer
             ^-
              ^TSIDL:0=continue in idle mode
               ^^^^^^-
                     ^TGATE:0=disable gated time accumulation
                      ^^TCKPS:00=1/1 prescale
                        ^-
                         ^TSYNC:ignored for internal clock
                          ^TCS:0=internal clock (FOSC/4)
                           ^-
  */

  IEC0bits.T1IE = 1;
}

void __attribute__((interrupt, no_auto_psv))
_T1Interrupt(void) {
  static int nr;

  // Run this about once a second (see T1CON)
  if (nr++ == 30) {
    nr = 0;
    led_blink();
  }

  IFS0bits.T1IF = 0;
}


int
main(void) {
  static int c = 'a';

  led_init();

  uart_init();
  uart_puts("reset\r\n");

  timer_init();

  lcd_init();
  spi_init();

  ssr_init();

  for (;;) {
    __delay_ms(500);
    //led_blink();

    if (c > 'z') c = 'a';
    lcd_locate(0, 0);
    lcd_putc(c);
    uart_putc(c++);

    // See MAX6675 DS
    uint16_t ret = spi_send(0);
    uint8_t max6675_status = ret & 0x7;
    int16_t max6675_result = ret >> 6;
    printf("s=%d, t=%d\r\n", max6675_status, max6675_result);

    ssr_control();
  }

  return 0;
}
