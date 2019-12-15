
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "xc.h"
#include "libpic30.h"
#include "uart.h"

#include "hd44780.h"

#include "pt.h"

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

// Global context to pass around
volatile struct ctx_t {
  uint32_t tick; // systick
  int tc_curr; // current temperature
  int tc_goal; // temperature to aim for
  int tc_disp; // displayed temperature for user control
} ctx;

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

int
write(int handle, uint8_t *buf, size_t len) {
  int i;
  for (i = 0; i < len; i++) {
    lcd_putc(buf[i]);
  }
  return len;
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
ui_init(void) {
  ADPCFG = 0xFF;     // all ADC/GPIO mux pins in GPIO mode (FRM 17, Register 17-5)
  TRISB  = 0b111000; // RB[012]: output, RB[345]: input
  LATB   = 0b000000; // RB[012]: X,      RB[345]: 0
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
tick_init(void) {
  //
  // Create "systick" timer that triggers every 1ms.
  // 8MHz / 4 / 1KHz = 2000
  //
  PR1 = 2000;
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
  // do nothing - just wake up
  ctx.tick++;
  IFS0bits.T1IF = 0;
}

PT_THREAD(blink_task(struct pt *pt)) {
  static uint32_t next_timing;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 1000;

    led_blink();
  }
  PT_END(pt);
}

PT_THREAD(run_task(struct pt *pt)) {
  static uint32_t next_timing;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 500;

    lcd_locate(0, 0);
    printf("%d -> %d %c  ",
           ctx.tc_curr, ctx.tc_disp, ctx.tc_disp == ctx.tc_goal ? ' ' : '?');

    ssr_control();
  }
  PT_END(pt);
}

typedef enum { MS_INIT, MS_WAIT } measure_state_t;

PT_THREAD(measure_task(struct pt *pt)) {
  static measure_state_t state = MS_INIT;
  static uint32_t next_timing;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);

    switch (state) {
    case MS_INIT:
      LATDbits.LATD1 = 0;
      __delay_us(1);
      SPI1BUF = 0; // dummy write to drive SPI clock
      state = MS_WAIT;
      next_timing = ctx.tick + 6; // data should be clocked in after 6ms
      break;
    case MS_WAIT:
      LATDbits.LATD1 = 1;

      // extract temperature in 10bit resolution
      // See MAX6675 DS
      ctx.tc_curr = SPI1BUF >> 6;

      state = MS_INIT;
      next_timing = ctx.tick + 1000; // redo after 1000ms
      break;
    }
  }
  PT_END(pt);
}

PT_THREAD(ui_task(struct pt *pt)) {
  static uint32_t next_timing;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 300;

    LATBbits.LATB0 ^= 1;
  }

  PT_END(pt);
}

typedef enum { BS_INIT, BS_CHECK_PRESS, BS_CHECK_RELEASE } button_state_t;

#define ASSERTED 0

PT_THREAD(button_task(struct pt *pt)) {
  static uint32_t next_timing;
  static button_state_t state;
  uint8_t delay = 1;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);

    switch (state) {
    case BS_INIT:
      if (PORTBbits.RB3 == ASSERTED) {
        state = BS_CHECK_PRESS;
        delay = 30;
      }
      break;
    case BS_CHECK_PRESS:
      if (PORTBbits.RB3 == ASSERTED) {
        // update goal temperature
        ctx.tc_goal = ctx.tc_disp;
        state = BS_CHECK_RELEASE;
      }
      else {
        state = BS_INIT;
      }
      break;
    case BS_CHECK_RELEASE:
      if (PORTBbits.RB3 != ASSERTED) {
        state = BS_INIT;
      }
      break;
    }

    next_timing = ctx.tick + delay;
  }

  PT_END(pt);
}

PT_THREAD(rotenc_task(struct pt *pt)) {
  static uint32_t next_timing;
  static const int dir[] = { 0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 };
  static uint8_t ab;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 1;

    ab = (ab << 2) + ((PORTB >> 4) & 0b11);

    int n = dir[ab & 0xF];
    if (n) {
      ctx.tc_disp += n;
      if (ctx.tc_disp < 0) {
        ctx.tc_disp = 0;
      }
    }
  }

  PT_END(pt);
}

int
main(void) {
  led_init();

  uart_init();
  uart_puts("reset\r\n");

  tick_init();

  ui_init();
  lcd_init();
  spi_init();

  ssr_init();

  struct pt pt1, pt2, pt3, pt4, pt5, pt6;

  for (;;) {
    Idle();

    rotenc_task(&pt6);
    button_task(&pt5);
    ui_task(&pt4);

    measure_task(&pt3);
    blink_task(&pt2);
    run_task(&pt1);
  }

  return 0;
}
