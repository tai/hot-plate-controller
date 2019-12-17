/**********************************************************************
 * Hot Plate Controller
 * 
 * This code controls external SSR based on temperature sensor data.
 * User can change/set temperature using a rotary encoder and a button.
 * LCD shows both current and target temperature.
 * LEDs indicates aliveness and when SSR is being turned on.
 * 
 **********************************************************************/

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

/**********************************************************************
 * config bits
 **********************************************************************/

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

/**********************************************************************
 * PINOUT
 **********************************************************************/

// for blinking LED
#define LED_IO D0

// for SSR control
#define SSR_IO C15

// for SPI IO
#define SPI_MISO F2
#define SPI_MOSI F3
#define SPI_SCK E8
#define SPI_CS D1

// for LCD output
#define LCD_PORT E
#define LCD_B4 E0
#define LCD_B5 E1
#define LCD_B6 E2
#define LCD_B7 E3
#define LCD_EN E4
#define LCD_RS E5

// for rotary encoder + button input
#define ROTENC_BUTTON B3
#define ROTENC_A B4
#define ROTENC_B B5

// for alternate UART
#define UART_TX C13
#define UART_RX C14

/**********************************************************************
 * Global macros and variables
 **********************************************************************/

#define BAUDRATE 9600
#define ASSERTED 0

// Global context to pass around
volatile struct ctx_t {
  uint32_t tick; // systick

  // temperature management
  int tc_curr; // current temperature
  int tc_goal; // temperature to aim for
  int tc_disp; // displayed temperature for user control
} ctx;

/**********************************************************************
 * UART
 **********************************************************************/

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

/**********************************************************************
 * STDIO for libpic30.
 * See DS50001456J, 16-Bit Language Tools Libraries Reference Manual.
 * 
 * Predefined handles are: 0=stdin, 1=stdout, 2=stderr.
 * Here, I tried to enhance with LCD output, but open/fopen overwrite
 * does not seem to be working. So I simply overwrote stderr to be LCD.
 **********************************************************************/

//FILE *lcdout;
#define lcdout stderr

enum {
  handle_stdin, 
  handle_stdout, 
  handle_stderr, 
  handle_lcdout,
};

#if 0
int
open(const char *name, int access, int mode) {
  uart_puts(name);
  switch (name[0]) {
  case 'i': return handle_stdin;
  case 'o': return handle_stdout;
  case 'e': return handle_stderr;
  case 'L': return handle_lcdout;
  default:  return handle_stderr;
  }
}
#endif

int
write(int handle, uint8_t *buf, size_t len) {
  int i;

  switch (handle) {
  case handle_stderr:
    for (i = 0; i < len; i++) {
      lcd_putc(buf[i]);
    }
    break;
  default:
    for (i = 0; i < len; i++) {
      uart_putc(buf[i]);
    }
  }

  return len;
}

/**********************************************************************
 * LED blink task, just to show the chip is alive.
 **********************************************************************/

static inline void
led_init(void) {
  TRISDbits.TRISD0 = 0;
}

PT_THREAD(led_task(struct pt *pt)) {
  static uint32_t next_timing;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 1000;

    LATDbits.LATD0 ^= 1;
  }
  PT_END(pt);
}

/**********************************************************************
 * LCD task - show current temperature and user configuration
 * 
 * A port of ChaN's HD44780 control code is used for actual control.
 **********************************************************************/

PT_THREAD(lcd_task(struct pt *pt)) {
  static uint32_t next_timing;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 500;

    lcd_locate(0, 0);
    fprintf(lcdout, "%d -> %d %c  \r\n",
            ctx.tc_curr, ctx.tc_disp, ctx.tc_disp == ctx.tc_goal ? ' ' : '?');
  }
  PT_END(pt);
}

/**********************************************************************
 * SSR control task
 *
 * Controlling AC power with SSR is tricky because SSR only
 * turns on/off at zero-cross point, not when CPU controlled the port.
 * 
 * Also, using fast-paced PWM (one used for DC power control) does
 * not work, because AC power is a sine wave and not evenly distributed
 * over time. If SSR was turned on during near-zero point, it won't
 * provide power like you expect with DC.
 * 
 * Here, slower-paced PWM is used to control AC power.
 * SSR is controlled with 16bit value, where each bit represents
 * power state of each 83ms period. As AC is 50/60Hz, 83ms contains
 * at least 3 complete cycles, avoiding problem with the fast PWM.
 * 
 * This 16bit value is handled as follows:
 * 
 * 11111111 11111111 all-periods turned on (full power)
 * 11111111 11111110 15 periods turned on
 * 11111111 11111100 14 periods turned on
 * 11111111 11111000 ...
 * 00000000 00000000 all-periods turned off (no power)
 * 
 * So 16-levels of power output can be managed.
 * 
 **********************************************************************/

static inline void
control_init(void) {
  TRISCbits.TRISC15 = 0;
}

PT_THREAD(control_task(struct pt *pt)) {
  static uint32_t next_timing;
  static uint16_t power_bits;
  static uint8_t i;

  PT_BEGIN(pt);
  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 83;

    // control SSR
    LATCbits.LATC15 = (power_bits >> (i++ & 0xF)) & 1;

    // update power schedule - 2 levels for now
    power_bits = (ctx.tc_goal > ctx.tc_curr) ? 0xFFFF : 0x0000;
  }
  PT_END(pt);
}

/**********************************************************************
 * SPI IO task
 **********************************************************************/

static inline void
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

PT_THREAD(spi_sensor_task(struct pt *pt)) {
  typedef enum { MS_INIT, MS_WAIT } measure_state_t;

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
    default:
      LATDbits.LATD1 = 1;

      // extract temperature in 10bit resolution
      // See MAX6675 DS
      ctx.tc_curr = SPI1BUF >> 6;

      state = MS_INIT;
      next_timing = ctx.tick + 1000; // rescan after 1000ms
    }
  }
  PT_END(pt);
}

/**********************************************************************
 * UI task - handle user inputs from a rotary encoder and a button
 **********************************************************************/

static inline void
ui_init(void) {
  ADPCFG = 0xFF;     // all ADC/GPIO mux pins in GPIO mode (FRM 17, Register 17-5)
  TRISB  = 0b111000; // RB[012]: output, RB[345]: input
  LATB   = 0b000000; // RB[012]: X,      RB[345]: 0
}

PT_THREAD(ui_button_task(struct pt *pt)) {
  typedef enum { BS_INIT, BS_CHECK_PRESS, BS_CHECK_RELEASE } button_state_t;

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
    default:
      if (PORTBbits.RB3 != ASSERTED) {
        state = BS_INIT;
      }
    }

    next_timing = ctx.tick + delay;
  }

  PT_END(pt);
}

PT_THREAD(ui_rotenc_task(struct pt *pt)) {
  static uint32_t next_timing;
  static const int dir[] = { 0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 };
  static uint8_t ab;
  static int8_t raw, fix;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt, ctx.tick >= next_timing);
    next_timing = ctx.tick + 1;

    ab = (ab << 2) + ((PORTB >> 4) & 0b11);
    raw -= dir[ab & 0xF]; // add/sub op depends on AB or BA wiring

    // Fixup to update temperature by 1 for each encoder click.
    // A divide does the trick as current encoder generates pulse
    // by 4 count for each click.
    if (raw >= 4) {
      fix = raw >> 2;
      if (fix == 0) fix = 1;
      raw = 0;
    }
    else if (raw <= -4) {
      fix = -(-raw >> 2);
      if (fix == 0) fix = -1;
      raw = 0;
    }

    if (fix != 0) {
      ctx.tc_disp += fix;
      fix = 0;
      if (ctx.tc_disp < 0) {
        ctx.tc_disp = 0;
      }
    }
  }

  PT_END(pt);
}

/**********************************************************************
 * systick - 1ms tick timer for driving tasks
 **********************************************************************/

static inline void
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

/**********************************************************************
 * app main
 **********************************************************************/

int
main(void) {
  led_init();

  uart_init();
  uart_puts("reset\r\n");

  tick_init();

  ui_init();
  lcd_init();
  spi_init();

  control_init();

  struct pt pt1, pt2, pt3, pt4, pt5, pt6, pt7;

  for (;;) {
    Idle();

    ui_rotenc_task(&pt6);
    ui_button_task(&pt5);

    control_task(&pt7);

    spi_sensor_task(&pt3);

    led_task(&pt2);
    lcd_task(&pt1);
  }

  return 0;
}
