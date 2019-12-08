#include <xc.h>
#include <libpic30.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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

int
main(void) {
#if 0
  // Skipping due to following description (DS 19.2.7):
  //
  // > If configuration bits FCKSM<1:0> = 1x, then the clockswitching
  // > and fail-safe clock monitor functions are dis-abled.
  // > This is the default configuration bit setting.
  // >
  // > If clock switching is disabled, then the FOS<1:0> and
  // > FPR<3:0> bits directly control the oscillator selection
  // > and the COSC<1:0> bits do not control the clock selection.
  // > However, these bits will reflect the clocksource selection.
  //

  //
  // 19.2.8 PROTECTION AGAINST ACCIDENTAL WRITES TO OSCCON
  //

  // low byte programming sequence
  OSCCON = 0x46;
  OSCCON = 0x57;
  OSCCON = 0b00000000; /*
             ^^POST<1:0> - system clock postscaler (FRM 7.16, Figure 7-12)
               ^LOCK - readonly bit to flag PLL lock state
                ^-
                 ^CF - readonly bit to flag ClockFail state
                  ^-
                   ^LPOSCEN - 1 to disable timer/counter mode (DS 9.5.1)
                    ^OSWEN - readonly bit to flag clock transition
  */

  // high byte programming sequence
  OSCCON = 0x46;
  OSCCON = 0x57;
  OSCCON = 0b00010001; /*
             ^^TUN<3:2> - FRC clock tuning (DS 19.2.5 - table 19-4)
               ^^COSC<1:0> - FRC as clock source (DS 19.2.5)
                 ^^TUN<1:0>
                   ^^NOSC<1:0> - Same as FSC<1:0>? (DS 19.2.7)
  */
#endif

  TRISD = 0x0000;

  for (;;) {
    __delay_ms(100);
    LATD ^= 0xFFFF;
  }

  return 0;
}
