#include "avr/io.h"
#include "avr/interrupt.h"
#include "include/gamecube_to_nes.h"

/*
 * Gamecube serial transmission interrupt handler
 * Idea: put gamecubePWMBuffer into gpio, saves 6 cycles per interrupt?
 */
.section .text.TCA0_OVF_vect
.globl TCA0_OVF_vect
TCA0_OVF_vect:
  // Interrupt prologue, save working...
  push r16                                                 // 1

  // Immediately update compare buffer to minimize the chances of starvation
  // This can come before most of the prologue, as no status flags are affected
  in r16, gamecubePWMBuffer + 0                            // 1
  sts TCA0_SINGLE_CMP0BUF + 0, r16                         // 2
  in r16, gamecubePWMBuffer + 1                            // 1
  sts TCA0_SINGLE_CMP0BUF + 1, r16                         // 2

  // ...and status registers
  in r16, _SFR_IO_ADDR(SREG)                               // 1
  push r16                                                 // 1
  push r26                                                 // 1
  push r27                                                 // 1

  // Acknowledge pending interrupt
  ldi r16, TCA_SINGLE_OVF_bm                               // 1
  sts TCA0_SINGLE_INTFLAGS, r16                            // 2

  // Read a new bit from the shift register
  lds r16, gamecubeTxShiftRegister                         // 3
  lsl r16                                                  // 1
  brne 0f                                                  // 1 / 2
    // Since we shifted in a sentinel bit, a result of zero means that the shift register is exhausted
    lds r16, gamecubeTxLength                              // 3
    dec r16                                                // 1
    breq .LTCA0_OVF_vect.end_tx                            // 1 / 2
    sts gamecubeTxLength, r16                              // 2

    // If we haven't reached the end of our data packet, read a new byte
    lds r26, gamecubeTxPtr + 0                             // 3
    lds r27, gamecubeTxPtr + 1                             // 3
    ld r16, X+                                             // 2
    sts gamecubeTxPtr + 0, r26                             // 2
    sts gamecubeTxPtr + 1, r27                             // 2

    // Conveniently, we can shift the same sentinel bit right back in
    rol r16                                                // 1
0:
  sts gamecubeTxShiftRegister, r16                         // 2

  // Convert carry bit into PWM value
  ldi r26, lo8(GC_PWM_TRUE)                                // 1
  ldi r27, hi8(GC_PWM_TRUE)                                // 1
  brcs .LTCA0_OVF_vect.update_pwm_buffer                   // 1 / 2
    ldi r26, lo8(GC_PWM_FALSE)                             // 1
    ldi r27, hi8(GC_PWM_FALSE)                             // 1
.LTCA0_OVF_vect.update_pwm_buffer:
  out gamecubePWMBuffer + 0, r26                           // 1
  out gamecubePWMBuffer + 1, r27                           // 1

  // Interrupt epilogue, restore working and status registers
.LTCA0_OVF_vect.epilogue:
  pop r27                                                  // 2
  pop r26                                                  // 2
  pop r16                                                  // 2
  out _SFR_IO_ADDR(SREG), r16                              // 1
  pop r16                                                  // 2
  reti                                                     // 4 / 5

  // If we've reached the end of the transmission, we need to start shutting down the data stream
.LTCA0_OVF_vect.end_tx:
  lds r16, gamecubeTxState
  inc r16
  sts gamecubeTxState, r16
  // Checking for the final stage has tighter timing requirements
  cpi r16, 4
  breq .LTCA0_OVF_vect.disable_peripheral
  cpi r16, 1
  breq .LTCA0_OVF_vect.send_stop_bit
  cpi r16, 2
  breq .LTCA0_OVF_vect.halt_oscillator
  rjmp .LTCA0_OVF_vect.epilogue

  // On the first cycle
.LTCA0_OVF_vect.send_stop_bit:
  ldi r26, lo8(GC_PWM_STOP)
  ldi r27, hi8(GC_PWM_STOP)
  rjmp .LTCA0_OVF_vect.update_pwm_buffer

  // second cycle
.LTCA0_OVF_vect.halt_oscillator:
  ldi r26, lo8(0)
  ldi r27, hi8(0)
  rjmp .LTCA0_OVF_vect.update_pwm_buffer

  // Fourth cycle
.LTCA0_OVF_vect.disable_peripheral:
  // Disable LUT1 to reclaim access to it's corresponding pin
  lds r16, CCL_LUT1CTRLA
  andi r16, lo8(~(CCL_ENABLE_bm))
  sts CCL_LUT1CTRLA, r16

  // Enable TCB0 peripheral to measure ...
  lds r16, TCB0_CTRLA
  ori r16, lo8(TCB_ENABLE_bm)
  sts TCB0_CTRLA, r16

  // Reconfigure TCA0 to be used as a watchdog timer
  ldi r16, 0xff
  sts TCA0_SINGLE_PER + 0, r16
  sts TCA0_SINGLE_PER + 1, r16
  rjmp .LTCA0_OVF_vect.epilogue



/*
 * Gamecube serial receipt interrupt handler
 */
.section .text.TCB0_INT_vect
.globl TCB0_INT_vect
TCB0_INT_vect:
  // Interrupt prologue, save working and status registers
  push r16                                                 // 1
  in r16, _SFR_IO_ADDR(SREG)                               // 1
  push r16                                                 // 1
  push r26                                                 // 1
  push r27                                                 // 1

  // Debug
  ldi r16, _BV(2)
  sts PORTB_OUTTGL, r16

  // Read pulse width and acknowledge interrupt
  lds r26, TCB0_CCMP + 0
  lds r27, TCB0_CCMP + 1

  // Reset watchdog timeout
  ldi r16, 0
  sts TCA0_SINGLE_CNT + 0, r16
  sts TCA0_SINGLE_CNT + 1, r16

  // Compare pulse width against reference threshold
  subi r26, lo8(GC_PWM_STOP)
  sbci r27, hi8(GC_PWM_STOP)

  // Shift data bit into serial stream
  lds r16, gamecubeRxShiftRegister
  ror r16
  brcc 0f
    // We initialized the shift register with a sentinel bit, so carry set means that it's full
    lds r26, gamecubeRxPtr + 0
    lds r27, gamecubeRxPtr + 1
    st X+, r16
    sts gamecubeRxPtr + 0, r26
    sts gamecubeRxPtr + 1, r27

    // Increment the length counter
    lds r16, gamecubeRxLength
    inc r16
    sts gamecubeRxLength, r16

    // Reset shift register
    ldi r16, 0x80
0:
  sts gamecubeRxShiftRegister, r16

  // Debug
  ldi r16, _BV(2)
  sts PORTB_OUTTGL, r16

  // Interrupt epilogue, restore working and status registers
  pop r27
  pop r26
  pop r16
  out _SFR_IO_ADDR(SREG), r16
  pop r16
  reti



/*
 *
 */
.section .text.TCA0_CMP2_vect
.globl TCA0_CMP2_vect
TCA0_CMP2_vect:
  //
  push r16

  // Acknowledge pending interrupt
  ldi r16, TCA_SINGLE_CMP2_bm                              // 1
  sts TCA0_SINGLE_INTFLAGS, r16                            // 2

  //
  pop r16
  reti



/*
 * NES serial transmission interrupt handler
 * Uses ...
 * Can probably be non-blocking? We can shove the buffered byte into another GPIOR register
 *//*
.section .text.SPI0_INT_vect
.globl SPI0_INT_vect
SPI0_INT_vect:
  // Interrupt prologue
  push r16
  in r16, _SFR_IO_ADDR(SREG)
  push r16
  push r26
  push r27

  // Read new data byte from stream and provide it to the SPI controller
  lds r26, nesTxPtr + 0
  lds r27, nesTxPtr + 1
  ld r16, X+
  sts nesTxPtr + 0
  sts nesTxPtr + 1
  sts SPI0_DATA, r16

  // Decrement our length counter
  lds r16, nesTxLength
  dec
  sts nesTxLength, r16
  brne 0f
    // End transmission here
0:

  // Interrupt epilogue
  pop r27
  pop r26
  pop r16
  out _SFR_IO_ADDR(SREG), r16
  pop r16
  reti*/

/*
 * NES strobe interrupt handler
 */
/*
.globl 
 */