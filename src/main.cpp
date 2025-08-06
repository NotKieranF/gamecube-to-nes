#include "avr/interrupt.h"
#include "avr/io.h"
#include "gamecube_to_nes.h"

/*
 * Gamecube serial transmissions are encoded using the TCA0 peripheral using
 * the double-buffered single-shot PWM mode. An interrupt is triggered on timer
 * overflow, and a new compare threshold must be written within one period
 */
unsigned char gamecubeTxState = 0;
unsigned char gamecubeTxLength = 4;
char gamecubeTxShiftRegister = 0x80;
char gamecubeTxBufferA[3] = {0x40, 0x00, 0x00};
char gamecubeTxBufferB[3];
char *gamecubeTxPtr = gamecubeTxBufferA;

/*
 * Gamecube serial transmissions are decoded using the TCB0 peripheral using the
 * pulse-width capture mode. An interrupt is triggered on capture end, and the
 * result must be read and serialized
 */
unsigned char gamecubeRxLength = 0;
char gamecubeRxShiftRegister = 0x80;
char gamecubeRxBufferA[10];
char gamecubeRxBufferB[10];
char *gamecubeRxPtr = gamecubeRxBufferA;

/*
 * NES serial transmissions are encoded using the SPI0 peripheral in buffered
 * slave mode.
 */
unsigned char nesTxLength;
char *nesTxPtr;
char nesTxBufferA[5];
char nesTxBufferB[5];

// Gamecube command packets
void transmitGamecubePacket() {
  // Set GCDATA pin to be output
  // Prime TCA.CMP0 with first bit
  // Buffer second bit into TCA.CMP0BUF
  // Prime output to be high
  //  TCA0.SINGLE.CTRLC = TCA_SINGLE_CMP0OV_bm;
  // Enable LUT1
  // Enable TCA
}

// Gamecube transmissions are handled by TCA and CCL peripherals, and output on
// pin PC6
void initGamecubeTx() {
  // Perform a hard reset on the TCA0 unit, switch to normal mode
  TCA0.SINGLE.CTRLA = 0;
  TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc;
  TCA0.SINGLE.CTRLD = 0;

  // TCA unit is directly driven by the peripheral clock, no prescaling, and no
  // event inputs
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
  TCA0.SINGLE.EVCTRL = 0;
  // Gamecube controllers expect a new bit every 5us
  TCA0.SINGLE.PER = GC_PWM_PERIOD;
  // Trigger interrupt on overflow, so we can buffer a new bit into CMP0
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;

  // Use single slope PWM mode on compare channel 0 to generate waveform
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
  // Notably, the output of compare channel 0 in normal mode cannot be mapped
  // directly to a GPIO pin on the attiny212, so we must pipe it through the CCL
  // unit first, specifically LUT1
  CCL.SEQCTRL0 = CCL_SEQSEL1_DISABLE_gc;
  CCL.LUT1CTRLB = CCL_INSEL0_TCA0_gc | CCL_INSEL1_MASK_gc;
  CCL.LUT1CTRLC = CCL_INSEL2_MASK_gc;
  CCL.TRUTH1 = 0b10101010;
  CCL.LUT1CTRLA = CCL_FILTSEL_DISABLE_gc | CCL_OUTEN_bm;

  // LUT must be enabled separately, after all other initialization
  // (This should also be done separately)
  CCL.LUT1CTRLA |= CCL_ENABLE_bm;
  CCL.CTRLA = CCL_ENABLE_bm;

  // Invert direction??
  PORTC.PIN6CTRL = 0b10000000 | PORT_ISC_INTDISABLE_gc;
  PORTC.DIRCLR = 0b01000000;

  // This is a temporary thing, we don't really need to initialize the pwm here
  TCA0.SINGLE.CMP0 = 0;
  TCA0.SINGLE.CMP0BUF = 0;
  gamecubePWMBuffer = 0;
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
  PORTMUX.CCLROUTEA = PORTMUX_LUT1_bm;
}

int main() {
  // Disable clock prescaling so we can run at the full 20MHz
  CCP = 0xd8;
  CLKCTRL.MCLKCTRLB = 0;
  asm volatile("sei" ::: "memory");

  //
  TCB0.CTRLB = TCB_CNTMODE_PW_gc;
  TCB0.EVCTRL = 0b01010001;
  TCB0.INTCTRL = 0b00000001;
  TCB0.CTRLA = /*TCB_ENABLE_bm |*/ TCB_CLKSEL_CLKDIV1_gc;
  EVSYS.CHANNEL2 = EVSYS_GENERATOR_PORT0_PIN6_gc;
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL2_gc;

  initGamecubeTx();

  PORTB.DIRSET = 0b00000100;
  PORTB.OUTSET = 0b00000100;

  while (1) {
    /*
     * Steps per iteration:
     *  1) Initiate controller poll
     *  2) Wait for response to be written to rx buffer
     *  3) Copy response to tx buffer
     *  4) Loop?
     */
  }
}