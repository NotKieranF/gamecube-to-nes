#include "include/gamecube_to_nes.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * Map:
 *  PA0 - SPI0 /SS - Should be tied low
 *  PA1 - SPI0 MOSI - Connected to NES_OUT, triggers an interrupt on falling edge
 *  PA2 - SPI0 MISO - Connected to NES_D0
 *  PA3 - SPI0 SCK - Connected to NES_CLK, clocks shift register on rising edge
 *  PA6 - NC
 *  PA7 - TCA0 WO0 - Connected to GC_DATA via a 5V-3.3V level shifter
 */

/*
 *  PA0 - TCA0 WO0
 *  PE0 - SPI0 MOSI
 *  PE1 - SPI0 MISO
 *  PE2 - SPI0 SCK
 *  PE3 - SPI0 /SS
 */

#define GC_DATA_PIN (0)
#define NES_OUT_PIN (0)
#define NES_CLK_PIN (2)
#define NES_D0_PIN (1)
#define SPI_SS_PIN (3)

nesInputPacket inputBufferA;
nesInputPacket inputBufferB;
nesInputPacket inputBufferC;
nesInputPacket *frontBuffer;
nesInputPacket *backBuffer;
nesInputPacket *nextBuffer;

/*
 * Gamecube serial transmissions are encoded using the TCA0 peripheral using
 * the double-buffered single-shot PWM mode on compare channel 0. An interrupt
 * is triggered on timer overflow, and a new compare threshold must be written
 * within one period (5us)
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
 * result must be read and serialized within one period (4us)
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
unsigned char nesTxLength = 5;
char nesTxBufferA[5] = {0xf0, 0x0f, 0xff, 0x00, 0xaa};
char nesTxBufferB[5];
char *nesTxPtr = nesTxBufferA;

//
void pollGamecubeController() {
    // Initialize TCA0 buffer chain to be initially high
    TCA0.SINGLE.CMP0BUF = 0;
    TCA0.SINGLE.CMP0 = 0;
    gamecubePWMBuffer = 0;

    // Enabling LUT1 overrides pin direction and begins output
    CCL.LUT1CTRLA |= CCL_ENABLE_bm;
    CCL.CTRLA = CCL_ENABLE_bm;

    // Enabling TCA0 begins command transmission
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;

        // Wait for controller response
        while (1);
}

// Initialize TCA0 peripheral for communication with GameCube controller
void initTCA0(void) {
    // Perform a hard reset on the TCA0 unit, switch to normal mode
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc;
    TCA0.SINGLE.CTRLD = 0;

    // Drive TCA0 directly by the peripheral clock without prescaling and event inputs
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
    TCA0.SINGLE.EVCTRL = 0;

    // Trigger interrupt on overflow so we can buffer a new bit into CMP0 during transmit,
    // trigger interrupt on CMP2 match for receive timeout
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP2_bm;

    // Use single slope PWM mode on compare channel 0 to generate waveform
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    // Wait for signal line to be idle for five bit periods before considering the receipt complete
    TCA0.SINGLE.CMP2 = GC_PWM_RX_TIMEOUT;
    TCA0.SINGLE.CMP2BUF = GC_PWM_RX_TIMEOUT;
}

// Initialize TCB0 peripheral for communication with GameCube controller
void initTCB0(void) {
  // Measure pulse width of signal
  TCB0.CTRLB = TCB_CNTMODE_PW_gc;

  // TCB0 monitors the status of the GC_DATA pin via the event system
  EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT0_PIN0_gc;
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;
  TCB0.EVCTRL = TCB_FILTER_bm | TCB_EDGE_bm | TCB_CAPTEI_bm;

  // Trigger interrupt on capture so we can serialize the result
  TCB0.INTCTRL = TCB_CAPT_bm;

  // No prescaling, initially disabled
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc;
}

// Initialize SPI0 peripheral for communication with NES
void initSPI0(void) {
    // Place SPI interface on pins PE[3:0]
    PORTMUX.TWISPIROUTEA = (PORTMUX.TWISPIROUTEA & ~PORTMUX_SPI0_gm) | PORTMUX_SPI0_ALT2_gc;

    // Slave mode, MSB first, initially disabled
    SPI0.CTRLA = 0;

    // SPI operates in buffered mode, sampling on the rising edge of sck
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_MODE_3_gc;

    // Enable data register empty interrupts so that we can refresh the serial data buffer
    SPI0.INTCTRL = SPI_DREIE_bm;

    // NES_OUT -> MOSI, NES_D0 -> MISO, NES_CLK -> SCK
    PORTE.DIRSET = _BV(NES_D0_PIN);
    PORTE.DIRCLR = _BV(NES_OUT_PIN) | _BV(NES_CLK_PIN) | _BV(SPI_SS_PIN);

    // Inputs need pullups for compatibility with PAL consoles
    PORTE.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;
    PORTE.PIN2CTRL = PORT_PULLUPEN_bm;

    // Prioritize servicing the NES's status request
    CPUINT.LVL1VEC = PORTE_PORT_vect_num;
}

int main() {
    // Disable clock prescaling so we can run at the full 20MHz
    CCP = 0xd8;
    CLKCTRL.MCLKCTRLB = 0;

    // Initialize peripherals prior to enabling interrupts
//    initTCA0();
//    initTCB0();
    initSPI0();
    asm volatile("sei" ::: "memory");

    // This is used as a debugging port
    PORTB.DIRSET = 0b00000010;
    PORTB.OUTSET = 0b00000010;

    while (1) {
        /*
         * Steps per iteration:
         *  1) Initiate controller poll
         *  2) Wait for response to be written to rx buffer
         *  3) Process response
         *  4) Copy response to tx buffer
         *  5) Loop?
         */
    }
}