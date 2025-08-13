#include "include/gamecube_to_nes.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * Map:
 *  PA0 - SPI0 /SS - Should be tied low
 *  PA1 - SPI0 MOSI - Connected to NES_OUT, triggers an interrupt on falling edge
 *  PA2 - SPI0 MISO - Connected to NES_D0
 *  PA3 - SPI0 SCK - Connected to NES_CLK, clocks shift register on rising edge
 *  PA7 - TCA0 WO0 - Connected to GC_DATA via a 5V-3.3V level shifter
 */

/*
 *  PA0 - TCA0 WO0
 *  PE0 - SPI0 MOSI
 *  PE1 - SPI0 MISO
 *  PE2 - SPI0 SCK
 *  PE3 - SPI0 /SS
 */

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
 * slave mode. ...10 cycles 
 */
unsigned char nesTxLength = 5;
char nesTxBufferA[5] = {0x00, 0x01, 0x02, 0x03};
char nesTxBufferB[5];
char *nesTxPtr = nesTxBufferA;

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
    TCA0.SINGLE.PER = GC_PWM_TX_PERIOD;
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
    TCA0.SINGLE.CMP0BUF = 0;
    TCA0.SINGLE.CMP0 = 0;
    gamecubePWMBuffer = 0;
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
    PORTMUX.CCLROUTEA = PORTMUX_LUT1_bm;
}

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

// Initialize SPI0 peripheral for communication with NES
void initSPI0(void) {
    // Place SPI interface on pins PE[3:0]
    PORTMUX.TWISPIROUTEA = (PORTMUX.TWISPIROUTEA & ~PORTMUX_SPI0_gm) | PORTMUX_SPI0_ALT2_gc;

    // Slave mode, MSB first, initially disabled
    SPI0.CTRLA = 0;

    // SPI operates in buffered mode, sampling on the rising edge of sck
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_MODE_3_gc;

    // Enable transfer complete interrupts so that we can refresh the serial data buffer
    SPI0.INTCTRL = SPI_TXCIE_bm;

    // NES_OUT -> MOSI, NES_D0 -> MISO, NES_CLK -> SCK
    PORTE.DIRSET = _BV(NES_D0_PIN);
    PORTE.DIRCLR = _BV(NES_OUT_PIN) | _BV(NES_CLK_PIN) | _BV(SPI_SS_PIN);

    // Inputs need pullups for compatibility with PAL consoles
    PORTE.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;
    PORTE.PIN2CTRL = PORT_PULLUPEN_bm;
}

int main() {
    // Disable clock prescaling so we can run at the full 20MHz
    CCP = 0xd8;
    CLKCTRL.MCLKCTRLB = 0;

    // Initialize peripherals prior to enabling interrupts
    initSPI0();
    asm volatile("sei" ::: "memory");

    //
//    TCB0.CTRLB = TCB_CNTMODE_PW_gc;
//    TCB0.EVCTRL = 0b01010001;
//    TCB0.INTCTRL = 0b00000001;
//    TCB0.CTRLA = /*TCB_ENABLE_bm |*/ TCB_CLKSEL_CLKDIV1_gc;
//    EVSYS.CHANNEL2 = EVSYS_GENERATOR_PORT0_PIN6_gc;
//    EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL2_gc;
//
//    initGamecubeTx();
//    TCA0.SINGLE.CMP2BUF = GC_PWM_RX_TIMEOUT;
//    TCA0.SINGLE.CMP2 = GC_PWM_RX_TIMEOUT;
//
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