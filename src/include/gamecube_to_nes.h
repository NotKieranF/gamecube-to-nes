#ifndef _GAMECUBE_TO_NES_H_INCLUDED
#define _GAMECUBE_TO_NES_H_INCLUDED

#define gamecubePWMBuffer GPIOR0

#ifndef __ASSEMBLER__
#include <stdint.h>

//
struct gamecubeInputPacket {
  // Digital inputs and status
  bool a : 1;
  bool b : 1;
  bool x : 1;
  bool y : 1;
  bool start : 1;
  bool : 1;
  bool : 1;
  bool : 1;
  bool left : 1;
  bool right : 1;
  bool down : 1;
  bool up : 1;
  bool z : 1;
  bool r : 1;
  bool l : 1;
  bool : 1;

  // Analog inputs
  uint8_t joystick_x : 8;
  uint8_t joystick_y : 8;
  uint8_t cstick_x : 8;
  uint8_t cstick_y : 8;
  uint8_t l_analog : 8;
  uint8_t r_analog : 8;
} __attribute__((packed));

//
struct nesInputPacket {
  // NES standard controller compatible subset
  bool a : 1;
  bool b : 1;
  bool z : 1;
  bool start : 1;
  bool up : 1;
  bool down : 1;
  bool left : 1;
  bool right : 1;

  // SNES standard controller compatible subset
  bool x : 1;
  bool y : 1;
  bool l : 1;
  bool r : 1;
  uint8_t id : 4;

  // Analog inputs
  uint8_t joystick_x : 4;
  uint8_t joystick_y : 4;
  uint8_t cstick_x : 4;
  uint8_t cstick_y : 4;
  uint8_t l_analog : 4;
  uint8_t r_analog : 4;

  // Revision numbers
  uint8_t hardware_rev : 8;
  uint8_t software_rev : 8;
} __attribute__((packed));
#endif

// Gamecube serial communication timing constants
#define GC_PWM_TX_PERIOD (F_CPU / 200000)
#define GC_PWM_TX_TRUE ((1 * GC_PWM_TX_PERIOD) / 4)
#define GC_PWM_TX_FALSE ((3 * GC_PWM_TX_PERIOD) / 4)
#define GC_PWM_TX_STOP ((1 * GC_PWM_TX_PERIOD) / 4)
#define GC_PWM_TX_THRESHOLD ((2 * GC_PWM_TX_PERIOD) / 4)

#define GC_PWM_RX_PERIOD (F_CPU / 250000)
#define GC_PWM_RX_TRUE ((1 * GC_PWM_RX_PERIOD) / 4)
#define GC_PWM_RX_FALSE ((3 * GC_PWM_RX_PERIOD) / 4)
#define GC_PWM_RX_STOP ((2 * GC_PWM_RX_PERIOD) / 4)
#define GC_PWM_RX_THRESHOLD ((2 * GC_PWM_RX_PERIOD) / 4)
#define GC_PWM_RX_TIMEOUT (5 * GC_PWM_RX_PERIOD)

#endif