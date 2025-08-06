#ifndef _GAMECUBE_TO_NES_H_INCLUDED
#define _GAMECUBE_TO_NES_H_INCLUDED

#define GC_PWM_PERIOD (100)
#define GC_PWM_TIMEOUT (GC_PWM_PERIOD * 5)
#define GC_PWM_TRUE ((1 * GC_PWM_PERIOD) / 4)
#define GC_PWM_FALSE ((3 * GC_PWM_PERIOD) / 4)
#define GC_PWM_STOP ((2 * GC_PWM_PERIOD) / 4)
#define gamecubePWMBuffer GPIOR0

#endif