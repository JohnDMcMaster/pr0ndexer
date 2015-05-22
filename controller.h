#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

/*
WARNING WARNING WARNING
Even with the UART buffering, using the serial port to TX (printf) for some reason seems to still
cause noticible step impact
Some of this could be mitigated by switching to timer (which is what really should use)
but regardless be warned...
*/
#if 0
#define dbg(x, ...)     printf("DBG %s:%d: " x "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define dbg(...)
#endif

//Constants calculated using lowest setting of 400 steps/rev on driver
//Microstepped to 2000
//TODO: think out a little more whether the microstepping adjustment should be done here or on the PC
//probably should be done on the PC
//#define MICROSTEPPING   5
//Higher! (10000)
#define MICROSTEPPING   25

/*
Think in terms of velocity
step width determines frequency, opposite of velocity
Motor speed = k * f = k / step_period

Instead track velocity from -10,000 to +10,000
Convert at step time from velocity to step size
10,000 => 2,000
0 => infinite

step_hwidth = 2,000 * 10,000 / abs(velocity)
*/

typedef struct {
    /*
    Remaining steps
    All pins are initialized to 0
    Then go through high step for one cycle and then low step
    */
    int step;
    int stepped;
    //Target half step delay
    //unsigned int hstep_dly;
    //Used to figure out acceleration
    //negative value means going backward
    //int last_hstp_dly;
    /*
    Current steps / second
    from -370 to +370
    */
    int velocity;
    
    uint32_t      step_gpioport;
    uint32_t      step_gpios;

    uint32_t      dir_gpioport;
    uint32_t      dir_gpios;
} axis_t;

void service_axis(axis_t *axis);
void service_axis_loop(axis_t *axis);
void nop_delay(unsigned hstep_dly);

//In actual pulse units
//Step acceleration per loop
extern unsigned int g_accel;
//Min velocity
extern unsigned int g_velmin;
//Max velocity
extern unsigned int g_velmax;
//Timing constant used to convert velocity to step widths
//See above for details
extern unsigned int g_hstep_c;


#endif

