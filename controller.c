#include "controller.h"

#include <stdbool.h>

//See header for details
int g_accel = 13 * MICROSTEPPING;
int g_velmin = 10;
int g_velmax = 370 * MICROSTEPPING;
int g_hstep_c = 740000;


/*
Looked at some more proper solutions, just have an approximation for now
http://www.ti.com/lit/an/slyt482/slyt482.pdf
fs: starting frequency
http://www.ti.com/lit/an/slva488/slva488.pdf

How to determine starting frequency?
Driver: PMC VS series
    Current: FIXME
Motor: 17PM-K318-04V
    17PM-K3** => L: 38 (1.50)
    17PM-K3 seem to have:
        rotor inertia: 50 g cm**2
        Detent torque: 11.3 mNm
No idea what load on shaft is
So lets just make up some numbers because why not
200 steps / rev
I know that 370 steps/second is a stable (1.85 RPS => 111.6 RPM)
And arbitrarily say that we can accelerate/decelerate to max speed in 0.25 second

Targetting delays down to 2,000, say up to 100x that...200,000
Don't want steps longer than say 100 ms certainly

stepvel = k / halfstep
370 = k / 2000, k = 740000
stepvel = 740000 / halfstep

Say limit to 10 steps / second min:
halfstep = 740000 / 10 = 74000

370 steps/s over 0.25s = 1480 steps/s**2
    
Velocity of 13
step, time, velocity, halfstep
1, 43, 23, 32173
2, 71, 36, 20555
3, 91, 49, 15102
4, 107, 62, 11935
5, 121, 75, 9866
6, 132, 88, 8409
7, 142, 101, 7326
8, 151, 114, 6491
9, 159, 127, 5826
10, 166, 140, 5285
11, 172, 153, 4836
12, 178, 166, 4457
13, 184, 179, 4134
14, 189, 192, 3854
15, 194, 205, 3609
16, 199, 218, 3394
17, 203, 231, 3203
18, 207, 244, 3032
19, 211, 257, 2879
20, 215, 270, 2740
21, 218, 283, 2614
22, 221, 296, 2500
23, 225, 309, 2394
24, 228, 322, 2298
25, 231, 335, 2208
26, 234, 348, 2126
27, 236, 361, 2049
28, 239, 370, 2000
*/
unsigned int vel2halfstep(int velocity) {
    //Only care about magnitude
    velocity = velocity > 0 ? velocity : -velocity;
    
    //Minimum velocity such that its even safe to switch direction at this speed
    //round off below this to avoid excessively long steps
    if (velocity < g_velmin) {
        velocity = g_velmin;
    }
    return g_hstep_c / velocity;
}

void service_axis_loop(axis_t *axis) {
    unsigned int hstep_dly;
    
    /*
    WARNING: if the user changes target while we are still moving we may not come to a good stop
    and overshoot from rotor inertia
    */
    if (axis->step == 0) {
        dbg("Reached 0 step in loop %d, velocity %d, stepped %d", j, axis->velocity, axis->stepped);
        axis->velocity = 0;
        return;
    }
    
    //Delays of 8000 (16000 total) give about 370 steps/second
    //skips at 800, didn't even move
    //2000 was pretty smooth and no slipping observed
    //2200 seemed pretty smooth
    //3000 was more vibration
    //1500 seems more than 2000
    //1000 skips        
    
    /*
    PI controller
    
    Linear function that we want to start at min velocity and end at max velocity
    Arbitrarily say min velocity is 10x the nice delay, 2000*10 = 20000
    hstep_dly = m x + c
    20000 = m * 0 + c
    20000 = c
    
    2000 = m * accel_steps + c
    2000 = m * 370/4 + 20000        
    2000 - 20000 = m * 370/4
    (2000 - 20000) / (370/4) = m
    −194.594594595 = m
    hstep_dly = -195 * steps_from_stop + 20000
    
    Using 195 gives us error of:
    hstep_dly = -195 * 370/4 + 20000 = 1962.5
    errror = 2000 - 1962.5 = 37.5
    since this is less than a step increase doesn't really matter
    large error probably from doing the multiplications
    
    TODO: consider generating LUT instead
    with only 200 elements this is much faster and probably less space
    */
    
    //Forward direction
    if (axis->step > 0) {
        if (axis->velocity <= g_velmin) {
            axis->velocity = g_velmin;
        }
        
        //Is velocity exceeding our ability to stop?
        //Note g_accel**2 is more proper but g_accel is just conservative
        if (axis->velocity >= axis->step * g_accel / 2) {
            axis->velocity -= g_accel;
        } else {
            axis->velocity += g_accel;
        }
        
        if (axis->velocity > g_velmax) {
            axis->velocity = g_velmax;
        }
    //Reverse direction
    } else {
        if (axis->velocity >= -g_velmin) {
            axis->velocity = -g_velmin;
        }
        
        //Is velocity exceeding our ability to stop?
        //Note g_accel**2 is more proper but g_accel is just conservative
        if (axis->velocity <= axis->step * g_accel) {
            axis->velocity += g_accel;
        } else {
            axis->velocity -= g_accel;
        }
        
        if (axis->velocity < -g_velmax) {
            axis->velocity = -g_velmax;
        }
    }
            
    //NOTE: just because we are targetted towards one direction doesn't mean that we have
    //switched direction yet
    //thus the direction pin is based on current velocity, not target direction
    //we are also ignoring setup times which could be problematic
    //assume we lose at most one step which is acceptable error at this time
    if (axis->velocity > 0) {
        gpio_clear(axis->dir_gpioport, axis->dir_gpios);
    } else {
        gpio_set(axis->dir_gpioport, axis->dir_gpios);
    }
    hstep_dly = vel2halfstep(axis->velocity);
    
    //Do a steps
    gpio_set(axis->step_gpioport, axis->step_gpios);
    nop_delay(hstep_dly);
    gpio_clear(axis->step_gpioport, axis->step_gpios);
    nop_delay(hstep_dly);
    
    //Register step completed, for better or worse
    //FIXME: really we should take in axis->velocity
    //since the step might not have been in the right direction
    if (axis->step > 0) {
        --axis->step;
        ++axis->stepped;
    } else {
        ++axis->step;
        --axis->stepped;
    }
}

void service_axis(axis_t *axis) {
    if (!axis->step) {
        axis->velocity = 0;
        return;
    }
    
    //In the future will want to do some sort of timer based precision timing
    //in the meantime this will suffice
    //only do a bit a at a time to keep the serial port responsive 
    //"Factory trimmed 8 MHz RC oscillator and 40 kHz for RTC and watchdog."
    //no idea what I was stepping at before...start with this and speed it up from there
    for (int j = 0; j < 10; ++j) {
        service_axis_loop(axis);
    }
}

