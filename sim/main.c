#include <stdio.h>

void gpio_clear(int port, int pin);
void gpio_set(int port, int pin);

#include "../controller.h"
#include "../controller.c"

axis_t axes[3] = {
    //X
    {
        //.hstep_dly = 2000,
        
        .step = 0,
        .stepped = 0,
        .velocity = 0,
        
        .step_gpioport = 'A',
        .step_gpios = 0,

        .dir_gpioport = 'A',
        .dir_gpios = 1,
    },
    //Y
    {
        //.hstep_dly = 2000,
        
        .step = 0,
        .stepped = 0,
        .velocity = 0,
        
        .step_gpioport = 'A',
        .step_gpios = 2,

        .dir_gpioport = 'A',
        .dir_gpios = 3,
    },
    //Z
    {
        //.hstep_dly = 2000,
        
        .step = 0,
        .stepped = 0,
        .velocity = 0,
        
        .step_gpioport = 'A',
        .step_gpios = 4,

        .dir_gpioport = 'A',
        .dir_gpios = 5,
    },
};


void gpio_clear(int port, int pin) {
}

void gpio_set(int port, int pin) {
}

int g_nop_delay_hstep_dly = -1;
void nop_delay(unsigned hstep_dly) {
    g_nop_delay_hstep_dly = hstep_dly;
}

int main(void) {
    axis_t *axis = &axes[0];
   
    g_accel = 325;
    g_velmin = 325;
    g_velmax = 9250;
    g_hstep_c = 740000;
    
    axis->step = 100;
    
    int i = 0;
    printf("%4s %8s %8s %8s\n", "i", "step", "vel", "hstp_dly");
    while (true) {
        printf("% 4d % 8d % 8d % 8d\n", i, axis->step, axis->velocity, g_nop_delay_hstep_dly);
        if (axis->step == 0) {
            break;
        }
        g_nop_delay_hstep_dly = -1;
        service_axis_loop(axis);
        i += 1;
    }
    return 0;
}

