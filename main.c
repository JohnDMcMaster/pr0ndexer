/*
Relavent documents:
-UM0919 User Manual STM32VLDISCOVERY STM32 value line Discovery 
    24 pages
    Details on crystals used, physical board layout, etc
    CD00267113.pdf
-Chip reference manual
    Details on package pinout
    CD00251732.pdf
-RM0041 Reference manual STM32F100xx advanced ARM-based 32-bit MCUs
    Details on peripherals like the USARTs
    http://www.st.com/web/en/resource/technical/document/reference_manual/CD00246267.pdf
-Cortex M3 manual
    Details on CPU instructions
        384 pages
    Does not contain information on peripherals
    http://infocenter.arm.com/help/topic/com.arm.doc.ddi0337e/DDI0337E_cortex_m3_r1p1_trm.pdf

USART1 TX: PA9 
USART1 RX: PA10

X step: PA0,
X dir:  PA1
Y step: PA2
Y dir:  PA3
Z step: PA4
Z dir:  PA5

USART_CR1: RM0041 pg 611
USART_CR2: RM0041 pg 613
USART_CR3: RM0041 pg 614 
*/

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
"The SLIP protocol defines two special characters: END and ESC. END is
octal 300 (decimal 192) and ESC is octal 333 (decimal 219) not to be
confused with the ASCII ESCape character; for the purposes of this
discussion, ESC will indicate the SLIP ESC character.  To send a
packet, a SLIP host simply starts sending the data in the packet.  If
a data byte is the same code as END character, a two byte sequence of
ESC and octal 334 (decimal 220) is sent instead.  If it the same as
an ESC character, an two byte sequence of ESC and octal 335 (decimal
221) is sent instead.  When the last byte in the packet has been
sent, an END character is then transmitted."
*/

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "controller.h"

#define SLIP_END        192
#define SLIP_ESC        219

#define BUFFER_SIZE 1024

#define toggle_blue()   gpio_toggle(GPIOC, GPIO8)
#define toggle_green()   gpio_toggle(GPIOC, GPIO9)

typedef struct {
    /*
    inverted modular sum checksum
    maybe consider CRC later
    */
    uint8_t checksum;
    /*
    Monotonically increasing sequence number
    */
    uint8_t seq;
    /*
    High bit indicates write
    */
    uint8_t reg;
    /*
    Value specific to reg
    Should be set to 0 if unused
    */
    uint32_t value;
} __attribute__((packed)) packet_t;



//A packet is invalid if it doesn't end by the time buffer is consumed
static uint8_t g_rx_buff[sizeof(packet_t)];
//large value indicates no packet is in progress
unsigned int g_rx_n;
//Last received sequence number of 0x100 if never
#define SEQ_NEVER       0x100
unsigned int g_seq_n;
axis_t g_axes[3];
#define N_AXES      3
bool g_escape;

struct ring output_ring;
uint8_t output_ring_buffer[BUFFER_SIZE];

struct ring input_ring;
uint8_t input_ring_buffer[BUFFER_SIZE];

#define USART_CONSOLE       USART1

#define XYZ_STATUS          0x00
#define XYZ_CONTROL         0x01     
//Set the step register to value (2's compliment)
#define XYZ_STEP_SET        0x02
//Adjust the step register by argument
#define XYZ_STEP_ADD        0x03
//Adjust the step register by argument
//#define XYZ_STEP_ADD    0x04
//Minimum velocity in steps/second     
#define XYZ_VELMIN          0x05
//Maximum velocity in steps/second
#define XYZ_VELMAX          0x06
//Acceleration/decceleration in steps/second**2 
#define XYZ_ACL             0x07
#define XYZ_HSTEP_C         0x08
#define XYZ_NET_STEP        0x09
//#define XYZ_USTEP           0x0A

//not really a register
#define REG_NOP             0x03
#define REG_ACK             0x02

#define REG_WRITE           0x80

#define PACKET_INVALID      0xFF

#define SEQ_NEVER           0x100
unsigned int g_seq_num_rx = SEQ_NEVER;
uint8_t g_checksum_rx;
unsigned int g_seq = 0;


#include "ring.h"
#include "ring.c"
#include "controller.c"

uint8_t checksum(const uint8_t *data, size_t data_size) {
    uint8_t ret = 0;
    for (unsigned int i = 0; i < data_size; ++i) {
        ret += data[i];
    }
    return ~ret;
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_24mhz();

    /* Enable GPIOC clock. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);

    /* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
}

static void usart_setup(void)
{
    /* Initialize output ring buffer. */
    ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);
    ring_init(&input_ring, input_ring_buffer, BUFFER_SIZE);
    
    /* Enable the USART1 interrupt. */
    nvic_enable_irq(NVIC_USART1_IRQ);
    
    /* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    /* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
              GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    /* Setup UART parameters. */
    // usart_set_baudrate(USART1, 38400);
    /* TODO usart_set_baudrate() doesn't support 24MHz clock (yet). */
    /* This is the equivalent: */
    USART_BRR(USART1) = (uint16_t)((24000000 << 4) / (38400 * 16));

    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Enable USART1 Receive interrupt. */
    USART_CR1(USART1) |= USART_CR1_RXNEIE;
    
    /* Finally enable the USART. */
    usart_enable(USART1);
}

void usart1_isr(void) {
    /* Check if we were called because of RXNE. */
    /*
    Bit 5 RXNEIE: RXNE interrupt enable
    This bit is set and cleared by software.
    0: Interrupt is inhibited
    1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR register
    
    whats the point of checking if the interrupt is enabled if one is active?
    */
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        toggle_green();
        
        /* Indicate that we got data. */
        //gpio_toggle(GPIOC, GPIO12);

        /* Retrieve the data from the peripheral. */
        ring_write_ch(&input_ring, usart_recv(USART1));

        /* Enable transmit interrupt so it sends back the data. */
        USART_CR1(USART1) |= USART_CR1_TXEIE;
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

        int32_t data;

        data = ring_read_ch(&output_ring, NULL);

        if (data == -1) {
            /* Disable the TXE interrupt, it's no longer needed. */
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        } else {
            /* Put data into the transmit register. */
            usart_send(USART1, data);
        }
    }
}

int _write(int file, char *ptr, int len)
{
    int ret;

    if (file == 1) {
        ret = ring_write(&output_ring, (uint8_t *)ptr, len);

        if (ret < 0)
            ret = -ret;

        USART_CR1(USART1) |= USART_CR1_TXEIE;

        return ret;
    }

    errno = EIO;
    return -1;
}

void nop_delay(unsigned hstep_dly) {
    for (int i = 0; i < hstep_dly; i++) {
        __asm__("NOP");
    }
}

void packet_write(uint8_t reg, uint32_t value) {
    packet_t packet;
    uint8_t *packetb = (uint8_t *)&packet;

    packet.seq = g_seq++;
    packet.reg = reg;
    packet.value = value;
    packet.checksum = checksum(packetb + 1, sizeof(packet) - 1);

    ring_write_ch(&output_ring, SLIP_END);
    for (unsigned int i = 0; i < sizeof(packet); ++i) {
        uint8_t b = packetb[i];
        
        if (b == SLIP_END) {
            //If a data byte is the same code as END character, a two byte sequence of
            //ESC and octal 334 (decimal 220) is sent instead.  
            ring_write_ch(&output_ring, SLIP_ESC);
            ring_write_ch(&output_ring, 220);
        } else if (b == SLIP_ESC) {
            //If it the same as an ESC character, an two byte sequence of ESC and octal 335 (decimal
            //221) is sent instead
            ring_write_ch(&output_ring, SLIP_ESC);
            ring_write_ch(&output_ring, 221);
        } else {
            ring_write_ch(&output_ring, b);
        }
    }
    //When the last byte in the packet has been
    //sent, an END character is then transmitted
    ring_write_ch(&output_ring, SLIP_END);
    
    //Start blasting out chars if not already doing so
    USART_CR1(USART1) |= USART_CR1_TXEIE;
}

void axis_process_command(axis_t *axis, const packet_t *packet) {
    uint8_t reg = packet->reg & 0x1F;
    
    if (packet->reg & REG_WRITE) {
        switch (reg) {
        case XYZ_STEP_SET:
            axis->step = (int32_t)packet->value;
            dbg("axis set step: %d", (int)axis->step);
            break;
        case XYZ_STEP_ADD:
            axis->step += (int32_t)packet->value;
            dbg("axis adjust step: %d", (int)axis->step);
            break;

        //XXX: global, should be per axis?
        case XYZ_ACL:
            g_accel = packet->value;
            break;
        case XYZ_VELMIN:
            g_velmin = packet->value;
            break;
        case XYZ_VELMAX:
            g_velmax = packet->value;
            break;
        case XYZ_HSTEP_C:
            g_hstep_c = packet->value;
            break;
        default:
            dbg("Drop packet: unknown axis reg 0x%02X (reg 0x%02X)", reg, packet->reg);
        }
    } else {
        //dbg("Drop packet: FIXME read not implemented, reg 0x%02X (reg 0x%02X)", reg, packet->reg);
        switch (reg) {
        case XYZ_STEP_SET:
            //Number of steps outstanding
            packet_write(packet->reg, (axis->step));
            dbg("axis step read: %d", (int)axis->step);
            break;
        case XYZ_NET_STEP:
            //Number of steps completed + outstanding
            packet_write(packet->reg, axis->step + axis->stepped);
            dbg("axis net step read");
            break;

        case XYZ_ACL:
            packet_write(packet->reg, g_accel);
            break;
        case XYZ_VELMIN:
            packet_write(packet->reg, g_velmin);
            break;
        case XYZ_VELMAX:
            packet_write(packet->reg, g_velmax);
            break;
        case XYZ_HSTEP_C:
            packet_write(packet->reg, g_hstep_c);
            break;
        default:
            dbg("Drop packet: unknown axis reg 0x%02X (reg 0x%02X)", reg, packet->reg);
        }
    }
}

void process_command(void) {
    const packet_t *packet = (const packet_t *)&g_rx_buff;
    uint8_t computed_checksum;
    
    //Verify checksum
    computed_checksum = checksum(g_rx_buff + 1, sizeof(g_rx_buff) - 1);
    if (packet->checksum != computed_checksum) {
        //nope!
        dbg("Drop packet: checksum mismatch, got: 0x%02X, compute: 0x%02X", packet->checksum, computed_checksum);
        return;
    }
    
    /*
    //Retransmit?
    if (packet->seq == g_seq_num_rx) {
        if (packet->checksum != g_checksum_rx) {
            //TODO: add flags in upper value bits
            packet_write(REG_ACK, g_seq_num_rx);
        } else {
            //Don't actually execute the command, just retransmit ack
            //TODO: add flags in upper value bits
            packet_write(REG_ACK, g_seq_num_rx);
        return;
    }
    */
    g_seq_num_rx = packet->seq;
    g_checksum_rx = packet->checksum;
    //TODO: add flags in upper value bits
    packet_write(REG_ACK, g_seq_num_rx);
    
    //0x20         X block
    //0x40         Y block
    //0x60         Z block
    if ((packet->reg & 0x60) == 0x20) {
        axis_process_command(&g_axes[0], packet);
    } else if ((packet->reg & 0x60) == 0x40) {
        axis_process_command(&g_axes[1], packet);
    } else if ((packet->reg & 0x60) == 0x60) {
        axis_process_command(&g_axes[2], packet);
    } else {
        dbg("unrecognized block, reg: 0x%02X", packet->reg);
    }
}

void rx_char(uint8_t c) {
    dbg("RX char 0x%02X, cur size: %d", c, g_rx_n);
    //usart_send_blocking(USART1, c);
    //printf("Force RX: 0x%04X\n", c);
    if (g_escape) {
        //enough room?
        if (g_rx_n >= sizeof(g_rx_buff)) {
            dbg("overflow");
            g_rx_n = PACKET_INVALID;
        } else {
            //If a data byte is the same code as END character, a two byte sequence of
            //ESC and octal 334 (decimal 220) is sent instead.  
            if (c == 220) {
                g_rx_buff[g_rx_n++] = SLIP_END;
            //If it the same as an ESC character, an two byte sequence of ESC and octal 335 (decimal
            //221) is sent instead
            } else if (c == 221) {
                g_rx_buff[g_rx_n++] = SLIP_ESC;
            } else {
                g_rx_n = PACKET_INVALID;
            }
        }
        g_escape = false;
    } else if (c == SLIP_END) {
        dbg("end RX");
        
        //Not the right size? drop it
        if (g_rx_n != sizeof(g_rx_buff)) {
            dbg("Drop packet: got %d / %d packet bytes", g_rx_n, sizeof(g_rx_buff));
        } else {
            toggle_blue();
            process_command();
        }
        g_rx_n = 0;
        g_escape = false;
    } else if (c == SLIP_ESC) {
        dbg("escape RX");
        g_escape = true;
    //Ordinary character
    } else {
        //enough room?
        if (g_rx_n >= sizeof(g_rx_buff)) {
            g_rx_n = PACKET_INVALID;
        } else {
            g_rx_buff[g_rx_n++] = c;
        }
    }
}

const axis_t axes_init[3] = {
    //X
    {
        //.hstep_dly = 2000,
        
        .step = 0,
        .stepped = 0,
        
        .step_gpioport = GPIOA,
        .step_gpios = GPIO0,

        .dir_gpioport = GPIOA,
        .dir_gpios = GPIO1,
    },
    //Y
    {
        //.hstep_dly = 2000,
        
        .step = 0,
        .stepped = 0,
        
        .step_gpioport = GPIOA,
        .step_gpios = GPIO2,

        .dir_gpioport = GPIOA,
        .dir_gpios = GPIO3,
    },
    //Z
    {
        //.hstep_dly = 2000,
        
        .step = 0,
        .stepped = 0,
        
        .step_gpioport = GPIOA,
        .step_gpios = GPIO4,

        .dir_gpioport = GPIOA,
        .dir_gpios = GPIO5,
    },
};


static void gpio_setup(void)
{
    //blue lower right
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    //green
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
}

int main(void) {
    clock_setup();
    gpio_setup();
    usart_setup();
    
    //no packet yet
    g_rx_n = 0;
    g_escape = false;
    g_seq_n = SEQ_NEVER;
    for (unsigned int i = 0; i < N_AXES; ++i) {
        axis_t *axis = &g_axes[i];        
        *axis = axes_init[i];
        
        gpio_set_mode(axis->dir_gpioport, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, axis->dir_gpios);              
        gpio_clear(axis->dir_gpioport, axis->dir_gpios);
        gpio_set_mode(axis->step_gpioport, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, axis->step_gpios);              
        gpio_clear(axis->step_gpioport, axis->step_gpios);
        
        axis->velocity = 0;
    }

    //Blink the LED (PC9) on the board with every transmitted byte
    while (1) {
        //Priotize on new commands over stepping
        //otherwise we can get backlog nastiness and miss a stop command
        while (1) {
            int32_t c = ring_read_ch(&input_ring, NULL);
            if (c < 0) {
                break;
            }
            rx_char(c);
        }
        
        for (unsigned int i = 0; i < N_AXES; ++i) {
            axis_t *axis = &g_axes[i];
            
            service_axis(axis);
        }
    }
    return 0;
}

