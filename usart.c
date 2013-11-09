/*
USART1 TX: PA9 
USART1 RX: PA10 
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

#if 1
#define dbg(x, ...)     printf("DBG %s:%d: " x "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define dbg(...)
#endif

#define SLIP_END        192
#define SLIP_ESC        219

#define BUFFER_SIZE 1024

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
    uint8_t opcode;
    /*
    Value specific to opcode
    Should be set to 0 if unused
    */
    uint32_t value;
} __attribute__((packed)) packet_t;

typedef struct {
    /*
    Remaining steps
    All pins are initialized to 0
    Then go through high step for one cycle and then low step
    */
    int32_t step;
    //half step delay
    unsigned int hstep_dly;
    
    uint32_t      step_gpioport;
    uint32_t      step_gpios;

    uint32_t      dir_gpioport;
    uint32_t      dir_gpios;
} axis_t;


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

#define USART_CONSOLE       USART1

/******************************************************************************
 * Simple ringbuffer implementation from open-bldc's libgovernor that
 * you can find at:
 * https://github.com/open-bldc/open-bldc/tree/master/source/libgovernor
 *****************************************************************************/

typedef int32_t ring_size_t;

struct ring {
	uint8_t *data;
	ring_size_t size;
	uint32_t begin;
	uint32_t end;
};

#define RING_SIZE(RING)  ((RING)->size - 1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin == (RING)->end)


static void ring_init(struct ring *ring, uint8_t *buf, ring_size_t size);
//static int32_t ring_write_ch(struct ring *ring, uint8_t ch);
//static int32_t ring_write(struct ring *ring, uint8_t *data, ring_size_t size);
//static int32_t ring_read_ch(struct ring *ring, uint8_t *ch);
//static int32_t ring_read(struct ring *ring, uint8_t *data, ring_size_t size);

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

#if 0
static void usart_setup(void)
{
	/* Initialize output ring buffer. */
	ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 230400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);
}
#endif

static void usart_setup(void)
{
	/* Initialize output ring buffer. */
	ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);
	
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

    /* Finally enable the USART. */
    usart_enable(USART1);
}

/*
void usart_set_baudrate(uint32_t usart, uint32_t baud);
void usart_set_databits(uint32_t usart, int bits);
void usart_set_stopbits(uint32_t usart, enum usart_stopbits);
void usart_set_parity(uint32_t usart, enum usart_parity);
void usart_set_mode(uint32_t usart, enum usart_mode);
void usart_set_flow_control(uint32_t usart, enum usart_flowcontrol);
void usart_enable(uint32_t usart);
void usart_disable(uint32_t usart);
void usart_send(uint32_t usart, uint16_t data);
uint16_t usart_recv(uint32_t usart);
void usart_wait_send_ready(uint32_t usart);
void usart_wait_recv_ready(uint32_t usart);
void usart_send_blocking(uint32_t usart, uint16_t data);
uint16_t usart_recv_blocking(uint32_t usart);
void usart_enable_rx_interrupt(uint32_t usart);
void usart_disable_rx_interrupt(uint32_t usart);
*/


void ring_init(struct ring *ring, uint8_t *buf, ring_size_t size)
{
	ring->data = buf;
	ring->size = size;
	ring->begin = 0;
	ring->end = 0;
}

static int32_t ring_write_ch(struct ring *ring, uint8_t ch)
{
	if (((ring->end + 1) % ring->size) != ring->begin) {
		ring->data[ring->end++] = ch;
		ring->end %= ring->size;
		return (uint32_t)ch;
	}

	return -1;
}

static int32_t ring_write(struct ring *ring, uint8_t *data, ring_size_t size)
{
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_write_ch(ring, data[i]) < 0)
			return -i;
	}

	return i;
}

static int32_t ring_read_ch(struct ring *ring, uint8_t *ch)
{
	int32_t ret = -1;

	if (ring->begin != ring->end) {
		ret = ring->data[ring->begin++];
		ring->begin %= ring->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}

/* Not used!
static int32_t ring_read(struct ring *ring, uint8_t *data, ring_size_t size)
{
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_read_ch(ring, data + i) < 0)
			return i;
	}

	return -i;
}
*/
void usart1_isr(void)
{
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		//gpio_toggle(GPIOC, GPIO12);

		/* Retrieve the data from the peripheral. */
		ring_write_ch(&output_ring, usart_recv(USART1));

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

void service_axis(axis_t *axis) {
    int steps;
    
    if (!axis->step) {
        return;
    }
    
    if (axis->step > 0) {
        steps = axis->step > 10 ? 10 : axis->step;
        axis->step -= steps;
        gpio_clear(axis->dir_gpioport, axis->dir_gpios);
    } else {
        steps = axis->step < -10 ? 10 : -axis->step;
        axis->step += steps;
        gpio_set(axis->dir_gpioport, axis->dir_gpios);
    }
    
    //In the future will want to do some sort of timer based precision timing
    //in the meantime this will suffice
    //only do a bit a at a time to keep the serial port responsive 
    //"Factory trimmed 8 MHz RC oscillator and 40 kHz for RTC and watchdog."
    //no idea what I was stepping at before...start with this and speed it up from there
    for (int j = 0; j < steps; ++j) {
        //Delays of 8000 (16000 total) give about 370 steps/second
        //skips at 800, didn't even move
        //2000 was pretty smooth and no slipping observed
        //2200 seemed pretty smooth
        //3000 was more vibration
        //1500 seems more than 2000
        //1000 skips
        
        gpio_set(axis->step_gpioport, axis->step_gpios);
        for (int i = 0; i < axis->hstep_dly; i++) {
            __asm__("NOP");
        }
        gpio_clear(axis->step_gpioport, axis->step_gpios);
        for (int i = 0; i < axis->hstep_dly; i++) {
            __asm__("NOP");
        }
    }
}

#define XYZ_STATUS         0x00
#define XYZ_CONTROL     0x01     
//Set the step register to value (2's compliment)
#define XYZ_STEP_SET     0x02
//Adjust the step register by argument
#define XYZ_STEP_ADD     0x03
//Adjust the step register by argument
//#define XYZ_STEP_ADD    0x04
//Minimum velocity in steps/second     
#define XYZ_VELMIN         0x05
//Maximum velocity in steps/second
#define XYZ_VELMAX         0x06
//Acceleration/decceleration in steps/second**2 
#define XYZ_ACL         0x07
#define XYZ_HSTEP_DLY   0x08

#define OPCODE_WRITE    0x80

#define PACKET_INVALID 0xFF

void axis_process_command(axis_t *axis, const packet_t *packet) {
    uint8_t reg = packet->opcode & 0x1F;
    
    if (packet->opcode & OPCODE_WRITE) {
        switch (reg) {
        case XYZ_STEP_SET:
            axis->step = (int32_t)packet->value;
            dbg("axis set step: %d", (int)axis->step);
            break;
        case XYZ_STEP_ADD:
            axis->step += (int32_t)packet->value;
            dbg("axis adjust step: %d", (int)axis->step);
            break;
        case XYZ_HSTEP_DLY:
            axis->hstep_dly = packet->value;
            dbg("axis adjust hstep dly: %d", axis->hstep_dly);
            break;
        default:
            dbg("Drop packet: unknown axis reg 0x%02X (opcode 0x%02X)", reg, packet->opcode);
        }
    } else {
        dbg("Drop packet: FIXME read not implemented, reg 0x%02X (opcode 0x%02X)", reg, packet->opcode);
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
    if (packet->seq == g_seq_num && g_seq_num != SEQ_NEVER) {
        //TODO: send ack
        return;
    }
    g_seq_num = packet->seq;
    */
    
    //0x20         X block
    //0x40         Y block
    //0x60         Z block
    if ((packet->opcode & 0x60) == 0x20) {
        axis_process_command(&g_axes[0], packet);
    } else if ((packet->opcode & 0x60) == 0x40) {
        axis_process_command(&g_axes[1], packet);
    } else if ((packet->opcode & 0x60) == 0x60) {
        axis_process_command(&g_axes[2], packet);
    } else {
        dbg("unrecognized block, opcode: 0x%02X", packet->opcode);
        /*
        switch (opcode) {
        case OPCODE_
        }
        */
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
        //green led
        gpio_toggle(GPIOC, GPIO9);
        
        //Not the right size? drop it
        if (g_rx_n != sizeof(g_rx_buff)) {
            dbg("Drop packet: got %d / %d packet bytes", g_rx_n, sizeof(g_rx_buff));
        } else {
            gpio_toggle(GPIOC, GPIO9);
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
        .hstep_dly = 2000,
        
        .step_gpioport = GPIOA,
        .step_gpios = GPIO0,

        .dir_gpioport = GPIOA,
        .dir_gpios = GPIO1,
    },
    //Y
    {
        .hstep_dly = 2000,
        
        .step_gpioport = GPIOA,
        .step_gpios = GPIO2,

        .dir_gpioport = GPIOA,
        .dir_gpios = GPIO3,
    },
    //Z
    {
        .hstep_dly = 2000,
        
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
    
    /*
    DBG usart.c:381: sizeof(char): 1
    DBG usart.c:382: sizeof(short): 2
    DBG usart.c:383: sizeof(int): 4
    DBG usart.c:384: sizeof(long): 4
    */
    /*
    dbg("sizeof(char): %d", sizeof(char));
    dbg("sizeof(short): %d", sizeof(short));
    dbg("sizeof(int): %d", sizeof(int));
    dbg("sizeof(long): %d", sizeof(long));
    */
    
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
    }

    
    
    //Blink the LED (PC9) on the board with every transmitted byte
    while (1) {
        //See if ready to receive
        //void usart_wait_recv_ready(uint32_t usart)
        //Wait until the data is ready to be received
        //while ((USART_SR(usart) & USART_SR_RXNE) == 0);
        if ((USART_SR(USART1) & USART_SR_RXNE) != 0) {
            //blue led
            gpio_toggle(GPIOC, GPIO8);
                rx_char(usart_recv_blocking(USART1));
        }
        
        for (unsigned int i = 0; i < N_AXES; ++i) {
            axis_t *axis = &g_axes[i];
            
            service_axis(axis);
        }
    }
    return 0;
}

