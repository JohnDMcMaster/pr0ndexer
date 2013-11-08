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
#define SLIP_END        192
#define SLIP_ESC        219


#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

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

//A packet is invalid if it doesn't end by the time buffer is consumed
static uint8_t g_rx_buff[sizeof(packet_t)];
//Negative value indicates no packet is in progress
int g_rx_n;

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
    /* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

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

static void gpio_setup(void)
{
    /* Set GPIO9 (in GPIO port C) to 'output push-pull'. [LED] */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
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

#define USART_CONSOLE       USART1


//for printf
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

int main(void) {
    clock_setup();
    gpio_setup();
    usart_setup();
    
    //printf("\nHello, world!\n");
    //no packet yet
    g_rx_n = -1;

    //Blink the LED (PC9) on the board with every transmitted byte
    while (1) {
        //See if ready to receive
        //void usart_wait_recv_ready(uint32_t usart)
	    //Wait until the data is ready to be received
	    //while ((USART_SR(usart) & USART_SR_RXNE) == 0);
        if ((USART_SR(USART1) & USART_SR_RXNE) != 0) {
            //blue led
            gpio_toggle(GPIOC, GPIO8);
            
            uint16_t c = usart_recv_blocking(USART1);
            //usart_send_blocking(USART1, c);
            printf("Force RX: 0x%04X\n", c);
            
        }

        //green led
        gpio_toggle(GPIOC, GPIO9);
        //Wait a bit
        for (int i = 0; i < 800000; i++)
            __asm__("NOP");
    }
    return 0;
}

/*
int main(void)
{
    int i, j = 0, c = 0;

    clock_setup();
    gpio_setup();
    usart_setup();

    //Blink the LED (PC9) on the board with every transmitted byte
    while (1) {
        //LED on/off
        gpio_toggle(GPIOC, GPIO9);    
        //USART1: Send byte
        usart_send_blocking(USART1, c + '0');
        //Increment c
        c = (c == 9) ? 0 : c + 1;    
        //Newline after line full
        if ((j++ % 80) == 0) {        
            usart_send_blocking(USART1, '\r');
            usart_send_blocking(USART1, '\n');
        }
        //Wait a bit
        for (i = 0; i < 800000; i++)
            __asm__("NOP");
    }

    return 0;
}
*/
