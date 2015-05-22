Target board: STM32VL-DISCOVERY
Target MCU family: STM32F1
Target MCU: STM32F100RBT6B
My actual CPU label
	ARM7
	32F100
	RBT68
	991PU 93
	MYS 99 019
	ST


get this in your $PATH: https://launchpadlibrarian.net/151487636/gcc-arm-none-eabi-4_7-2013q3-20130916-linux.tar.bz2
start stlink: sudo st-util
    I'm using 73dccb68ed61dfe3364202dd8cd0d66bb9ec62e3

sudo st-util -1
Program to flash: make && stdl *.elf
    make flash doesn't work...?




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


