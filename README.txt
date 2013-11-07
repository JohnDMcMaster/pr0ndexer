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


arm-none-eabi-gcc -Os -g -Wall -Wextra -Wimplicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -Wundef -Wshadow -I../../../../../libopencm3/include -fno-common -mthumb -mcpu=cortex-m3 -msoft-float -MD -DSTM32L1 -I../../../../../libopencm3/include -o usart.o -c usart.c
arm-none-eabi-gcc -o usart.elf usart.o -lopencm3_stm32l1 --static -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group -L../../../../../libopencm3/lib -T../../../../../libopencm3/lib/stm32/l1/stm32l15xxb.ld -nostartfiles -Wl,--gc-sections -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -L../../../../../libopencm3/lib -L../../../../../libopencm3/lib/stm32/l1 -Wl,--print-gc-sections

arm-none-eabi-objcopy -Obinary usart.elf usart.bin
arm-none-eabi-objcopy -Oihex usart.elf usart.hex
arm-none-eabi-objcopy -Osrec usart.elf usart.srec
arm-none-eabi-objdump -S usart.elf > usart.list



arm-none-eabi-gcc -Os -g -Wall -Wextra -Wimplicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -Wundef -Wshadow -I/home/mcmaster/document/external/libopencm3-examples/libopencm3/include -fno-common -mthumb -mcpu=cortex-m3 -msoft-float -MD -DSTM32F1 -o usart.o -c usart.c
arm-none-eabi-gcc -o usart.elf usart.o -lopencm3_stm32f1 --static -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group -L/home/mcmaster/document/external/libopencm3-examples/libopencm3/lib -Tstm32l15xxb.ld -nostartfiles -Wl,--gc-sections -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -Wl,--print-gc-sections

arm-none-eabi-objcopy -Obinary usart.elf usart.bin
arm-none-eabi-objcopy -Oihex usart.elf usart.hex
arm-none-eabi-objcopy -Osrec usart.elf usart.srec
arm-none-eabi-objdump -S usart.elf > usart.list


