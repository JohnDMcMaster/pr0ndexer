BINARY = main

LDSCRIPT = stm32vl-discovery.ld

#OBJS		+= $(BINARY).o

STLINK_PORT=localhost:4242

include Makefile.include

