PROJECT=dogmtool

MCU = atmega644pa
MCU_AVRDUDE = atmega644p


ifeq ($(OSTYPE),)
OSTYPE      = $(shell uname)
endif
ifneq ($(findstring Darwin,$(OSTYPE)),)
USB_DEVICE = /dev/cu.usbserial-A400eRcF
else
USB_DEVICE = /dev/ttyUSB0
endif


#########################################################################

SRC=$(wildcard *.c)
OBJECTS=$(SRC:.c=.o) 
DFILES=$(SRC:.c=.d) 
HEADERS=$(wildcard *.h)



#  Compiler Options
GCFLAGS = -mmcu=$(MCU) -I. -gstabs -DF_CPU=20000000 -O2 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes  -std=gnu99 -MD -MP
#LDFLAGS =  -Wl,-Map=pwbl.map,--cref    -lm -Wl,--section-start=.text=0x1800
LDFLAGS = -mmcu=$(MCU)  

#  Compiler Paths
GCC = avr-gcc
REMOVE = rm -f
SIZE = avr-size
OBJCOPY = avr-objcopy

#########################################################################

all: $(PROJECT).hex Makefile stats

$(PROJECT).hex: $(PROJECT).tmphex Makefile
	./crypt $(PROJECT).tmphex $(PROJECT).hex 

$(PROJECT).tmphex: $(PROJECT).elf Makefile
	$(OBJCOPY) -R .eeprom -O ihex $(PROJECT).elf $(PROJECT).tmphex 

$(PROJECT).elf: $(OBJECTS) Makefile
	$(GCC) $(LDFLAGS) $(OBJECTS) -o $(PROJECT).elf

stats: $(PROJECT).elf Makefile
	$(SIZE)  -C --mcu=$(MCU)  $(PROJECT).elf

clean:
	$(REMOVE) $(OBJECTS)
	$(REMOVE) $(PROJECT).hex
	$(REMOVE) $(PROJECT).tmphex
	$(REMOVE) $(DFILES)
	$(REMOVE) $(PROJECT).elf

#########################################################################

%.o: %.c Makefile $(HEADERS)
	$(GCC) $(GCFLAGS) -o $@ -c $<

#########################################################################

flash: all
	avrdude -F -p $(MCU_AVRDUDE) -P $(USB_DEVICE) -c avr109  -b 115200  -U flash:w:$(PROJECT).hex


