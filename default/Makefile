###############################################################################
# Makefile for the project AT861USB0
###############################################################################

## General Flags
PROJECT = AT861USB0
MCU = attiny861
TARGET = AT861USB0.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                 -DF_CPU=16500000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=AT861USB0.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"D:\Operator1\20100115 USB NANOSENSOR\02 SOFTWARE\AT861USB7 ALL ADC real pcb\vusb-20090822\usbdrv" -I"D:\Operator1\20100115 USB NANOSENSOR\02 SOFTWARE\AT861USB7 ALL ADC real pcb\." 

## Objects that must be built in order to link
OBJECTS = AT861USB0.o oddebug.o usbdrv.o usbdrvasm.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) AT861USB0.hex AT861USB0.eep AT861USB0.lss size

## Compile
AT861USB0.o: ../AT861USB0.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

oddebug.o: ../vusb-20090822/usbdrv/oddebug.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usbdrv.o: ../vusb-20090822/usbdrv/usbdrv.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usbdrvasm.o: ../vusb-20090822/usbdrv/usbdrvasm.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) AT861USB0.elf dep/* AT861USB0.hex AT861USB0.eep AT861USB0.lss AT861USB0.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

