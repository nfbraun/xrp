MCU=atmega8
MCUFREQ=-DF_CPU=8000000UL
PROGTYPE=butterfly
BAUDRATE=38400
SOURCES=pendulum.c
LIBDIR=../lib
LIBS=motor.c twi.c ssi.c serial.c scons.c
INCDIR=../include
HEADERS=bitset.h motor.h serial.h twi.h ssi.h
TARGET=pendulum

.PHONY: build upload

build: $(TARGET).hex

upload: build
	avrdude -p $(MCU) -c $(PROGTYPE) -b $(BAUDRATE) -U flash:w:$(TARGET).hex

$(TARGET).elf: $(SOURCES) $(HEADERS:%=$(INCDIR)/%) Makefile
	avr-gcc -I$(INCDIR) $(MCUFREQ) -Os -o $@ -mmcu=$(MCU) $(SOURCES) $(LIBS:%=$(LIBDIR)/%)
	
$(TARGET).hex: $(TARGET).elf Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
