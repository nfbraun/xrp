MCU=atmega8
MCUFREQ=-DF_CPU=8000000UL
SERPORT=/dev/ttyUSB0
PROGTYPE=butterfly
BAUDRATE=38400
# SOURCES=motor_test_2.c speedctrl.c
SOURCES=pendulum.c speedctrl.c
# SOURCES=main.c speedctrl.c
LIBDIR=../lib
LIBS=motor.c twi.c ssi.c serial.c scons.c serencode.c adc.c ir.c
INCDIR=../include
HEADERS=bitset.h motor.h serial.h twi.h ssi.h serencode.h adc.h
TARGET=main

.PHONY: build upload

build: $(TARGET).hex

upload: build
	avrdude -P $(SERPORT) -p $(MCU) -c $(PROGTYPE) -b $(BAUDRATE) -U flash:w:$(TARGET).hex

$(TARGET).elf: $(SOURCES) $(HEADERS:%=$(INCDIR)/%) Makefile
	avr-gcc -I$(INCDIR) $(MCUFREQ) -Os -o $@ -mmcu=$(MCU) $(SOURCES) $(LIBS:%=$(LIBDIR)/%)
	
$(TARGET).hex: $(TARGET).elf Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
