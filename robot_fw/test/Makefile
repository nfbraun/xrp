MCU=atmega8
MCUFREQ=-DF_CPU=8000000UL
SERPORT=/dev/ttyUSB0
PROGTYPE=butterfly
BAUDRATE=38400
SOURCES=adc.c
LIBDIR=../lib
LIBS=$(LIBDIR)/serial.c $(LIBDIR)/serencode.c
# LIBS=$(LIBDIR)/serial.c $(LIBDIR)/serencode.c $(LIBDIR)/twi.c
INCDIR=../include
TARGET=adc

.PHONY: build upload

build: $(TARGET).hex

upload: build
	avrdude -P $(SERPORT) -p $(MCU) -c $(PROGTYPE) -b $(BAUDRATE) -U flash:w:$(TARGET).hex

$(TARGET).elf: $(SOURCES) $(HEADERS) Makefile
	avr-gcc -I$(INCDIR) $(MCUFREQ) -Os -o $@ -mmcu=$(MCU) $(LIBS) $(SOURCES)

$(TARGET).hex: $(TARGET).elf Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
