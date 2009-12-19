MCU=atmega8
MCUFREQ=-DF_CPU=8000000UL
POSTSTATE=noreset
PROGTYPE=dapa
SOURCES=blink.c
LIBDIR=../lib
# LIBS=$(LIBDIR)/serial.c
INCDIR=../include
TARGET=blink

.PHONY: build upload

build: $(TARGET).hex

upload: build
	avrdude -p $(MCU) -c $(PROGTYPE) -E $(POSTSTATE) -U flash:w:$(TARGET).hex

$(TARGET).elf: $(SOURCES) $(HEADERS) Makefile
	avr-gcc -I$(INCDIR) $(MCUFREQ) -Os -o $@ -mmcu=$(MCU) $(LIBS) $(SOURCES)
	
$(TARGET).hex: $(TARGET).elf Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
