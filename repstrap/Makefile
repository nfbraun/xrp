MCU=atmega8
POSTSTATE=noreset
PROGTYPE=dapa
SOURCES=pcctrl.c
TARGET=pcctrl

.PHONY: build upload

build: $(TARGET).hex

upload: build
	avrdude -p $(MCU) -c $(PROGTYPE) -E $(POSTSTATE) -U flash:w:$(TARGET).hex

$(TARGET).elf: $(SOURCES) $(HEADERS) Makefile
	avr-gcc -Os -o $@ -mmcu=$(MCU) $(SOURCES)
	
$(TARGET).hex: $(TARGET).elf Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
