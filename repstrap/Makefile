MCU=atmega8
MCUFREQ=-DF_CPU=8000000UL
SERPORT=/dev/ttyUSB0
PROGTYPE=butterfly
BAUDRATE=38400
SOURCES=first.c #pcctrl.c
TARGET=pcctrl

.PHONY: build upload

build: $(TARGET).hex

upload: build
	avrdude -P $(SERPORT) -p $(MCU) -c $(PROGTYPE) -b $(BAUDRATE) -U flash:w:$(TARGET).hex

$(TARGET).elf: $(SOURCES) $(HEADERS) Makefile
	avr-gcc -Os -o $@ -mmcu=$(MCU) $(MCUFREQ) $(SOURCES)
	
$(TARGET).hex: $(TARGET).elf Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
