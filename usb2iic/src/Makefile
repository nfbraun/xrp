# Makefile
XCC=sdcc
XCCFLAGS=-I../include --no-xinit-opt
XLDFLAGS=-L../lib -mmcs51 --no-xinit-opt --code-loc 0x0000 --code-size 0x1800 --xram-loc 0x1800 --xram-size 0x0800
LIBS=libfx2.lib
XAS=asx8051
RELS=vectors.rel main.rel usb_descriptors.rel _startup.rel
PROG_HOST=rubia.ikp.uni-koeln.de

.PHONY: upload

main.ihx: $(RELS) Makefile
	$(XCC) $(XLDFLAGS) $(LIBS) -o main.ihx -mmcs51 $(RELS)
	
upload: main.ihx
	scp main.ihx $(PROG_HOST):xrp-1/usb2iic/fw_bin/main.ihx
	ssh $(PROG_HOST) "/home/braun/bin/cycfx2prog prg:xrp-1/usb2iic/fw_bin/main.ihx run"
	
main.rel: main.c Makefile
	$(XCC) $(XCCFLAGS) -mmcs51 -c main.c
	
%.rel: %.a51 Makefile
	$(XAS) -losg $<
	
