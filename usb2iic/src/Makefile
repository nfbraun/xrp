# Makefile
XCC=sdcc
XCCFLAGS=-I../include --no-xinit-opt
XLDFLAGS=-L../lib -mmcs51 --no-xinit-opt --code-loc 0x0000 --code-size 0x1800 --xram-loc 0x1800 --xram-size 0x0800
LIBS=libfx2.lib
XAS=asx8051
RELS=vectors.rel main.rel usb_descriptors.rel _startup.rel

.PHONY: upload

main.ihx: $(RELS) Makefile
	$(XCC) $(XLDFLAGS) $(LIBS) -o main.ihx -mmcs51 $(RELS)
	
upload: main.ihx
	scp main.ihx kiwi.ikp.uni-koeln.de:xrp-1/fx2/main.ihx
	ssh kiwi.ikp.uni-koeln.de "/home/braun/bin/cycfx2prog prg:xrp-1/fx2/main.ihx run"
	
main.rel: main.c Makefile
	$(XCC) $(XCCFLAGS) -mmcs51 -c main.c
	
%.rel: %.a51 Makefile
	$(XAS) -losg $<
	
