CFLAGS = -I../common $(shell ode-config --cflags)
LDFLAGS = -L../common
LIBS = -lode -lmechsim

.PHONY: all

all: simprint simview legtest lagrange

simprint: McGeer.cxx McGeer.h simprint.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) McGeer.cxx simprint.cxx

simview: McGeer.cxx McGeer.h simview.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) McGeer.cxx simview.cxx

legtest: McGeer.cxx McGeer.h legtest.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) McGeer.cxx legtest.cxx

lagrange: lagrange.c
	gcc -o lagrange -lgsl -lblas lagrange.c