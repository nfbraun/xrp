CFLAGS = -I../common $(shell ode-config --cflags)
LDFLAGS = -L../common
LIBS = -lode -lmechsim

.PHONY: all

all: simprint simview

simprint: Acrobot.cxx Acrobot.h simprint.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Acrobot.cxx simprint.cxx

simview: Acrobot.cxx Acrobot.h simview.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Acrobot.cxx simview.cxx

