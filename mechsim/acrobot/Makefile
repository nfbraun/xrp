CFLAGS = -I../common/mechsim -O3 -I../../eigen-eigen-3.0.1 $(shell ode-config --cflags)
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim

.PHONY: all

all: simprint simview

simprint: Acrobot.cxx Acrobot.h simprint.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Acrobot.cxx simprint.cxx

simview: Acrobot.cxx Acrobot.h simview.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Acrobot.cxx simview.cxx

