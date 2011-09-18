SOURCES = ChainSegment.cxx FootSegment.cxx Hobbelen.cxx Spline.cxx
HEADERS = BodyConf.h ChainSegment.h FootSegment.h Hobbelen.h
OBJS = $(SOURCES:.cxx=.o)
CFLAGS = -O3 -Wall -I../common/mechsim -I../../eigen-eigen-3.0.1 $(shell ode-config --cflags) -g
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim -lgsl -lblas

.PHONY: all

all: simprint simview

simprint: $(OBJS) simprint.o
	g++ -o $@ $(LDFLAGS) $(LIBS) $(OBJS) simprint.o

simview: $(OBJS) simview.o
	g++ -o $@ $(LDFLAGS) $(LIBS) $(OBJS) simview.o

springtest: springtest.o
	g++ -o $@ -lode springtest.o

pdtest: pdtest.o
	g++ -o $@ -lode pdtest.o

%.o: %.cxx $(HEADERS) Makefile
	g++ $(CFLAGS) -o $@ -c $<
