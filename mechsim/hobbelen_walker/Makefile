SOURCES = ChainSegment.cxx FootSegment.cxx Hobbelen.cxx
HEADERS = BodyConf.h ChainSegment.h FootSegment.h Hobbelen.h
OBJS = $(SOURCES:.cxx=.o)
CFLAGS = -Wall -I../common $(shell ode-config --cflags)
LDFLAGS = -L../common
LIBS = -lode -lmechsim

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
