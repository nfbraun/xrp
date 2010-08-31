HEADERS = Lagrange.h ODEGenerator.h Acrobot.h
OBJS = Lagrange.o ODEGenerator.o Acrobot.o
TARGET = acrobot
GENERATOR = acrobot_gen
GEN_INTERMEDIATE = _acrobot_intermed

OCTAVELIBS = -L/usr/lib/octave-3.2.3 -loctave -loctinterp

.PHONY: all

all: $(TARGET) linear octtest

octtest: octtest.o Octave.h Octave.o
	g++ -g -o octtest $(OCTAVELIBS) Octave.o octtest.o

linear: linear.o $(HEADERS) $(OBJS) ODESolver.o Octave.o Makefile
	g++ -O3 -o linear -lginac -lgsl $(OCTAVELIBS) linear.o ODESolver.o Octave.o $(OBJS)

$(TARGET): $(TARGET).o ODESolver.h ODESolver.o $(GEN_INTERMEDIATE).o Makefile
	g++ -O3 -o $(TARGET) -lgsl -lblas $(TARGET).o $(GEN_INTERMEDIATE).o ODESolver.o

$(GENERATOR): $(GENERATOR).o $(HEADERS) $(OBJS) Makefile
	g++ -O3 -o $(GENERATOR) -lginac $(GENERATOR).o $(OBJS)

$(GEN_INTERMEDIATE).cxx: $(GENERATOR)
	./$(GENERATOR) > $(GEN_INTERMEDIATE).cxx

%.o: %.cxx $(HEADERS) Makefile
	g++ -g -c $<

