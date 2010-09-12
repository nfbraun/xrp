HEADERS = Lagrange.h ODEArray.h RawODESolver.h ODESolver.h Acrobot.h
OBJS = Lagrange.o RawODESolver.o ODESolver.o Acrobot.o
TARGET = acrobot
GIJIT_INCL = -I../../common/gijit
GIJIT_LIBS = -L../../common/gijit -lgijit
OCTAVE_LIBS = -L/usr/lib/octave-3.2.3 -loctave -loctinterp

.PHONY: all clean

all: $(TARGET) nauttest linear octtest

$(TARGET): $(TARGET).o $(OBJS) Makefile
	g++ -O3 -o $(TARGET) -lgsl -lblas $(GIJIT_LIBS) $(TARGET).o $(OBJS)

nauttest: nauttest.o $(OBJS) Makefile
	g++ -O3 -o nauttest -lgsl -lblas $(GIJIT_LIBS) nauttest.o $(OBJS)

octtest: octtest.o Octave.h Octave.o
	g++ -g -o octtest $(OCTAVE_LIBS) Octave.o octtest.o

linear: linear.o $(HEADERS) Acrobot.o Lagrange.o RawODESolver.o Octave.o Makefile
	g++ -O3 -o linear -lginac -lgsl $(OCTAVE_LIBS) linear.o Acrobot.o Lagrange.o RawODESolver.o Octave.o

%.o: %.cxx $(HEADERS) Makefile
	g++ -g $(GIJIT_INCL) -c $<

clean:
	rm -f *.o
	rm -f acrobot nauttest octtest linear
