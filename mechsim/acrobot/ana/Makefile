HEADERS = Lagrange.h ODEArray.h RawODESolver.h ODESolver.h Acrobot.h
OBJS = Lagrange.o RawODESolver.o ODESolver.o Acrobot.o
GIJIT_INCL = -I../../common/gijit
GIJIT_LIBS = -L../../common/gijit -lgijit
OCTAVE_LIBS = -L/usr/lib/octave-3.2.3 -loctave -loctinterp

.PHONY: clean

acrobot: acrobot.o $(OBJS) Octave.o LQR.o Makefile
	g++ -O3 -o acrobot -lgsl $(GIJIT_LIBS) $(OCTAVE_LIBS) acrobot.o $(OBJS) Octave.o LQR.o

energy: energy.o $(OBJS) Makefile
	g++ -O3 -o energy -lgsl $(GIJIT_LIBS) $(OCTAVE_LIBS) energy.o $(OBJS)

nauttest: nauttest.o $(OBJS) Makefile
	g++ -O3 -o nauttest -lgsl -lblas $(GIJIT_LIBS) nauttest.o $(OBJS)

octtest: octtest.o Octave.h Octave.o
	g++ -g -o octtest $(OCTAVE_LIBS) -lginac Octave.o octtest.o

%.o: %.cxx $(HEADERS) Makefile
	g++ -g $(GIJIT_INCL) -c $<

clean:
	rm -f *.o
	rm -f acrobot energy nauttest octtest
