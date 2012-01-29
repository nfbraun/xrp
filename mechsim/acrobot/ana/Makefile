HEADERS = Lagrange.h ODEArray.h RawODESolver.h ODESolver.h Acrobot.h
OBJS = Lagrange.o RawODESolver.o ODESolver.o Acrobot.o
GIJIT_INCL = -I../../common/gijit
GIJIT_LIBS = -L../../common/gijit -lgijit
OCTAVE_LIBS = -L/usr/lib/octave-3.2.3 -loctave -loctinterp
EIGEN_INCL = -I../../../eigen-eigen-3.0.1/

.PHONY: clean

tracking: tracking.o $(OBJS) CSVFile.o GSLInterp.o LTVLQR.o Makefile
	g++ -O3 -o tracking -lgsl -lblas $(GIJIT_LIBS) tracking.o CSVFile.o GSLInterp.o LTVLQR.o $(OBJS)

ene_capt: ene_capt.o $(OBJS) Octave.o LQR.o Makefile
	g++ -O3 -o ene_capt -lgsl $(GIJIT_LIBS) $(OCTAVE_LIBS) ene_capt.o $(OBJS) Octave.o LQR.o

free_acrobot: free_acrobot.o $(OBJS) Makefile
	g++ -O3 -o free_acrobot -lgsl -lblas $(GIJIT_LIBS) free_acrobot.o $(OBJS)

energy: energy.o $(OBJS) Makefile
	g++ -O3 -o energy -lgsl $(GIJIT_LIBS) $(OCTAVE_LIBS) energy.o $(OBJS)

nauttest: nauttest.o $(OBJS) Makefile
	g++ -O3 -o nauttest -lgsl -lblas $(GIJIT_LIBS) nauttest.o $(OBJS)

octtest: octtest.o Octave.h Octave.o
	g++ -g -o octtest $(OCTAVE_LIBS) -lginac Octave.o octtest.o

%.o: %.cxx $(HEADERS) Makefile
	g++ -g $(GIJIT_INCL) $(EIGEN_INCL) -c $<

clean:
	rm -f *.o
	rm -f tracking ene_capt free_acrobot energy nauttest octtest
