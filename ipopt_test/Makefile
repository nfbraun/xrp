INCLUDE_DIRS=-I/home/braun/Projects/Robot/opt/include
LIB_DIRS=-L/home/braun/Projects/Robot/opt/lib

.PHONY: all

all: brachistochrone doubleint pendulum acrobot acrodyn_deriv_test

brachistochrone: brachistochrone.cxx Brachistochrone.cxx Makefile
	g++ -o brachistochrone $(INCLUDE_DIRS) $(LIB_DIRS) -ladolc -lipopt -ldmumps_seq-4.9.2 brachistochrone.cxx Brachistochrone.cxx

doubleint: doubleint.cxx DoubleInt.cxx Makefile
	g++ -o doubleint $(INCLUDE_DIRS) $(LIB_DIRS) -ladolc -lipopt -ldmumps_seq-4.9.2 doubleint.cxx DoubleInt.cxx

pendulum: pendulum.cxx Pendulum.cxx Makefile
	g++ -O3 -I../eigen-eigen-3.0.1/ -o pendulum $(INCLUDE_DIRS) $(LIB_DIRS) -ladolc -lipopt -ldmumps_seq-4.9.2 pendulum.cxx Pendulum.cxx

acrobot: acrobot.cxx Acrobot.cxx AcroDyn.cxx CSVFile.cxx Makefile
	g++ -O3 -I../eigen-eigen-3.0.1/ -o acrobot $(INCLUDE_DIRS) $(LIB_DIRS) -ladolc -lipopt -ldmumps_seq-4.9.2 acrobot.cxx Acrobot.cxx AcroDyn.cxx CSVFile.cxx

acrodyn_deriv_test: acrodyn_deriv_test.cxx AcroDyn.cxx Makefile
	g++ -O3 -I../eigen-eigen-3.0.1/ -o acrodyn_deriv_test $(INCLUDE_DIRS) $(LIB_DIRS) -ladolc -lipopt -ldmumps_seq-4.9.2 acrodyn_deriv_test.cxx AcroDyn.cxx
