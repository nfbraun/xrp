EIGEN_INCL = -I../../eigen-eigen-3.2.0/

.PHONY: all

all: kintree cartwheel cartwheel2 rne_crb_test

kintree: test.o KinTree.o Makefile
	g++ -O3 -o $@ KinTree.o test.o

cartwheel: cartwheel.o CWJoints.o KinTree.o Makefile
	g++ -O3 -o $@ ../acrobot/ana/RawODESolver.o KinTree.o CWJoints.o cartwheel.o -lgsl -lblas

cartwheel2: cartwheel2.o Spatial.o RNE_CRB.o Robot.o Makefile
	g++ -O3 -o $@ ../acrobot/ana/RawODESolver.o Spatial.o RNE_CRB.o Robot.o cartwheel2.o -lgsl -lblas

invdyn: invdyn.o CSVFile.o Robot.o Spatial.o RNE_CRB.o
	g++ -O3 -o $@ invdyn.o CSVFile.o Robot.o Spatial.o RNE_CRB.o

invdyn2: invdyn2.o CSVFile.o Robot.o RobotLeg.o Spatial.o RNE_CRB.o
	g++ -O3 -o $@ invdyn2.o CSVFile.o Robot.o RobotLeg.o Spatial.o RNE_CRB.o

rne_crb_test: rne_crb_test.o KinTree.o CWJoints.o Spatial.o RNE_CRB.o Robot.o Makefile
	g++ -o $@ KinTree.o CWJoints.o Spatial.o RNE_CRB.o Robot.o rne_crb_test.o

%.o: %.cxx Makefile
	g++ $(EIGEN_INCL) -O3 -I../acrobot/ana -o $@ -c $<
