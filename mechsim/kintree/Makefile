EIGEN_INCL = -I../../eigen-eigen-3.0.1/

.PHONY: all

all: kintree cartwheel rne_crb_test

kintree: test.o KinTree.o Makefile
	g++ -O3 -o $@ KinTree.o test.o

cartwheel: cartwheel.o CWJoints.o KinTree.o Makefile
	g++ -O3 -o $@ -lgsl -lblas ../acrobot/ana/RawODESolver.o KinTree.o CWJoints.o cartwheel.o

rne_crb_test: rne_crb_test.o KinTree.o CWJoints.o Spatial.o RNE_CRB.o Makefile
	g++ -o $@ KinTree.o CWJoints.o Spatial.o RNE_CRB.o rne_crb_test.o

%.o: %.cxx Makefile
	g++ $(EIGEN_INCL) -O3 -I../acrobot/ana -o $@ -c $<
