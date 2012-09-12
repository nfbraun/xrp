EIGEN_INCL = -I../../eigen-eigen-3.0.1/

.PHONY: all

all: kintree cartwheel

kintree: kintree.cxx Makefile
	g++ -O3 $(EIGEN_INCL) -o kintree KinTree.cxx kintree.cxx

cartwheel: cartwheel.cxx KinTree.cxx Makefile
	g++ -O3 -o $@ -lgsl -lblas $(EIGEN_INCL) -I../acrobot/ana ../acrobot/ana/RawODESolver.o KinTree.cxx cartwheel.cxx