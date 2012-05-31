CFLAGS = -g -I../../../eigen-eigen-3.0.1/ -O3 -I../../common/mechsim $(shell ode-config --cflags)
LDFLAGS = -L../../common/mechsim
LIBS = -lode -lmechsim
GIJIT_INCL = -I../../common/gijit
GIJIT_LIBS = -L../../common/gijit -lgijit
EIGEN_INCL = -I../../../eigen-eigen-3.0.1/

.PHONY: all

# joint3_ana and joint4_ana do not compile
# (they were steps during the development of the KinTree class)
all: rotation rotation_ana joint joint_ana joint2 joint2_ana joint3 joint5_ana

rotation: rotation.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) rotation.cxx

rotation_ana: rotation_ana.cxx Makefile
	g++ -g -o rotation_ana -lgsl -lblas $(GIJIT_INCL) -I../../acrobot/ana $(GIJIT_LIBS) ../../acrobot/ana/Lagrange.o ../../acrobot/ana/ODESolver.o ../../acrobot/ana/RawODESolver.o rotation_ana.cxx

joint: joint.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) joint.cxx

joint_ana: joint_ana.cxx Makefile
	g++ -O3 -o joint_ana -lgsl -lblas $(GIJIT_INCL) -I../../acrobot/ana $(GIJIT_LIBS) ../../acrobot/ana/Lagrange.o ../../acrobot/ana/ODESolver.o ../../acrobot/ana/RawODESolver.o joint_ana.cxx
	
joint2: joint2.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) joint2.cxx

joint2_ana: joint2_ana.cxx Makefile
	g++ -O3 -o joint2_ana -lgsl -lblas $(GIJIT_INCL) -I../../acrobot/ana $(GIJIT_LIBS) ../../acrobot/ana/Lagrange.o ../../acrobot/ana/LagrODESolver.o ../../acrobot/ana/RawODESolver.o joint2_ana.cxx

joint3: joint3.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) joint3.cxx

joint3_ana: joint3_ana.cxx ../KinTree.cxx Makefile
	g++ -O3 -o joint3_ana -lgsl -lblas $(EIGEN_INCL) $(GIJIT_INCL) -I.. -I../../acrobot/ana $(GIJIT_LIBS) ../../acrobot/ana/Lagrange.o ../../acrobot/ana/LagrODESolver.o ../../acrobot/ana/RawODESolver.o ../KinTree.cxx joint3_ana.cxx

joint4_ana: joint4_ana.cxx ../KinTree.cxx Makefile
	g++ -O3 -o joint4_ana -lgsl -lblas $(EIGEN_INCL) $(GIJIT_INCL) -I.. -I../../acrobot/ana $(GIJIT_LIBS) ../../acrobot/ana/Lagrange.o ../../acrobot/ana/LagrODESolver.o ../../acrobot/ana/RawODESolver.o ../KinTree.cxx joint4_ana.cxx

joint5_ana: joint5_ana.cxx ../KinTree.cxx Makefile
	g++ -O3 -o joint5_ana -lgsl -lblas $(EIGEN_INCL) -I.. -I../../acrobot/ana ../../acrobot/ana/RawODESolver.cxx ../KinTree.cxx joint5_ana.cxx

