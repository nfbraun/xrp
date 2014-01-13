EIGEN_INCL_DIR = ../../eigen-eigen-3.2.0/
CFLAGS = -g -I$(EIGEN_INCL_DIR) -O3 -I../common/mechsim $(shell ode-config --cflags)
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim -lGLU -lGL

.PHONY: all

all: asphere ballslide cube bunny dp hftest

asphere: AsymSphere.cxx AsymSphere.h asphere_main.cxx
	g++ -o $@ $(CFLAGS) AsymSphere.cxx asphere_main.cxx $(LDFLAGS) $(LIBS)

ballslide: BallSlide.cxx BallSlide.h ballslide_main.cxx
	g++ -o $@ $(CFLAGS) BallSlide.cxx ballslide_main.cxx $(LDFLAGS) $(LIBS)

bunny: Bunny.cxx Bunny.h bunny_main.cxx
	g++ -o $@ $(CFLAGS) Bunny.cxx bunny_main.cxx $(LDFLAGS) $(LIBS)

cube: Cube.cxx Cube.h cube_main.cxx
	g++ -o $@ $(CFLAGS) Cube.cxx cube_main.cxx $(LDFLAGS) $(LIBS)

dp: DoublePendulum.cxx DoublePendulum.h dp_main.cxx
	g++ -o $@ $(CFLAGS) DoublePendulum.cxx dp_main.cxx $(LDFLAGS) $(LIBS)

hftest: HFTest.cxx HFTest.h hftest_main.cxx
	g++ -o $@ $(CFLAGS) HFTest.cxx hftest_main.cxx $(LDFLAGS) $(LIBS)

