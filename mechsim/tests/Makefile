CFLAGS = -g -I../../eigen-eigen-3.0.1/ -O3 -I../common/mechsim $(shell ode-config --cflags)
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim

.PHONY: all

all: asphere ballslide cube bunny dp hftest

asphere: AsymSphere.cxx AsymSphere.h asphere_main.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) AsymSphere.cxx asphere_main.cxx

ballslide: BallSlide.cxx BallSlide.h ballslide_main.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) BallSlide.cxx ballslide_main.cxx

bunny: Bunny.cxx Bunny.h bunny_main.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Bunny.cxx bunny_main.cxx

cube: Cube.cxx Cube.h cube_main.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Cube.cxx cube_main.cxx

dp: DoublePendulum.cxx DoublePendulum.h dp_main.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) DoublePendulum.cxx dp_main.cxx

hftest: HFTest.cxx HFTest.h hftest_main.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) HFTest.cxx hftest_main.cxx

