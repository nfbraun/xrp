CFLAGS = -g -I../common/mechsim $(shell ode-config --cflags)
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim

.PHONY: all

all: asphere ballslide cube bunny dp

asphere: AsymSphere.cxx AsymSphere.h asphere.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) AsymSphere.cxx asphere.cxx

ballslide: BallSlide.cxx BallSlide.h ballslide.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) BallSlide.cxx ballslide.cxx

bunny: Bunny.cxx Bunny.h bunny.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Bunny.cxx bunny.cxx

cube: Cube.cxx Cube.h cube.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) Cube.cxx cube.cxx

dp: DoublePendulum.cxx DoublePendulum.h dp.cxx
	g++ -o $@ $(CFLAGS) $(LDFLAGS) $(LIBS) DoublePendulum.cxx dp.cxx
