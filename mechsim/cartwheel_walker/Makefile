SOURCES = Cartwheel.cxx CWRobot.cxx DynTransform.cxx DynInfo.cxx
HEADERS = Cartwheel.h CWRobot.h StaticRobotInfo.h RobotState.h AffineTransform.h DynTransform.h DynInfo.h

CW_SOURCES = \
cartwheel/MathLib/Quaternion.cxx \
cartwheel/MathLib/Segment.cxx \
cartwheel/MathLib/Vector3d.cxx \
cartwheel/Physics/PhysicsGlobals.cxx \
cartwheel/Core/ContactInfo.cxx \
cartwheel/Core/RobotInfo.cxx \
cartwheel/Core/TorqueController.cxx \
cartwheel/Core/InvPendulum.cxx \
cartwheel/Core/SimGlobals.cxx \
cartwheel/Core/TurnController.cxx \
cartwheel/Core/WalkController.cxx \
cartwheel/Core/CWController.cxx \
cartwheel/Core/StateMachine.cxx \
cartwheel/Core/LegIK.cxx \
cartwheel/Core/SwingController.cxx

CW_HEADERS = \
cartwheel/MathLib/MathLib.h \
cartwheel/MathLib/Point3d.h \
cartwheel/MathLib/Quaternion.h \
cartwheel/MathLib/Segment.h \
cartwheel/MathLib/Trajectory.h \
cartwheel/MathLib/Vector3d.h \
cartwheel/Physics/ContactPoint.h \
cartwheel/Physics/PhysicsGlobals.h \
cartwheel/Core/ContactInfo.h \
cartwheel/Core/RobotInfo.h \
cartwheel/Core/Controller.h \
cartwheel/Core/TorqueController.h \
cartwheel/Core/InvPendulum.h \
cartwheel/Core/SimGlobals.h \
cartwheel/Core/TurnController.h \
cartwheel/Core/WalkController.h \
cartwheel/Core/WorldOracle.h \
cartwheel/Core/CWController.h \
cartwheel/Core/StateMachine.h \
cartwheel/Core/LegIK.h \
cartwheel/Core/SwingController.h \
cartwheel/Core/Debug.h

OBJS = $(SOURCES:.cxx=.o) $(CW_SOURCES:.cxx=.o)
CARTWHEEL=cartwheel
CFLAGS = -I../common/mechsim -I$(CARTWHEEL) -I../../eigen-eigen-3.0.1 $(shell ode-config --cflags) -g
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim -lgsl -lblas

.PHONY: all tests clean

all: simview simprint
tests: test_trans test_sim

clean:
	rm -f $(OBJS)
	rm -f simview.o
	rm -f simview

simview: $(OBJS) simview.o
	g++ -o $@ $(LDFLAGS) $(LIBS) $(CARTWHEEL_LIBS) $(OBJS) simview.o

simprint: $(OBJS) simprint.o
	g++ -o $@ $(LDFLAGS) $(LIBS) $(CARTWHEEL_LIBS) $(OBJS) simprint.o

test_trans: DynTransform.o test_trans.o
	g++ -o $@ DynTransform.o test_trans.o

test_sim: $(OBJS) test_sim.o
	g++ -o $@ $(LDFLAGS) $(LIBS) $(CARTWHEEL_LIBS) $(OBJS) test_sim.o

%.o: %.cxx $(HEADERS) $(CW_HEADERS) Makefile
	g++ $(CFLAGS) -O2 -o $@ -c $<
