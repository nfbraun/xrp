SOURCES = Cartwheel.cxx CWRobot.cxx
HEADERS = Cartwheel.h CWRobot.h

CW_SOURCES = \
cartwheel/MathLib/Quaternion.cxx \
cartwheel/MathLib/Segment.cxx \
cartwheel/MathLib/Vector3d.cxx \
cartwheel/Physics/ArticulatedFigure.cxx \
cartwheel/Physics/Joint.cxx \
cartwheel/Physics/PhysicsGlobals.cxx \
cartwheel/Physics/RigidBody.cxx \
cartwheel/Core/BehaviourController.cxx \
cartwheel/Core/Character.cxx \
cartwheel/Core/IKVMCController.cxx \
cartwheel/Core/InvPendulum.cxx \
cartwheel/Core/PoseController.cxx \
cartwheel/Core/SimBiController.cxx \
cartwheel/Core/SimGlobals.cxx \
cartwheel/Core/TurnController.cxx \
cartwheel/Core/TwoLinkIK.cxx \
cartwheel/Core/VirtualModelController.cxx \
cartwheel/Core/CWController.cxx

CW_HEADERS = \
cartwheel/MathLib/MathLib.h \
cartwheel/MathLib/Point3d.h \
cartwheel/MathLib/Quaternion.h \
cartwheel/MathLib/Segment.h \
cartwheel/MathLib/Trajectory.h \
cartwheel/MathLib/Vector3d.h \
cartwheel/Physics/ArticulatedFigure.h \
cartwheel/Physics/ArticulatedRigidBody.h \
cartwheel/Physics/ContactPoint.h \
cartwheel/Physics/Joint.h \
cartwheel/Physics/PhysicsGlobals.h \
cartwheel/Physics/RBState.h \
cartwheel/Physics/RigidBody.h \
cartwheel/Core/BehaviourController.h \
cartwheel/Core/Character.h \
cartwheel/Core/Controller.h \
cartwheel/Core/IKVMCController.h \
cartwheel/Core/InvPendulum.h \
cartwheel/Core/PoseController.h \
cartwheel/Core/SimBiConState.h \
cartwheel/Core/SimBiController.h \
cartwheel/Core/SimGlobals.h \
cartwheel/Core/TurnController.h \
cartwheel/Core/TwoLinkIK.h \
cartwheel/Core/VirtualModelController.h \
cartwheel/Core/WorldOracle.h \
cartwheel/Core/CWController.h

OBJS = $(SOURCES:.cxx=.o) $(CW_SOURCES:.cxx=.o)
CARTWHEEL=cartwheel
CFLAGS = -I../common/mechsim -I$(CARTWHEEL) -I../../eigen-eigen-3.0.1 $(shell ode-config --cflags) -g
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim -lgsl -lblas

.PHONY: all clean

all: simview simprint

clean:
	rm -f $(OBJS)
	rm -f simview.o
	rm -f simview

simview: $(OBJS) simview.o
	g++ -o $@ $(LDFLAGS) $(LIBS) $(CARTWHEEL_LIBS) $(OBJS) simview.o

simprint: $(OBJS) simprint.o
	g++ -o $@ $(LDFLAGS) $(LIBS) $(CARTWHEEL_LIBS) $(OBJS) simprint.o

%.o: %.cxx $(HEADERS) $(CW_HEADERS) Makefile
	g++ $(CFLAGS) -o $@ -c $<
