SOURCES = Cartwheel.cxx CWRobot.cxx DynTransform.cxx DynInfo.cxx MotorModel.cxx Reaction.cxx kintree/RNE_CRB.cxx kintree/Robot.cxx kintree/Spatial.cxx
HEADERS = Cartwheel.h CWRobot.h StaticRobotInfo.h RobotState.h DynTransform.h DynInfo.h MotorModel.h Reaction.h kintree/SE3Tr.h kintree/KinChain2.h kintree/RNE_CRB.h kintree/Robot.h kintree/Spatial.h

CW_SOURCES = \
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
cartwheel/Core/SwingController.cxx \
cartwheel/Core/CWConfig.cxx

CW_HEADERS = \
cartwheel/MathLib/MathLib.h \
cartwheel/MathLib/Trajectory.h \
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
cartwheel/Core/CWConfig.h \
cartwheel/Core/Debug.h

OBJS = $(SOURCES:.cxx=.o) $(CW_SOURCES:.cxx=.o)
CARTWHEEL=cartwheel
EIGEN_INCL_DIR = ../../eigen-eigen-3.2.0/
CFLAGS = -I../common/mechsim -I$(CARTWHEEL) -I$(EIGEN_INCL_DIR) $(shell ode-config --cflags) -g
LDFLAGS = -L../common/mechsim
LIBS = -lode -lmechsim -lgsl -lblas -lGL

.PHONY: all tests clean

all: simview simprint
tests: print_mass test_trans test_sim

clean:
	rm -f $(OBJS)
	rm -f simview.o
	rm -f simview

simview: $(OBJS) simview.o
	g++ -o $@ $(LDFLAGS) $(OBJS) simview.o $(LIBS)

simprint: $(OBJS) simprint.o
	g++ -o $@ $(LDFLAGS) $(OBJS) simprint.o $(LIBS)

print_invdyn: $(OBJS) print_invdyn.o
	g++ -o $@ $(LDFLAGS) $(OBJS) print_invdyn.o $(LIBS)

print_mass: print_mass.o
	g++ -o $@ print_mass.o

test_trans: DynTransform.o test_trans.o
	g++ -o $@ DynTransform.o test_trans.o

test_sim: $(OBJS) test_sim.o
	g++ -o $@ $(LDFLAGS) $(OBJS) test_sim.o $(LIBS)

%.o: %.cxx $(HEADERS) $(CW_HEADERS) Makefile
	g++ $(CFLAGS) -O2 -o $@ -c $<
