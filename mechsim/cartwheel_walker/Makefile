SOURCES = Cartwheel.cxx CWRobot.cxx DynTransform.cxx DynInfo.cxx MotorModel.cxx Reaction.cxx kintree/RNE_CRB.cxx kintree/Robot.cxx kintree/Spatial.cxx
HEADERS = Cartwheel.h CWRobot.h StaticRobotInfo.h RobotState.h DynTransform.h DynInfo.h MotorModel.h Reaction.h Torques.h kintree/SE3Tr.h kintree/KinChain2.h kintree/RNE_CRB.h kintree/Robot.h kintree/Spatial.h

CW_SOURCES = \
cartwheel/ContactInfo.cxx \
cartwheel/RobotInfo.cxx \
cartwheel/TorqueController.cxx \
cartwheel/InvPendulum.cxx \
cartwheel/SimGlobals.cxx \
cartwheel/TurnController.cxx \
cartwheel/WalkController.cxx \
cartwheel/CWController.cxx \
cartwheel/StateMachine.cxx \
cartwheel/LegIK.cxx \
cartwheel/SwingController.cxx \
cartwheel/CWConfig.cxx

CW_HEADERS = \
cartwheel/MathLib.h \
cartwheel/Trajectory.h \
cartwheel/ContactPoint.h \
cartwheel/ContactInfo.h \
cartwheel/RobotInfo.h \
cartwheel/TorqueController.h \
cartwheel/InvPendulum.h \
cartwheel/SimGlobals.h \
cartwheel/TurnController.h \
cartwheel/WalkController.h \
cartwheel/WorldOracle.h \
cartwheel/CWController.h \
cartwheel/StateMachine.h \
cartwheel/LegIK.h \
cartwheel/SwingController.h \
cartwheel/CWConfig.h \
cartwheel/Debug.h

OBJS = $(SOURCES:.cxx=.o) $(CW_SOURCES:.cxx=.o)
EIGEN_INCL_DIR = ../../eigen-eigen-3.2.0/
CFLAGS = -I../common/mechsim -I. -I$(EIGEN_INCL_DIR) $(shell ode-config --cflags) -g
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
