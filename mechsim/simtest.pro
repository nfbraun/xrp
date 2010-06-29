# CONFIG += debug
QMAKE_CXXFLAGS += $$system(ode-config --cflags)
HEADERS = GLWidget.h SimulationViewer.h Vector.h Matrix.h Rotation.h RotMotion.h SolidBody.h Simulation.h CachedSimulation.h McGeer.h
SOURCES = GLWidget.cxx SimulationViewer.cxx Rotation.cxx SolidBody.cxx simprint.cxx McGeer.cxx
LIBS += -lode
QT += opengl
