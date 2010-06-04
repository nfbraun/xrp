# CONFIG += debug
QMAKE_CXXFLAGS += $$system(ode-config --cflags)
HEADERS = GLWidget.h SimulationViewer.h Vector.h Rotation.h Simulation.h CachedSimulation.h Test.h
SOURCES = GLWidget.cxx SimulationViewer.cxx Rotation.cxx simprint.cxx Test.cxx
LIBS += -lode
QT += opengl
