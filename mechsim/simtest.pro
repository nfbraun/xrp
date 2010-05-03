# CONFIG += debug
QMAKE_CXXFLAGS += $$system(ode-config --cflags)
HEADERS = GLWidget.h SimulationViewer.h Vector.h Simulation.h CachedSimulation.h McGeer.h
SOURCES = GLWidget.cxx SimulationViewer.cxx simtest.cxx McGeer.cxx
LIBS += -lode
QT += opengl
