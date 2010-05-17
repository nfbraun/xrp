# CONFIG += debug
QMAKE_CXXFLAGS += $$system(ode-config --cflags)
HEADERS = GLWidget.h SimulationViewer.h Vector.h Rotation.h Simulation.h CachedSimulation.h STLReader.h Bunny.h
SOURCES = GLWidget.cxx SimulationViewer.cxx Rotation.cxx STLReader.cxx simtest.cxx Bunny.cxx
LIBS += -lode
QT += opengl
