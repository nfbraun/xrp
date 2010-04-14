# CONFIG += debug
QMAKE_CXXFLAGS += $$system(ode-config --cflags)
HEADERS = GLWidget.h SimulationViewer.h Vector.h Simulation.h CachedSimulation.h BallSlide.h
SOURCES = GLWidget.cxx SimulationViewer.cxx simtest.cxx BallSlide.cxx
LIBS += -lode
QT += opengl
