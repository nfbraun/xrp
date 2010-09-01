TEMPLATE = lib
# CONFIG += debug
QMAKE_CXXFLAGS += $$system(ode-config --cflags)
HEADERS = GLHelper.h ODEHelper.h GLWidget.h SimulationWidget.h Vector.h Matrix.h Rotation.h RotMotion.h SolidBody.h Simulation.h CachedSimulation.h SimulationViewer.h STLReader.h
SOURCES = GLHelper.cxx GLWidget.cxx SimulationWidget.cxx Rotation.cxx SolidBody.cxx SimulationViewer.cxx STLReader.cxx
LIBS += -lode
QT += opengl
