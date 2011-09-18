TEMPLATE = lib
CONFIG += debug
QMAKE_CXXFLAGS += $$system(ode-config --cflags) -O3 -I../../../eigen-eigen-3.0.1/
HEADERS = GLHelper.h ODEHelper.h GLWidget.h SimulationWidget.h DataViewWidget.h ChannelDialog.h CamCtrlWidget.h Spherical.h RotMotion.h SolidBody.h Simulation.h SyncSimulation.h AsyncSimulation.h SimulationViewer.h STLReader.h dataplot/VChannel.h dataplot/Channel.h dataplot/File.h dataplot/Data.h dataplot/CSVFile.h dataplot/VPTransform.h dataplot/ScalePainter.h dataplot/ChannelWidget.h dataplot/VCWidget.h
SOURCES = GLHelper.cxx GLWidget.cxx SimulationWidget.cxx DataViewWidget.cxx ChannelDialog.cxx CamCtrlWidget.cxx SolidBody.cxx SimulationViewer.cxx STLReader.cxx dataplot/File.cxx dataplot/Data.cxx dataplot/Channel.cxx dataplot/CSVFile.cxx dataplot/VPTransform.cxx dataplot/ScalePainter.cxx dataplot/ChannelWidget.cxx dataplot/VCWidget.cxx
LIBS += -lode
QT += opengl
