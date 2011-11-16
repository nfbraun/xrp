INCLUDE_DIRS=-I/home/braun/Projects/Robot/opt/include
LIB_DIRS=-L/home/braun/Projects/Robot/opt/lib

brachistochrone: brachistochrone.cxx Brachistochrone.cxx Makefile
	g++ -o brachistochrone $(INCLUDE_DIRS) $(LIB_DIRS) -ladolc -lipopt -ldmumps_seq-4.9.2 brachistochrone.cxx Brachistochrone.cxx
