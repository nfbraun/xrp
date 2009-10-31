TARGET= simtest
HEADERS= Display.h Simulation.h Vector.h
SOURCES= simtest.cxx Display.cxx Simulation.cxx
OBJ= $(SOURCES:.cxx=.o)
LFLAGS= -lglut -lode
CPP= g++

.PHONY: clean

$(TARGET): $(OBJ) Makefile
	$(CPP) -o $(TARGET) $(LFLAGS) $(OBJ)
    
%.o: %.cxx $(HEADERS) Makefile
	$(CPP) $(CFLAGS) -fpic -c $<
	
clean:
	-/bin/rm -f *~
	-/bin/rm -f $(OBJ)
	-/bin/rm -f $(TARGET)
