HEADERS = GiJIT.h CodeGenImp.h Visitor.h FuncFactory.h
SOURCES = CodeGenImp.cxx Visitor.cxx FuncFactory.cxx
OBJS = $(SOURCES:.cxx=.o)
CXXFLAGS = $(shell llvm-config --cxxflags) -fexceptions -fPIC
LDFLAGS = $(shell llvm-config --ldflags)
LIBS = $(shell llvm-config --libs core jit native interpreter) -lginac
TARGET = libgijit.so.1

.PHONY: clean

$(TARGET): $(OBJS) Makefile
	g++ -shared -Wl,-soname,$(TARGET) -o $(TARGET) $(OBJS) $(LDFLAGS) $(LIBS)

%.o: %.cxx $(HEADERS) Makefile
	g++ -c -o $@ $(CXXFLAGS) $<

test: test.cxx $(TARGET) Makefile
	g++ -o test -L. -lgijit -lginac test.cxx

clean:
	rm -f *.o
	rm -f test
