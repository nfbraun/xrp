# Makefile for USB MCA example program
#

CXX=g++
LIBS=-lusb
SRCS=usbdev.cxx
OBJS=$(SRCS:.c=.o)

.PHONY: clean

test: $(OBJS) Makefile
	$(CXX) $(LIBS) -o test $(OBJS)
	
%.o: %.c $(HEADERS) Makefile
	$(CXX) -c $<
	
clean:
	-/bin/rm -f *.o test
