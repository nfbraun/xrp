PSOPT_DIR=/home/braun/Projects/Robot/Psopt3
DMATRIXDIR=$(PSOPT_DIR)/dmatrix
LUSOL=/home/braun/Projects/Robot/lusol/csrc

IPOPTINCDIR = /usr/include/coin
ADOL_INCLUDE=/home/braun/Projects/Robot/opt/include
ADOL_LIB=/home/braun/Projects/Robot/opt/lib
SUITESPARSE=/usr/include/suitesparse

PSOPTSRCDIR = $(PSOPT_DIR)/PSOPT/src

SPARSE_LIBS   = $(LUSOL)/liblusol.a  /usr/lib/libcxsparse.a -ldl
IPOPT_LIBS    = -lipopt -ldmumps_seq-4.9.2
ADOLC_LIBS    = -ladolc
PSOPT_LIBS    = $(PSOPT_DIR)/PSOPT/lib/libpsopt.a
DMATRIX_LIBS  = $(DMATRIXDIR)/lib/libdmatrix.a

LDFLAGS	      = -Wl,--rpath -Wl,$(USERHOME)/Ipopt-3.5.4/lib -L$(ADOL_LIB)  -lm  -ldl    -lgcc_s


CXX           = /usr/bin/g++
CC            = /usr/bin/gcc
CXXFLAGS      = -O0 -g -I$(DMATRIXDIR)/include  -I$(PSOPTSRCDIR) -DLAPACK -DUNIX -DSPARSE_MATRIX -DUSE_IPOPT -I$(CXSPARSE)/Include -I$(LUSOL) -I$(SUITESPARSE) -I$(ADOL_INCLUDE) -I$(IPOPTINCDIR) -fomit-frame-pointer -pipe -DNDEBUG -pedantic-errors -Wimplicit -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wconversion -fPIC -DHAVE_MALLOC

CFLAGS        = -O0 -fPIC

ALL_LIBRARIES = $(PSOPT_LIBS) $(DMATRIX_LIBS)  $(FLIBS) $(SPARSE_LIBS) $(IPOPT_LIBS)  $(ADOLC_LIBS)

.PHONY: all

all: acrobot acrobot2

%.o : %.cxx
	$(CXX) -c $(CXXFLAGS) $< -o $@

acrobot: acrobot.o $(PSOPT_LIBS) $(DMATRIX_LIBS) $(SPARSE_LIBS)
	$(CXX) $(CXXFLAGS) $^ -o $@ -L$(LIBDIR) $(ALL_LIBRARIES) $(LDFLAGS)

acrobot2: acrobot2.o $(PSOPT_LIBS) $(DMATRIX_LIBS) $(SPARSE_LIBS)
	$(CXX) $(CXXFLAGS) $^ -o $@ -L$(LIBDIR) $(ALL_LIBRARIES) $(LDFLAGS)

pendulum: pendulum.o $(PSOPT_LIBS) $(DMATRIX_LIBS) $(SPARSE_LIBS)
	$(CXX) $(CXXFLAGS) $^ -o $@ -L$(LIBDIR) $(ALL_LIBRARIES) $(LDFLAGS)

