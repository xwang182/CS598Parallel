CHARMC=/home/kchndrs2/charm/netlrts-linux-x86_64/bin/charmc

# CHANGEME: Additional flags for compilation (e.g., include flags)
ADDINCFLAGS =

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile the      #
#  COIN package.                                                         #
##########################################################################

COIN_HAS_PKGCONFIG = TRUE
COIN_CXX_IS_CL = #TRUE
COIN_HAS_SAMPLE = TRUE
COIN_HAS_NETLIB = TRUE

WHOAMI = $(shell whoami)

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -O3 -pipe -DNDEBUG -DCLP_BUILD -std=c++0x

# additional C++ Compiler options for linking
CXXLINKFLAGS =  -Wl,--rpath -Wl,/home/$(WHOAMI)/coin-Clp/lib

# C Compiler command
CC = gcc

# C Compiler options
CFLAGS = -O3 -pipe -DNDEBUG -pedantic-errors -Wimplicit -Wparentheses -Wsequence-point -Wreturn-type -Wcast-qual -Wall -Wno-unknown-pragmas -Wno-long-long   -DCLP_BUILD

# Sample data directory
ifeq ($(COIN_HAS_SAMPLE), TRUE)
  ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
    CXXFLAGS += -DSAMPLEDIR=\"`PKG_CONFIG_PATH=/home/$(WHOAMI)/coin-Clp/lib64/pkgconfig:/home/$(WHOAMI)/coin-Clp/lib/pkgconfig:/home/$(WHOAMI)/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatasample`\"
      CFLAGS += -DSAMPLEDIR=\"`PKG_CONFIG_PATH=/home/$(WHOAMI)/coin-Clp/lib64/pkgconfig:/home/$(WHOAMI)/coin-Clp/lib/pkgconfig:/home/$(WHOAMI)/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatasample`\"
  else
    CXXFLAGS += -DSAMPLEDIR=\"\"
      CFLAGS += -DSAMPLEDIR=\"\"
  endif
endif

# Netlib data directory
ifeq ($(COIN_HAS_NETLIB), TRUE)
  ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
    CXXFLAGS += -DNETLIBDIR=\"`PKG_CONFIG_PATH=/home/$(WHOAMI)/coin-Clp/lib64/pkgconfig:/home/$(WHOAMI)/coin-Clp/lib/pkgconfig:/home/$(WHOAMI)/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatanetlib`\"
      CFLAGS += -DNETLIBDIR=\"`PKG_CONFIG_PATH=/home/$(WHOAMI)/coin-Clp/lib64/pkgconfig:/home/$(WHOAMI)/coin-Clp/lib/pkgconfig:/home/$(WHOAMI)/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatanetlib`\"
  else
    CXXFLAGS += -DNETLIBDIR=\"\"
      CFLAGS += -DNETLIBDIR=\"\"
  endif
endif

# Include directories (we use the CYGPATH_W variables to allow compilation with Windows compilers)
ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
  INCL = `PKG_CONFIG_PATH=/home/$(WHOAMI)/coin-Clp/lib64/pkgconfig:/home/$(WHOAMI)/coin-Clp/lib/pkgconfig:/home/$(WHOAMI)/coin-Clp/share/pkgconfig: pkg-config --cflags clp`
else
  INCL = 
endif
INCL += $(ADDINCFLAGS)

# Linker flags
ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
  LIBS = `PKG_CONFIG_PATH=/home/$(WHOAMI)/coin-Clp/lib64/pkgconfig:/home/$(WHOAMI)/coin-Clp/lib/pkgconfig:/home/$(WHOAMI)/coin-Clp/share/pkgconfig: pkg-config --libs osi-clp`
else
  ifeq ($(COIN_CXX_IS_CL), TRUE)
    LIBS = -link -libpath:`$(CYGPATH_W) /home/$(WHOAMI)/coin-Clp/lib` libClpSolver.lib libClp.lib 
  else
    LIBS = -L/home/$(WHOAMI)/coin-Clp/lib -lClpSolver -lClp 
  endif
endif

all: project coarse_ofb ilp ilpprune

project: project.o
	$(CHARMC) project.o -o project -language charm++ $(CXXLINKFLAGS) $(CXXFLAGS) $(LIBS) $(ADDLIBS)

project.o : project.C project.def.h project.decl.h
	$(CHARMC) -c $(CXXFLAGS) $(INCL) project.C

project.def.h project.decl.h : project.ci
	$(CHARMC) project.ci

coarse_ofb: coarse_ofb.o
	$(CHARMC) coarse_ofb.o -o coarse_ofb -language charm++ $(CXXLINKFLAGS) $(CXXFLAGS) $(LIBS) $(ADDLIBS)

coarse_ofb.o : coarse_ofb.C coarse_ofb.def.h coarse_ofb.decl.h
	$(CHARMC) -c $(CXXFLAGS) $(INCL) coarse_ofb.C

coarse_ofb.def.h coarse_ofb.decl.h : coarse_ofb.ci
	$(CHARMC) coarse_ofb.ci

ilp: ilp.o
	$(CHARMC) ilp.o -o ilp -language charm++ $(CXXLINKFLAGS) $(CXXFLAGS) $(LIBS) $(ADDLIBS)

ilp.o : ilp.C ilp.def.h ilp.decl.h
	$(CHARMC) -c $(CXXFLAGS) $(INCL) ilp.C

ilp.def.h ilp.decl.h : ilp.ci
	$(CHARMC) ilp.ci

ilpprune: ilpprune.o
	$(CHARMC) ilpprune.o -o ilpprune -language charm++ $(CXXLINKFLAGS) $(CXXFLAGS) $(LIBS) $(ADDLIBS)

ilpprune.o : ilpprune.C ilpprune.def.h ilpprune.decl.h
	$(CHARMC) -c $(CXXFLAGS) $(INCL) ilpprune.C

ilpprune.def.h ilpprune.decl.h : ilpprune.ci
	$(CHARMC) ilpprune.ci

test_project: project
	./charmrun ++p 16 ++local ./project $(TSPFILE) $(CITYLIMIT)

test_coarse_ofb: coarse_ofb
	./charmrun ++p 16 ++local ./coarse_ofb $(TSPFILE) $(CITYLIMIT)

test_ilp: ilp
	./charmrun ++p 16 ++local ./ilp $(TSPFILE) $(CITYLIMIT)

test_ilpprune: ilpprune
	./charmrun ++p 16 ++local ./ilpprune $(TSPFILE) $(CITYLIMIT) $(CACHESIZE) $(GRANULARITY)

clean:
	rm -f conv-host *.o charmrun project coarse_ofb ilp ilpprune
	rm -f *.def.h *.decl.h
	rm -f *.log.gz *.projrc *.topo *.sts *.sum

