# Modify it to your installation path...
INCLUDE = /Library/gurobi700/mac64/include/
LIBRARY = /Library/gurobi700/mac64/lib/
gurobi:
	g++ tsp_c.cpp -o tsp -I ${INCLUDE} -L${LIBRARY} -lgurobi_c++ -lgurobi70


# Coin-CLP
DRIVER = driver

# CHANGEME: This should be the name of your executable
EXE = $(DRIVER)

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
OBJS =  $(DRIVER).o 

# CHANGEME: Additional libraries
ADDLIBS =

# CHANGEME: Additional flags for compilation (e.g., include flags)
ADDINCFLAGS =

# CHANGEME: Directory to the sources for the (example) problem definition
# files
SRCDIR = .


##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile the      #
#  COIN package.                                                         #
##########################################################################

COIN_HAS_PKGCONFIG = TRUE
COIN_CXX_IS_CL = #TRUE
COIN_HAS_SAMPLE = TRUE
COIN_HAS_NETLIB = TRUE

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -O3 -pipe -DNDEBUG -pedantic-errors -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wconversion -Wno-unknown-pragmas -Wno-long-long   -DCLP_BUILD

# additional C++ Compiler options for linking
CXXLINKFLAGS =  -Wl,--rpath -Wl,/home/ywang189/project/CS598Parallel/coin-Clp/lib

# C Compiler command
CC = gcc

# C Compiler options
CFLAGS = -O3 -pipe -DNDEBUG -pedantic-errors -Wimplicit -Wparentheses -Wsequence-point -Wreturn-type -Wcast-qual -Wall -Wno-unknown-pragmas -Wno-long-long   -DCLP_BUILD

# Sample data directory
ifeq ($(COIN_HAS_SAMPLE), TRUE)
  ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
    CXXFLAGS += -DSAMPLEDIR=\"`PKG_CONFIG_PATH=/home/ywang189/project/CS598Parallel/coin-Clp/lib64/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/lib/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatasample`\"
      CFLAGS += -DSAMPLEDIR=\"`PKG_CONFIG_PATH=/home/ywang189/project/CS598Parallel/coin-Clp/lib64/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/lib/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatasample`\"
  else
    CXXFLAGS += -DSAMPLEDIR=\"\"
      CFLAGS += -DSAMPLEDIR=\"\"
  endif
endif

# Netlib data directory
ifeq ($(COIN_HAS_NETLIB), TRUE)
  ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
    CXXFLAGS += -DNETLIBDIR=\"`PKG_CONFIG_PATH=/home/ywang189/project/CS598Parallel/coin-Clp/lib64/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/lib/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatanetlib`\"
      CFLAGS += -DNETLIBDIR=\"`PKG_CONFIG_PATH=/home/ywang189/project/CS598Parallel/coin-Clp/lib64/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/lib/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatanetlib`\"
  else
    CXXFLAGS += -DNETLIBDIR=\"\"
      CFLAGS += -DNETLIBDIR=\"\"
  endif
endif

# Include directories (we use the CYGPATH_W variables to allow compilation with Windows compilers)
ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
  INCL = `PKG_CONFIG_PATH=/home/ywang189/project/CS598Parallel/coin-Clp/lib64/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/lib/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/share/pkgconfig: pkg-config --cflags clp`
else
  INCL = 
endif
INCL += $(ADDINCFLAGS)

# Linker flags
ifeq ($(COIN_HAS_PKGCONFIG), TRUE)
  LIBS = `PKG_CONFIG_PATH=/home/ywang189/project/CS598Parallel/coin-Clp/lib64/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/lib/pkgconfig:/home/ywang189/project/CS598Parallel/coin-Clp/share/pkgconfig: pkg-config --libs osi-clp`
else
  ifeq ($(COIN_CXX_IS_CL), TRUE)
    LIBS = -link -libpath:`$(CYGPATH_W) /home/ywang189/project/CS598Parallel/coin-Clp/lib` libClpSolver.lib libClp.lib 
  else
    LIBS = -L/home/ywang189/project/CS598Parallel/coin-Clp/lib -lClpSolver -lClp 
  endif
endif

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

# Here we list all possible generated objects or executables to delete them
CLEANFILES = \
	tsp_clp.o tsp_clp
	example_clp.o example_clp 

all:	$(EXE)

.SUFFIXES: .cpp .c .o .obj

$(EXE): $(OBJS)
	bla=;\
	for file in $(OBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXLINKFLAGS) $(CXXFLAGS) -o $@ $$bla $(LIBS) $(ADDLIBS)

clean:
	rm -rf $(CLEANFILES)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `test -f '$<' || echo '$(SRCDIR)/'`$<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `if test -f '$<'; then $(CYGPATH_W) '$<'; else $(CYGPATH_W) '$(SRCDIR)/$<'; fi`

.c.o:
	$(CC) $(CFLAGS) $(INCL) -c -o $@ `test -f '$<' || echo '$(SRCDIR)/'`$<


.c.obj:
	$(CC) $(CFLAGS) $(INCL) -c -o $@ `if test -f '$<'; then $(CYGPATH_W) '$<'; else $(CYGPATH_W) '$(SRCDIR)/$<'; fi`