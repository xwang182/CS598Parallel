CHARMC =~/charm/net-darwin-x86_64/bin/charmc -module liveViz $(OPTS) -g
TESTOPTS = ++local

OBJS = particle.o

N = 10000
K = 16

all: particle

particle: $(OBJS)
	$(CHARMC) -O3 -language charm++ -o particle $(OBJS)

particle.decl.h: particle.ci
	$(CHARMC)  particle.ci

clean:
	rm -f *.decl.h *.def.h conv-host *.o particle charmrun

cleanp:
	rm -f *.sts *.gz *.projrc *.topo *.out

particle.o: particle.C particle.decl.h
	$(CHARMC) -c particle.C

test: all
	./charmrun +p4 ./particle $(N) $(K) $(TESTOPTS) ++server ++server-port 1235
