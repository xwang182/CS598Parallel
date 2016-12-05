CHARMC=/home/kchndrs2/charm/netlrts-linux-x86_64/bin/charmc

all: project

project: project.o
	$(CHARMC) project.o -o project -language charm++ 

project.o : project.C project.def.h project.decl.h
	$(CHARMC) -c project.C

project.def.h project.decl.h : project.ci
	$(CHARMC) project.ci

TSPFILE=data/test3.tsp

test: all
	./charmrun +p6 ++local ./project $(TSPFILE)

clean:
	rm -f conv-host *.o charmrun project
	rm -f *.def.h *.decl.h
	rm -f *.log.gz *.projrc *.topo *.sts *.sum

