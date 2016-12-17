#!/bin/sh
#PBS -q cse
#PBS -l nodes=8:ppn=12
#PBS -N numt
#PBS -j oe
#PBS -l walltime=00:10:00

cd /home/gong15/CS598Parallel
~/gong15/taub_scripts/gennodelist.sh > nodelist


./charmrun ./ilpprune data/test20.tsp 19 1 5 ++p 96 ++nodelist nodelist
./charmrun ./ilpprune data/test20.tsp 19 5 5 ++p 96 ++nodelist nodelist

for (( i=10; i<=100; i+=10 )); do
	./charmrun ./ilpprune data/test20.tsp 19 $i 5 ++p 96 ++nodelist nodelist
done

