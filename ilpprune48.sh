#!/bin/sh
#PBS -q cse
#PBS -l nodes=4:ppn=12
#PBS -N numt
#PBS -j oe
#PBS -l walltime=00:20:00

cd /home/gong15/CS598Parallel
~/gong15/taub_scripts/gennodelist.sh > nodelist

for (( i=12; i<20; i++ )); do
	./charmrun ./ilpprune data/test20.tsp $i 30 5 ++p 48 ++nodelist nodelist
done

