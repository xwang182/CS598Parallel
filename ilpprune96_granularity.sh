#!/bin/sh
#PBS -q cse
#PBS -l nodes=8:ppn=12
#PBS -N numt
#PBS -j oe
#PBS -l walltime=00:10:00

cd /home/gong15/CS598Parallel
~/gong15/taub_scripts/gennodelist.sh > nodelist

for (( i=0; i<=30; i+=5 )); do
	./charmrun ./ilpprune data/test20.tsp 19 30 $i ++p 96 ++nodelist nodelist
done

