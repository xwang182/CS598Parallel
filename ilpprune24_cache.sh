#!/bin/sh
#PBS -q cse
#PBS -l nodes=2:ppn=12
#PBS -N numt
#PBS -j oe
#PBS -l walltime=00:20:00

cd /home/gong15/CS598Parallel
~/gong15/taub_scripts/gennodelist.sh > nodelist


./charmrun ./ilpprune data/test20.tsp 19 1 5 ++p 24 ++nodelist nodelist
./charmrun ./ilpprune data/test20.tsp 19 5 5 ++p 24 ++nodelist nodelist

for (( i=10; i<=50; i+=10 )); do
        ./charmrun ./ilpprune data/test20.tsp 19 $i 5 ++p 24 ++nodelist nodelist
done

for (( i=100; i<=300; i+=50 )); do
        ./charmrun ./ilpprune data/test20.tsp 19 $i 5 ++p 24 ++nodelist nodelist
done   
