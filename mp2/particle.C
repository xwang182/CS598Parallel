#include <stdlib.h>
#include <vector>
#include "pup_stl.h"
#include "Particle.h"
#include "ParticleExercise.decl.h"

#define ITERATION (100)
#define step (10)

/*readonly*/ CProxy_Main mainProxy;
/*readonly*/ CProxy_Cell cellProxy;
/*readonly*/ int particlesPerCell;
/*readonly*/ int cellDimension;
/*readonly*/ double delta;

using namespace std;

class Main: public CBase_Main {
  public:
    int results[ITERATION/step][3]; //0 is iter, 1 is max, 2 is total
    int iterprint;

    Main(CkArgMsg* m) {
      if(m->argc < 3) CkAbort("USAGE: ./charmrun +p<number_of_processors> ./particle <number of particles per cell> <size of array>");

      mainProxy = thisProxy;
      particlesPerCell = atoi(m->argv[1]);
      cellDimension = atoi(m->argv[2]);
      delta = 100.0/cellDimension;
      delete m;

      for(int it=0;it<ITERATION/step;it++){
        results[it][0]=0;
        results[it][1]=0;
        results[it][2]=0;
      }

      iterprint = 0;

      //Done; TODO: Create the grid and start the simulation by calling run()
      cellProxy = CProxy_Cell::ckNew(cellDimension,cellDimension);
      cellProxy.run();
    }

    //Done; TODO: Add entry methods which will be a target of the reduction for avg
    //and max counts and exiting when the iterations are done
    void maxCheck(int maxArray[2]){
      int iter = maxArray[0];
      int max = maxArray[1];
      int index = iter/step - 1;
      results[index][0] = iter;
      results[index][1] = max;
      // CkPrintf("Max: ITER %d, MAX: %d\n", iter, max);
    }

    void totalCheck(int totalArray[2]){
      int iter = totalArray[0]/(cellDimension*cellDimension);
      int total = totalArray[1];
      int index = iter/step - 1;
      results[index][0] = iter;
      results[index][2] = total;
      // CkPrintf("Total: ITER %d, TOTAL: %d\n", iter, total);
    }

    void done() { 
      // CkPrintf("We finished all\n");
      for(int it=0;it<ITERATION/step;it++){
        int iter = results[it][0];
        int max = results[it][1];
        int total = results[it][2];
        printTotal(total, max, iter);
      }
      CkExit(); 
    }

    void printTotal(int total, int max, int iter){
      CkPrintf("ITER %d, MAX: %d, TOTAL: %d\n", iter, max, total);
    }

    void printCheck(int check){
      // CkPrintf("ITER %d, chare finish: %d\n", iterprint, check);
      iterprint = iterprint+1;
    }
};

// This class represent the cells of the simulation.
// Each cell contains a vector of particle.
// On each time step, the cell perturbs the particles and moves them to neighboring cells as necessary.
class Cell: public CBase_Cell {
  Cell_SDAG_CODE

  public:
    int iteration;
    vector<Particle> particles;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    int wait;
    //Done; TODO: more variables might be needed

    Cell() {
      __sdag_init();
      iteration = 0;
      min_x = thisIndex.y * delta;
      max_x = (thisIndex.y+1) * delta;
      min_y = thisIndex.x * delta;
      max_y = (thisIndex.x+1) * delta;
      populateCell(particlesPerCell); //creates random particles within the cell

    }
    Cell(CkMigrateMessage* m) {}

    void pup(PUP::er &p){
      CBase_Cell::pup(p);
      __sdag_pup(p);
      p|iteration;
      p|particles;
      p|min_x;
      p|max_x;
      p|min_y;
      p|max_y;
      p|wait;
      //Done; TODO: if you added more variable, decide if they need to go into the
      //pup method
    }

    void updateParticles() {
      //Done; TODO:use perturb function for the location of new particles
      for(int i=0;i<particles.size();i++){
        perturb(&particles[i]);
      }

      //Done; TODO:move the particles
      vector<Particle> left;
      vector<Particle> right;
      vector<Particle> top;
      vector<Particle> bottom;
      vector<Particle> leftTop;
      vector<Particle> rightTop;
      vector<Particle> leftBottom;
      vector<Particle> rightBottom;
      for (int i=0;i<particles.size();){
        double x_p = particles[i].x;
        double y_p = particles[i].y;
        if (x_p>=min_x && x_p<max_x && y_p>=min_y && y_p<max_y )
          i = i + 1;
        else{
          if (x_p<min_x && x_p<max_x && y_p>=min_y && y_p<max_y )
            left.push_back(particles[i]);
          if (x_p>=min_x && x_p>=max_x && y_p>=min_y && y_p<max_y )
            right.push_back(particles[i]);
          if (x_p>=min_x && x_p<max_x && y_p<min_y && y_p<max_y )
            top.push_back(particles[i]);
          if (x_p>=min_x && x_p<max_x && y_p>=min_y && y_p>=max_y )
            bottom.push_back(particles[i]);
          if (x_p<min_x && x_p<max_x && y_p<min_y && y_p<max_y )
            leftTop.push_back(particles[i]);
          if (x_p>=min_x && x_p>=max_x && y_p<min_y && y_p<max_y )
            rightTop.push_back(particles[i]);
          if (x_p<min_x && x_p<max_x && y_p>=min_y && y_p>=max_y )
            leftBottom.push_back(particles[i]);
          if (x_p>=min_x && x_p>=max_x && y_p>=min_y && y_p>=max_y )
            rightBottom.push_back(particles[i]);
          particles.erase(particles.begin()+i);
        }
      }
        int topChange = thisIndex.x-1;
        if(topChange<0)
          topChange = cellDimension - 1;
        int bottomChange = thisIndex.x+1;
        if(bottomChange>cellDimension - 1)
          bottomChange = 0;
        int leftChange = thisIndex.y-1;
        if(leftChange<0)
          leftChange = cellDimension - 1;
        int rightChange = thisIndex.y+1;
        if(rightChange>cellDimension - 1)
          rightChange = 0;
        // CkPrintf("topChange %d, bottomChange: %d, leftChange: %d, rightChange: %d\n", topChange, bottomChange, leftChange, rightChange);
        thisProxy(thisIndex.x,leftChange).updateNeighbor(iteration, left);
        thisProxy(thisIndex.x,rightChange).updateNeighbor(iteration, right);
        thisProxy(topChange,thisIndex.y).updateNeighbor(iteration, top);
        thisProxy(bottomChange,thisIndex.y).updateNeighbor(iteration, bottom);
        thisProxy(topChange,leftChange).updateNeighbor(iteration, leftTop);
        thisProxy(topChange,rightChange).updateNeighbor(iteration, rightTop);
        thisProxy(bottomChange,leftChange).updateNeighbor(iteration, leftBottom);
        thisProxy(bottomChange,rightChange).updateNeighbor(iteration, rightBottom);
      
    }

    //you can add more methods if you want or need to

  private:

    void populateCell(int initialElements) {
      //Done; create random particles and add then to the particles vector
      for(int i=0;i<initialElements;i++){
        double rand_x = drand48()*(100.0/cellDimension) + min_x;
        double rand_y = drand48()*(100.0/cellDimension) + min_y;

        Particle one(rand_x,rand_y);
        particles.push_back(one);
      }
    }

    //change the location of the particle within the range of 8 neighbours
    //the location of the particles might exceed the bounds of the chare array
    //as a result of this functions, so you need to handle that case when deciding 
    //which particle to go which neighbour chare
    //e.g. the right neighbour of chare indexed[k-1,0] is chare [0,0]
    void perturb(Particle* particle) {
      //drand48 creates a random number between [0-1]	
      double deltax = (drand48()-drand48())*(100.0/cellDimension);
      double deltay = (drand48()-drand48())*(100.0/cellDimension);

      particle->x += deltax;
      particle->y += deltay;
    }


};

#include "ParticleExercise.def.h"
