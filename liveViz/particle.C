#include <stdlib.h>
#include <vector>
#include "liveViz.h"
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

      cellProxy = CProxy_Cell::ckNew(cellDimension,cellDimension);

      CkArrayOptions opts(cellDimension,cellDimension);
      CkCallback c(CkIndex_Cell::requestNextFrame(0),cellProxy);
      liveVizConfig cfg(liveVizConfig::pix_color,true);
      liveVizInit(cfg,cellProxy,c, opts);
      CkPrintf("Particles start moving");

      cellProxy.run();
    }

    void maxCheck(int maxArray[2]){
      int iter = maxArray[0];
      int max = maxArray[1];
      int index = iter/step - 1;
      results[index][0] = iter;
      results[index][1] = max;
      if(results[index][2]!=0)
        printTotal(results[index][2], results[index][1], results[index][0]);
    }

    void totalCheck(int totalArray[2]){
      int iter = totalArray[0]/(cellDimension*cellDimension);
      int total = totalArray[1];
      int index = iter/step - 1;
      results[index][0] = iter;
      results[index][2] = total;
      if(results[index][1]!=0)
        printTotal(results[index][2], results[index][1], results[index][0]);
    }

    void done() { 
      CkExit(); 
    }

    void printTotal(int total, int max, int iter){
      CkPrintf("ITER %d, MAX: %d, TOTAL: %d\n", iter, max, total);
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

    Cell() {
      __sdag_init();
      iteration = 0;
      min_x = thisIndex.y * delta;
      max_x = (thisIndex.y+1) * delta;
      min_y = thisIndex.x * delta;
      max_y = (thisIndex.x+1) * delta;
      populateCell(particlesPerCell);

    }
    Cell(CkMigrateMessage* m) {}

    void requestNextFrame(liveVizRequestMsg *m){ 
      int cellLength= (int)(100/cellDimension);
      int sx=thisIndex.y*cellLength;
      int sy=thisIndex.x*cellLength;
      int w=cellLength;
      int h=cellLength;

      unsigned char *intensity= new unsigned char[3*w*h];
      for(int i=0;i<particles.size();i++){
        Particle particle = particles[i];
        int x_p = (int)(particle.x);
        x_p = x_p - sx;
        int y_p = (int)(particle.y);
        y_p = y_p - sy;

        // if(x_p<0)
        //   x_p = x_p * (-1);
        // if(x_p>=cellLength)
        //   x_p = cellLength-1;
        
        // if(y_p<0)
        //   y_p = y_p * (-1);
        // if(y_p>=cellLength)
        //   y_p = cellLength-1;

        if(x_p<0 || x_p>=cellLength || x_p<0 ||  y_p>=cellLength)
          continue;

        int index = 3*(y_p * w + x_p);
        char color = particle.color;

        if(color == 'r'){
          intensity[index+0] = 255;
          intensity[index+1] = 0;
          intensity[index+2] = 0;
        } else if(color == 'g'){
          intensity[index+0] = 0;
          intensity[index+1] = 255;
          intensity[index+2] = 0;
        }else if(color == 'b'){
          intensity[index+0] = 0;
          intensity[index+1] = 0;
          intensity[index+2] = 255; 
        }
      }

      liveVizDeposit(m, sx,sy, w,h, intensity, this);
      delete[] intensity;
    }    

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
    }

    Particle adjustBorder(Particle particle){
      double x_p = particle.x;
      double y_p = particle.y;
      char color_p = particle.color;
      if(x_p<0.0)
        x_p = 100.0 + x_p;
      if(x_p>=100.0)
        x_p = x_p - 100.0;
      if(y_p<0.0)
        y_p = 100.0 + y_p;
      if(y_p>=100.0)
        y_p = y_p - 100.0;
      Particle oneNew(x_p,y_p,color_p);
      return oneNew;
    }

    void updateParticles() {
      for(int i=0;i<particles.size();i++){
        perturb(&particles[i]);
      }

      vector<Particle> left;
      vector<Particle> right;
      vector<Particle> top;
      vector<Particle> bottom;
      vector<Particle> leftTop;
      vector<Particle> rightTop;
      vector<Particle> leftBottom;
      vector<Particle> rightBottom;

      vector<Particle> newParticle;
      for (int i=0;i<particles.size();i=i+1){
        double x_p = particles[i].x;
        double y_p = particles[i].y;
        if (x_p>=min_x && x_p<max_x && y_p>=min_y && y_p<max_y )
          newParticle.push_back(adjustBorder( particles[i]) );
        else{
          if (x_p<min_x && x_p<max_x && y_p>=min_y && y_p<max_y )
            left.push_back( adjustBorder( particles[i] ) );
          if (x_p>=min_x && x_p>=max_x && y_p>=min_y && y_p<max_y )
            right.push_back( adjustBorder( particles[i] ) );
          if (x_p>=min_x && x_p<max_x && y_p<min_y && y_p<max_y )
            top.push_back( adjustBorder( particles[i] ) );
          if (x_p>=min_x && x_p<max_x && y_p>=min_y && y_p>=max_y )
            bottom.push_back( adjustBorder( particles[i] ) );
          if (x_p<min_x && x_p<max_x && y_p<min_y && y_p<max_y )
            leftTop.push_back( adjustBorder( particles[i] ) );
          if (x_p>=min_x && x_p>=max_x && y_p<min_y && y_p<max_y )
            rightTop.push_back( adjustBorder( particles[i] ) );
          if (x_p<min_x && x_p<max_x && y_p>=min_y && y_p>=max_y )
            leftBottom.push_back( adjustBorder( particles[i] ) );
          if (x_p>=min_x && x_p>=max_x && y_p>=min_y && y_p>=max_y )
            rightBottom.push_back( adjustBorder( particles[i] ) );
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

        thisProxy(thisIndex.x,leftChange).updateNeighbor(iteration, left);
        thisProxy(thisIndex.x,rightChange).updateNeighbor(iteration, right);
        thisProxy(topChange,thisIndex.y).updateNeighbor(iteration, top);
        thisProxy(bottomChange,thisIndex.y).updateNeighbor(iteration, bottom);
        thisProxy(topChange,leftChange).updateNeighbor(iteration, leftTop);
        thisProxy(topChange,rightChange).updateNeighbor(iteration, rightTop);
        thisProxy(bottomChange,leftChange).updateNeighbor(iteration, leftBottom);
        thisProxy(bottomChange,rightChange).updateNeighbor(iteration, rightBottom);
      particles = newParticle;
    }


  private:

    void populateCell(int particlesPerCell) {
      int x = thisIndex.x;
      int y = thisIndex.y;
      if(x <= y){
        // upper right,blue
        char color = 'b';
        for(int i=0;i<particlesPerCell;i++){
          double rand_x = drand48()*(100.0/cellDimension) + min_x;
          double rand_y = drand48()*(100.0/cellDimension) + min_y;
          
          Particle one(rand_x,rand_y,color);
          particles.push_back(one);
        }
      }
      if(x >= y){
        // lower left,green
        char color = 'g';
        for(int i=0;i<particlesPerCell;i++){
          double rand_x = drand48()*(100.0/cellDimension) + min_x;
          double rand_y = drand48()*(100.0/cellDimension) + min_y;
          
          Particle one(rand_x,rand_y,color);
          particles.push_back(one);
        }
      }
      float middle_Start = 3.0*cellDimension/8.0;
      float middle_End = 5.0*cellDimension/8.0;
      if(middle_Start<=x && x<=middle_End && middle_Start<=y && y<=middle_End){
        // middle,red
        char color = 'r';
        for(int i=0;i<2*particlesPerCell;i++){
          double rand_x = drand48()*(100.0/cellDimension) + min_x;
          double rand_y = drand48()*(100.0/cellDimension) + min_y;
          
          Particle one(rand_x,rand_y,color);
          particles.push_back(one);
        }
      }
    }

    //change the location of the particle within the range of 8 neighbours
    //the location of the particles might exceed the bounds of the chare array
    //as a result of this functions, so you need to handle that case when deciding 
    //which particle to go which neighbour chare
    //e.g. the right neighbour of chare indexed[k-1,0] is chare [0,0]
    void perturb(Particle* particle) {
      //drand48 creates a random number between [0-1] 
      double factor = 1.0;
      if(particle->color == 'r')
        factor = 1.0;
      if(particle->color == 'g')
        factor = 0.5;
      if(particle->color == 'b')
        factor = 0.25;
      double deltax = (drand48()-drand48())*(100.0/cellDimension);
      double deltay = (drand48()-drand48())*(100.0/cellDimension);

      particle->x += deltax;
      particle->y += deltay;
    }


};

#include "ParticleExercise.def.h"