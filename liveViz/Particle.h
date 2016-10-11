#ifndef PARTICLE_H
#define PARTICLE_H

/*
*Particle object with x&y coordinate components
*/

class Particle  {
public:
    double x;
    double y;
    char color;

    Particle() { }
    Particle(double a, double b,char colorInput) { 
      x=a; y=b; 
      color = colorInput;
    }

    void pup(PUP::er &p){
      p|x;
      p|y;
      p|color;
    }

};

#endif
