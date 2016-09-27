#include "project.decl.h"
#include <math.h> 
/* readonly */ CProxy_Master mainProxy;
/* readonly */ int numElements;

class Master : public CBase_Master {
  public:
    CProxy_Project projectArray;

    Master(CkArgMsg* msg){
      if(msg->argc > 1){
        numElements = atoi(msg->argv[1]);
      }
      mainProxy = thisProxy;
      projectArray = CProxy_Project::ckNew(numElements);
      projectArray.run();
    }
    Master(CkMigrateMessage* msg){}

    void done(int value) { 
      CkPrintf("all together value: %d\n",value);
      CkExit(); 
   }
};



class Project : public CBase_Project {
  Project_SDAG_CODE

  public:
    Project(){
      CkPrintf("%d. Initial number of array %d\n", thisIndex, numElements);
    }
    Project(CkMigrateMessage* msg){}
};


#include "project.def.h"