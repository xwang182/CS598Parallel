#include <math.h> 
#include <vector>
#include <fstream>
#include <limits>
#include <algorithm>
#include "OsiClpSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"

using namespace std;


typedef vector<int> int_vec_t;
typedef vector<double> double_vec_t;

typedef vector< pair<double, double> > map_t;
typedef vector< vector<double> > distance_t;
typedef vector< vector<int> > path_map_t;
typedef path_map_t idx_map_t;
typedef vector< int_vec_t > constraint_vec_t;

#include "ilp.decl.h"

/* readonly */ CProxy_Master mainProxy;
/* readonly */ CProxy_Cache cacheProxy;
/* readonly */ map_t problem;
/* readonly */ distance_t dist;
/* readonly */ idx_map_t variable_map;
/* readonly */ double_vec_t objective_vec;
/* readonly */ double_vec_t col_lb_vec;
/* readonly */ double_vec_t col_ub_vec;
/* readonly */ constraint_vec_t constraint_vec;
/* readonly */ int n_cols;


vector<int> findShortestPath(path_map_t graph) {
  vector<int> visited;
  visited.resize(graph.size());

  vector<int> path;

  while(true) {
    bool all_visited = true;
    int source = 0;
    for (int i = 0; i < visited.size(); i++) {
      if (!visited[i]) {
        all_visited = false;
        source = i;
        break;
      }
    }
    if (all_visited) {
      break;
    }

    vector<int> temp_path;
    int original_source = source;
    temp_path.push_back(source);
    visited[source] = 1;
    while (true) {
      for (int i = 0; i < graph.size(); i++) {
        if ((i != temp_path[temp_path.size() - 2]/*|| temp_path.size() == 1*/) && graph[source][i]) {
          temp_path.push_back(i);
          visited[i] = 1;
          source = i;
          break;
        }
      }
      if (source == original_source) {
        temp_path.pop_back(); // remove 0
        break;
      }
    }

    if (temp_path.size() <= path.size() || path.empty()) {
      path = temp_path;
    }
  }

  return path;
}


class Master : public CBase_Master {
private:
  double ofub_;
  double *sol_;
  bool qd_;

public:

  Master(CkArgMsg* msg) {

    if(msg->argc < 2){

      CkPrintf("Usage: %s tsp_file", msg->argv[0]);
      CkExit();  
    }

    int city_limit = numeric_limits<int>::max();
    if (msg->argc > 2) {
      city_limit = atoi(msg->argv[2]);
    }

    ofub_ = numeric_limits<double>::infinity();
    qd_ = false;

    problem = readTspFile(msg->argv[1], city_limit);
    dist = calcDistanceMap(problem);

    CkPrintf("Number of city: %d\n", problem.size());

    n_cols = problem.size() * (problem.size() - 1) / 2;
    sol_ = new double[n_cols];
    // define the objective coefficients
    // minimize Sum: d_ij x_ij

    for (int i = 0; i < problem.size(); i++) {
      vector<int> row;
      row.resize(problem.size());
      variable_map.push_back(row);
    }

    int count = 0;
    for (int i = 0; i < problem.size() - 1; i++) {
      for (int j = i + 1; j < problem.size(); j++) {
        objective_vec.push_back(dist[i][j]);

        // set variable map
        variable_map[i][j] = count;
        variable_map[j][i] = count;
        count++;
      }
    }

    for (int i = 0; i < n_cols; i++) {
      col_lb_vec.push_back(0);
      col_ub_vec.push_back(1);
    }

    for (int i = 0; i < problem.size(); i++) {
      int_vec_t vec;
      for (int j = 0; j < problem.size(); j++) {
        if (i == j) continue;
        vec.push_back(variable_map[i][j]);
      }

      // Sum: x_ij = 2
      constraint_vec.push_back(vec);
    } 

    mainProxy = thisProxy;
    cacheProxy = CProxy_Cache::ckNew(ofub_);

  }
  Master(CkMigrateMessage* msg){}

  void groupCreated(CkReductionMsg *msg) {

    /*SlaveMessage *root_msg = new (0, 0, 8 * sizeof(int)) SlaveMessage(0, 0.0);
    * (int *) CkPriorityPtr(root_msg) = 0;
    CkSetQueueing(root_msg, CK_QUEUEING_ILIFO);

    CProxy_Slave::ckNew(root_msg, 1);*/
    CProxy_Slave::ckNew(0.0, int_vec_t(), double_vec_t(), idx_map_t());
  }

  void updateOFUB(double ofub, int size, const double *solution) {
    if (ofub < ofub_) {
      ofub_ = ofub;
      memcpy(sol_, solution, n_cols * sizeof(double));
      CkPrintf("Global OFUB updated to %lf\n", ofub);
    }

    if (!qd_) {
      qd_ = true;
      CkStartQD(CkCallback(CkIndex_Master::done(), mainProxy));
    }
  }

  void done() {
    CkPrintf("Final CP: %lf\n", ofub_);
    CkPrintf("Time: %lfs\n", CkTimer());
    CkExit();
  }

private:

  map_t readTspFile(string file_path, int city_limit) {
    vector< pair<double, double> > output_vector;
    ifstream infile(file_path);
    string line;
    int count = 0;
    bool foundEntry = false;
    while (getline(infile, line)) {
      if (line[0] >= '0' && line[0] <= '9') {
        foundEntry = true;
      }
      if (foundEntry) { // begin to read data
        std::istringstream iss(line);
        int index;
        double x, y;
        if (!(iss >> index >> x >> y)) { break; } // error

        output_vector.push_back(make_pair(x, y));

        if (++count == city_limit) {
          break;
        }
      }
    }

    return output_vector;
  }

  double calcDistance(const pair<double, double> &a, const pair<double, double> &b) {
    double dx = a.first - b.first;
    double dy = a.second - b.second;

    return sqrt(dx * dx + dy * dy);
  }

  distance_t calcDistanceMap(const map_t &problem) {
    distance_t dist;

    for (int i = 0; i < problem.size(); ++i) {
      vector<double> tmp;
      const pair<double, double> &a = problem[i];
      for (int j = 0; j < problem.size(); ++j) {

        if (i == j) {
          tmp.push_back(numeric_limits<double>::infinity());
          continue;
        }

        const pair<double, double> &b = problem[j];
        tmp.push_back(calcDistance(a, b));
      }
      dist.push_back(tmp);
    }

    return dist;
  }

};

class Cache : public CBase_Cache {
private:
  CmiNodeLock lock_;
  double ofub_;

public:
  Cache(double ofub) {
    lock_ = CmiCreateLock();
    CmiLock(lock_);
    ofub_ = ofub;
    CmiUnlock(lock_);

    contribute(0, NULL, CkReduction::nop,
      CkCallback(CkReductionTarget(Master, groupCreated), mainProxy)
    );
  }

  virtual ~Cache() {
    CmiDestroyLock(lock_);
  }

  void updateOFUB(double ofub) {
    CmiLock(lock_);
    ofub_ = ofub;
    //CkPrintf("Local OFUB updated to %lf\n", ofub);
    CmiUnlock(lock_);
  }

  bool checkCPP(double cpp) {
    bool ret;
    CmiLock(lock_);
    ret = cpp < ofub_ ? true : false;
    CmiUnlock(lock_);

    return ret;
  }

  void sendCTP(double ctp, int size, const double *solution) {
    CmiLock(lock_);
    if (ctp < ofub_) {
      ofub_ = ctp;
      thisProxy.updateOFUB(ctp);
      mainProxy.updateOFUB(ctp, size, solution);
      //CkPrintf("Local OFUB updated to %lf\n", ctp);
    }
    CmiUnlock(lock_);
  }
};

class Slave : public CBase_Slave {

public:

  Slave(CkMigrateMessage* msg){}
  
  Slave(double parent_cost, int_vec_t idx_vec, double_vec_t val_vec, idx_map_t subtour_vec){

    Cache *cache = cacheProxy.ckLocalBranch();
    if (!cache->checkCPP(parent_cost)) {
      return;
    }

    //CkPrintf("Visited %d cities, remained %d cities, slave initiated\n", visited.size(), remained.size());

    // Create a problem pointer.  We use the base class here.
    OsiSolverInterface *si;

    // When we instantiate the object, we need a specific derived class.
    si = new OsiClpSolverInterface;
    CoinMessageHandler *msg_hdl = si->messageHandler();
    msg_hdl->setLogLevel(0);

    double *objective = objective_vec.data();
    double *col_lb = col_lb_vec.data();
    double *col_ub = col_ub_vec.data();

    // Constraint
    // Sum: x_ij = 2
    size_t n_rows = problem.size(); // 1; // V;
    double *row_lb = new double[n_rows + idx_vec.size() + subtour_vec.size()]; //the row lower bounds
    double *row_ub = new double[n_rows + idx_vec.size() + subtour_vec.size()]; //the row upper bounds

    // define the constraint matrix
    CoinPackedMatrix *matrix = new CoinPackedMatrix(false, 0, 0);
    matrix->setDimensions(0, n_cols);
    
    for (int i = 0; i < n_rows; i++) {
      CoinPackedVector vec((int) constraint_vec[i].size(), constraint_vec[i].data(), 1.0);
      row_lb[i] = 2;
      row_ub[i] = 2;
      matrix->appendRow(vec);
    }

    for (int i = 0; i < idx_vec.size(); ++i) {
      CoinPackedVector vec;
      vec.insert(idx_vec[i], 1.0);
      row_lb[n_rows + i] = val_vec[i];
      row_ub[n_rows + i] = val_vec[i];
      matrix->appendRow(vec);
    }

    for (int i = 0; i < subtour_vec.size(); ++i) {
      vector<int> &path = subtour_vec[i];

      CoinPackedVector vec;
      int path_size = path.size();
      for (int i = 0; i < path_size; i++) {
        int row = path[i];
        int col = path[(i + 1) % path_size];

        vec.insert(variable_map[row][col], 1.0);
      }
      row_lb[n_rows + idx_vec.size() + i] = 0.0;
      row_ub[n_rows + idx_vec.size() + i] = (double)(path_size - 1);
      matrix->appendRow(vec);
    }

    si->loadProblem(*matrix, col_lb, col_ub, objective, row_lb, row_ub);

    // Solve the (relaxation of the) problem
    si->initialSolve();
    if (si->isProvenOptimal()) {
      int n = si->getNumCols();
      const double* solution = si->getColSolution();

      /*for (int i = 0; i < n; ++i) {
        CkPrintf("%d: %lf\n", i, solution[i]);
      }*/

      double cost = si->getObjValue();

      if (cache->checkCPP(cost)) {

        bool is_int = true;
        for (int i = 0; i < n; ++i) {
          if (solution[i] == 1 || solution[i] == 0) {
            continue;
          }
          is_int = false;

          /*int len = idx_vec.size() + 1;
          SlaveMessage *child0_msg = new (len, len, 8 * sizeof(int)) SlaveMessage(len, cost);
          memcpy(child0_msg->idx_vec, idx_vec, sizeof(int) * (len - 1));
          memcpy(child0_msg->val_vec, val_vec, sizeof(double) * (len - 1));

          child0_msg->idx_vec[len - 1] = i;
          child0_msg->val_vec[len - 1] = 1.0;

          * (int *) CkPriorityPtr(child0_msg) = (int) cost;
          CkSetQueueing(child0_msg, CK_QUEUEING_ILIFO);

          CProxy_Slave::ckNew(child0_msg);*/
          
          /*SlaveMessage *child1_msg = new (len, len, 8 * sizeof(int)) SlaveMessage(len, cost);
          memcpy(child1_msg->idx_vec, idx_vec, sizeof(int) * (len - 1));
          memcpy(child1_msg->val_vec, val_vec, sizeof(double) * (len - 1));

          child1_msg->idx_vec[len - 1] = i;
          child1_msg->val_vec[len - 1] = 0.0;

          * (int *) CkPriorityPtr(child1_msg) = (int) cost;
          CkSetQueueing(child1_msg, CK_QUEUEING_ILIFO);

          CProxy_Slave::ckNew(child1_msg, 1);*/

          idx_vec.push_back(i);
          val_vec.push_back(0.0);
          CkEntryOptions opts;
          opts.setPriority((int) cost);
          opts.setQueueing(CK_QUEUEING_ILIFO);
          CProxy_Slave::ckNew(cost, idx_vec, val_vec, subtour_vec, CK_PE_ANY, &opts);
          val_vec.back() = 1.0;
          CProxy_Slave::ckNew(cost, idx_vec, val_vec, subtour_vec, CK_PE_ANY, &opts);
          idx_vec.pop_back();
          val_vec.pop_back();

        }
        if (is_int) {

          path_map_t graph; // adjacency matrix
          for (int i = 0; i < dist.size(); i++) {
            vector<int> row;
            row.resize(dist.size());
            graph.push_back(row);
          }

          // set up graph
          for (int i = 0; i < dist.size() - 1; i++) {
            for (int j = i + 1; j < dist.size(); j++) {
              graph[i][j] = (int)(solution[variable_map[i][j]]);
              graph[j][i] = (int)(solution[variable_map[j][i]]);
            }
          }

          vector<int> path = findShortestPath(graph);

          if (path.size() == dist.size()) {
            CkPrintf("Total cost = %lf #constraint = %d ", cost, idx_vec.size());
            for (int i = 0; i < idx_vec.size(); ++i) {
              CkPrintf("%d=%lf ", idx_vec[i], val_vec[i]);
            }
            CkPrintf("#subtour_constraint = %d\n", subtour_vec.size());
            cache->sendCTP(cost, n, solution);
          } else {
            //CkPrintf("Add subtour %d constraints\n", subtour_vec.size() + 1);
            subtour_vec.push_back(path);
            CkEntryOptions opts;
            opts.setPriority(0);
            opts.setQueueing(CK_QUEUEING_IFIFO);
            CProxy_Slave::ckNew(cost, idx_vec, val_vec, subtour_vec, CK_PE_ANY, &opts);
          }
        } else {
          //CkPrintf("Partial cost = %lf #constraint = %d\n", cost, msg->vec_length);
        }

      } else {
        //CkPrintf("Dropped cost = %lf #constraint = %d\n", cost, msg->vec_length);
      }
    }

    delete row_ub;
    delete row_lb;
    delete si;

  }



};


#include "ilp.def.h"