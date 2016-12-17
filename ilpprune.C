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
typedef vector<double_vec_t> distance_t;
typedef vector<int_vec_t> path_map_t;
typedef path_map_t idx_map_t;
typedef set<int_vec_t> subtour_set_t;
typedef vector< int_vec_t > constraint_vec_t;
typedef pair<int, bool> constraint_pair_t;
typedef set<constraint_pair_t> constraint_set_t;
typedef pair<constraint_set_t, subtour_set_t> constraint_t;

#include "ilpprune.decl.h"

/* readonly */ CProxy_Master mainProxy;
/* readonly */ CProxy_Cache cacheProxy;
/* readonly */ map_t problem;
/* readonly */ distance_t dist;
/* readonly */ idx_map_t variable_map;
/* readonly */ double_vec_t objective_vec;
/* readonly */ double_vec_t col_lb_vec;
/* readonly */ double_vec_t col_ub_vec;
/* readonly */ constraint_vec_t base_constraint_vec;
/* readonly */ int n_cols;
/* readonly */ int cache_size;
/* readonly */ int granularity;


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

    cache_size = 10;
    if (msg->argc > 3) {
      cache_size = atoi(msg->argv[3]);
    }

    granularity = 1;
    if (msg->argc > 4) {
      granularity = atoi(msg->argv[4]);
    }

    ofub_ = numeric_limits<double>::infinity();
    qd_ = false;

    problem = readTspFile(msg->argv[1], city_limit);
    dist = calcDistanceMap(problem);

    CkPrintf("Number of city: %d Cache size: %d Granularity: %d\n", problem.size(), cache_size, granularity);

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
      base_constraint_vec.push_back(vec);
    } 

    mainProxy = thisProxy;
    cacheProxy = CProxy_Cache::ckNew(ofub_);

  }
  Master(CkMigrateMessage* msg){}

  void groupCreated(CkReductionMsg *msg) {
    CProxy_Slave::ckNew(0.0, constraint_t(), -1);
  }

  void getStat(CkReductionMsg *msg) {

    int size;
    CkReduction::tupleElement* results;

    msg->toTuple(&results, &size);
    int finished_cut_cnt  = * (int *) results[0].data;
    int known_cut_cnt = * (int *) results[1].data;
    int cost_cut_cnt = * (int *) results[2].data;
    int eval_cnt = * (int *) results[3].data;

    CkPrintf("# cut by seeing a finished constraint: %d\n", finished_cut_cnt);
    CkPrintf("# cut by seeing a known constraint: %d\n", known_cut_cnt);
    CkPrintf("# cut by dropping cost higher than the bound: %d\n", cost_cut_cnt);
    CkPrintf("# of LP evaluated: %d\n", eval_cnt);

    CkExit();
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
    cacheProxy.reportStat();
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

  CmiNodeLock ofub_lock_, known_lock_, finished_lock_, dropped_lock_, eval_lock_;
  double ofub_;

  set<constraint_t> known_constraints_;
  set<constraint_t> cached_known_constraints_;
  set<constraint_set_t> finished_constraints_;
  set<constraint_set_t> dropped_constraints_;
  set<constraint_set_t> cached_dropped_constraints_;

  int finished_cut_cnt_;
  int known_cut_cnt_;
  int dropped_cut_cnt_;
  int cost_cut_cnt_;
  int eval_cnt_;

public:
  Cache(double ofub) {
    ofub_lock_ = CmiCreateLock();
    known_lock_ = CmiCreateLock();
    finished_lock_ = CmiCreateLock();
    dropped_lock_ = CmiCreateLock();
    eval_lock_ = CmiCreateLock();
    CmiLock(ofub_lock_);
    ofub_ = ofub;
    cost_cut_cnt_ = 0;
    CmiUnlock(ofub_lock_);

    CmiLock(finished_lock_);
    finished_cut_cnt_ = 0;
    CmiUnlock(finished_lock_);

    CmiLock(known_lock_);
    known_cut_cnt_ = 0;
    CmiUnlock(known_lock_);

    CmiLock(dropped_lock_);
    finished_cut_cnt_ = 0;
    CmiUnlock(dropped_lock_);

    CmiLock(eval_lock_);
    eval_cnt_ = 0;
    CmiUnlock(eval_lock_);

    contribute(0, NULL, CkReduction::nop,
      CkCallback(CkReductionTarget(Master, groupCreated), mainProxy)
    );
  }

  virtual ~Cache() {
    CmiDestroyLock(ofub_lock_);
    CmiDestroyLock(known_lock_);
    CmiDestroyLock(finished_lock_);
    CmiDestroyLock(dropped_lock_);
    CmiDestroyLock(eval_lock_);
  }

  void updateOFUB(double ofub) {
    CmiLock(ofub_lock_);
    ofub_ = ofub;
    CmiUnlock(ofub_lock_);
  }

  bool checkCPP(double cpp) {
    bool ret;
    CmiLock(ofub_lock_);
    ret = true;
    if (cpp >= ofub_) {
      ret = false;
      cost_cut_cnt_++;
    }
    CmiUnlock(ofub_lock_);

    return ret;
  }

  void sendCTP(double ctp, int size, const double *solution, const constraint_set_t &constraints) {
    CmiLock(ofub_lock_);
    if (ctp < ofub_) {
      ofub_ = ctp;
      thisProxy.updateOFUB(ctp);
      mainProxy.updateOFUB(ctp, size, solution);
      //CkPrintf("Local OFUB updated to %lf\n", ctp);
    }
    CmiUnlock(ofub_lock_);
    addFinishedConstraint(constraints);
  }

  void finishEval() {
    CmiLock(eval_lock_);
    eval_cnt_++;
    CmiUnlock(eval_lock_);
  }

  void addFinishedConstraint(const constraint_set_t &constraints) {
    CmiLock(known_lock_);
    for (set<constraint_set_t>::iterator i = finished_constraints_.begin(); i != finished_constraints_.end(); ++i) {
      if (i->size() < constraints.size()) {
        continue;
      }
      if (includes(i->begin(), i->end(), constraints.begin(), constraints.end())) {
        if (i->size() != constraints.size()) {
          set<constraint_set_t>::iterator tmp = i++;
          finished_constraints_.erase(tmp);
        }
      }
    }
    finished_constraints_.insert(constraints);
    thisProxy.postFinishedConstraint(constraints);
    CmiUnlock(known_lock_);
  }

  void postFinishedConstraint(constraint_set_t constraints) {
    CmiLock(known_lock_);
    for (set<constraint_set_t>::iterator i = finished_constraints_.begin(); i != finished_constraints_.end(); ++i) {
      if (i->size() < constraints.size()) {
        continue;
      }
      if (includes(i->begin(), i->end(), constraints.begin(), constraints.end())) {
        if (i->size() != constraints.size()) {
          set<constraint_set_t>::iterator tmp = i++;
          finished_constraints_.erase(tmp);
        }
      }
    }
    finished_constraints_.insert(constraints);
    CmiUnlock(known_lock_);
  }

  void addDroppedConstraint(const constraint_set_t &constraints) {
    CmiLock(dropped_lock_);
    bool found = false;
    for (set<constraint_set_t>::iterator i = cached_dropped_constraints_.begin(); i != cached_dropped_constraints_.end(); ++i) {
      if (i->size() < constraints.size()) {
        continue;
      }
      if (includes(i->begin(), i->end(), constraints.begin(), constraints.end())) {
        if (i->size() != constraints.size()) {
          cached_dropped_constraints_.erase(i);
          cached_dropped_constraints_.insert(constraints);
        }
        found = true;
        break;
      }
    }
    if (!found) {
      cached_dropped_constraints_.insert(constraints);
    }
    if (cached_dropped_constraints_.size() > cache_size) {
      thisProxy.postDroppedConstraint(cached_dropped_constraints_);
      cached_dropped_constraints_.clear();
    }
    CmiUnlock(dropped_lock_);
  }

  void postDroppedConstraint(set<constraint_set_t> constraints) {
    CmiLock(dropped_lock_);
    for (set<constraint_set_t>::iterator k = constraints.begin(); k != constraints.end(); ++k) {
      const constraint_set_t &constraint_set = *k;
      bool found = false;
      for (set<constraint_set_t>::iterator i = dropped_constraints_.begin(); i != dropped_constraints_.end(); ++i) {
        if (i->size() < constraint_set.size()) {
          continue;
        }
        if (includes(i->begin(), i->end(), constraint_set.begin(), constraint_set.end())) {
          if (i->size() != constraint_set.size()) {
            dropped_constraints_.erase(i);
            dropped_constraints_.insert(constraint_set);
          }
          found = true;
          break;
        }
      }
      if (!found) {
        dropped_constraints_.insert(constraint_set);
      }
    }
    CmiUnlock(dropped_lock_);
  }
  void postConstraint(set<constraint_t> constraints) {
    CmiLock(known_lock_);
    known_constraints_.insert(constraints.begin(), constraints.end());
    CmiUnlock(known_lock_);
  }

  bool addConstraint(const constraint_t &constraints) {
    CmiLock(finished_lock_);
    const constraint_set_t &constraint_set = constraints.first;
    for (set<constraint_set_t>::iterator i = finished_constraints_.begin(); i != finished_constraints_.end(); ++i) {
      if (constraint_set.size() < i->size()) {
        continue;
      }
      if (includes(constraint_set.begin(), constraint_set.end(), i->begin(), i->end())) {
        finished_cut_cnt_++;
        CmiUnlock(finished_lock_);
        return false;
      }
    }
    CmiUnlock(finished_lock_);
    /*CmiLock(dropped_lock_);
    for (set<constraint_set_t>::iterator i = dropped_constraints_.begin(); i != dropped_constraints_.end(); ++i) {
      if (constraint_set.size() < i->size()) {
        continue;
      }
      if (includes(constraint_set.begin(), constraint_set.end(), i->begin(), i->end())) {
        finished_cut_cnt_++;
        CmiUnlock(dropped_lock_);
        return false;
      }
    }
    CmiUnlock(dropped_lock_);*/
    if (known_constraints_.find(constraints) != known_constraints_.end()) {
      CmiUnlock(known_lock_);
      known_cut_cnt_++;
      return false;
    }
    if (cached_known_constraints_.insert(constraints).second == false) {
      CmiUnlock(known_lock_);
      known_cut_cnt_++;
      return false;
    }
    if (cached_known_constraints_.size() >= cache_size) {
      thisProxy.postConstraint(cached_known_constraints_);
      cached_known_constraints_.clear();
    }
    CmiUnlock(known_lock_);
    return true;
  }

  bool testConstraint(const constraint_t &constraints) {
    CmiLock(known_lock_);
    if (known_constraints_.find(constraints) != known_constraints_.end()) {
      known_cut_cnt_++;
      CmiUnlock(known_lock_);
      return false;
    }
    if (cached_known_constraints_.find(constraints) != cached_known_constraints_.end()) {
      known_cut_cnt_++;
      CmiUnlock(known_lock_);
      return false;
    }
    CmiUnlock(known_lock_);
    /*CmiLock(dropped_lock_);
    const constraint_set_t &constraint_set = constraints.first;
    for (set<constraint_set_t>::iterator i = dropped_constraints_.begin(); i != dropped_constraints_.end(); ++i) {
      if (constraint_set.size() < i->size()) {
        continue;
      }
      if (includes(constraint_set.begin(), constraint_set.end(), i->begin(), i->end())) {
        finished_cut_cnt_++;
        CmiUnlock(dropped_lock_);
        return false;
      }
    }
    CmiUnlock(dropped_lock_);*/
    CmiLock(finished_lock_);
    const constraint_set_t &constraint_set = constraints.first;
    for (set<constraint_set_t>::iterator i = finished_constraints_.begin(); i != finished_constraints_.end(); ++i) {
      if (constraint_set.size() < i->size()) {
        continue;
      }
      if (includes(constraint_set.begin(), constraint_set.end(), i->begin(), i->end())) {
        finished_cut_cnt_++;
        CmiUnlock(finished_lock_);
        return false;
      }
    }
    CmiUnlock(finished_lock_);
    return true;
  }

  void reportStat() {

    //CkPrintf("# finished constraints: %d # known constraints: %d\n", finished_constraints_.size(), known_constraints_.size());

    CkReduction::tupleElement tuple_red_n[] = {
      CkReduction::tupleElement(sizeof(int), &finished_cut_cnt_, CkReduction::sum_int),
      CkReduction::tupleElement(sizeof(int), &known_cut_cnt_, CkReduction::sum_int),
      CkReduction::tupleElement(sizeof(int), &cost_cut_cnt_, CkReduction::sum_int),
      CkReduction::tupleElement(sizeof(int), &eval_cnt_, CkReduction::sum_int)
    };

    CkReductionMsg* msg = CkReductionMsg::buildFromTuple(tuple_red_n, 4);

    CkCallback cb(CkReductionTarget(Master, getStat), mainProxy);

    msg->setCallback(cb);

    contribute(msg);
  }
};

class Slave : public CBase_Slave {

  int iter_;
  OsiClpSolverInterface *si_;
  CoinPackedMatrix matrix;

public:

  Slave(CkMigrateMessage* msg){}
  
  Slave(double parent_cost, constraint_t constraints, int branch_col) : matrix(false, 0, 0) {

    si_ = NULL;

    Cache *cache = cacheProxy.ckLocalBranch();
    if (!cache->checkCPP(parent_cost)) {
      cache->addFinishedConstraint(constraints.first);
      return;
    }

    iter_ = 0;

    si_ = new OsiClpSolverInterface;

    CoinMessageHandler *msg_hdl = si_->messageHandler();
    msg_hdl->setLogLevel(0);

    calcLP(parent_cost, constraints, branch_col);

  }

  virtual ~Slave() {
    if (si_) {
      delete si_;
    }
  }

  void calcLP(double parent_cost, constraint_t constraints, int branch_col){

    constraint_set_t &constraint_set = constraints.first;
    subtour_set_t &subtour_set = constraints.second;

    int nbranch = 2;
    if (branch_col == -1) {
      nbranch = 1;
    }

    Cache *cache = cacheProxy.ckLocalBranch();

    for (int branch = 0; branch < nbranch; ++branch) {

      constraint_set_t::iterator cons_itr;
      if (branch_col != -1) {
        cons_itr = constraint_set.insert(constraint_pair_t(branch_col, branch)).first;
      }

      if (!cache->addConstraint(constraints)) {
        continue;
      }

      //CkPrintf("Visited %d cities, remained %d cities, slave initiated\n", visited.size(), remained.size());

      // Create a problem pointer.  We use the base class here.

      // When we instantiate the object, we need a specific derived class.


      double *objective = objective_vec.data();
      double *col_lb = col_lb_vec.data();
      double *col_ub = col_ub_vec.data();

      // Constraint
      // Sum: x_ij = 2
      size_t n_rows = problem.size(); // 1; // V;
      double *row_lb = new double[n_rows + constraint_set.size() + subtour_set.size()]; //the row lower bounds
      double *row_ub = new double[n_rows + constraint_set.size() + subtour_set.size()]; //the row upper bounds

      // define the constraint matrix
      matrix.clear();
      matrix.setDimensions(0, n_cols);
      
      for (int i = 0; i < n_rows; i++) {
        CoinPackedVector vec((int) base_constraint_vec[i].size(), base_constraint_vec[i].data(), 1.0);
        row_lb[i] = 2;
        row_ub[i] = 2;
        matrix.appendRow(vec);
      }

      int cnt = 0;
      for (constraint_set_t::iterator i = constraint_set.begin(); i != constraint_set.end(); ++i) {
        CoinPackedVector vec;
        vec.insert(i->first, 1.0);
        row_lb[n_rows + cnt] = i->second;
        row_ub[n_rows + cnt++] = i->second;
        matrix.appendRow(vec);
      }

      cnt = 0;
      for (subtour_set_t::iterator i = subtour_set.begin(); i != subtour_set.end(); ++i) {
        const vector<int> &path = *i;

        CoinPackedVector vec;
        int path_size = path.size();
        for (int j = 0; j < path_size; j++) {
          int row = path[j];
          int col = path[(j + 1) % path_size];

          vec.insert(variable_map[row][col], 1.0);
        }
        row_lb[n_rows + constraint_set.size() + cnt] = 0.0;
        row_ub[n_rows + constraint_set.size() + cnt++] = (double)(path_size - 1);
        matrix.appendRow(vec);
      }

      si_->loadProblem(matrix, col_lb, col_ub, objective, row_lb, row_ub);
      si_->initialSolve();
      cache->finishEval();

      if (si_->isProvenOptimal()) {
        int n = si_->getNumCols();
        double* solution = new double[n];
        memcpy(solution, si_->getColSolution(), sizeof(double) * n);

        double cost = si_->getObjValue();

        if (cache->checkCPP(cost)) {

          bool is_int = true;
          for (int i = 0; i < n; ++i) {
            if (solution[i] == 1 || solution[i] == 0) {
              continue;
            }
            is_int = false;

            if (++iter_ >= granularity) {
              CkEntryOptions opts;
              opts.setPriority((int) cost);
              opts.setQueueing(CK_QUEUEING_ILIFO);
              CProxy_Slave::ckNew(cost, constraints, i, CK_PE_ANY, &opts);
            } else {
              calcLP(cost, constraints, i);
            }

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
                graph[i][j] = (int) (solution[variable_map[i][j]]);
                graph[j][i] = (int) (solution[variable_map[j][i]]);
              }
            }

            vector<int> path = findShortestPath(graph);

            if (path.size() == dist.size()) {
              /*CkPrintf("Total cost = %lf #constraint = %d ", cost, constraint_set.size());
              for (constraint_set_t::iterator i = constraint_set.begin(); i != constraint_set.end(); ++i) {
                CkPrintf("%d=%d ", i->first, (int) i->second);
              }
              CkPrintf("#subtour_constraint = %d\n", subtour_set.size());*/
              cache->sendCTP(cost, n, solution, constraint_set);
            } else {
              //CkPrintf("Add subtour %d constraints\n", subtour_vec.size() + 1);
              subtour_set.insert(path);
              CkEntryOptions opts;
              opts.setPriority(0);
              opts.setQueueing(CK_QUEUEING_IFIFO);
              if (cache->testConstraint(constraints)) {
                CProxy_Slave::ckNew(cost, constraints, -1, CK_PE_ANY, &opts);
              }
            }
          } else {
            //CkPrintf("Partial cost = %lf #constraint = %d\n", cost, msg->vec_length);
          }

        } else {
          cache->addFinishedConstraint(constraint_set);
          //CkPrintf("Dropped cost = %lf #constraint = %d\n", cost, msg->vec_length);
        }
      }

      delete row_ub;
      delete row_lb;

      if (branch_col != -1) {
        constraint_set.erase(cons_itr);
      }
    }

  }


};


#include "ilpprune.def.h"