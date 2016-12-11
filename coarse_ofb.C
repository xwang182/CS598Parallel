#include <math.h> 
#include <vector>
#include <fstream>
#include <limits>
#include <algorithm>

using namespace std;

typedef vector< pair<double, double> > map_t;
typedef vector< vector<double> > distance_t;
typedef vector< vector<int> > path_map_t;
typedef vector<int> path_t;

#include "coarse_ofb.decl.h"

/* readonly */ CProxy_Master mainProxy;
/* readonly */ CProxy_Cache cacheProxy;
/* readonly */ map_t problem;
/* readonly */ distance_t dist;
/* readonly */ CkNodeGroupID cacheID;

class Master : public CBase_Master {
private:
  double ofub_, oflb_;
  path_t path_;
  CProxy_Slave root_;
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

    mainProxy = thisProxy;
    cacheProxy = CProxy_Cache::ckNew(ofub_);

    cacheID = cacheProxy;
    CkEntryOptions opts;
    opts.setGroupDepID(cacheID);

    path_t visited, remained;
    visited.push_back(0);
    for (int i = 1; i < problem.size(); ++i) {
      remained.push_back(i);
    }
    root_ = CProxy_Slave::ckNew(visited, remained, 1, &opts);

    oflb_ = calcOFLB();
    CkPrintf("OFLB: %lf\n", oflb_);
  }
  Master(CkMigrateMessage* msg){}

  void updateOFUB(double ofub, path_t path) {
    if (ofub < ofub_) {
      ofub_ = ofub;
      path_ = path;
      CkPrintf("Global OFUB updated to %lf\n", ofub);
    }

    if (!qd_) {
      qd_ = true;
      CkStartQD(CkCallback(CkIndex_Master::done(), mainProxy));
    }
  }

  void done() {
    CkPrintf("Final CP: %lf, OFLB: %lf\n", ofub_, oflb_);
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

  double calcOFLB() {
    double sum = 0;
    for (int i = 0; i < problem.size(); ++i) {
      vector<double> row = dist[i];
      sort(row.begin(), row.end());
      for (int j = 0; j < min(2, (int) row.size()); ++j) {
        if (row[j] < numeric_limits<double>::infinity()) {
          sum += row[j];
        }
      }
    }

    return sum / 2;    
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

  void sendCTP(double ctp, path_t path) {
    CmiLock(lock_);
    if (ctp < ofub_) {
      ofub_ = ctp;
      thisProxy.updateOFUB(ctp);
      mainProxy.updateOFUB(ctp, path);
      //CkPrintf("Local OFUB updated to %lf\n", ctp);
    }
    CmiUnlock(lock_);
  }
};


class Slave : public CBase_Slave {
private:

  CProxy_Slave children_;
  path_map_t path_;
  int progress_;
  double cp_;

public:
  Slave(path_t visited, path_t remained){

    //CkPrintf("Visited %d cities, remained %d cities, slave initiated\n", visited.size(), remained.size());

    progress_ = visited.size();

    for (int i = 0; i < problem.size(); ++i) {
      path_.push_back(vector<int>());
    }

    int last_city = visited[0];
    for (int i = 1; i < visited.size(); ++i) {
      int x = last_city;
      int y = visited[i];
      path_[x].push_back(y);
      path_[y].push_back(x);

      last_city = y;
    }

    for (int i = 0; i < remained.size(); ++i) {

      int cur_city = remained[i];
      path_[last_city].push_back(cur_city);
      path_[cur_city].push_back(last_city);

      cp_ = calcCP(path_);

      path_[last_city].pop_back();
      path_[cur_city].pop_back();

      Cache *cache = cacheProxy.ckLocalBranch();

      visited.push_back(cur_city);
      if (visited.size() < problem.size()) {
        bool pass = cache->checkCPP(cp_);

        if (pass) {

          //CkPrintf("Visited %d cities, CPP is %lf, branching...\n", visited.size(), cp_);

          remained.erase(remained.begin() + i);


          CkEntryOptions opts;
          opts.setGroupDepID(cacheID);
          children_ = CProxy_Slave::ckNew(visited, remained, 1, &opts);

          remained.insert(remained.begin() + i, cur_city);

        } else {
          //CkPrintf("Visited %d cities, CPP is %lf, terminated!\n", visited.size(), cp_);
        }
      } else {
        cache->sendCTP(cp_, visited);
        //CkPrintf("CP is %lf\n", cp_);
      }
      visited.pop_back();
    }


  }
  Slave(CkMigrateMessage* msg){}

private:
  double calcCP(const path_map_t &path) {
    double sum = 0;
    for (int i = 0; i < problem.size(); ++i) {
      vector<double> row = dist[i];
      for (int j = 0; j < path_[i].size(); ++j) {
        sum += row[path_[i][j]];
        row[path_[i][j]] = numeric_limits<double>::infinity();
      }

      if (path_[i].size() != 2) {
        sort(row.begin(), row.end());
        for (int j = 0; j < 2 - path_[i].size(); ++j) {
          if (row[j] < numeric_limits<double>::infinity()) {
            sum += row[j];
          }
        }
      }
    }

    return sum / 2;
  }


};


#include "coarse_ofb.def.h"