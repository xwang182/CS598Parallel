// Example of using COIN-OR OSI, building the instance internally
// with sparse matrix object

#include <iostream>
#include "OsiClpSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"

#include <vector>
#include <queue>
#include <set>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
using namespace std;

typedef vector< pair<double, double> > map_t;
typedef vector< vector<double> > distance_t;
typedef vector< vector<int> > path_map_t;
typedef path_map_t idx_map_t;
typedef vector<int> path_t;

static double best_cost = -1;

string itos(int i) {stringstream s; s << i; return s.str(); }

map_t readTspFile(string file_path)
{
  map_t output_vector;
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
    }

    count++;
  }

  return output_vector;
}


double calcDistance(const pair<double, double> &a, const pair<double, double> &b)
{
  double dx = a.first - b.first;
  double dy = a.second - b.second;

  return sqrt(dx * dx + dy * dy);
}

distance_t calcDistanceMap(const map_t &problem)
{
  distance_t dist;

  for (size_t i = 0; i < problem.size(); ++i) {
    vector<double> tmp;
    const pair<double, double> &a = problem[i];
    for (size_t j = 0; j < problem.size(); ++j) {

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

/*
distance_t copyDistanceMap(distance_t d) {
  distance_t dist;
  for (int i = 0; i < d.size(); i++) {
    vector<double> row = d[i];
    dist.push_back(row);
  }
  return dist;
}
*/

double calculateCost(double* objective, const double* solution, int num_sols)
{
  double cost = 0;
  for (int i = 0; i < num_sols; i++) {
    cost += objective[i] * solution[i];
  }
  return cost;
}

class Constraint
{
public:
  Constraint();
  Constraint(double p);
  Constraint(const Constraint &obj, double parent_cost);
  bool operator< (const Constraint &right) const;
  double* getRowLowerBound();
  double* getRowUpperBound();
  void addConstraint(double lb, double ub, CoinPackedVector vec);
  vector<CoinPackedVector> getPackedVectors();

private:
  vector<double> row_lb;
  vector<double> row_ub;
  vector<CoinPackedVector> vecs;
  double parent_cost;
};

Constraint::Constraint() {
  parent_cost = 0;
}

Constraint::Constraint(double p) {
  parent_cost = p;
}

Constraint::Constraint(const Constraint &obj, double parent_cost)
{
  row_lb = obj.row_lb;
  row_ub = obj.row_ub;
  vecs = obj.vecs;
  parent_cost = parent_cost;
}

bool Constraint::operator< (const Constraint &right) const
{
  return parent_cost < right.parent_cost;
}

double* Constraint::getRowLowerBound()
{
  return (double*)(&row_lb[0]);
}
double* Constraint::getRowUpperBound()
{
  return (double*)(&row_ub[0]);
}
vector<CoinPackedVector> Constraint::getPackedVectors()
{
  return vecs;
}
void Constraint::addConstraint(double lb, double ub, CoinPackedVector vec)
{
  row_lb.push_back(lb);
  row_ub.push_back(ub);
  vecs.push_back(vec);
}

bool sortFunc(pair<double, int> v1, pair<double, int> v2)
{
  return abs(v1.first - 0.5) < abs(v2.first - 0.5);
}

void printPath(vector<size_t> path)
{
  cout << "Path: ";
  for (size_t i = 0; i < path.size(); i++) {
    cout << path[i] << " ";
  }
  cout << "" <<endl;
}

vector<size_t> findShortestPath(path_map_t graph) {
  vector<size_t> visited;
  visited.resize(graph.size());

  vector<size_t> path;

  while(true) {
    bool all_visited = true;
    size_t source = 0;
    for (size_t i = 0; i < visited.size(); i++) {
      if (!visited[i]) {
	all_visited = false;
	source = i;
	break;
      }
    }
    if (all_visited)
      break;

    vector<size_t> temp_path;
    size_t original_source = source;
    temp_path.push_back(source);
    visited[source] = 1;
    while (true) {
      for (size_t i = 0; i < graph.size(); i++) {
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

int
main(int argc,
     char* argv[])
{
  if (argc < 2) {
    cout << "Usage: tsp_clp .tsp_file_path" << endl;
    return 1;
  }

  string file_path(argv[1]);
  map_t coords = readTspFile(file_path);
  distance_t dist = calcDistanceMap(coords);
  // solveLP(dist, -1, -1);

  size_t n_cols = dist.size() * (dist.size() - 1) / 2;
  double *objective = new double[n_cols];
  double *col_lb = new double[n_cols];
  double *col_ub = new double[n_cols];

  // define the objective coefficients
  // minimize Sum: d_ij x_ij
  int count = 0;
  idx_map_t variable_map;

  for (size_t i = 0; i < dist.size(); i++) {
    vector<int> row;
    /*for (size_t j = 0; j < dist.size(); j++) {
      row.push_back(-1);
    }*/
    row.resize(dist.size());
    variable_map.push_back(row);
  }

  for (size_t i = 0; i < dist.size() - 1; i++) {
    for (size_t j = i + 1; j < dist.size(); j++) {
      objective[count] = dist[i][j];

      // set variable map
      variable_map[i][j] = count;
      variable_map[j][i] = count;

      count++;
    }
  }

  // define the variable lower/upper bounds
  // 0 <= x_ij <= 1
  for (size_t i = 0; i < n_cols; i++) {
    col_lb[i] = 0;
    col_ub[i] = 1;
  }

  // initial constraints
  Constraint initial_constraint;
  size_t n_rows = dist.size();
  for (size_t i = 0; i < n_rows; i++) {
    CoinPackedVector vec;
    for (size_t j = 0; j < n_rows; j++) {
      if (i == j) continue;
      vec.insert(variable_map[i][j], 1.0);
    }
    initial_constraint.addConstraint(2.0, 2.0, vec);
  }

  // DFS
  set<Constraint> constraints;
  constraints.insert(initial_constraint);

  const double* final_solution = NULL;
  int final_num_sols = -1;
  vector<size_t> final_path;

  int iter = 0;
  while (!constraints.empty()) {
    // cout << "Iteration: " << (iter) << endl;
    iter++;

    Constraint constraint = *(constraints.begin());
    constraints.erase(constraints.begin());
    // constraints.pop_back();

    double *row_lb = constraint.getRowLowerBound();
    double *row_ub = constraint.getRowUpperBound();
    vector<CoinPackedVector> vecs = constraint.getPackedVectors();

    // define the constraint matrix
    CoinPackedMatrix *matrix = new CoinPackedMatrix(false, 0, 0);
    matrix->setDimensions(0, (int)n_cols);

    for (size_t i = 0; i < vecs.size(); i++) {
      matrix->appendRow(vecs[i]);
    }
    //    cout << "row_constraint: " << vecs.size() << endl;
    //    cout << "n_cols: " << n_cols << endl;

    // Create a problem pointer.  We use the base class here.
    // When we instantiate the object, we need a specific derived class.
    OsiSolverInterface *si = new OsiClpSolverInterface;

    // remove message level
    CoinMessageHandler *msg = si->messageHandler();
    msg->setLogLevel(0);

    si->loadProblem(*matrix, col_lb, col_ub, objective, row_lb, row_ub);
    si->initialSolve();


    if (si->isProvenOptimal()) {
      int n = si->getNumCols();
      // cout << "n: " << n << endl;
      const double* solution = si->getColSolution();

      int all_integers = true;
      for (int i = 0; i < n; i++) {
        if (!(solution[i] == 1 || solution[i] == 0)) {
          all_integers = false;
          break;
        }
        // std::cout << si->getColName(i) << " = " << solution[i] << std::endl;
      }


      double cost = si->getObjValue(); // calculateCost(objective, solution, n)
      if (best_cost != -1 && cost > best_cost) continue; // prune

      if (all_integers) {
        // subtour
        // 1 component
        //    compare to best_cost
        // > 1 component
        //    add subtour constraint
        /*
      	if (best_cost == -1 || best_cost > cost) {
      	  best_cost = cost;
      	  final_solution = solution;
          final_num_sols = n;
      	}*/

        path_map_t graph; // adjacency matrix
        for (size_t i = 0; i < dist.size(); i++) {
          vector<int> row;
          row.resize(dist.size());
          graph.push_back(row);
        }

        // set up graph
        for (size_t i = 0; i < dist.size() - 1; i++) {
          for (size_t j = i + 1; j < dist.size(); j++) {
            graph[i][j] = (int)(solution[variable_map[i][j]]);
            graph[j][i] = (int)(solution[variable_map[j][i]]);
          }
        }

	vector<size_t> path = findShortestPath(graph);

        // find subtour
        if (path.size() == dist.size()) { // no subtour
	  // printPath(path);
	  //if (best_cost == -1 || cost <= best_cost) {
	    best_cost = cost;
	    final_solution = solution;
	    final_num_sols = n;
	    final_path = path;
	    //}
        } else {
	  // cout << "subtour" << endl;
	  // printPath(path);
	  /*
	  // cout << graph[0][26] << " " << graph[26][0] << endl;

	  for (size_t i = 0; i < dist.size(); i++) {
	    cout << i << ": ";
	    for (size_t j = 0; j < dist.size(); j++) {
	      cout << graph[i][j] << " ";
	    }
	    cout << "" << endl;
	  }*/


          Constraint new_constraint(constraint, cost); // new constraint

          CoinPackedVector vec;
          size_t path_size = path.size();
	  //	  if (path_size == 2) continue; // path_size = 1;
          for (size_t i = 0; i < path_size; i++) {
            size_t row = path[i];
            size_t col = path[(i+1) % path_size];

            vec.insert(variable_map[row][col], 1.0);
          }

          new_constraint.addConstraint(1.0, (double)(path_size - 1), vec);

          constraints.insert(new_constraint);
        }
      } else {
        vector<pair<double, int>> non_integer_sols;

        for (int i = 0; i < n; i++) {
          if (solution[i] == 1 || solution[i] == 0) continue;
	  // cout << solution[i] << endl;
          non_integer_sols.push_back(make_pair(solution[i], i));
	}

	// sort non_integer_sols so that by the order of i ~ 0.5
	std::sort(non_integer_sols.begin(), non_integer_sols.end(), sortFunc);

        for (size_t i = 0; i < non_integer_sols.size(); i++) {
	  // cout << non_integer_sols[i].first << " " << non_integer_sols[i].second << endl;

	  // int offset = i;
          int offset = non_integer_sols[i].second;
          // branch and bound
          // LEFT: 0
          Constraint new_constraint_1(constraint, cost); // new constraint
          CoinPackedVector vec_1;
          vec_1.insert(offset, 1.0);
          new_constraint_1.addConstraint(0.0, 0.0, vec_1);

          // RIGHT: 1
          Constraint new_constraint_2(constraint, cost); // new constraint
          CoinPackedVector vec_2;
          vec_2.insert(offset, 1.0);
          new_constraint_2.addConstraint(1.0, 1.0, vec_2);

          //	  cout << new_constraint_1.getPackedVectors().size() << endl;
          //	  cout << new_constraint_2.getPackedVectors().size() << endl;

          constraints.insert(new_constraint_1);
          constraints.insert(new_constraint_2);
        }
      }
    } else {
      // no optimal solution found...
    }
  }

  cout << "Iterations: " << iter << endl;
  cout << "Best Cost: " << best_cost << endl;
  // cout << "n_cols: " << n_cols << endl;
  cout << "Path Size: " << final_path.size() << endl;
  cout << "final_num_sols: " << final_num_sols << endl;
  if (final_solution) {
    printPath(final_path);
  }

  return 0;
}
