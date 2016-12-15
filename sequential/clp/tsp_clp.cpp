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
#include <algorithm>
using namespace std;

typedef vector< pair<double, double> > map_t;
typedef vector< vector<double> > distance_t;
typedef vector< vector<int> > path_map_t;
typedef set< pair<int, double> > var_coeff_set_t;
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

double calculateCost(double* objective, const double* solution, int num_sols)
{
  double cost = 0;
  for (int i = 0; i < num_sols; i++) {
    cost += objective[i] * solution[i];
  }
  return cost;
}

static int ROW_COUNTER = 0;
class Row
{
public:
  Row(double lb, double ub);
  Row(double lb, double ub, var_coeff_set_t var_coeffs);
  bool operator< (const Row &right) const;
  bool operator== (const Row &right) const;
  void addVarCoeff(int idx, double coeff);
  double getLowerBound();
  double getUpperBound();
  CoinPackedVector getPackedVector();
  int counter;
private:
  // int counter;
  double lb;
  double ub;
  var_coeff_set_t var_coeffs;
};

Row::Row(double l, double u)
{
  lb = l;
  ub = u;
  counter = ROW_COUNTER;
  ROW_COUNTER++;
}

Row::Row(double l, double u, var_coeff_set_t c)
{
  lb = l;
  ub = u;
  var_coeffs = c;
  counter = ROW_COUNTER;
  ROW_COUNTER++;
}

bool Row::operator< (const Row &right) const
{
  return counter < right.counter;
}
bool Row::operator== (const Row &right) const
{
  return (counter == right.counter) || (lb == right.lb && ub == right.ub && var_coeffs == right.var_coeffs);
}
void Row::addVarCoeff(int idx, double coeff)
{
  var_coeffs.insert(pair<int, double>(idx, coeff));
}

double Row::getLowerBound()
{
  return lb;
}
double Row::getUpperBound()
{
  return ub;
}
CoinPackedVector Row::getPackedVector()
{
  CoinPackedVector vec;
  for (auto p : var_coeffs) {
    int idx = p.first;
    double coeff = p.second;
    vec.insert(idx, coeff);
  }
  return vec;
}

static int COUNTER = 0;
class Constraint
{
public:
  Constraint();
  Constraint(double p);
  Constraint(const Constraint &obj, double cost);
  bool operator< (const Constraint &right) const;
  bool operator== (const Constraint &right) const;
  double* getRowLowerBound();
  double* getRowUpperBound();
  vector<CoinPackedVector> getPackedVectors();
  double getParentCost();
  void addRow(Row row);
  void calculateRowsInfo();
  int counter;

private:
  set< Row > rows;
  double parent_cost;
  // int counter;

  vector<CoinPackedVector> vecs;
  vector<double> row_lb;
  vector<double> row_ub;
};

Constraint::Constraint() {
  parent_cost = 0;
  this->counter = COUNTER;
  COUNTER++;
}

Constraint::Constraint(double p) {
  parent_cost = p;
  this->counter = COUNTER;
  COUNTER++;
}

Constraint::Constraint(const Constraint &obj, double cost)
{
  rows = obj.rows;
  parent_cost = cost;
  this->counter = COUNTER;
  COUNTER++;
}

bool Constraint::operator< (const Constraint &right) const
{
  if (parent_cost == right.parent_cost){
    return counter < right.counter;
  }
  return parent_cost < right.parent_cost;
}

bool Constraint::operator== (const Constraint &right) const
{
  if (counter == right.counter) return true;
  if (rows == right.rows) return true;
  return false;
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
void Constraint::addRow(Row row)
{
  rows.insert(row);
}

double Constraint::getParentCost() { return parent_cost; }

void Constraint::calculateRowsInfo()
{
  for (auto row : rows)
  {
    double lb = row.getLowerBound();
    double ub = row.getUpperBound();
    CoinPackedVector vec = row.getPackedVector();

    row_lb.push_back(lb);
    row_ub.push_back(ub);
    vecs.push_back(vec);
  }
}



bool sortFunc(pair<double, int> v1, pair<double, int> v2)
{
  return abs(v1.first - 0.5) < abs(v2.first - 0.5);
}

void printPath(vector<size_t> path)
{
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
    if (path.empty() || temp_path.size() <= path.size()) {
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
    Row row(2.0, 2.0);
    for (size_t j = 0; j < n_rows; j++) {
      if (i == j) continue;
      row.addVarCoeff(variable_map[i][j], 1.0);
    }
    initial_constraint.addRow(row);
  }

  // DFS
  set<Constraint> constraints;
  set<Constraint> global_constraints;
  constraints.insert(initial_constraint);

  const double* final_solution = NULL;
  int final_num_sols = -1;
  vector<size_t> final_path;

  int iter = 0;
  while (!constraints.empty()) {
    // cout << "Iteration: " << (iter) << " " << constraints.size() << endl;
    iter++;

    Constraint constraint = *(constraints.begin());
    constraints.erase(constraints.begin());

    // cout << "size: " << global_constraints.size() << endl;
    if (std::find(global_constraints.begin(), global_constraints.end(), constraint) != global_constraints.end()) {
      cout << "found" << endl;
      continue;
    } else {
      global_constraints.insert(constraint);
    }

    constraint.calculateRowsInfo();
    double *row_lb = constraint.getRowLowerBound();
    double *row_ub = constraint.getRowUpperBound();
    vector<CoinPackedVector> vecs = constraint.getPackedVectors();

    // define the constraint matrix
    CoinPackedMatrix *matrix = new CoinPackedMatrix(false, 0, 0);
    matrix->setDimensions(0, (int)n_cols);

    for (size_t i = 0; i < vecs.size(); i++) {
      matrix->appendRow(vecs[i]);
    }

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
      const double* solution = si->getColSolution();

      int all_integers = true;
      for (int i = 0; i < n; i++) {
        if (!(solution[i] == 1 || solution[i] == 0)) {
          all_integers = false;
          break;
        }
      }


      double cost = si->getObjValue(); // calculateCost(objective, solution, n)
      if (best_cost != -1 && cost > best_cost) continue; // prune

      if (all_integers) {
        // subtour
        // 1 component
        //    compare to best_cost
        // > 1 component
        //    add subtour constraint

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
	        best_cost = cost;
          final_solution = solution;
	        final_num_sols = n;
	        final_path = path;
        } else {
          Constraint new_constraint(constraint, cost); // new constraint

	  size_t path_size = path.size();	    
          Row row(-si->getInfinity(), (double)(path_size - 1));
          for (size_t i = 0; i < path_size; i++) {
            size_t x = path[i];
            size_t y = path[(i+1) % path_size];
            row.addVarCoeff(variable_map[x][y], 1.0);
          }
          new_constraint.addRow(row);

          constraints.insert(new_constraint);
        }
      } else {
        vector<pair<double, int>> non_integer_sols;

        for (int i = 0; i < n; i++) {
          if (solution[i] == 1 || solution[i] == 0) continue;
          non_integer_sols.push_back(make_pair(solution[i], i));
        }

	      // sort non_integer_sols so that by the order of i ~ 0.5
	      std::sort(non_integer_sols.begin(), non_integer_sols.end(), sortFunc);

        for (size_t i = 0; i < non_integer_sols.size(); i++) {
          int offset = non_integer_sols[i].second;
          // branch and bound
          // LEFT: 0
          Constraint new_constraint_1(constraint, cost); // new constraint
          Row row_1(0.0, 0.0);
          row_1.addVarCoeff(offset, 1.0);
          new_constraint_1.addRow(row_1);

          // RIGHT: 1
          Constraint new_constraint_2(constraint, cost); // new constraint
          Row row_2(1.0, 1.0);
          row_2.addVarCoeff(offset, 1.0);
          new_constraint_2.addRow(row_2);

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
