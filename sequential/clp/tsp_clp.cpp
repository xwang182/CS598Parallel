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

// lower bound, upper bound, variable coeffs
typedef pair< pair<double, double>, var_coeff_set_t > row_t;
typedef set< row_t > constraints_t;
typedef pair< pair<double*, double*>, vector<CoinPackedVector> > rows_result_t;

// constraint
//       parent cost,   rows
typedef pair< double, constraints_t > node;

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

void addVarCoeff(var_coeff_set_t &var_coeffs, int idx, double coeff)
{
  var_coeffs.insert(pair<int, double>(idx, coeff));
}


node_t node_new(double parent_cost)
{
  return node_t(parent_cost, set< row_t >());
}

node_t node_new(node_t &node, double parent_cost)
{
  return node_t(parent_cost, node.second);
}

void node_addRow(node_t &node, double lb, double ub, var_coeff_set_t var_coeffs)
{
  row_t row(pair<double, double>(lb, ub), var_coeffs);
  node.second.insert(row);
}

rows_result_t node_getRowsResult(node_t &node)
{
  vector<CoinPackedVector> vecs;
  constraints_t rows = node.second;
  double* row_lb = new double[rows.size()];
  double* row_ub = new double[rows.size()];
  size_t i = 0;
  for (auto row : rows) {
    double lb = row.first.first;
    double ub = row.first.second;
    var_coeff_set_t var_coeffs = row.second;

    row_lb[i] = lb;
    row_ub[i] = ub;

    CoinPackedVector vec;
    for (auto p : var_coeffs) {
      int idx = p.first;
      double coeff = p.second;
      vec.insert(idx, coeff);
    }
    vecs.push_back(vec);
    i++;
  }
  return rows_result_t(
    pair<double*, double*>(
      row_lb,
      row_ub),
    vecs);
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
  node_t root = node_new(0);
  size_t n_rows = dist.size();
  for (size_t i = 0; i < n_rows; i++) {
    var_coeff_set_t var_coeffs;
    for (size_t j = 0; j < n_rows; j++) {
      if (i == j) continue;
      addVarCoeff(var_coeffs, variable_map[i][j], 1.0);
    }
    node_addRow(root, 2.0, 2.0, var_coeffs);
  }

  // DFS
  set<node_t> nodes;
  set<constraints_t> global_constraints;
  nodes.insert(root);

  const double* final_solution = NULL;
  int final_num_sols = -1;
  vector<size_t> final_path;

  int iter = 0;
  while (!nodes.empty()) {
    iter++;

    node_t node = *(nodes.begin());
    nodes.erase(nodes.begin());

    if (global_constraints.find(node.second) != global_constraints.end()) {
      cout << "found" << endl;
      continue;
    } else {
      global_constraints.insert(node.second);
    }

    rows_result_t rows_result = node_getRowsResult(node);
    double *row_lb = rows_result.first.first;
    double *row_ub = rows_result.first.second;
    vector<CoinPackedVector> vecs = rows_result.second;

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
          node_t new_code = node_new(node, cost); // new node

	  size_t path_size = path.size();
          var_coeff_set_t var_coeffs;
          for (size_t i = 0; i < path_size; i++) {
            size_t x = path[i];
            size_t y = path[(i+1) % path_size];
            addVarCoeff(var_coeffs, variable_map[x][y], 1.0);
          }
          node_addRow(new_code,
                            -si->getInfinity(),
                            (double)(path_size - 1),
                            var_coeffs);

          nodes.insert(new_code);
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
          node_t new_code_1 = node_new(node, cost); // new node
          var_coeff_set_t var_coeffs_1;
          addVarCoeff(var_coeffs_1, offset, 1.0);
          node_addRow(new_code_1,
                            0.0,
                            0.0,
                            var_coeffs_1);

          // RIGHT: 1
          node_t new_code_2 = node_new(node, cost); // new node
          var_coeff_set_t var_coeffs_2;
          addVarCoeff(var_coeffs_2, offset, 1.0);
          node_addRow(new_code_2,
                            1.0,
                            1.0,
                            var_coeffs_2);

          nodes.insert(new_code_1);
          nodes.insert(new_code_2);
        }
      }
    } else {
      // no optimal solution found...
    }

    delete[] row_lb;
    delete[] row_ub;
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
