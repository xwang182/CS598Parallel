// Example of using COIN-OR OSI, building the instance internally
// with sparse matrix object

#include <iostream>
#include "OsiClpSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"

#include <vector>
#include <queue>
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
  ifstream infile(file_path.c_str());
  string line;
  int count = 0;
  bool foundEntry = false;
  while (getline(infile, line)) {
    if (line[0] == '1') {
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

void solveLP(distance_t dist, int variable_offset, int variable_value)
{
  // Create a problem pointer.  We use the base class here.
  OsiSolverInterface *si;

  // When we instantiate the object, we need a specific derived class.
  si = new OsiClpSolverInterface;

  // distance_t dist = copyDistanceMap(d);
  size_t V = dist.size();

  size_t n_cols = dist.size() * (dist.size() - 1) / 2;
  double *objective = new double[n_cols];
  double *col_lb = new double[n_cols];
  double *col_ub = new double[n_cols];

  // define the objective coefficients
  // minimize Sum: d_ij x_ij

  int count = 0;
  idx_map_t variable_map;
  /*

  for (int i = 0; i < dist.size(); i++) {
    for (int j = 0; j < dist.size(); j++) {
      if (i < dist.size() - 1 && j >= dist.size() - 1 - i) {

      }
    }
  }
  */
  for (size_t i = 0; i < dist.size(); i++) {
    vector<int> row;
    row.resize(dist.size());
    variable_map.push_back(row);
  }

  for (size_t i = 0; i < dist.size() - 1; i++) {
    for (size_t j = i + 1; j < dist.size(); j++) {
      objective[count] = dist[i][j];
      // cout << count << ": " << objective[count] << endl;

      // set variable map
      variable_map[i][j] = count;
      variable_map[j][i] = count;

      count++;
    }
  }

  /*
  for (int i = 0; i < variable_map.size(); i++) {
    for (int j = 0; j < variable_map[i].size(); j++) {
      cout << i << " " << j << ": " << variable_map[i][j] << endl;
    }
  }
  */

  // TODO: refactor
  // define the variable lower/upper bounds
  // 0 <= x_ij <= 1
  for (size_t i = 0; i < n_cols; i++) {
    col_lb[i] = 0;
    col_ub[i] = 1;
  }

  // Constraint
  // Sum: x_ij = 2
  size_t n_rows = V; // 1; // V;
  double *row_lb = new double[n_rows]; //the row lower bounds
  double *row_ub = new double[n_rows]; //the row upper bounds

  // define the constraint matrix
  CoinPackedMatrix *matrix = new CoinPackedMatrix(false, 0, 0);
  matrix->setDimensions(0, (int)n_cols);


  for (size_t i = 0; i < n_rows; i++) {
    CoinPackedVector vec;
    // cout << "\n@ " << i << endl;
    for (size_t j = 0; j < n_rows; j++) {
      if (i == j) continue;
      vec.insert(variable_map[i][j], 1.0);
      // cout << variable_map[i][j] << endl;
    }

    // Sum: x_ij = 2
    row_lb[i] = 2;
    row_ub[i] = 2;
    matrix->appendRow(vec);
  }

  si->loadProblem(*matrix, col_lb, col_ub, objective, row_lb, row_ub);

  si->writeMps("tsp");


  // Solve the (relaxation of the) problem
  si->initialSolve();
  if (si->isProvenOptimal()) {
    int n = si->getNumCols();
    const double* solution = si->getColSolution();

    // cout << "Solution: " << endl;
    bool all_integers = true;
    for (int i = 0; i < n; i++) {
      if (!(solution[i] == 1 || solution[i] == 0)) {
        all_integers = false;
        break;
      }
    }

    if (all_integers) {
      // subtour
      // 1 component
      //    compare to best_cost
      // > 1 component
      //    add subtour constraint
    } else {
      for (int i = 0; i < n; i++) {
        // if (solution[i] == 1 || solution[i] == 0) continue;
        // solveLP(dist);
      }
    }

  } else { // no solution
    return;
  }
  return;
}

class Constraint
{
public:
  Constraint();
  Constraint(const Constraint &obj);
  double* getRowLowerBound();
  double* getRowUpperBound();
  void addConstraint(double lb, double ub, CoinPackedVector vec);
  vector<CoinPackedVector> getPackedVectors();

private:
  vector<double> row_lb;
  vector<double> row_ub;
  vector<CoinPackedVector> vecs;
};

Constraint::Constraint() {}
Constraint::Constraint(const Constraint &obj)
{
  row_lb = obj.row_lb;
  row_ub = obj.row_ub;
  vecs = obj.vecs;
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
  vector<Constraint> constraints;
  constraints.push_back(initial_constraint);

  const double* final_solution;
  int final_num_sols;

  int iter = 0;
  while (constraints.size() != 0) {
    cout << "\nIteration: " << (iter) << endl;
    iter++;

    Constraint constraint = constraints.back();
    constraints.pop_back();

    double *row_lb = constraint.getRowLowerBound();
    double *row_ub = constraint.getRowUpperBound();
    vector<CoinPackedVector> vecs = constraint.getPackedVectors();

    // define the constraint matrix
    CoinPackedMatrix *matrix = new CoinPackedMatrix(false, 0, 0);
    matrix->setDimensions(0, (int)n_cols);

    for (size_t i = 0; i < vecs.size(); i++) {
      matrix->appendRow(vecs[i]);
    }
    cout << "row_constraint: " << vecs.size() << endl;

    // Create a problem pointer.  We use the base class here.
    // When we instantiate the object, we need a specific derived class.
    OsiSolverInterface *si; = new OsiClpSolverInterface;
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
        // std::cout << si->getColName(i) << " = " << solution[i] << std::endl;
      }

      double cost = calculateCost(objective, solution, n);
      if (best_cost == -1 || best_cost > cost) {
        best_cost = cost;
      } else { // prune
        continue;
      }

      if (all_integers) {
        // subtour
        // 1 component
        //    compare to best_cost
        // > 1 component
        //    add subtour constraint
        if (cost == best_cost) {
          final_solution = solution;
          final_num_sols = n;
        }
      } else {
        for (int i = 0; i < n; i++) {
          if (solution[i] == 1 || solution[i] == 0) continue;
          // branch and bound
          // LEFT: 0
          Constraint new_constraint_1(constraint); // new constraint
          CoinPackedVector vec_1;
          vec_1.insert(i, 1.0);
          new_constraint_1.addConstraint(0.0, 0.0, vec_1);

          // RIGHT: 1
          Constraint new_constraint_2(constraint); // new constraint
          CoinPackedVector vec_2;
          vec_2.insert(i, 1.0);
          new_constraint_1.addConstraint(1.0, 1.0, vec_1);

          constraints.push_back(new_constraint_1);
          constraints.push_back(new_constraint_2);
        }
      }
    } else {
      // no optimal solution found...
    }
  }

  cout << "Best Cost: " << best_cost << endl;
  cout << "n_cols: " << n_cols << endl;
  cout << "final_num_sols: " << final_num_sols << endl;
  // for (size_t i = 0; i < final_num_sols; i++) {
    // cout << i << ": " << final_solution[i] << endl;
  // }



  // Build our own instance from scratch

  /*
   * Traveling Salesman Problem
   *
   *  http://examples.gurobi.com/traveling-salesman-problem/
   *
   *  minimize Sum: d_ij * x_ij
   *  s.t       Sum: x_ij = 2
   *            Sum: x_ij < |S| - 1
   */
   /*
  int n_cols = 2;
  double *objective    = new double[n_cols];//the objective coefficients
  double *col_lb       = new double[n_cols];//the column lower bounds
  double *col_ub       = new double[n_cols];//the column upper bounds

  //Define the objective coefficients.
  //minimize -1 x0 - 1 x1
  objective[0] = -1.0;
  objective[1] = -1.0;

  //Define the variable lower/upper bounds.
  // x0 >= 0   =>  0 <= x0 <= infinity
  // x1 >= 0   =>  0 <= x1 <= infinity
  col_lb[0] = 0.0;
  col_lb[1] = 0.0;
  col_ub[0] = si->getInfinity();
  col_ub[1] = si->getInfinity();

  int n_rows = 2;
  double *row_lb = new double[n_rows]; //the row lower bounds
  double *row_ub = new double[n_rows]; //the row upper bounds

  //Define the constraint matrix.
  CoinPackedMatrix *matrix =  new CoinPackedMatrix(false,0,0);
  matrix->setDimensions(0, n_cols);

  //1 x0 + 2 x1 <= 3  =>  -infinity <= 1 x0 + 2 x2 <= 3
  CoinPackedVector row1;
  row1.insert(0, 1.0);
  row1.insert(1, 2.0);
  row_lb[0] = -1.0 * si->getInfinity();
  row_ub[0] = 3.0;
  matrix->appendRow(row1);

  //2 x0 + 1 x1 <= 3  =>  -infinity <= 2 x0 + 1 x1 <= 3
  CoinPackedVector row2;
  row2.insert(0, 2.0);
  row2.insert(1, 1.0);
  row_lb[1] = -1.0 * si->getInfinity();
  row_ub[1] = 3.0;
  matrix->appendRow(row2);

  //load the problem to OSI
  si->loadProblem(*matrix, col_lb, col_ub, objective, row_lb, row_ub);

  //write the MPS file to a file called example.mps
  si->writeMps("example");



  // Solve the (relaxation of the) problem
  si->initialSolve();

  // Check the solution
  if ( si->isProvenOptimal() ) {
    std::cout << "Found optimal solution!" << std::endl;
    std::cout << "Objective value is " << si->getObjValue() << std::endl;

    int n = si->getNumCols();
    const double* solution = si->getColSolution();

    // We can then print the solution or could examine it.
    for( int i = 0; i < n; ++i )
      std::cout << si->getColName(i) << " = " << solution[i] << std::endl;

  } else {
    std::cout << "Didn't find optimal solution." << std::endl;
    // Could then check other status functions.
  }*/

  return 0;
}