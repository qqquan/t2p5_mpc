#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


typedef struct{
    vector<double> actuations;
    vector<double> predicted_x;
    vector<double> predicted_y;
    vector<double> predicted_psi;

} IpoptSolution_Type;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  IpoptSolution_Type Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
