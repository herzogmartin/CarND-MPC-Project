#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// Number of time steps for prediction in MPC
#define MPC_N 10
// maximum duration of prediction steps in ms
#define MPC_MAX_DT 100
// latency in ms
#define MPC_LATENCY 100

// Target speed to be reached
#define MPC_V_REF 100.0

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
#define MPC_LF 2.67

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double viewRange,
                       vector<double> &mpc_x_vals, vector<double> &mpc_y_vals);

  double delta_prev {0};
  double a_prev {0.1};
  int calcTime {20};
};

#endif /* MPC_H */
