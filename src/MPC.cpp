#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
// N = 25, dt=.05, slow to process
// N = 13, dt=.1, works well
// N = 13, dt=.15, Looks really far out, rough around sharp corners
// N = 13, dt=.1, works well
// N = 7, dt=.1, doesnt look far enough ahead
// N = 7, dt=.15, does not get a good formed polynomial
// N = 9, dt=.15, Does not work well around sharp corners
// N = 10, dt=.1, works well
// N = 11, dt=.11, works well
// N = 11, dt=.09, works well
//N = 10, dt=.1, Used this in the end to make latency 'delay' easier as dt=.1 is equivalent
//to 100ms for our delay in processing.


size_t N = 10;
double dt = 0.1;

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
const double Lf = 2.67;

//Costs
const int cte_cost = 15;
const int epsi_cost = 15;
const int v_cost = 1;
const int delta_cost = 5;
const int a_cost = 1;
const int delta_d_cost = 1000;
const int delta_a_cost = 5;

//Reference

const double cte_ref = 0;
const double epsi_ref = 0;
const double v_ref = 40;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

  //  Calculate Costs
    fg[0] = 0;
    
    for ( int i = 0; i < N; i++ ) {
    	fg[0] += cte_cost * CppAD::pow( vars[ cte_start + i ] - cte_ref, 2 );
    	fg[0] += epsi_cost * CppAD::pow( vars[ epsi_start + i ] - epsi_ref, 2 );
    	fg[0] += v_cost * CppAD::pow( vars[ v_start + i ] - v_ref, 2 );
    }
    
    for ( int i = 0; i < N - 1; i++ ) {
    	fg[0] += delta_cost * CppAD::pow( vars[ delta_start + i ], 2 );
    	fg[0] += a_cost * CppAD::pow( vars[ a_start + i ], 2 );
    }
    
    for ( int i = 0; i < N - 2; i++ ) {
    	fg[0] += delta_d_cost * CppAD::pow( vars[ delta_start + i + 1 ] - 
    		vars[ delta_start + i], 2 );
    	fg[0] += delta_a_cost * CppAD::pow( vars[ a_start + i + 1 ] - 
    		vars[ a_start + i], 2 );
  }
  
  //Initial Constraints (Pulled from Lesson 19)
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];
	
		for (int t = 1; t < N; t++) {
			// The state at time t+1 .
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];

			// The state at time t.
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];

			// Only consider the actuation at time t.
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];
			AD<double> x0_2 = x0 * x0;
			AD<double> x0_3 = x0_2 * x0;
			AD<double> f0 = coeffs[0] + ( coeffs[1] * x0 ) + ( coeffs[2] * x0_2 ) + ( coeffs[3] * x0_3 );
			AD<double> psides0 = CppAD::atan( coeffs[1] + ( 2 * coeffs[2] * x0 ) + ( 3 * coeffs[3] * x0_2 ) );
		
			fg[ 1 + x_start + t ] = x1 - ( x0 + v0 * CppAD::cos( psi0 ) * dt );
			fg[ 1 + y_start + t ] = y1 - ( y0 + v0 * CppAD::sin( psi0 ) * dt );
			fg[ 1 + psi_start + t ] = psi1 - ( psi0 + v0 * delta0 / Lf * dt );
			fg[ 1 + v_start + t ] = v1 - ( v0 + a0 * dt );
			fg[ 1 + cte_start + t ] = cte1 - ( ( f0 - y0 ) + ( v0 * CppAD::sin( epsi0 ) * dt ) );
			fg[ 1 + epsi_start + t ] = epsi1 - ( (psi0 - psides0) + v0 * delta0 / Lf * dt );
		}
	}\
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  //State and Actuator variables
  int n_state = 6;
  int n_actuator = 2;
  int latency = 1;
  size_t n_vars = n_state * N + n_actuator * ( N - 1 );
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  double steering_angle = state[6];
  double throttle = state[7];
  

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  
  for ( int i = 0; i < delta_start; i++ ) {
  	vars_lowerbound[i] = -1.0e19;
  	vars_upperbound[i] = 1.0e19;
  }
  //Bound the upper and lower bounds to current steering angle for the length of our latency.
  for ( int i = delta_start; i < delta_start + latency; i++) {
  	vars_lowerbound[i] = steering_angle;
  	vars_upperbound[i] = steering_angle;
  }
  for ( int i = delta_start + latency; i < a_start; i++ ) {
  	vars_lowerbound[i] = -0.4363;
  	vars_upperbound[i] = 0.4363;
  }
  //Bound the upper and lower bounds to current throttle for the length of our latency.
  for ( int i = a_start; i < a_start + latency; i++) {
  	vars_lowerbound[i] = throttle;
  	vars_upperbound[i] = throttle;
  }
  for ( int i = a_start + latency; i < n_vars; i++ ) {
  	vars_lowerbound[i] = -1.0;
  	vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  vector<double> results;
  results.push_back( solution.x[ delta_start + latency ] ); // skip the 'latency' frame
  results.push_back( solution.x[ a_start + latency ] );
  for( int i = 1; i < N; i++ ) {
  	results.push_back( solution.x[ x_start + i ] );
  	results.push_back( solution.x[ y_start + i ] );
  }
  return results;
}
