#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"

#include <uWS/uWS.h>

#include <vector>
#include <string>

class Twiddle {
 public:
  /**
   * Constructor
   */
  Twiddle();

  /**
   * Destructor.
   */
  virtual ~Twiddle();

  /**
   * Initialize PID.
   * @param pid: the pid object to be modified
   * @param start_iter: starting point to accumulate error
   * @param end_iter: Point to use the accumulated error and do a Twiddle step
   * @param twiddle_tol: Tolerance to be reached by twiddle to stop
   * @param twiddle_max_iter: Maximum number of Twiddle steps
   * @param Kp, Ki, Kd: Initial values for the parameters
   * @param dKp, dKi, dKd: Initial values for the delta values of the parameters
   */
  void Init(PID &pid_, int start_iter_, int end_iter_, double twiddle_tol_, int twiddle_max_iter_,
                    double Kp, double Ki, double Kd, double dKp, double dKi, double dKd);

  /**
   * Update the PID error variables given cross track error and call Execute as needed.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

   /**
   * Execute one step of Twiddel.
   * @param 'none'
   */
  void Execute();

   /**
   * Return the best found combination.
   * @param 'none'
   */
  std::vector<double> Get_best();

 /*
  * Taken from https://github.com/bguisard/CarND-PID-Control-Project
  * Restarts the robot.
  */
  void RestartSim();

  /*
  * Taken from https://github.com/bguisard/CarND-PID-Control-Project
  * Stores server info.
  * @param ws: The WebSocket server
  */
  void SetServer(uWS::WebSocket<uWS::SERVER> ws);

  bool twiddle_converged;
  bool twiddle_done;
  int twiddle_iter;

 private:
  /**
   * PID controller
   */
  PID *pid;

  /**
   * Errors:
   */
  double error;
  double best_error;

  /**
   * Iteration counter, start and end point for error calculation, pid.Init(p[0], p[1], p[2]);
   */
  int start_iter;
  int end_iter;
  double twiddle_tol;
  int twiddle_max_iter;
  int simulation_iter;
  int parameter_index;
  int best_iteration;
  std::string next_twiddle_step;
  std::string prev_twiddle_step;
  std::vector<double> best_p;
  std::vector<double> p;
  std::vector<double> dp;

  /**
   * Web socet server
   */
  uWS::WebSocket<uWS::SERVER> server;
};

#endif  // TWIDDLE_H