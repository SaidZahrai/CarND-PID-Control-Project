#include "Twiddle.h"
#include "PID.h"

#include <vector>
#include <string>

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(PID &pid_, int start_iter_, int end_iter_, double twiddle_tol_, int twiddle_max_iter_,
                    double Kp, double Ki, double Kd, double dKp, double dKi, double dKd) {
    pid = &pid_;

    start_iter = start_iter_;
    end_iter = end_iter_;

    twiddle_tol = twiddle_tol_;
    twiddle_max_iter = twiddle_max_iter_;

    twiddle_iter = 0;
    twiddle_converged = false;
    twiddle_done = false;
    simulation_iter = 0;
    best_iteration = 0;

    parameter_index = 0;

    p = {Kp, Ki, Kd};
    best_p = {Kp, Ki, Kd};
    dp = {dKp, dKi, dKd};
    pid->Init(p[0], p[1], p[2]);

    error = 0.0;
    best_error = 0.0;

    next_twiddle_step = "START";
    prev_twiddle_step = "START";

}

void Twiddle::UpdateError(double cte) {
    //std::cout << "Simulation iter: " << simulation_iter << std::endl;
    if (twiddle_done) return;
    if (simulation_iter > start_iter){
        error += cte*cte;
    }
    if (simulation_iter == end_iter) {
        Execute();
    }
    simulation_iter++;
}


void Twiddle::Execute() {
    std::cout << "Twiddle iteration: " << twiddle_iter << " parameter " << parameter_index << std::endl << std::endl;
    std::cout << "p = [" << p[0] << "," << p[1] << "," << p[2] << "]" << std::endl;
    std::cout << "dp = [" << dp[0] << "," << dp[1] << "," << dp[2] << "]" << std::endl;
    std::cout << "Current error: " << error << std::endl << std::endl;
    if (next_twiddle_step == "START"){
        best_error = error;
        p[parameter_index] += dp[parameter_index];

        prev_twiddle_step = next_twiddle_step;
        next_twiddle_step = "CHECK_PLUS";
        error = 0.0;
        simulation_iter = 0;
        RestartSim();
    } else if (next_twiddle_step == "CHECK_PLUS"){
        if (parameter_index == 0){
            twiddle_iter ++;
            std::cout << "Twiddle iteration " << twiddle_iter << " Best error = " << best_error << 
                            " found at iteration: " << best_iteration << std::endl;
            std::cout << "best p = [" << best_p[0] << "," << best_p[1] << "," << best_p[2] << "]" << std::endl;
            if ((std::fabs(dp[0]) + std::fabs(dp[0]) + std::fabs(dp[0])) < twiddle_tol) {
                twiddle_converged = true;
                twiddle_done = true;
                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl; 
                std::cout << "Twiddle converged after " << twiddle_iter << "iterations. Error = " << error << std::endl; 
                std::cout << "Twiddle converged. p[0] = " << p[0] << " p[1] = " << p[1] << " p[1] = " << p[2] << std::endl; 
                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl; 
            }
            if (twiddle_iter > twiddle_max_iter) {
                twiddle_converged = false;
                twiddle_done = true;
                std::cout << "--------------------------------------------------" << std::endl; 
                std::cout << "Twiddle did not converge after " << twiddle_iter << "iterations. Error = " << error << std::endl; 
                std::cout << "Twiddle converged. p[0] = " << p[0] << " p[1] = " << p[1] << " p[1] = " << p[2] << std::endl; 
                std::cout << "--------------------------------------------------" << std::endl; 
            }
        }
        if (error < best_error) {
            best_error = error;
            best_p[parameter_index] = p[parameter_index];
            best_iteration = twiddle_iter;
            dp[parameter_index] *= 1.1;
            parameter_index = (parameter_index + 1) % p.size();

            p[parameter_index] += dp[parameter_index];
            pid->Init(p[0], p[1], p[2]);

            prev_twiddle_step = next_twiddle_step;
            next_twiddle_step = "CHECK_PLUS";
            error = 0.0;
            simulation_iter = 0;
            RestartSim();
        } else {
            p[parameter_index] -= 2.0 * dp[parameter_index];

            pid->Init(p[0], p[1], p[2]);

            prev_twiddle_step = next_twiddle_step;
            next_twiddle_step = "CHECK_MINUS";
            error = 0.0;
            simulation_iter = 0;
            RestartSim();
        }
    } else if (next_twiddle_step == "CHECK_MINUS"){
        if (error < best_error) {
            best_error = error;
            best_p[parameter_index] = p[parameter_index];
            best_iteration = twiddle_iter;
            dp[parameter_index] *= 1.1;
            parameter_index = (parameter_index + 1) % p.size();

            p[parameter_index] += dp[parameter_index];
            pid->Init(p[0], p[1], p[2]);

            prev_twiddle_step = next_twiddle_step;
            next_twiddle_step = "CHECK_PLUS";
            error = 0.0;
            simulation_iter = 0;
            RestartSim();
        } else {
            p[parameter_index] += dp[parameter_index];

            dp[parameter_index] *= 0.9;
            parameter_index = (parameter_index + 1) % p.size();

            p[parameter_index] += dp[parameter_index];
            pid->Init(p[0], p[1], p[2]);

            prev_twiddle_step = next_twiddle_step;
            next_twiddle_step = "CHECK_PLUS";
            error = 0.0;
            simulation_iter = 0;
            RestartSim();
        }
    }
}

   /**
   * Return the best found combination.
   * @param 'none'
   */
std::vector<double> Twiddle::Get_best(){
    return best_p;
};


 /*
  * Taken from https://github.com/bguisard/CarND-PID-Control-Project
  * Restarts the robot.
  */
void Twiddle::RestartSim() {
  std::string reset_msg = "42[\"reset\",{}]";
  server.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

 /*
  * Taken from https://github.com/bguisard/CarND-PID-Control-Project
  * Stores server info.
  */
void Twiddle::SetServer(uWS::WebSocket<uWS::SERVER> ws) {
  server = ws;
}