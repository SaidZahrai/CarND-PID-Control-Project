#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

#include <stdlib.h> 

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[])  {
  uWS::Hub h;

  PID steer_pid, speed_pid;
  Twiddle speed_twiddle, steer_twiddle;
  bool do_speed_tuning = false, do_steer_tuning = false;
  /**
   * TODO: Initialize the pid variable.
   */

  // For manually found initial conditions try: ./pid 0.3 0.001 6.0 0.5 0.001 0.5
  /* First iteration of Twiddle. First speed and then steering angle:
  double v_Kp = 0.167406, v_Ki = 0.00191441, v_Kd = 0.000739548;
  double a_Kp = 0.2217, a_Ki = 0.00118119, a_Kd = 11.5692;
  */
  // Second iteration of Twiddle. First speed and then steering angle:
  // best p speed = [0.217266,0.00118119,17.9162]
  double v_Kp = 0.193591, v_Ki = 0.00191441, v_Kd = 0.000781322;
  double a_Kp = 0.217266, a_Ki = 0.00118119, a_Kd = 17.9162;

  steer_pid.Init(a_Kp, a_Ki, a_Kd);
  speed_pid.Init(v_Kp, v_Ki, v_Kd);

  if (argc == 7) {
    a_Kp = strtod (argv[1], NULL);
    a_Ki = strtod (argv[2], NULL);
    a_Kd = strtod (argv[3], NULL);
    v_Kp = strtod (argv[4], NULL);
    v_Ki = strtod (argv[5], NULL);
    v_Kd = strtod (argv[6], NULL);
    std::cout << "Read values are:" << std::endl;
    std::cout << "a_Kp: " << a_Kp << "a_Ki: " << a_Ki << "a_Kd: " << a_Kd << std::endl;
    std::cout << "v_Kp: " << v_Kp << "v_Ki: " << v_Ki << "v_Kd: " << v_Kd << std::endl;
    std::cout << "Respond with Y if you want to continue." << std::endl;
    string c;
    std::cin >> c;
    if (c.compare(0,1,"Y") != 0) exit(EXIT_FAILURE);
    steer_pid.Init(a_Kp, a_Ki, a_Kd);
    speed_pid.Init(v_Kp, v_Ki, v_Kd);
  }
  else if (argc == 3)
  {
    string arg1 = argv[1];
    string arg2 = argv[2];
    if ((arg1.compare(0, 7, "twiddle") == 0) && (arg2.compare(0, 5, "steer") == 0)){
      std::cout << "Optimizing coefficients for steering using Twiddle algorithm." << std::endl; 
      do_steer_tuning = true;
      steer_twiddle.Init(steer_pid, 600, 1000, 0.001, 100, a_Kp, a_Ki, a_Kd, a_Kp/5.0, a_Ki/5.0, a_Kd/5.0);
    } else if ((arg1.compare(0, 7, "twiddle") == 0) && (arg2.compare(0, 5, "speed") == 0)){
      std::cout << "Optimizing coefficients for speed using Twiddle algorithm." << std::endl; 
      do_speed_tuning = true;
      speed_twiddle.Init(speed_pid, 600, 800, 0.001, 100, v_Kp, v_Ki, v_Kd, v_Kp/5.0, v_Ki/5.0, v_Kd/5.0);
    } else {
        std::cout << "Incorrect arguments." <<  std::endl;
        std::cout << "Try with Kp, Ki and Kd for steering and speed (6 numbers)" <<  std::endl;
        std::cout << "or ./pid twiddle [speed/steer] for Twiddle optimization" <<  std::endl;
        std::cout << "or simply with ./pid for default execution." <<  std::endl;
        exit(EXIT_FAILURE);
    }
  }
  else if (argc > 1)
  {
    std::cout << "Incorrect arguments." <<  std::endl;
    std::cout << "Try with 6 numbers as input for Kp, Ki and Kd for steering and the same for speed" <<  std::endl;
    std::cout << "or ./pid twiddle [speed/steer] for Twiddle optimization" <<  std::endl;
    std::cout << "or simply with ./pid for default execution." <<  std::endl;
    exit(EXIT_FAILURE);
  }
  else if (argc == 1)
  {
    std::cout << "Start of simulation with default parameters:" << std::endl;
    std::cout << "a_Kp: " << a_Kp << ", a_Ki: " << a_Ki << ", a_Kd: " << a_Kd << std::endl;
    std::cout << "v_Kp: " << v_Kp << ", v_Ki: " << v_Ki << ", v_Kd: " << v_Kd << std::endl;
  }

  h.onMessage([&do_steer_tuning, &steer_twiddle, &do_speed_tuning, &speed_twiddle, &steer_pid, &speed_pid]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value = 0;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          double set_speed = 20.0;
          double throttle_value = 0.0;

          if (do_speed_tuning) speed_twiddle.UpdateError(speed - set_speed);
          if (do_steer_tuning) steer_twiddle.UpdateError(cte);

          steer_pid.UpdateError(cte);
          steer_value = - steer_pid.TotalError();

          speed_pid.UpdateError(speed - set_speed);
          throttle_value = 0.5 - speed_pid.TotalError();
          
          // DEBUG
          if (false) {
            std::cout << "STR: " << cte << " Steering Value: " << steer_value 
                      << std::endl;
            std::cout << "SPD: " << speed - set_speed << " Throttle Value: " << throttle_value 
                      << std::endl;
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
/*
          std::cout << msg;
          if (do_tuning) std::cout << " - Twiddle iteration " << twiddle.twiddle_iter;
          std::cout << std::endl;
*/
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h, &steer_twiddle, &speed_twiddle](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    steer_twiddle.SetServer(ws);
    speed_twiddle.SetServer(ws);
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}