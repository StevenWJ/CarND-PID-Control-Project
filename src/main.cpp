#include <math.h>
#include <bits/stdc++.h> 
#include <unistd.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <time.h>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

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

double setprec(double number, int digits) 
{
  double base = pow(10.0, digits);
  return double(int(number*base))/base;
}

int main() {
  uWS::Hub h;

  class pid_t {
    public:
      PID steer;
      PID throt;
      double accel_step;
      double prev_accel;
      double target_speed; 
      clock_t prev_ticks;
      double prev_speed;
      double prev_steer;
      double prev_throt;
      double steer_check_dist; 
      double throt_check_time;
      double steer_curr_dist;
      double steer_curr_time;
      double throt_curr_dist;
      double throt_curr_time;
      double total_dist;
      double total_time;
      bool initialized;
  } pid;
  /**
   * TODO: Initialize the pid variable.
   */
  pid.steer.Init("steer", 0.109901, 0.0388477, 0.591048, 1.0, -1.0, 100, 0.1, 0.01);
  pid.throt.Init("throt", 0.11, 0.0, 0.0, 1.0, -1.0, 50, 0.5, 0.01);
  pid.prev_ticks = clock();
  pid.accel_step = 8.0;
  pid.prev_accel = pid.accel_step;
  pid.target_speed = 50.0;
  pid.prev_speed = 0.0;
  pid.prev_steer = 0.0;
  pid.prev_throt = 0.0;
  pid.steer_check_dist = 0.02;
  pid.throt_check_time = 0.01;
  pid.steer_curr_dist = 0.0;
  pid.steer_curr_time = 0.0;
  pid.throt_curr_dist = 0.0;
  pid.throt_curr_time = 0.0;
  pid.total_dist = 0.0;
  pid.total_time = 0.0;
  pid.initialized = false;
  pid.steer.enable_twiddle = true;
  pid.throt.enable_twiddle = false;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double s_cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value = 0.0;
          double throt_value = 0.0;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          clock_t curr_ticks = clock();
          clock_t elasped_ticks = curr_ticks - pid.prev_ticks;
          double elapsed_time = (double(elasped_ticks)/CLOCKS_PER_SEC);
          double expected_travel_dist = pid.prev_speed * elapsed_time;
          
          double delta_speed = speed - pid.target_speed;
          double v_cte = ((delta_speed/pid.target_speed)/exp(fabs(s_cte)) + fabs(s_cte)*0.1)*20.0;
          if (v_cte > 0.0) { v_cte /= 100.0; }

          pid.steer_curr_dist += expected_travel_dist;
          pid.throt_curr_dist += expected_travel_dist;
          pid.steer_curr_time += elapsed_time;
          pid.throt_curr_time += elapsed_time;
          pid.total_dist += expected_travel_dist;
          pid.total_time += elapsed_time;

          steer_value = pid.steer.getNextAction();
          if (pid.initialized && pid.steer_curr_dist >= pid.steer_check_dist) {
            pid.steer.UpdateError(s_cte, speed, angle/25.0, pid.steer_curr_time);
            pid.steer.TotalError();
            pid.steer_curr_dist = 0.0;
            pid.steer_curr_time = 0.0;
          } 

          throt_value = pid.throt.getNextAction();
          if (pid.initialized && pid.throt_curr_time >= pid.throt_check_time) {
            pid.throt.UpdateError(v_cte, speed, angle/25.0, pid.throt_curr_time);
            pid.throt.TotalError();
            pid.throt_curr_dist = 0.0;
            pid.throt_curr_time = 0.0;
          } 


          // Values limiter for safety
          if (steer_value < -1.0) {
            steer_value = -1.0;
          }
          if (steer_value > 1.0) {
            steer_value = 1.0;
          }

          if (throt_value < -1.0) {
            throt_value = -1.0;
          }
          if (throt_value > 1.0) {
            throt_value = 1.0;
          }

          pid.prev_steer = steer_value;
          pid.prev_throt = throt_value;
          pid.prev_ticks = curr_ticks;
          pid.prev_speed = speed;
          pid.prev_steer = steer_value;
          pid.prev_throt = throt_value;
          pid.initialized = true;
          
          vector<double> s_params = pid.steer.getParams();
          vector<double> s_errors = pid.steer.getErrors();
          vector<double> v_params = pid.throt.getParams();
          vector<double> v_errors = pid.throt.getErrors();
         
          // DEBUG
          std::cout << " time: " << pid.total_time << " dist: " << pid.total_dist << " speed: " << speed  << std::endl;
          std::cout << " s_cte: " << s_cte << " v_cte: " << v_cte << std::endl;
          std::cout << " steer_value: " << steer_value << " throt_value: " << throt_value << std::endl;
          std::cout << " s_Kp:" << s_params[0] << " s_Ki:" << s_params[1] << " s_Kd: " << s_params[2] << std::endl;
          std::cout << " s_Ep:" << s_errors[0] << " s_Ei:" << s_errors[1] << " s_Ed: " << s_errors[2] << std::endl;
          std::cout << " v_Kp:" << v_params[0] << " v_Ki:" << v_params[1] << " v_Kd: " << v_params[2] << std::endl;
          std::cout << " v_Ep:" << v_errors[0] << " v_Ei:" << v_errors[1] << " v_Ed: " << v_errors[2] << std::endl;
          std::cout << " steer pending: " << pid.steer.getSchedActionCount() << " throt pending: " << pid.throt.getSchedActionCount() << std::endl;
          std::cout << std::endl;


          json msgJson;
          msgJson["steering_angle"] = setprec(steer_value, 5);
          msgJson["throttle"] =  setprec(throt_value, 5);
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // std::cout << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    
    h.getDefaultGroup<uWS::SERVER>().close();
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
