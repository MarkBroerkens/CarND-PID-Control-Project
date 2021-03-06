#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <algorithm>    // std::max
#include "json.hpp"
#include "PID.h"
#include "PIDTuner.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

const double MAX_SPEED = 100.;
const double MAX_ANGLE = 25.;
const double MAX_THROTTLE = 0.3;

const bool twiddle = false;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
  {
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }


int main() {
  uWS::Hub h;

  PID pid_steering_angle, pid_throttle;


  // Lesson 16: PID Control, 11 PID Implementation
  // pid_steering_angle.Init(0,2, 0,004, 3)

  // manual optimzations
  // pid_steering_angle.Init(0.14, 0.00027, 6);
  // pid_throttle.Init(.1, 0.0, 1);


  // twiddle optimizations
  // with constant throttle = 0.3
  pid_steering_angle.Init(0.135, 0.0015, 8.51561);
  pid_throttle.Init(0.6, 0, 1);


  PIDTuner pidtuner(pid_steering_angle, pid_throttle, 0.1);



  h.onMessage(
      [&pid_steering_angle, &pid_throttle, &pidtuner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
          auto s = hasData(std::string(data).substr(0, length));
          if (s != "") {
            auto j = json::parse(s);
            std::string event = j[0].get<std::string>();
            if (event == "telemetry") {
              // j[1] is the data JSON object
              double cte = std::stod(j[1]["cte"].get<std::string>());
              double speed = std::stod(j[1]["speed"].get<std::string>());
              double angle = std::stod(j[1]["steering_angle"].get<std::string>());
              // DEBUG
              // std::cout << "CTE: " << cte << " angle: " << angle << " speed: " << speed << std::endl;

              pid_steering_angle.UpdateError(cte);
              double steer_value = pid_steering_angle.GetControl();

              double target_speed = std::max(0.0, MAX_SPEED * ( 1.0 - fabs(angle/MAX_ANGLE*cte) / 4));
              target_speed = std::min(100.0, target_speed);
              pid_throttle.UpdateError(speed - target_speed);
              // DEBUG
              // std::cout << "Throttle Error: " << pid_throttle.TotalError() << " Target Speed: " << target_speed << std::endl;
              double throttle_value = std::min(MAX_THROTTLE, 0.7 + pid_throttle.GetControl());

              // DEBUG
              // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << std::endl;

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              // msgJson["throttle"] = .3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              // DEBUG
              // std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


              if (twiddle) {
                pidtuner.accumulateCTE(cte);
                if (pidtuner.hasFinishedRun() || pidtuner.isOffTrack(cte, speed)) {
                  pidtuner.twiddle();
                  pidtuner.print();
                  reset_simulator(ws);
                }
              }
            }
          } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest(
      [](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
          res->end(s.data(), s.length());
        }
        else
        {
          // i guess this should be done more gracefully?
          res->end(nullptr, 0);
        }
      });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection(
      [&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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




