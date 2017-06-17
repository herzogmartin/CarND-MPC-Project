#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        //set t0 for duration calculation
        chrono::high_resolution_clock::time_point t0 = chrono::high_resolution_clock::now();
        
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"]; 
          double accel = j[1]["throttle"];

          
          // transform waypoints to vehicle coordinate system (vcs)
          assert(ptsx.size() == ptsy.size());
          Eigen::VectorXd ptsx_vcs(ptsx.size());
          Eigen::VectorXd ptsy_vcs(ptsy.size());
          const double cos_psi = cos(psi);
          const double sin_psi = sin(psi);
          for (int i=0; i < ptsx.size(); i++) {
            ptsx_vcs[i] =  (ptsx[i] - px) * cos_psi + (ptsy[i] - py) * sin_psi;
            ptsy_vcs[i] = -(ptsx[i] - px) * sin_psi + (ptsy[i] - py) * cos_psi;
          }
          
          //fit polynomial to waypoints in car coordinates
          auto coeffs = polyfit(ptsx_vcs, ptsy_vcs, 3);
          
          //calculate view range (length of waypoints)
          double viewRange = 0.0;
          for (int i=1; i < ptsx_vcs.size(); i++) {
            viewRange += std::sqrt(
                              std::pow((ptsx_vcs[i]-ptsx_vcs[i-1]),2)
                            + std::pow((ptsy_vcs[i]-ptsy_vcs[i-1]),2) );
           }
          
          // calculation is done in vehicle coordinate system
          // -> px = py = psi = 0.0;
          
          // calculate the cross track error
          double cte = coeffs[0];
          // calculate the orientation error
          double epsi = atan(coeffs[1]);

          //initialize state
          //x,y,psi of car is 0 because of transformation to car coordinate system
          Eigen::VectorXd state(6);
          state << 0.0,0.0,0.0,v,cte,epsi;
          
          //variables for displaying the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //run MPC
          auto vars = mpc.Solve(state, coeffs, viewRange, mpc_x_vals, mpc_y_vals);
          
          //get actuator controls and convert to [-1 ... 1]
          // var[0] ^= steering
          // var[1] ^= throttle 
          const double steer_value = -vars[0] / 0.436332;
          const double throttle_value = vars[1]; 
          
          // store steering and throttle
          mpc.delta_prev = vars[0];
          mpc.a_prev = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          
          next_x_vals.resize(ptsx_vcs.size());
          next_y_vals.resize(ptsy_vcs.size());
          Eigen::VectorXd::Map(&next_x_vals[0], ptsx_vcs.size()) = ptsx_vcs;
          Eigen::VectorXd::Map(&next_y_vals[0], ptsy_vcs.size()) = ptsy_vcs;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          
          //calculate comutation time pf MPC (addinional delay))
          chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
          mpc.calcTime = chrono::duration_cast<chrono::milliseconds>( t1 - t0 ).count();
          std::cout << "calculation duration MPC [ms]: " << mpc.calcTime << std::endl;
          
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(MPC_LATENCY));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
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
