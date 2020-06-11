#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include "map.h"
#include "behavior_planner.h"
#include "trajectory_generator.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  string map_file = "../data/highway_map.csv";
  Map map;
  map.load(map_file);

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // int lane = 1;
  // double ref_vel = 0.0;  //mph

  TrajectoryGenerator tg(map);
  BehaviorPlanner bp;


  h.onMessage([&map, &bp, &tg]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
 

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds. That is a vector of x's (next_x_vals) 
           *   and a vector of y's (next_y_vals) for the path points.
           */
          
          Vehicle ego_car(EGOCAR_ID, car_x, car_y, car_s, car_d, car_yaw);
          
          Behavior behavior = bp.next_behavior();


          Trajectory trajectory = tg.generate_trajectory(behavior, ego_car);

          // vector<double> next_x_vals;
          // vector<double> next_y_vals;

          // double dist_inc = 0.5;
          // for (int i = 0; i < 50; ++i)
          // {
          //   next_x_vals.push_back(car_x + (i * dist_inc) * cos(deg2rad(car_yaw)));
          //   next_y_vals.push_back(car_y + (i * dist_inc) * sin(deg2rad(car_yaw)));
          // }


          msgJson["next_x"] = trajectory.xs;
          msgJson["next_y"] = trajectory.ys;

          // msgJson["next_x"] = next_x_vals;
          // msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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