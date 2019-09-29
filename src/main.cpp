#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double ref_vel = 50; // 50 miles/hour 

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

    // start in lane 1
    int lane = 1;
    // Have a reference velocity to target
    double ref_vel = 0; // mph

  Vehicle a;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
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

          // We retrieve the vehicle information from sensor fusion
          vector<Vehicle> vehicleInfo = getVehicles(sensor_fusion);

          int prev_size = previous_path_x.size();

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // We will work on sensor fusion. The simulator is reporting a list of all the other cars on the road. 
          // To avoid hitting another car, we should go to the sensor fusion list and see if the car is in our lane or not
          // To avoid hitting other car, if we detect that the car is too close, we do some action. 


          // Craete a list of widely spaced (x,y) waypoints, evenly space at 30 m
          // Later we will interpoolate these waypoints with a spline and fill it in with more points that control speed.

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;

          // find ref_v to use
          for (int i = 0; i<sensor_fusion.size(); i++) {
            // car is in my lane
            float d = sensor_fusion[i][6];
            if (d<(2+4*lane+2) && d > (2+4*lane-2)) { // range +2, -2, as each lane is four meters. If it is in the center lane, we check is between 4 and 8
              double vx = sensor_fusion[i][3]; // 'i' car of the road
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy); // the speed is important to predict where the car will be in the future
              double check_car_s = sensor_fusion[i][5];

              // We kind of want to look at what the car will be like in the future. 
              check_car_s += ((double) prev_size*.02*check_speed); // if using previous points can project s value outwards in time. If we are using our path points, we might be not there yet.

              // we check if our car s is close to this other cars_s
              if ((check_car_s > car_s) /*if the car is in front of us*/ && ((check_car_s-car_s) < 30) /*the gap is smaller than 30 meters*/) {
                // we need to do some logic, lower reference velocity so we dont crash into the car in front of use, could also flag to tray to change lanes
                //ref_vel = 29.5; // mph
                too_close = true; // we are not setting a velocity, we will incrementing or decrementing velocity, to have a smooth acceleration
                if (lane > 0)
                {
                  lane = 0;
                }

              }
            }
          }

          if (too_close) {
            ref_vel -= .224; // 5m/s^2
          } else if (ref_vel < 49.5) {
            ref_vel += .224;
          }

          vector<double> ptsx;
          vector<double> ptsy;  

          // reference x,y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths and point 
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2) {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw); // go backwards in time based on the angle.
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // use the previous path's end point as starting reference
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangetnt to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          // When we change the lanes, we add new waypoints for the 30, 60 and 90 meters with new d.
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // We make the first point as our reference (angle 0, and position (0,0))
          for (int i = 0; i<ptsx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline s;

          // set (x,y) poitns to the spline
          s.set_points(ptsx, ptsy); // add the anchor points, not the previous path point

          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all tof the previous path points from last time 
          // Instead of recreating the path from scracth every single time, just add points onto it and work with what is still had left from last time
          // If in one iteration we have 50 points, but in the following iteration, the car only have gone through 3 of the points of the planned points,
          // then we are reusing 47 of the last points, and generate 3 points more. 
          for (int i = 0; i<previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Need to check the space between these points in order to know the desired speed. We want to space all the points in a way so that the car
          // goes at the desired speed. 

          // Set a distance, 30 m. you calculate the y value of the spline at this 30 m. If you want to split the segment in N number of points
          // then N*0.02*vel = d where d is the distance between current position of the car and the (30m, y). The 0.02 means the car visit a point every 0.02 seconds.
          // We need to calculate the N          
          // The projected N points, on the X axis, is used together, with the spline function, to get the (x,y) points of the spline. 

          // Calculate how to break up spline points so that we travel at our desired reference velocity 
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous point, here we will always output 50 points
          for (int i = 1; i<= 50-previous_path_x.size(); i++) {
            double N = (target_dist/(.02*ref_vel/2.24)); // dividing by 2.24 because this is in miles per hour and need to be meters per second
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier. Go back to the global coordinates. 
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


          /*double dist_inc = 0.5; // how much the points are spaced apart. that is makes it around 50mph.
          for (int i = 0; i<50; i++) { // we are using a constant size of 50 points for a pack planner.
            double next_s = car_s+(i+1)*dist_inc;  
            // we are in th emiddle lane, and the waypoints are measured from the double yellow line in the middle of the road 
            // So, we are like one and a half lanes from where the waypoints are
            // lanes are four meters wide
            double next_d =  6;
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }*/

          // END


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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