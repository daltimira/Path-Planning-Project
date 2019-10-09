#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <functional>


//using Eigen::MatrixXd;
//using Eigen::VectorXd;

// for convenience
using std::string;
using std::vector;

const float EFFICIENCY = 0.1;
const float OBSTACLE = 1.0;
const float DIFF_SPEED = 0.2;
const float SAFE_DISTANCE = 40;

namespace {
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

float speed_lane(double car_s, int lane, float max_speed, const vector<vector<double>> & sensorFusion)
{
	float speed = max_speed;
	// check the speed of the current lane based on other vehicles and speed limit
	for (int i = 0; i<sensorFusion.size(); i++) {
		double vx     = sensorFusion[i][3];
		double vy     = sensorFusion[i][4];
		double check_car_s      = sensorFusion[i][5];
		float check_car_d       = sensorFusion[i][6];
		double v = sqrt(vx*vx+vy*vy);

		int car_lane = -1;
		if (check_car_d>0 && check_car_d<4){
	      car_lane = 0;
	    } else if (check_car_d>4 && check_car_d<8) {
	      car_lane = 1;
	    } else if (check_car_d>8 && check_car_d<12) {
	      car_lane = 2;
	    }

	    if (car_lane == lane &&  ((car_s+20)>check_car_s) && (car_s<check_car_s) && speed>v ) {
	    	// we take the minimum speed of all vehicles that are ahead of our vehicle, within the safe distance
	    	speed = v;
	    }
  }
  return speed;
}

// float inefficiency_cost(double car_s, double car_v, int lane, float max_speed, int prev_size, const vector<vector<double>> & sensorFusion) {
// 	// cost becomes higher for lanes whose speed have traffic slower than the max_speed
// 	float currentSpeedLane = speed_lane(car_s, lane, max_speed, sensorFusion);
//   //printf("current speed lane %d: %f", lane, currentSpeedLane);
// 	float cost = (2.0*max_speed - currentSpeedLane)/max_speed;

//   return cost;
// }

// float diffspeed_cost(double car_s, double car_v, int lane, float max_speed, int prev_size, const vector<vector<double>> & sensorFusion) {
// 	// cost becomes higher for lanes whose different speed with current car is larger
// 	float currentSpeedLane = speed_lane(car_s, lane, max_speed, sensorFusion);
//   //printf("current speed lane %d: %f", lane, currentSpeedLane);
// 	float cost = (2.0*car_v - currentSpeedLane)/max_speed;

//   return cost;
// }

float obstacles_cost(double car_s, int car_lane, double car_v, int lane, float max_speed, int prev_size, const vector<vector<double>> & sensorFusion) {
  bool obstacle = false;

  bool look_vehicle_ahead = car_lane == lane; // if we are evaluating the cost of the lane where is located our vehicle
  bool look_vehicle_next_lane =  abs(car_lane - lane) == 1; // if we are evaluating the cost of the lane next to where our vehicle is located

  if (!look_vehicle_ahead && !look_vehicle_next_lane) {
    return 1.0f;
  }

  for (int i = 0; i<sensorFusion.size(); i++) {
    // car is in my lane
    float d = sensorFusion[i][6];
    int check_car_lane = -1;
    double check_car_s = sensorFusion[i][5];
    double vx = sensorFusion[i][3]; // 'i' car of the road
    double vy = sensorFusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy); // the speed is important to predict where the car will be in the future
    check_car_s += ((double) prev_size*.02*check_speed); // if using previous points can project s value outwards in time. If we are using our path points, we might be not there yet.

    if (d>=0 && d<=4){
      check_car_lane = 0;
    } else if (d>=4 && d<=8) {
      check_car_lane = 1;
    } else if (d>=8 && d<=12) {
      check_car_lane = 2;
    }

    if (check_car_lane == lane) {
      if (look_vehicle_ahead) {
        // we check for vehicles that are in the same lane
        if (check_car_s > car_s && (check_car_s-car_s) < SAFE_DISTANCE) {
          return 1.0f; // car in front is too close, so cost 1
        }
      } else {
        if (car_s < (check_car_s + SAFE_DISTANCE) && car_s > (check_car_s - SAFE_DISTANCE)) {
          return 1.0f;
        }
      }
    }
  }

 return 0.0f;
}


float calculate_cost(double car_s, int car_lane, double car_v, int lane, float max_speed, int prev_size, const vector<vector<double>> & sensorFusion) {
	float cost = 0.0;
	vector<std::function<float(double, int, double, int, float, int, const vector<vector<double>> &) >> cf_list = {/*inefficiency_cost,*/ obstacles_cost/*, diffspeed_cost*/};
	vector<float> weight_list = {/*EFFICIENCY,*/ OBSTACLE /*, DIFF_SPEED*/};
	for (int i = 0; i < cf_list.size(); ++i) {
	    float new_cost = weight_list[i]*cf_list[i](car_s, car_lane, car_v, lane, max_speed, prev_size, sensorFusion);
	    cost += new_cost;
	  }

	  return cost;
}

std::vector<int> successor_states(int lane, int num_lanes) {
	std::vector<int> lanes;
	if (lane>0) {
		lanes.push_back(lane-1);
	}
  lanes.push_back(lane);
	if (lane<num_lanes-1) {
		lanes.push_back(lane+1);
	}
	return lanes;
}

// this is the transition function, where we move from one state to another (change lanes)
int choose_next_lane(double car_s, double car_v,  int lane, float max_speed, int prev_size, const vector<vector<double>> & sensorFusion) {
  int numLanes = 3;
	std::vector<int> possible_successor_lanes = successor_states(lane, numLanes); // give as a parameter current lanes and the number of lanes of the highway
	std::vector<float> costs(numLanes);

  for (int iLane = 0; iLane < numLanes; iLane++) {
    costs[iLane] = 1000.0f; // init with large cost. The successor states (lanes) will overwrite this cost
  }

	 for (int iLane = 0; iLane<possible_successor_lanes.size(); iLane++) {
        //vector<Vehicle> trajectory_for_state = generate_trajectory(possible_successor_lanes[iLane], sensorFusion);
        double cost_for_state = 0;
        costs[possible_successor_lanes[iLane]] = calculate_cost(car_s, lane, car_v, possible_successor_lanes[iLane], max_speed, prev_size, sensorFusion);
    }

    for (int iLane = 0; iLane < numLanes; iLane++) {
      printf(" %f, ", costs[iLane]);
    }
    printf("\n");
    int bestLane = lane;
    float min_cost = costs[bestLane];

    for (int iLane = 0; iLane<numLanes; iLane++) {
        //int tmp_lane =  possible_successor_lanes[iLane];
        float cost = costs[iLane];
        if (cost < min_cost) {
            min_cost = cost;
            bestLane = iLane;
        }
    }
    return bestLane;
}

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  /*MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;*/

  //MatrixXd Ai = A.Eigen3::inverse();

  /*MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;*/

  return vector <double> ();
}
}

#endif  // HELPERS_H