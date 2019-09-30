#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>
#include <map> 

using std::string;
using std::vector;
using std::map;

class Vehicle {

public: 
    Vehicle();
    Vehicle(float d, float s, float v, float a, string state = "CS");
    virtual ~Vehicle();
    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> & predictions);
    vector<string> successor_states();
    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> & predictions);
    vector<float> get_kinematics(map<int, vector<Vehicle>> & predictions, int lane);
    vector<Vehicle> constant_speed_trajectory();
    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> & predictions);
    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> & predictions);
    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> & predictions);
    float position_at(int t);
    bool get_vehicle_behind(map<int, vector<Vehicle>> & predictions, int lane, Vehicle & rVehicle);
    bool get_vehicle_ahead(map<int, vector<Vehicle>> & predictions, int lane, Vehicle & rVehicle);
    vector<Vehicle> generate_predictions(int horizon=2);
    void realize_next_state(vector<Vehicle> &trajectory);
    //void configure(vector<int> &road_data);
    void increment(int dt = 1);
    int getLane(float d);
    float getCenterLane(int lane);

    // public Vehicle variables
    struct collider{
        bool collision; // is there a collision?
        int  time; // time collision happens
    };

    map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                        {"LCR", -1}, {"PLCR", -1}};

    int L = 1;

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int lane, d, s, goal_lane, goal_s, lanes_available;

    float v, target_speed, a, max_acceleration;

    string state;

};

#endif //VEHICLE_H