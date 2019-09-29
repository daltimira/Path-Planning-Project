#include "vehicle.h"
#include "highway.h"
#include <map>
#include <vector>

using std::map;
using std::vector;

Highway::Highway(int num_lanes, int speed_limit) {
    this->num_lanes = num_lanes;
    this->speed_limit = speed_limit;
}

Highway::~Highway() {}

void Highway::update(Vehicle ego, const std::vector<Vehicle>& vehicles) {
    map<int, vector<Vehicle>> predictions;
    // For now we just assign an id of the vehicle every time (we do not keep track of the vehicle over time)
    // We could however, track the vehicle in every update to observer current behaviour in a larger amount of time
    int id = 0;
    for (Vehicle vehicle : vehicles) {
        vector<Vehicle> preds = vehicle.generate_predictions();
        predictions[id] = preds;
        id++;
    }
    vector<Vehicle> trajectory = ego.choose_next_state(predictions);

    // now we need to send the trajectory of the vehicle to the simulator.
    
    //ego.realize_next_state(trajectory);

    // 
}