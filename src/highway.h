#ifndef HIGHWAY_H
#define HIGHWAY_H

#include <vector>
#include "vehicle.h"

class Highway {

public:
    Highway(int num_lanes, int speed_limit);
    virtual ~Highway();
    void update(Vehicle ego, const std::vector<Vehicle>& vehicles);

private:
    int num_lanes;
    int speed_limit;

};

#endif // HIGHWAY_H