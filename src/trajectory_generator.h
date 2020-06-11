#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include "behavior_planner.h"
#include "vehicle.h"
#include "map.h"


using std::vector;

const double DISTANCE_INCREMENT = 0.5;
const int NUM_TRAJECTORY_POINTS = 50;

struct Trajectory  
{
    vector<double> xs;
    vector<double> ys;
};

class TrajectoryGenerator 
{
    public:
        // constructor
        TrajectoryGenerator(Map& map);

        // destructor
        ~TrajectoryGenerator();

        // functions
        Trajectory generate_trajectory(Behavior behavior, Vehicle& vehicle);

        // variables

    private:
        Map& map;

};


#endif //TRAJECTORY_GENERATOR_H