#include "trajectory_generator.h"
#include "helpers.h"


TrajectoryGenerator::TrajectoryGenerator(Map& map) : map(map) 
{
    
}

TrajectoryGenerator::~TrajectoryGenerator() {}

Trajectory TrajectoryGenerator::generate_trajectory(Behavior behavior, Vehicle& egocar)
{
    Trajectory trajectory;
    for (int i = 0; i < 50; ++i) {
        double next_s = egocar.s + (i + 1) * DISTANCE_INCREMENT;
        double next_d = egocar.d; 

        vector<double> xy = getXY(next_s, next_d, 
                                  map.waypoints_s, 
                                  map.waypoints_x, 
                                  map.waypoints_y);

        trajectory.xs.push_back(xy[0]);
        trajectory.ys.push_back(xy[1]);
    }


    return trajectory;
}