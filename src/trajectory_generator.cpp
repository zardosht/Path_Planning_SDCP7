#include "trajectory_generator.h"
#include "helpers.h"


TrajectoryGenerator::TrajectoryGenerator()
{

}

TrajectoryGenerator::~TrajectoryGenerator() {}

Trajectory TrajectoryGenerator::generate_trajectory(State state, Vehicle& egocar)
{
    Trajectory trajectory;

    double dist_inc = 0.5;
    for (int i = 0; i < NUM_TRAJECTORY_POINTS; ++i)
    {
        double new_x = egocar.x + (i * DISTANCE_INCREMENT) * cos(deg2rad(egocar.yaw));
        double new_y = egocar.y + (i * DISTANCE_INCREMENT) * sin(deg2rad(egocar.yaw));
        trajectory.xs.push_back(new_x);
        trajectory.ys.push_back(new_y);
    }

    return trajectory;
}