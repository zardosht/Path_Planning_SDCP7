#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "behavior_planner.h"
#include "vehicle.h"
#include "map.h"



using std::vector;

using Eigen::Array2Xd;

// how many point to calculate for a trajectory
const int NUM_TRAJECTORY_POINTS = 50;

// threshold to start planning a new trajectory; otherwise consume points from previous trajectory.
const int PLAN_NEW_TRAJECTORY_THRESHOLD = 50;

// the simulator reachs (consumes) each point of trajectory in TIMESTEP
const double TIMESTEP = 0.02;

const double MAX_SPEED = 48.5;  // mph

struct Trajectory  
{
    vector<double> xs;
    vector<double> ys;
    
    double end_s = -1.0;
    double end_d = -1.0;
    
    int size() {return xs.size();}

};

struct Behavior;

class TrajectoryGenerator 
{
    public:
        // constructor
        TrajectoryGenerator(Map& map);

        // destructor
        ~TrajectoryGenerator();

        // functions
        Trajectory generate_trajectory(Behavior behavior, Vehicle& vehicle, Trajectory& previous_path);

        // variables

    private:

        double get_d(Behavior behavior, Vehicle& egocar); 

        void initial_spline_points(Array2Xd& spline_knots, Vehicle& egocar, Trajectory& prev_path, double& ref_yaw);

        void end_spline_points(Array2Xd& spline_knots, Vehicle& egocar, double start_from_s, double target_d);

        void transform_to_local(Array2Xd& spline_knots, const double ref_x, const double ref_y, const double ref_yaw);

        vector<double> transform_to_global(const double x_local, const double y_local, const double ref_x, const double ref_y, const double ref_yaw);

        Map& map;

        double vel = 0.0;

};


#endif //TRAJECTORY_GENERATOR_H