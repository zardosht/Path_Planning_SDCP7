#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "behavior_planner.h"
#include "vehicle.h"
#include "map.h"



using std::vector;

// how many point to calculate for a trajectory
// const int NUM_TRAJECTORY_POINTS = 50;
const int NUM_TRAJECTORY_POINTS = 100;

// threshold to start planning a new trajectory; otherwise consume points from previous trajectory.
// const int PLAN_NEW_TRAJECTORY_THRESHOLD = 50;
const int PLAN_NEW_TRAJECTORY_THRESHOLD = 100;

// the simulator reachs (consumes) each point of trajectory 
// in 0.02 seconds (in other words 50 points per second)
const double TIMESTEP = 0.02;   // sec.  

const double PATH_PLANNING_DURATION = NUM_TRAJECTORY_POINTS * TIMESTEP; // sec.

const double MPH_TO_MS = 0.44704;

const double MAX_SPEED = 48.5 * MPH_TO_MS;  // m/s

//const double MAX_ACC = 10;   // m/s^2
const double MAX_ACC = 7;   // m/s^2


struct Behavior;

struct Trajectory  
{
    vector<double> xs;
    vector<double> ys;
    
    double end_s = -1.0;
    double end_d = -1.0;
    
    int size() {return xs.size();}

};

class TrajectoryGenerator 
{
    public:
        // constructor
        TrajectoryGenerator(Map& map);

        // destructor
        ~TrajectoryGenerator();

        // functions
        Trajectory generate_trajectory(Behavior& behavior, Vehicle& egocar, Trajectory& previous_path);

        // variables

    private:


        void initial_spline_points(vector<double>& knot_xs, vector<double>& knot_ys, Vehicle& egocar, Trajectory& prev_path, double& ref_yaw);

        void end_spline_points(vector<double>& knot_xs, vector<double>& knot_ys, double t_s, double target_v, double target_d);

        void transform_to_local(vector<double>& knot_xs, vector<double>& knot_ys, const double ref_x,  const double ref_y, const double ref_yaw);

        vector<double> transform_to_global(const double x_local, const double y_local, const double ref_x, const double ref_y, const double ref_yaw);

        Map& map;

        double ego_v = 0.0;

};


#endif //TRAJECTORY_GENERATOR_H