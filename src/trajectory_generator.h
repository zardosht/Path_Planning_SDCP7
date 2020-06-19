#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include "behavior_planner.h"
#include "vehicle.h"
#include "map.h"


using std::vector;


// how many point to calculate for a trajectory
const int NUM_TRAJECTORY_POINTS = 50;

// threshold to start planning a new trajectory; otherwise consume points from previous trajectory.
const int PLAN_NEW_TRAJECTORY_THRESHOLD = 50;

const double PATH_PLANNING_DURATION = 2.5; // sec.

// the simulator reachs (consumes) each point of trajectory 
// in 0.02 seconds (in other words 50 points per second)
const double TIMESTEP = 0.02;   // sec.  

const double MPH_TO_MS = 0.44704;

const double MAX_SPEED = 48.5 * MPH_TO_MS;  // m/s

const double MAX_ACC = 10;   // m/s^2


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

        double target_s(double ego_s, double ego_v, double target_v, double palnning_time = PATH_PLANNING_DURATION);

        void initial_spline_points(vector<double>& knot_xs, vector<double>& knot_ys, Vehicle& egocar, Trajectory& prev_path, double& ref_yaw);

        void end_spline_points(vector<double>& knot_xs, vector<double>& knot_ys, double t_s, double target_v, double target_d);

        void transform_to_local(vector<double>& knot_xs, vector<double>& knot_ys, const double ref_x, const double ref_y, const double ref_yaw);

        vector<double> transform_to_global(const double x_local, const double y_local, const double ref_x, const double ref_y, const double ref_yaw);

        Map& map;

};


#endif //TRAJECTORY_GENERATOR_H