#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "behavior_planner.h"
#include "vehicle.h"
#include "map.h"



using std::vector;

using Eigen::Array2Xd;


const int NUM_TRAJECTORY_POINTS = 50;
const double TIMESTEP = 0.02;
const double MAX_SPEED = 49.5; //mph 


// Defines general accleration values
// The behavior planner only tells if 
// trajectory generator shoule accelerate, 
// keep the speed, or deccelrate. 
// It is the responsibility of the trajectory
// generator to define proper values for acceleration
enum Accel 
{
    ACCEL, 
    ZERO, 
    DECEL
};


struct Trajectory  
{
    vector<double> xs;
    vector<double> ys;
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
        Trajectory generate_trajectory(Behavior behavior, Vehicle& vehicle, Trajectory& previous_path);

        // variables

    private:

        double get_d(Behavior behavior, Vehicle& egocar); 

        Accel get_accel(Behavior behavior);

        Trajectory generate_trajectory(Vehicle& egocar, double d, Accel accel, Trajectory& previous_path);

        void initial_spline_points(Array2Xd& spline_knots, Vehicle& egocar, Trajectory& prev_path, double& ref_yaw);

        void end_spline_points(Array2Xd& spline_knots, Vehicle& egocar, double target_d);

        void transform_to_local(Array2Xd& spline_knots, const double ref_x, const double ref_y, const double ref_yaw);

        Map& map;

};


#endif //TRAJECTORY_GENERATOR_H