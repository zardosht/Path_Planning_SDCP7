#include "trajectory_generator.h"
#include "spline.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;
using std::min;
using std::max;


TrajectoryGenerator::TrajectoryGenerator(Map& map) : map(map) 
{
    
}

TrajectoryGenerator::~TrajectoryGenerator() {}


Trajectory TrajectoryGenerator::generate_trajectory(Behavior behavior, Vehicle& egocar, Trajectory& previous_path)
{
    // find out the lane from behavior value
    // find out the acceleration from behavior value (ACCEL, ZERO, DECEL)
    
    int prev_path_size = previous_path.size();
    cout << "*** prev_path_size=" << prev_path_size << endl;
    if (prev_path_size >= PLAN_NEW_TRAJECTORY_THRESHOLD) 
        return previous_path;

    int ego_lane = egocar.get_lane();
    double target_d = behavior.target_d(ego_lane);
    double target_v = behavior.target_v;

   
    // get initial spline knots
    // get initial yaw (for transforming the points
    // into local coordinates and back to global coordinates)
    // get end spline knots
    vector<double> knot_xs;
    vector<double> knot_ys;
    double ref_yaw = deg2rad(egocar.yaw);  //inout
    initial_spline_points(knot_xs, knot_ys, egocar, previous_path, ref_yaw);

    double ref_x = knot_xs[0];
    double ref_y = knot_ys[0];

    double start_s = (prev_path_size > 2)? previous_path.end_s : egocar.s;
    //double t_s = target_s(start_s, egocar.speed, target_v);
    double t_s = start_s;
    end_spline_points(knot_xs, knot_ys, t_s, target_v, target_d);

    // transform spline knots into local coordinates
    transform_to_local(knot_xs, knot_ys, ref_x, ref_y, ref_yaw);

    // prepare spline
    tk::spline spl;
    spl.set_points(knot_xs, knot_ys);

    // add points from previus path to the beginning of the current path
    Trajectory trajectory;
    for(int i = 0; i < prev_path_size; ++i)
    {
        trajectory.xs.push_back(previous_path.xs[i]);
        trajectory.ys.push_back(previous_path.ys[i]);
    }
    
    // add the rest points to trajectory (get the xs by 
    // considering the speed and acceleration, get the ys from 
    // spline, transform back the x and y into global coordinates
    // add the x and y to trajectory)
    // Note that contrary to the solution presented in the Q&A video, 
    // we do not distribute trajectory points evenly, but based on planning distance.
    // Instead the points are smapled from spline based on planning duration
  
    // double x0 = knot_xs[0];
	// double y0 = knot_ys[0];
    // vector<double> xy_global = transform_to_global(x0, y0, ref_x, ref_y, ref_yaw);
    // trajectory.xs.push_back(xy_global[0]);
    // trajectory.ys.push_back(xy_global[1]);
    
    // double x1 = knot_xs[1];
    // double y1 = knot_ys[1];
    // xy_global = transform_to_global(x1, y1, ref_x, ref_y, ref_yaw);
    // trajectory.xs.push_back(xy_global[0]);
    // trajectory.ys.push_back(xy_global[1]);

    // double next_dist;
    // double ego_v = egocar.speed;
	// for (int dt = 1; dt * TIMESTEP <= PATH_PLANNING_DURATION - (prev_path_size * TIMESTEP); dt++) {
    //     x0 = x1;
	//     y0 = y1;
    // 	next_dist = TIMESTEP * ego_v;
    //     x1 = x0 + next_dist;
    //     y1 = spl(x1);
        
    //     xy_global = transform_to_global(x1, y1, ref_x, ref_y, ref_yaw);
    //     trajectory.xs.push_back(xy_global[0]);
    //     trajectory.ys.push_back(xy_global[1]);

    //     if (target_v > ego_v) {
    //         //accelerate
    //         ego_v = min(ego_v + MAX_ACC * TIMESTEP, MAX_SPEED);
    //     } else {
    //         //decelerate
    //         ego_v = min(ego_v - MAX_ACC * TIMESTEP, MAX_SPEED);
    //     }
        
	// }
    
    // return trajectory;
    

    
    // add the rest points to trajectory (get the xs by 
    // considering the speed and acceleration, get the ys from 
    // spline, transform back the x and y into global coordinates
    // add the x and y to trajectory)

    // Define how far to go in the x direction.
    // We uniformly divide this distance into N points.
    // That gives us the x coordinates of the path points. 
    // The y coordinates are calculated by the spline. 
    // The time step of the simulator is 0.02. That is the 
    // simulator reachs each point in the path in 0.02 
    // seconds. The velocity (ref_vel) hence defines how 
    // dense the points are. Higher velocity means the points
    // are far apart. Lower velocity means the points are 
    // dence near eachother.   
    // The number of points N is then given by deviding the distance 
    // by the (speed * time_step). 

    double ego_v = (egocar.speed == 0)? 5 : egocar.speed;
    if (target_v > ego_v) {
        //accelerate
        ego_v = min(ego_v + MAX_ACC * TIMESTEP, MAX_SPEED);
    } else {
        //decelerate
        ego_v = min(ego_v - MAX_ACC * TIMESTEP, MAX_SPEED);
    }

    // double target_x = 30.0; //horizon
    double target_x = 100.0; //horizon
    double target_y = spl(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    // double vel = (egocar.speed > 0.0)? egocar.speed : 0.224;  // current speed in mph
    // double vel_mps = vel / 2.24;   // current speed in m/s
    double num_points = target_dist / (TIMESTEP * ego_v);

    double x_add_on = 0.0;
    for(int i = 1; i <= NUM_TRAJECTORY_POINTS - prev_path_size; i++) 
    { 
        double x = x_add_on + target_x / num_points;
        double y = spl(x);
        x_add_on = x;

        // convert to global coordinates
        vector<double> xy_global = transform_to_global(x, y, ref_x, ref_y, ref_yaw);
       
        trajectory.xs.push_back(xy_global[0]);
        trajectory.ys.push_back(xy_global[1]);
    
    }

    return trajectory;

}

// Returns the two points to start the new trajectory from. 
// These are either the car's position and a previous point
// in its direction, or the last two points of the previous
// trajectory.
// Also returns a ref_yaw for transforming points into local
// coordinate system.
void TrajectoryGenerator::initial_spline_points(vector<double>& knot_xs, vector<double>& knot_ys, Vehicle& egocar, Trajectory& prev_path, double& ref_yaw) 
{
    int prev_path_size = prev_path.size();
    if(prev_path_size < 2) 
    {
        double prelast_x = egocar.x - cos(deg2rad(egocar.yaw));
        double prelast_y = egocar.y - sin(deg2rad(egocar.yaw));

        knot_xs.push_back(prelast_x);
        knot_xs.push_back(egocar.x);
        knot_ys.push_back(prelast_y);
        knot_ys.push_back(egocar.y);

        ref_yaw = deg2rad(egocar.yaw);
    } 
    else 
    {
        double last_x = prev_path.xs[prev_path_size - 1];
        double last_y = prev_path.ys[prev_path_size - 1];
        double prelast_x = prev_path.xs[prev_path_size - 2];
        double prelast_y = prev_path.ys[prev_path_size - 2];

        knot_xs.push_back(prelast_x);
        knot_xs.push_back(last_x);
        knot_ys.push_back(prelast_y);
        knot_ys.push_back(last_y);

        ref_yaw = atan2(last_y - prelast_y, last_x - prelast_x);
        // egocar.speed = distance(prelast_x, prelast_y, last_x, last_y) / TIMESTEP;
    }
}


// double TrajectoryGenerator::target_s(double start_s, double start_v, double target_v, double palnning_time) {
// 	start_v = min(start_v, MAX_SPEED);
// 	if (fabs(start_v / target_v - 1) < 0.05) {
// 		// start_v is almost near target_v
// 		return start_s + target_v * palnning_time;
// 	}

//     // shortest time to ge to target velocity
// 	double acc_t = min(fabs(target_v - start_v)/MAX_ACC, palnning_time); 
// 	// considers both distance traveled during reaching of target v (with max acceleration)
// 	// and distance traveled with target v.
// 	double dist_target_v = (palnning_time - acc_t) * target_v;
// 	if (target_v > start_v){
//         // accelerate
// 		return start_s + acc_t * start_v + 0.5 * pow(acc_t,2) * MAX_ACC + dist_target_v;
// 	} else{
//         // decelerate
// 		return start_s + acc_t * start_v - 0.5 * pow(acc_t,2) * MAX_ACC + dist_target_v; 
// 	}
// }


void TrajectoryGenerator::end_spline_points(vector<double>& knot_xs, vector<double>& knot_ys, double t_s, double target_v, double target_d)
{
    // end knot points for the spline
    // the last point of spline is target_s
    // the two points before that are 5 and 10 trajectory points behind (num_points_behid)
    // with their distance from target_s given by target_v * TIMESTEP * num_points_behind 

    // vector<double> end_pt1 = map.get_xy(t_s - target_v * TIMESTEP * 10, target_d);
    // vector<double> end_pt2 = map.get_xy(t_s - target_v * TIMESTEP * 5, target_d);
    // vector<double> end_pt3 = map.get_xy(t_s, target_d);

    // vector<double> end_pt1 = map.get_xy(t_s, target_d);
    // vector<double> end_pt2 = map.get_xy(t_s + 30, target_d);
    // vector<double> end_pt3 = map.get_xy(t_s + 60, target_d);

    vector<double> end_pt1 = map.get_xy(t_s + 30, target_d);
    vector<double> end_pt2 = map.get_xy(t_s + 60, target_d);
    vector<double> end_pt3 = map.get_xy(t_s + 90, target_d);



    knot_xs.push_back(end_pt1[0]);
    knot_xs.push_back(end_pt2[0]);
    knot_xs.push_back(end_pt3[0]);

    knot_ys.push_back(end_pt1[1]);
    knot_ys.push_back(end_pt2[1]);
    knot_ys.push_back(end_pt3[1]);

}

void TrajectoryGenerator::transform_to_local(vector<double>& knot_xs, vector<double>& knot_ys, const double ref_x,  const double ref_y, const double ref_yaw)
{
    // transform the knot points in the ego car's 
    // coordinate system (the beginning point 
    // becomes (0, 0) and the beginnign yaw becomes
    // 0 degrees and the rest of the points are 
    // transformed accordingly)
    for(int i = 0; i < knot_xs.size(); ++i)
    {
        double shift_x = knot_xs[i] - ref_x;
        double shift_y = knot_ys[i] - ref_y;

        knot_xs[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
        knot_ys[i] = (shift_x * sin(ref_yaw) + shift_y * cos(-ref_yaw));
    }
    
}


vector<double> TrajectoryGenerator::transform_to_global(const double x_local, const double y_local, const double ref_x, const double ref_y, const double ref_yaw)
{        
        double x_global = (x_local * cos(ref_yaw) - y_local * sin(ref_yaw));
        double y_global = (x_local * sin(ref_yaw) + y_local * cos(ref_yaw));

        x_global += ref_x;
        y_global += ref_y;

        return {x_global, y_global};
}



