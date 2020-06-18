#include "trajectory_generator.h"
#include "spline.h"
#include <iostream>


using std::cout;
using std::endl;


TrajectoryGenerator::TrajectoryGenerator(Map& map) : map(map) 
{
    
}

TrajectoryGenerator::~TrajectoryGenerator() {}


Trajectory TrajectoryGenerator::generate_trajectory(Behavior behavior, Vehicle& egocar, Trajectory& previous_path)
{
    

    // the prediction gives the predicted trajectory of other cars (check the github repo)

    // the behavior planner checks for each behavior if collides with other cars or goes out of road 

    // the behavior gives the target lane and the speed (increas or decrease)
    // Or the behavior gives lane and acceleration? (because acceleration is the measure of how to increase or decrese speed)
    // there will be following behaviors:
    //     SlowDown = keep lane (same lane) and decrease speed
    //     SpeedUp = keep lane (same lane) and increase speed
    //     KeepLane = same lane, do not change speed
    //     ChangeLaneRight = change lane right (increase d) and increase speed
    //     ChangeLaneLeft = change lane left (decrease d) and increase speed

    
    // generate_trajectory(lane, accel[DECEL, ZERO, ACCEL], previous_path)
    //        prepares spline points for the given lane
    //        addds the points from previous path
    //        uses the spline to generate points for the trajectory
    
    
    
    // find out the lane from behavior value
    // find out the acceleration from behavior value (ACCEL, ZERO, DECEL)
    double target_d = get_d(behavior, egocar);
    int accel = get_accel(behavior);

    int prev_path_size = previous_path.size();
    if (prev_path_size >= PLAN_NEW_TRAJECTORY_THRESHOLD) 
        return previous_path;

    return generate_trajectory(egocar, target_d, accel, previous_path);

}


Trajectory TrajectoryGenerator::generate_trajectory(Vehicle& egocar, double target_d, int accel, Trajectory& previous_path) 
{

    int prev_path_size = previous_path.size();

    if (accel == Accel::ACCEL) {
        if (vel < MAX_SPEED) {  // convert v to mph for comparing
            cout << "****** Speeding up" << endl;
            vel += 0.224;
        }
    } else if (accel == Accel::DECEL) {
        if (vel > 1) {
            cout << "****** Slowing down" << endl;
            vel -= 0.224;
        }
    } else {
        cout << "****** Keep Speed" << endl;
    }


    // get initial spline knots
    // get initial yaw (for transforming the points
    // into local coordinates and back to global coordinates)
    // get end spline knots
    Array2Xd spline_knots(2, 5);
    double ref_yaw = deg2rad(egocar.yaw);  //inout
    initial_spline_points(spline_knots, egocar, previous_path, ref_yaw);

    double ref_x = spline_knots(0, 1);
    double ref_y = spline_knots(1, 1);

    double start_from_s = (prev_path_size > 0)? previous_path.end_s : egocar.s;
    end_spline_points(spline_knots, egocar, start_from_s, target_d);

    // transform spline knots into local coordinates
    transform_to_local(spline_knots, ref_x, ref_y, ref_yaw);

    // prepare spline
    tk::spline spl;
    vector<double> knot_xs(spline_knots.cols());
    vector<double> knot_ys(spline_knots.cols());
    for (int i = 0; i < spline_knots.cols(); ++i) 
    {
        knot_xs[i] = spline_knots(0, i);
        knot_ys[i] = spline_knots(1, i);
    }
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

    double target_x = 30.0; //horizon
    double target_y = spl(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    // double vel = (egocar.speed > 0.0)? egocar.speed : 0.224;  // current speed in mph
    double vel_mps = vel / 2.24;   // current speed in m/s
    double num_points = target_dist / (TIMESTEP * vel_mps);

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
void TrajectoryGenerator::initial_spline_points(Array2Xd& spline_knots, Vehicle& egocar, Trajectory& prev_path, double& ref_yaw) 
{
    int prev_path_size = prev_path.size();
    if(prev_path_size < 2) 
    {
        double prelast_x = egocar.x - cos(deg2rad(egocar.yaw));
        double prelast_y = egocar.y - sin(deg2rad(egocar.yaw));

        spline_knots(0, 0) = prelast_x;
        spline_knots(0, 1) = egocar.x;
        spline_knots(1, 0) = prelast_y;
        spline_knots(1, 1) = egocar.y;

        ref_yaw = deg2rad(egocar.yaw);
    } 
    else 
    {
        double last_x = prev_path.xs[prev_path_size - 1];
        double last_y = prev_path.ys[prev_path_size - 1];
        double prelast_x = prev_path.xs[prev_path_size - 2];
        double prelast_y = prev_path.ys[prev_path_size - 2];

        spline_knots(0, 0) = prelast_x;
        spline_knots(0, 1) = last_x;
        spline_knots(1, 0) = prelast_y;
        spline_knots(1, 1) = last_y;

        ref_yaw = atan2(last_y - prelast_y, last_x - prelast_x);
    }

}

void TrajectoryGenerator::end_spline_points(Array2Xd& spline_knots, Vehicle& egocar, double start_from_s, double target_d)
{
    // end knot points for the spline
    // pick points in the far distance to
    // have a smooth spline
    double factor = egocar.speed < 10? 1 : egocar.speed / MAX_SPEED;
    vector<double> end_pt1 = map.get_xy(start_from_s + factor * 30, target_d);
    vector<double> end_pt2 = map.get_xy(start_from_s + factor * 60, target_d);
    vector<double> end_pt3 = map.get_xy(start_from_s + factor * 90, target_d);

    //spline_knots already includes the two begining points
    spline_knots(0, 2) = end_pt1[0];
    spline_knots(0, 3) = end_pt2[0];
    spline_knots(0, 4) = end_pt3[0];

    spline_knots(1, 2) = end_pt1[1];
    spline_knots(1, 3) = end_pt2[1];
    spline_knots(1, 4) = end_pt3[1];

}

void TrajectoryGenerator::transform_to_local(Array2Xd& spline_knots,  const double ref_x,  const double ref_y, const double ref_yaw)
{
    // transform the knot points in the ego car's 
    // coordinate system (the beginning point 
    // becomes (0, 0) and the beginnign yaw becomes
    // 0 degrees and the rest of the points are 
    // transformed accordingly)
    for(int i = 0; i < spline_knots.cols(); ++i)
    {
        double shift_x = spline_knots(0, i) - ref_x;
        double shift_y = spline_knots(1, i) - ref_y;

        spline_knots(0, i) = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        spline_knots(1, i) = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
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


double TrajectoryGenerator::get_d(Behavior behavior, Vehicle& egocar) 
{
    // find current lane
    int lane = egocar.get_lane();
    if (behavior.name == BehaviorNames::ChangeLaneLeft) {
        if (lane > 0){
            cout << "****** Changing lane to left" << endl;
            lane += -1;
        }
    } else if (behavior.name == BehaviorNames::ChangeLaneRight) {
        if (lane < 2) {
            cout << "****** Changing lane to right" << endl;
            lane += 1;
        }
    }

    double d = (2.0 + 4.0 * lane);
    
    // this should never happen!
    if (d > 11){
        cout << "ERROR: d value too large! correcting to 10" << endl;
        d = 10;
    }
    if (d < 1) {
        cout << "ERROR: d value too small! correcting to 2" << endl;
        d = 2;
    }
    return d;
}   


int TrajectoryGenerator::get_accel(Behavior behavior)
{
    // SlowDown = keep lane (same lane) and decrease speed
    // SpeedUp = keep lane (same lane) and increase speed
    // KeepLane = same lane, do not change speed
    // ChangeLaneRight = change lane right (increase d) do not change speed
    // ChangeLaneLeft = change lane left (decrease d) do not change speed
    if (behavior.name == BehaviorNames::SpeedUp) {
        return Accel::ACCEL;
    } else if (behavior.name == BehaviorNames::SlowDown){
        return Accel::DECEL;
    } else {
        return Accel::ZERO;
    }
}
