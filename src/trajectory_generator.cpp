#include "trajectory_generator.h"
#include "spline.h"
#include <iostream>


using std::cout;
using std::endl;


TrajectoryGenerator::TrajectoryGenerator(Map& map) : map(map) 
{
    
}

TrajectoryGenerator::~TrajectoryGenerator() {}


void prepare_trajectory_spline(tk::spline& spl, 
                               double& ref_x, 
                               double& ref_y, 
                               double& ref_yaw, 
                               const double ego_s,
                               Map& map, 
                               Trajectory& previous_path, 
                               int prev_path_size, 
                               int lane); 

Trajectory TrajectoryGenerator::generate_trajectory(Behavior behavior, Vehicle& egocar, Trajectory previous_path)
{
    

    // first todo: move the helper methods to Map file

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

    
    // generate_trajectory(lane, speed, previous_path)
    //        prepares spline points for the given lane
    //        addds the points from previous path
    //        uses the spline to generate points for the trajectory
    
    
    
    Trajectory trajectory;

    std::cout << "previous path size: " << previous_path.size() << std::endl;

 
    int lane = 1;
    double ref_vel = 49.5;  //mph
    int prev_path_size = previous_path.size();

    double ref_x = egocar.x;
    double ref_y = egocar.y;
    double ref_yaw = deg2rad(egocar.yaw);


    // get frist two starting knot points of the new path
    // get the yaw of the new path (either yaw of the car or yaw of the last two starting points)
    // get end knot points of the spline
    // transform all knot points to car coordinates (using the yaw and x, y of the beginnig points)


    tk::spline spl;
    prepare_trajectory_spline(spl, ref_x, ref_y, ref_yaw, 
                              egocar.s, map, previous_path, 
                              prev_path_size, lane);
    
    // add the points from previous path to the 
    // beginning of current path
    for(int i = 0; i < prev_path_size; ++i)
    {
        trajectory.xs.push_back(previous_path.xs[i]);
        trajectory.ys.push_back(previous_path.ys[i]);
    }

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
    double ref_vel_m_per_sec = ref_vel / 2.24;
    double N = target_dist / (0.02 * ref_vel_m_per_sec);

    double x_add_on = 0;
    // fill the rest of current path (till we have 50 points)
    // with the points interpolated by the spline
    for(int i = 1; i <= 50 - prev_path_size; i++) 
    {
        double x_point = x_add_on + target_x / N;
        double y_point = spl(x_point);

        x_add_on = x_point;
        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        trajectory.xs.push_back(x_point);
        trajectory.ys.push_back(y_point);

    }

    return trajectory;
}

void prepare_trajectory_spline(tk::spline& spl, 
                               double& ref_x, 
                               double& ref_y, 
                               double& ref_yaw, 
                               const double ego_s,
                               Map& map, 
                               Trajectory& previous_path, 
                               int prev_path_size, 
                               int lane) 
{
    // *********************
    // * Set the knot points for the spline.
    // * The spline is then used to give us path
    // * points with a smooth transition between 
    // * points.
    // *********************
    
    vector<double> ptsx;
    vector<double> ptsy;

    // Beginning knot points for the spline.
    // Either take the corrent position of the car
    // (if the previous path is consumed, i.e. 
    // prev_size < 2), or take the last two points
    // from the previous path.
    // This helps in a smooth transition from 
    // previous path to the current path
    if(prev_path_size < 2) 
    {
        double prev_car_x = ref_x - cos(ref_yaw);
        double prev_car_y = ref_y - sin(ref_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);

    } 
    else 
    {
        ref_x = previous_path.xs[prev_path_size - 1];
        ref_y = previous_path.ys[prev_path_size - 1];

        double ref_x_prev = previous_path.xs[prev_path_size - 2];
        double ref_y_prev = previous_path.ys[prev_path_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // end knot points for the spline
    // pick points in the far distance to
    // have a smooth spline
    vector<double> next_wp0 = map.get_xy(ego_s + 30, 
                                    (2.0 + 4.0 * lane));
    vector<double> next_wp1 = map.get_xy(ego_s + 60, 
                                    (2.0 + 4.0 * lane));
    vector<double> next_wp2 = map.get_xy(ego_s + 90, 
                                    (2.0 + 4.0 * lane));
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]); 
    
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // transform the knot points in the ego car's 
    // coordinate system (the beginning point 
    // becomes (0, 0) and the beginnign yaw becomes
    // 0 degrees and the rest of the points are 
    // transformed accordingly)
    for(int i = 0; i < ptsx.size(); ++i)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    }
    
    // set the knot points
    spl.set_points(ptsx, ptsy);

}