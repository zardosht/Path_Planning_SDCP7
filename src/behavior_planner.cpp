#include "behavior_planner.h"
#include <iostream>

using std::cout;
using std::endl;


BehaviorPlanner::BehaviorPlanner()
{
    
}

BehaviorPlanner::~BehaviorPlanner() {}

int BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, vector<vector<double>>& sensor_fusion) 
{
    //return Behavior::KeepLane;

    int prev_path_size = prev_path.size();
    int ego_lane = egocar.get_lane();
    int behavior = Behavior::KeepLane;

    // where will our ego car be at the end of 
    // previous_path points
    double egocar_s = egocar.s;
    if (prev_path_size > 0) {
        egocar_s = prev_path.end_s;
    } 

    bool too_close = false;
    bool car_in_left = false;
    bool car_in_right = false;

    //reduce speed if a car is in our lane and 
    // we get too close to it
    cout << "sensor_fusion.size() = " << sensor_fusion.size() << endl;
    for (int i = 0; i < sensor_fusion.size(); ++i) 
    {
        // get the lane of other car based on its d value 
        int other_car_lane = Vehicle::get_lane(sensor_fusion[i][6]);
        double other_car_s = sensor_fusion[i][5];
        double other_car_vx = sensor_fusion[i][3];
        double other_car_vy = sensor_fusion[i][4];
        double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);

        // projection of s value of the car in the future 
        // (i.e. when we would have passed through all the 
        //  points in previous_path)
        other_car_s += (double)prev_path_size * 0.02 * other_car_speed;
        
        // gap between ego car and the car in front of us
        double front_gap = 30.0;
        double lane_change_gap = 15.0;
        // distance from ego car to the car in front of us
        double distance = other_car_s - egocar_s;
        //only consider cases where distance to other car is too close

        if (other_car_lane == ego_lane) {
            // other car in our lane.
            too_close |= distance > 0 && distance < front_gap;
        } else if(other_car_lane - ego_lane == -1) {
            // other car in left lane
            car_in_left |= fabs(distance) < lane_change_gap;
        } else if (other_car_lane - ego_lane == 1) {
            // other car in right lane
            car_in_right |= fabs(distance) < lane_change_gap;
        }
    }

    // 1) Descision to change to left or right? should be made based on which side has a longer free distance.
    // 2) sometimes does not slowdown when change lange to right!"


    // gradually decreas or increase the speed
    // to avoid jerk (both at the beginning and
    // when we are behind a car in our lane)
    if(too_close) {
        std::cout << "****************** too close!" << std::endl;
        behavior |= Behavior::SlowDown;
    } else {
        if (egocar.speed < 49.5){
            behavior |= Behavior::SpeedUp;
        }
    }

    if (too_close) {
        if (ego_lane > 0 && !car_in_left) {
            behavior = Behavior::ChangeLaneLeft;
        } else if (ego_lane < 2 && !car_in_right) {
            behavior = Behavior::ChangeLaneRight | Behavior::SlowDown;
        } 
    } 

    return behavior;
     
}

float BehaviorPlanner::calculate_cost(int behavior) 
{

}


