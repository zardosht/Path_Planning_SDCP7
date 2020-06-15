#include "behavior_planner.h"
#include <iostream>

BehaviorPlanner::BehaviorPlanner()
{
    
}

BehaviorPlanner::~BehaviorPlanner() {}

int BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, vector<vector<double>>& sensor_fusion) 
{
    //return Behavior::KeepLane;

    int prev_path_size = prev_path.size();
    int lane = egocar.get_lane();
    int behavior = Behavior::KeepLane;

    // where will our ego car be at the end of 
    // previous_path points
    double car_s = egocar.s;
    if (prev_path_size > 0) {
        car_s = prev_path.end_s;
    } 

    bool too_close = false;

    //reduce speed if a car is in our lane and 
    // we get too close to it
    for (int i = 0; i < sensor_fusion.size(); ++i) 
    {
        // get the d value of the car
        float d = sensor_fusion[i][6];

        // check if a car is in our lane
        if (d < (2 + 4.0 * lane + 2) && d > (2 + 4.0 * lane - 2)) 
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            // projection of s value of the car in the future 
            // (i.e. when we would have passed through all the 
            //  points in previous_path)
            check_car_s += (double)prev_path_size * 0.02 * check_speed;
            
            // gap between ego car and the car in front of us
            double gap = 30.0;
            // distance from ego car to the car in front of us
            double distance = check_car_s - car_s;
            // if the car is front of us (distance > 0) and our 
            // distance is less than allowed gap
            if (distance > 0 && distance < gap) 
            {
                std::cout << "****************** too close!" << std::endl;
                too_close = true;

                // change lane
                if (lane > 0) 
                {
                    behavior = Behavior::ChangeLaneLeft;
                }
            } 
        }
    }

    
    // gradually decreas or increase the speed
    // to avoid jerk (both at the beginning and
    // when we are behind a car in our lane)
    if (too_close) 
    {
        behavior |= Behavior::SlowDown;
    }
    else if (egocar.speed < 49.5)
    {
        behavior |= Behavior::SpeedUp;
    }

    return behavior;

}

float BehaviorPlanner::calculate_cost(int behavior) 
{

}


