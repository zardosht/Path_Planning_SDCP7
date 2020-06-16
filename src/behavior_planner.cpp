#include "behavior_planner.h"
#include <iostream>

using std::cout;
using std::endl;


BehaviorPlanner::BehaviorPlanner()
{
    
}

BehaviorPlanner::~BehaviorPlanner() {}

int BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred) 
{
    //return Behavior::KeepLane;

    int ego_lane = egocar.get_lane();
    int behavior = Behavior::KeepLane;


    // 1) Descision to change to left or right? should be made based on which side has a longer free distance.
    // 2) sometimes does not slowdown when change lange to right!"


    // gradually decreas or increase the speed
    // to avoid jerk (both at the beginning and
    // when we are behind a car in our lane)
    if(pred.too_close) {
        std::cout << "****************** too close!" << std::endl;
        behavior |= Behavior::SlowDown;
    } else {
        if (egocar.speed < 49.5){
            behavior |= Behavior::SpeedUp;
        }
    }

    if (pred.too_close) {
        if (ego_lane > 0 && !pred.car_left) {
            behavior = Behavior::ChangeLaneLeft;
        } else if (ego_lane < 2 && !pred.car_right) {
            behavior = Behavior::ChangeLaneRight | Behavior::SlowDown;
        } 
    } 

    return behavior;
     
}

float BehaviorPlanner::calculate_cost(int behavior) 
{

}


