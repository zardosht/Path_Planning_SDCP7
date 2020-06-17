#include "behavior_planner.h"
#include <iostream>

using std::cout;
using std::endl;


BehaviorPlanner::BehaviorPlanner()
{
    Behavior keep_lane;
    keep_lane.name = BehaviorNames::KeepLane;
    keep_lane.cost = 0.0;
    behaviors[BehaviorNames::KeepLane] = keep_lane;

    Behavior speed_up;
    speed_up.name = BehaviorNames::SpeedUp;
    speed_up.cost = LARGE_NUMBER;
    behaviors[BehaviorNames::SpeedUp] = speed_up;

    Behavior slow_down;
    slow_down.name = BehaviorNames::SlowDown;
    slow_down.cost = LARGE_NUMBER;
    behaviors[BehaviorNames::SlowDown] = slow_down;

    Behavior change_lane_left;
    change_lane_left.name = BehaviorNames::ChangeLaneLeft;
    change_lane_left.cost = LARGE_NUMBER;
    behaviors[BehaviorNames::ChangeLaneLeft] = change_lane_left;

    Behavior change_lane_right;
    change_lane_right.name = BehaviorNames::ChangeLaneRight;
    change_lane_right.cost = 0.0;
    behaviors[BehaviorNames::ChangeLaneRight] = change_lane_right;

}

BehaviorPlanner::~BehaviorPlanner() {}

Behavior BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred) 
{
    //return Behavior::KeepLane;

    int ego_lane = egocar.get_lane();
    Behavior behavior = behaviors[BehaviorNames::KeepLane];


    // 1) Descision to change to left or right? should be made based on which side has a longer free distance.
    // 2) sometimes does not slowdown when change lange to right!"


    update_costs();

    // gradually decreas or increase the speed
    // to avoid jerk (both at the beginning and
    // when we are behind a car in our lane)

    // if(pred.too_close) {
    //     std::cout << "****************** too close!" << std::endl;
    //     behavior |= Behavior::SlowDown;
    // } else {
    //     if (egocar.speed < 49.5){
    //         behavior |= Behavior::SpeedUp;
    //     }
    // }

    // if (pred.too_close) {
    //     if (ego_lane > 0 && !pred.car_left) {
    //         behavior = Behavior::ChangeLaneLeft;
    //     } else if (ego_lane < 2 && !pred.car_right) {
    //         behavior = Behavior::ChangeLaneRight | Behavior::SlowDown;
    //     } 
    // } 

    return best_behavior;
     
}

void BehaviorPlanner::update_costs() 
{

}


