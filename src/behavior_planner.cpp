#include "behavior_planner.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;


BehaviorPlanner::BehaviorPlanner()
{
    Behavior keep_lane;
    keep_lane.name = BehaviorNames::KeepLane;
    keep_lane.cost = LARGE_NUMBER;
    behaviors[BehaviorNames::KeepLane] = keep_lane;

    Behavior speed_up;
    speed_up.name = BehaviorNames::SpeedUp;
    //speed_up.cost = LARGE_NUMBER;
    speed_up.cost = 0.0;
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


    update_costs(pred);

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


void BehaviorPlanner::update_costs(Prediction& pred) 
{

    double min_cost = LARGE_NUMBER;
    cout << "*********** pred.dist_front = " << pred.dist_front << endl;

    for(const auto pair : behaviors) {
        int name = pair.first;
        Behavior behavior = pair.second;
        behavior.cost = LARGE_NUMBER;
        switch (name)
        {
        case BehaviorNames::KeepLane:
            // keep_lane, speed_up, and slow_down all depend on the distance between us and the front car.
            if (pred.dist_front >= 30 && pred.dist_front <= 70) {
                behavior.cost = 3;    
            } else {
                behavior.cost = 10;
            }
            break;
        case BehaviorNames::SpeedUp:
            if (pred.dist_front > 70){
                behavior.cost = 5;
            }else
            {
                behavior.cost = 10;
            }
            
            break;
        case BehaviorNames::SlowDown:
            if (pred.dist_front < 50){
                behavior.cost = 5;
            }else {
                behavior.cost = 10;
            }
        
            break;
        case BehaviorNames::ChangeLaneLeft:
            
            break;
        case BehaviorNames::ChangeLaneRight:
            
            break;

        default:
            break;
        }

        if(min_cost > behavior.cost) {
            min_cost = behavior.cost;
            best_behavior = behavior;
        }
    }
}


