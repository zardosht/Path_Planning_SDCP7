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
    change_lane_right.cost = LARGE_NUMBER;
    behaviors[BehaviorNames::ChangeLaneRight] = change_lane_right;

}

BehaviorPlanner::~BehaviorPlanner() {}

Behavior BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred) 
{
    int ego_lane = egocar.get_lane();
   
    // 1) Descision to change to left or right? should be made based on which side has a longer free distance.
    // 2) sometimes does not slowdown when change lange to right!"


    update_costs(pred, egocar);

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


void BehaviorPlanner::update_costs(Prediction& pred, Vehicle& egocar) 
{

    double min_cost = LARGE_NUMBER;
    cout << "*********** pred.dist_front = " << pred.dist_front << endl;

    if(egocar.speed == 0) {
        best_behavior = behaviors[BehaviorNames::SpeedUp];
        return;
    }

    for(const auto pair : behaviors) {
        int name = pair.first;
        Behavior behavior = pair.second;
        behavior.cost = LARGE_NUMBER;
        switch (name)
        {
        case BehaviorNames::KeepLane:
            break;

        case BehaviorNames::SpeedUp:
            behavior.cost = (pred.too_close) * 1 / pred.dist_front;
            break;

        case BehaviorNames::SlowDown:
            cout << "*********** 1- too close: " << 1 - pred.too_close << endl; 
            cout << "*********** speed up cost: " << (pred.too_close) * 1 / pred.dist_front << endl; 
            cout << "*********** slow down cost: " << (1 - pred.too_close) * 1 / pred.dist_front << endl;
            behavior.cost = (1 - pred.too_close) * 1 / pred.dist_front;
            break;

        case BehaviorNames::ChangeLaneLeft:
            if (pred.too_close && !pred.car_left) {
                cout << "*********** change lane left cost: " << -pred.dist_front_left << endl; 
                // if both options of going left and right are good, prefer to go left
                behavior.cost = -pred.dist_front_left - 0.1;  
            }
            break;

        case BehaviorNames::ChangeLaneRight:
            if (pred.too_close && !pred.car_right) {
                cout << "*********** change lane right cost: " << -pred.dist_front_right << endl;
                behavior.cost = -pred.dist_front_right;  
            }
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


