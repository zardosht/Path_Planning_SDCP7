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
    update_costs(pred, egocar);
    return best_behavior;
     
}


void BehaviorPlanner::update_costs(Prediction& pred, Vehicle& egocar) 
{

    double min_cost = LARGE_NUMBER;

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
            if(!pred.too_close) {
                behavior.cost = 0;
            } else {
                behavior.cost = 0.5;
            }
            break;

        case BehaviorNames::SlowDown:
            if (best_behavior.name == BehaviorNames::ChangeLaneLeft || best_behavior.name == BehaviorNames::ChangeLaneRight) {
                // do not slow down if we are changing lanes.    
                behavior.cost = 0.5;
            } else if (pred.too_close) {
                // do not slow down if we are changing lanes.    
                cout << "*********** pred.dist_front = " << pred.dist_front << endl;
                cout << "*********** slow down cost: " << 0 << endl; 
                behavior.cost = 0;
            } else {
                behavior.cost = 0.5;
            }

            break;

        case BehaviorNames::ChangeLaneLeft:
            if (pred.too_close && !pred.car_left) { 
                cout << "*********** pred.dist_front = " << pred.dist_front << endl;
                cout << "*********** change lane left cost: " << -pred.dist_front_left - 0.1 << endl; 
                // if both options of going left and right are good, prefer to go left
                behavior.cost = -pred.dist_front_left - 0.1;  
            }
            break;

        case BehaviorNames::ChangeLaneRight:
            if (pred.too_close && !pred.car_right) { 
                cout << "*********** pred.dist_front = " << pred.dist_front << endl;
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


