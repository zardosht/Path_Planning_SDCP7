#include "behavior_planner.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;


BehaviorPlanner::BehaviorPlanner()
{
    Behavior keep_lane;
    keep_lane.name = KeepLane;
    keep_lane.accel = 0;
    keep_lane.lane = 0;
    keep_lane.cost = LARGE_NUMBER;
    behaviors[KeepLane] = keep_lane;

    Behavior speed_up;
    speed_up.name = SpeedUp;
    speed_up.accel = 1;
    speed_up.lane = 0;
    speed_up.cost = LARGE_NUMBER;
    behaviors[SpeedUp] = speed_up;

    Behavior slow_down;
    slow_down.name = SlowDown;
    slow_down.accel = -1;
    slow_down.lane = 0;
    slow_down.cost = LARGE_NUMBER;
    behaviors[SlowDown] = slow_down;

    Behavior change_lane_left;
    change_lane_left.name = ChangeLaneLeft;
    change_lane_left.accel = 0;
    change_lane_left.lane = -1;
    change_lane_left.cost = LARGE_NUMBER;
    behaviors[ChangeLaneLeft] = change_lane_left;

    Behavior change_lane_right;
    change_lane_right.name = ChangeLaneRight;
    change_lane_right.accel = 0;
    change_lane_right.lane = 1;
    change_lane_right.cost = LARGE_NUMBER;
    behaviors[ChangeLaneRight] = change_lane_right;

}

BehaviorPlanner::~BehaviorPlanner() {}




Behavior BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred) 
{
    int ego_lane = egocar.get_lane();
    update_costs(pred, egocar);

    cout << "***** KeepLane.cost = " << behaviors[KeepLane].cost << endl;
    cout << "***** SlowDown.cost = " << behaviors[SlowDown].cost << endl;
    cout << "***** SpeedUp.cost = " << behaviors[SpeedUp].cost << endl;
    cout << "***** ChangeLaneRight.cost = " << behaviors[ChangeLaneRight].cost << endl;
    cout << "***** ChangeLaneLeft.cost = " << behaviors[ChangeLaneLeft].cost << endl;
    cout << "*********************************** best: " << best_behavior.name << endl;

    return best_behavior;
     
}


void BehaviorPlanner::update_costs(Prediction& pred, Vehicle& egocar) 
{

    double min_cost = LARGE_NUMBER;

    if(egocar.speed == 0) {
        best_behavior = behaviors[SpeedUp];
        return;
    }

    double coll_cost_w = 2;
    double speed_cost_w = 1;
    for (auto &pair : behaviors)
    {
        Behavior& behavior = pair.second;
        double coll_cost = cost_collision(behavior, egocar, pred);
        double speed_cost = cost_speed(behavior, egocar);
        //sigmoid
        double cost = 1 / (1 + exp(-(speed_cost_w * speed_cost + coll_cost_w * coll_cost)));
        behavior.cost = cost;
        if (cost < min_cost) {
            best_behavior = behavior;
            min_cost = cost;
        }
    }
}

double BehaviorPlanner::cost_collision(Behavior& behavior, Vehicle& egocar, Prediction& pred)
{
    // the higher collision probability, the higher the cost
    double collision_prob = 0.0;
    if (behavior.lane == 0) {
        collision_prob = pred.too_close * 1 / pred.dist_front;
    } else if (behavior.lane == -1) {
        collision_prob = pred.car_left * 1 / pred.dist_front_left;
    } else {
        collision_prob = pred.car_right * 1 / pred.dist_front_right;
    }

}


double BehaviorPlanner::cost_speed(Behavior& behavior, Vehicle& egocar) 
{
    // the higher proposed_speed, the lower the cost    
    double propoesed_speed = egocar.speed + behavior.accel * 0.224;
    double cost = 1 / propoesed_speed;
}


