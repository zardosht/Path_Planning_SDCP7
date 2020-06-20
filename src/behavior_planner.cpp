#include "behavior_planner.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;
using std::min;
using std::max;

const double MAX_COST = 500;

BehaviorPlanner::BehaviorPlanner()
{
    Behavior kl(0);
    kl.name = KeepLane;
    kl.cost = MAX_COST;
    behaviors.push_back(kl);

    Behavior cll(-1);
    cll.name = ChangeLaneLeft;
    cll.cost = MAX_COST;
    behaviors.push_back(cll);

    Behavior clr(1);
    clr.name = ChangeLaneRight;
    clr.cost = MAX_COST;
    behaviors.push_back(clr);

}

BehaviorPlanner::~BehaviorPlanner() {}

Behavior BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred) 
{

    if (egocar.speed < 10) {
        Behavior& kl = behaviors[0];  // keep lane
        kl.target_v = 20;   // m/s
        best_behavior = kl;
        return best_behavior;
    }

    cout << "\nego_lane = " << egocar.get_lane() << endl;
    cout << pred;

    int ego_lane = egocar.get_lane();
    update_costs(pred, egocar);

    cout << "***** KeepLane.cost = " << behaviors[0].cost << endl;
    cout << "***** ChangeLaneLeft.cost = " << behaviors[1].cost << endl;
    cout << "***** ChangeLaneRight.cost = " << behaviors[2].cost << endl;
    cout << "*********************************** best: " << best_behavior.name << endl;

    return best_behavior;
     
}


void BehaviorPlanner::update_costs(Prediction& pred, Vehicle& egocar) 
{
    double min_cost = MAX_COST;
    for (Behavior& b : behaviors)
    {

        double cost_dist = distance_cost(b, egocar, pred);
        double cost_v = speed_cost(b, egocar, pred);
        double cost_lane_change = lane_change_cost(b, egocar, pred);
        b.cost = cost_v + cost_dist + cost_lane_change;
        if (b.cost < min_cost) {
            best_behavior = b;
            min_cost = b.cost;
        }
    }

    if (pred.all_lanes_blocked) 
    {
        Behavior& kl = behaviors[0];  // keep lane
        // kl.target_v is already set in speed_cost()
        kl.cost = 0;
        best_behavior = kl;
    }
}


double BehaviorPlanner::lane_change_cost(Behavior& b, Vehicle& egocar, Prediction& pred) 
{
    if (b.name.compare(KeepLane) == 0) 
    {
        return 0.0;
    }

    int tl = b.target_lane(egocar.get_lane());
    Lane& lane = pred.lanes[tl];
    if (lane.blocked) {
        return 100;
    } else {
        return 0;
    }
}


double BehaviorPlanner::distance_cost(Behavior& b, Vehicle& egocar, Prediction& pred)
{
    // favor the lane with the farthest free distance ahead
    int tl = b.target_lane(egocar.get_lane());
    Lane& lane = pred.lanes[tl];
    double dist = lane.front_dist;
    double cost_dist = 1 / dist;
    if (lane.blocked) {
        return 100 + cost_dist;
    }
    return cost_dist;

}


double BehaviorPlanner::speed_cost(Behavior& b, Vehicle& egocar, Prediction& pred) 
{
    // get v for the target_lane of behavior
    int tl = b.target_lane(egocar.get_lane());
    Lane& lane = pred.lanes[tl]; 
    double v = lane.front_v;
    b.target_v = v;
    double cost_v = 1 - v / MAX_SPEED;
    if (lane.blocked) {
        return 100 + cost_v;
    } 
    return cost_v;
    
}

Behavior::Behavior() { }

Behavior::Behavior(int ls) : lane_shift(ls) {}


Behavior::~Behavior() {}


double Behavior::target_d(double current_lane) 
{
    // find current lane
    int lane = target_lane(current_lane);
    
    // this should never happen!
    if (lane < 0) {
        cout << "ERROR: target_lane negative. Correcting to 0" << endl;
        lane = 0;
    }
    if (lane > 2) {
        // we only have three lanes 0, 1, 2
        cout << "ERROR: target_lane larger than 2. Correcting to 2" << endl;
        lane = 2;
    }
    
    return 2.0 + 4.0 * lane;
}


int Behavior::target_lane(int current_lane)  
{
    return current_lane + lane_shift;
}