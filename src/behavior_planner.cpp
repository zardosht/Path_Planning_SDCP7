#include "behavior_planner.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;
using std::min;
using std::max;

const double MAX_COST = 5000;

BehaviorPlanner::BehaviorPlanner()
{
    Behavior kl(0);
    kl.id = 0;
    kl.name = KeepLane;
    kl.cost = MAX_COST;
    behaviors.push_back(kl);

    Behavior cll(-1);
    cll.id = 1;
    cll.name = ChangeLaneLeft;
    cll.cost = MAX_COST;
    behaviors.push_back(cll);

    Behavior clr(1);
    clr.id = 2;
    clr.name = ChangeLaneRight;
    clr.cost = MAX_COST;
    behaviors.push_back(clr);

    current_behavior = kl;
    previous_lane = 1; // the car starts at lane 1
    current_lane = 1;
    lane_changed = false;
    return_keep_lane = 0;

}

BehaviorPlanner::~BehaviorPlanner() {}

Behavior BehaviorPlanner::next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred) 
{

    cout << "\nego_lane = " << egocar.get_lane() << endl;
    cout << "current_behavior = "  << current_behavior.name << endl;
    cout << "previous_lane = " << previous_lane << endl;

    current_lane = egocar.get_lane();
    if(current_lane != previous_lane) {
        lane_changed = true;
        previous_lane = current_lane;
    } else {
        lane_changed = false;
    }
    
    cout << "current_lane = " <<  current_lane << endl;
    cout << "lane_changed = " << lane_changed << endl;
    cout << pred;

    if (egocar.speed < 10) {
        Behavior& kl = behaviors[0];  // keep lane
        kl.target_v = 20;   // m/s
        best_behavior = kl;

    } else {

        update_costs(pred, egocar);

        cout << "***** KeepLane.cost = " << behaviors[0].cost << endl;
        cout << "***** ChangeLaneLeft.cost = " << behaviors[1].cost << endl;
        cout << "***** ChangeLaneRight.cost = " << behaviors[2].cost << endl;
    }

    // if started a lane change, continue till it's finished
    if (current_behavior.name.compare(KeepLane) != 0) {
        // prevent the case for oscilating between keep lane and lane change
        // the other case for oscilating between opposite lane change behaviors
        // is taken care of in transition_cost (todo: move this check also to transitioin_cost)
        if(best_behavior.name.compare(KeepLane) == 0 && lane_changed != true) {
            best_behavior = current_behavior;
        }
    }


    // if a lane change finished and then best behavior is keep lane, then return it 50 times
    // (to make lane changes more stable)
    cout << "return_keep_lane = " << return_keep_lane << endl;
    if (current_behavior.name.compare(KeepLane) != 0 && best_behavior.name.compare(KeepLane) == 0) {
        return_keep_lane = 1;
    }
    if(return_keep_lane > 0 && return_keep_lane < 50) {
        best_behavior = behaviors[0]; // keep lane
        ++return_keep_lane;
    }else {
        return_keep_lane = 0;
    }
    cout << "*********************************** best: " << best_behavior.name << endl;

    current_behavior = best_behavior;
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
        double cost_transition = transition_cost(b);
        b.cost = cost_v + cost_dist + cost_lane_change + cost_transition;
        if (b.cost < min_cost) {
            best_behavior = b;
            min_cost = b.cost;
        }
    }

    if (pred.all_lanes_blocked) 
    {
        Behavior& kl = behaviors[0];  // keep lane. kl.target_v is already set in speed_cost()
        kl.cost = 0;
        best_behavior = kl;
    }
}


double BehaviorPlanner::transition_cost(Behavior& b)
{
    // Prevent immedate lane change in the opposite direction.
    // The driving more robust and prevents "dancing" car when undecided
    if (current_behavior.name.compare(ChangeLaneLeft) == 0) {
        // previous behavior was ChangeLaneLeft
        if (b.name.compare(ChangeLaneRight) == 0) {
            return 800.0;
        }
    } else if (current_behavior.name.compare(ChangeLaneRight) == 0) {
        // previous behavior was ChangeLaneRight
        if (b.name.compare(ChangeLaneLeft) == 0) {
            return 800.0;
        }
    } 
    
    if (lane_changed && b.name.compare(KeepLane) == 0) {
        // favor keep the lane after a lane change
        // for at least 40 behavior steps
        return -10.0;
    } 
    
    return 0.0;
    
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