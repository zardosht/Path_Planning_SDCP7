#include "behavior_planner.h"
#include <iostream>

BehaviorPlanner::BehaviorPlanner()
{
    
}

BehaviorPlanner::~BehaviorPlanner() {}

Behavior BehaviorPlanner::next_behavior() 
{
    return Behavior::KeepLane;
}


