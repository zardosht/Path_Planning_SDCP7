#include "behavior_planner.h"
#include <iostream>

BehaviorPlanner::BehaviorPlanner()
{
    
}

BehaviorPlanner::~BehaviorPlanner() {}

int BehaviorPlanner::next_behavior() 
{
    return Behavior::KeepLane;
}


