#include "behavior_planner.h"
#include <iostream>

BehaviorPlanner::BehaviorPlanner(int i)
{
    std::cout << i; 
}

BehaviorPlanner::~BehaviorPlanner() {}

State BehaviorPlanner::next_state() 
{
    return State::KL;
}


