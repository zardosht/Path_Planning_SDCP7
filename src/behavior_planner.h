#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <math.h>
#include <vector>

#include "vehicle.h"
#include "trajectory_generator.h"

using std::vector;

enum Behavior {
    // same lane, do not change speed
    KeepLane = 1<<0, // 1
    // keep lane (same lane) and decrease speed
    SlowDown = 1<<1, // 2
    // keep lane (same lane) and increase speed
    SpeedUp = 1<<2,  // 4
    // change lane right (increase d) and decrease speed
    ChangeLaneRight = 1<<3,  // 8
    // change lane left (decrease d) and increase speed
    ChangeLaneLeft = 1<<4 // 16
};

struct Trajectory; 

class BehaviorPlanner {
   public: 
        //constructor
        BehaviorPlanner();

        //desctructor
        ~BehaviorPlanner();

        //functions
        int next_behavior(Vehicle& egocar, Trajectory& prev_path, vector<vector<double>>& sensor_fusion);


        // variables
        int current_behavior; 
    
    private: 
        float calculate_cost(int behavior);

       
}; 

#endif  // BEHAVIOR_PLANNER_H 