#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <math.h>
#include <map>

#include "vehicle.h"
#include "trajectory_generator.h"
#include "prediction.h"

using std::map;

struct Prediction;

enum BehaviorNames {
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

struct Behavior {
    int name;
    double cost;
};

struct Trajectory; 

class BehaviorPlanner {
   public: 

        BehaviorPlanner();

        ~BehaviorPlanner();

        Behavior next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred);

        map<int, Behavior> behaviors; 
        Behavior best_behavior;

    
    private: 
        void update_costs();

       
}; 

#endif  // BEHAVIOR_PLANNER_H 