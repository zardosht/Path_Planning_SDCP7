#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <math.h>
#include <map>
#include <string>

#include "vehicle.h"
#include "trajectory_generator.h"
#include "prediction.h"

using std::map;
using std::string;

struct Prediction;

const string KeepLane = "KeepLane"; // same lane, do not change speed
const string SlowDown = "SlowDown"; // keep lane (same lane) and decrease speed
const string SpeedUp = "SpeedUp";   // keep lane (same lane) and increase speed
const string ChangeLaneRight = "ChangeLaneRight";   // change lane right (increase d) and same speed
const string ChangeLaneLeft = "ChangeLaneLeft";     // change lane left (decrease d) and increase speed


struct Behavior {
    string name;
    double accel; // -1 (deccel), 0 (keep speed), 1 (accel)
    double lane; // -1 (change lane left), 0 (keep), 1 (change lane right) 
    double cost;
};

struct Trajectory; 

class BehaviorPlanner {
   public: 

        BehaviorPlanner();

        ~BehaviorPlanner();

        Behavior next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred);

        map<string, Behavior> behaviors; 
        Behavior best_behavior;
        
    
    private: 
        void update_costs(Prediction& pred, Vehicle& egocar);
        vector<string> successor_behaviors(Behavior& behavior) ;
        double cost_speed(Behavior& behavior, Vehicle& egocar);
        double cost_collision(Behavior& behavior, Vehicle& egocar, Prediction& pred);
       
}; 

#endif  // BEHAVIOR_PLANNER_H 