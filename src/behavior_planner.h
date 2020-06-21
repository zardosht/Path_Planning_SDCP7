#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <math.h>
#include <vector>
#include <string>

#include "vehicle.h"
#include "trajectory_generator.h"
#include "prediction.h"


using std::string;
using std::vector;

struct Prediction;

const string KeepLane = "KeepLane"; 
const string ChangeLaneRight = "ChangeLaneRight";  
const string ChangeLaneLeft = "ChangeLaneLeft";    


struct Behavior {
    public: 
        Behavior();
        Behavior(int lane_shift);
        ~Behavior();

        int id;
        string name;
        double cost;
        double target_v;

        int target_lane(int current_lane);
        double target_d(double car_lane);

    private: 
        int lane_shift; // -1 (change lane left), 0 (keep), 1 (change lane right) 
};

struct Trajectory; 

class BehaviorPlanner {
   public: 

        BehaviorPlanner();

        ~BehaviorPlanner();

        Behavior next_behavior(Vehicle& egocar, Trajectory& prev_path, Prediction& pred);

        vector<Behavior> behaviors; 
        Behavior current_behavior;
        
    
    private: 
        void update_costs(Prediction& pred, Vehicle& egocar);
        double speed_cost(Behavior& behavior, Vehicle& egocar, Prediction& pred);
        double distance_cost(Behavior& behavior, Vehicle& egocar, Prediction& pred);
        double lane_change_cost(Behavior& b, Vehicle& egocar, Prediction& pred);
        double transition_cost(Behavior& b);
        
        Behavior best_behavior;
       
        int return_keep_lane; 
        
        int previous_lane;
        int current_lane;
        bool lane_changed;


       
}; 

#endif  // BEHAVIOR_PLANNER_H 