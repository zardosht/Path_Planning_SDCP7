#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H


enum Behavior {
    // go straight at constant speed
    CS,
    // keep the lane
    KL,
    // prepare lane change left
    PLCL,
    // prepare lane change right
    PLCR, 
    // lane change left
    LCL, 
    // Change lane right
    LCR 
};


class BehaviorPlanner {
   public: 
        //constructor
        BehaviorPlanner();

        //desctructor
        ~BehaviorPlanner();

        //functions
        Behavior next_behavior();


        // variables
        Behavior behavior; 
    
    private: 
        float calcuateCost(Behavior behavior);
        float sucessor_behaviors(Behavior behavior);
       
}; 

#endif  // BEHAVIOR_PLANNER_H 