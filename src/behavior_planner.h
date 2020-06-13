#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H


enum Behavior {
    // keep lane (same lane) and decrease speed
    SlowDown,
    // keep lane (same lane) and increase speed
    SpeedUp,
    // same lane, do not change speed
    KeepLane,
    // change lane right (increase d) and decrease speed
    ChangeLaneRight, 
    // change lane left (decrease d) and increase speed
    ChangeLaneLeft
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