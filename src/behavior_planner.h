#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H


enum Behavior {
    // same lane, do not change speed
    KeepLane = 1<<0,
    // keep lane (same lane) and decrease speed
    SlowDown = 1<<1,
    // keep lane (same lane) and increase speed
    SpeedUp = 1<<2,
    // change lane right (increase d) and decrease speed
    ChangeLaneRight = 1<<3, 
    // change lane left (decrease d) and increase speed
    ChangeLaneLeft = 1<<4
};


class BehaviorPlanner {
   public: 
        //constructor
        BehaviorPlanner();

        //desctructor
        ~BehaviorPlanner();

        //functions
        int next_behavior();


        // variables
        Behavior behavior; 
    
    private: 
        float calcuateCost(int behavior);
        float sucessor_behaviors(int behavior);
       
}; 

#endif  // BEHAVIOR_PLANNER_H 