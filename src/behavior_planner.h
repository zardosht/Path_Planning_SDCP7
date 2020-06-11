#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H


enum State {
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
        BehaviorPlanner(int i);

        //desctructor
        ~BehaviorPlanner();

        //functions
        State next_state();


        // variables
        State currentState; 
    
    private: 
        float calcuateCost(State state);
        float sucessor_states(State state);
       
}; 

#endif  // BEHAVIOR_PLANNER_H 