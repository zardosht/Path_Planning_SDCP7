#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include "behavior_planner.h"

using std::vector;

struct Trajectory  
{
    vector<double> xs;
    vector<double> ys;
};

class TrajectoryGenerator 
{
    public:
        // constructor
        TrajectoryGenerator();

        // destructor
        ~TrajectoryGenerator();

        // functions
        Trajectory generate_trajectory(State state);

        // variables

    private:
        

};


#endif //TRAJECTORY_GENERATOR_H