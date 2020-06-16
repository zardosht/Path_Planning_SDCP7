#ifndef PREDICTION_H
#define PREDICTION_H

#include <iostream>
#include <vector>
#include "vehicle.h"
#include "trajectory_generator.h"


using std::vector;

struct Trajectory;

const double FRONT_GAP = 40;
const double LANE_CHANGE_GAP_FRONT = 15;
const double LANE_CHANGE_GAP_REAR = 10;

struct Prediction 
{
    public:
        bool too_close;
        double dist_front;
        
        bool car_left;
        double dist_front_left;

        bool car_right;
        double dist_front_right;
        
        void update(vector<vector<double>>& sensor_fusion, Vehicle& egocar, Trajectory& prev_path);
};



#endif //PREDICTION_H
