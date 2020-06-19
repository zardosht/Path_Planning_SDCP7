#ifndef PREDICTION_H
#define PREDICTION_H

#include <iostream>
#include <vector>
#include "vehicle.h"
#include "trajectory_generator.h"


using std::vector;

struct Trajectory;

const double TOO_CLOSE_GAP = 30;
const double LANE_CHANGE_GAP_FRONT = 35;
const double LANE_CHANGE_GAP_REAR = -7;

const double LARGE_NUMBER = 100000;

struct Prediction 
{
    public:
        bool too_close;
        double dist_front;
        double speed_front;
        
        bool car_left;
        double dist_front_left;
        double speed_front_left;

        bool car_right;
        double dist_front_right;
        double speed_front_right;
        
        void update(vector<vector<double>>& sensor_fusion, Vehicle& egocar, Trajectory& prev_path);
};



#endif //PREDICTION_H
