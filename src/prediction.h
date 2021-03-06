#ifndef PREDICTION_H
#define PREDICTION_H

#include <iostream>
#include <vector>
#include "vehicle.h"
#include "trajectory_generator.h"


using std::vector;

struct Trajectory;

// const double TOO_CLOSE_GAP = 30; // m
const double TOO_CLOSE_GAP = 40; // m

// const double LANE_CHANGE_GAP_FRONT = 35;  // m
const double LANE_CHANGE_GAP_FRONT = 25;  // m

const double LANE_CHANGE_GAP_REAR = -10;   // m

const int NUM_LANES = 3;

const double PREDICTION_HIROZON = 120; // m


struct Lane 
{
    int id;
    bool blocked;
    double front_dist;
    double front_v;
};


struct Prediction 
{
    public:
        Prediction();
        ~Prediction();

        vector<Lane> lanes;
        void update(vector<vector<double>>& sensor_fusion, Vehicle& egocar, Trajectory& prev_path);

        friend std::ostream& operator<<(std::ostream &strm, const Prediction &pred);

        bool all_lanes_blocked;

    private: 
        void init_lanes();
        void reset_lanes();
};



#endif //PREDICTION_H
