#include <math.h>
#include <iostream>
#include <math.h>

#include "prediction.h"

using std::cout;
using std::endl;
using std::min;
using std::max;


Prediction::Prediction() 
{
    init_lanes();
}

Prediction::~Prediction() { }


void Prediction::update(vector<vector<double>>& sensor_fusion, Vehicle& egocar, Trajectory& prev_path)
{
    int prev_path_size = prev_path.size();
    int ego_lane = egocar.get_lane();
    double ego_s = egocar.s;
    if (prev_path_size > 2) {
        ego_s = prev_path.end_s;
    }  

    reset_lanes();

    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
        if (sensor_fusion[i][6] < 0) continue;

        Vehicle car((int)sensor_fusion[i][0],  //id
                    sensor_fusion[i][1],  //x
                    sensor_fusion[i][2],  //y
                    sensor_fusion[i][3],  //vx
                    sensor_fusion[i][4],  //vy
                    sensor_fusion[i][5],  //s
                    sensor_fusion[i][6]); //d

        // projection of s value of the car in the future 
        // (i.e. when we would have passed through all the 
        //  points in previous_path)
        int car_lane = car.get_lane();
        double car_speed = sqrt(car.vx*car.vx + car.vy*car.vy);  // m/s
        car.s += (double)prev_path_size * TIMESTEP * car_speed; 
  
        // distance from ego car to the car in front of us
        double dist = car.s - ego_s;
        Lane& lane = lanes[car_lane];

        
        if (lane.id == ego_lane) {
            if(dist >= 0 && dist < TOO_CLOSE_GAP) {
                lane.blocked = true;
            }
        } else {
            double delta_v = egocar.speed - car_speed;
            double adapted_lcgap_f = LANE_CHANGE_GAP_FRONT + delta_v;
            double adapted_lcgap_r = min(LANE_CHANGE_GAP_REAR + delta_v, -10.0);
            if (adapted_lcgap_r < dist && dist < adapted_lcgap_f) {
                lane.blocked = true;
            }
        }
        
        if(dist >= 0) {
            lane.front_dist = min(lane.front_dist, dist);
            if (dist < TOO_CLOSE_GAP) {
                lane.front_v = min(lane.front_v, car_speed);
            } else {
                lane.front_v = min(lane.front_v, MAX_SPEED);
            }
        }
      
    }

    bool all_bloked = true;
    for (int i = 0; i < NUM_LANES; i++) {
        all_bloked &= lanes[i].blocked;
    }
    all_lanes_blocked = all_bloked;

}

void Prediction::init_lanes() 
{
    
    for (int i = 0; i < NUM_LANES; i++) 
    {
        Lane lane;
        lane.id = i;
        lane.blocked = false;
        lane.front_dist = PREDICTION_HIROZON;
        lane.front_v = MAX_SPEED;
        lanes.push_back(lane);
    }
}

void Prediction::reset_lanes() 
{
    all_lanes_blocked = false;
    for (int i = 0; i < NUM_LANES; i++) 
    {
        lanes[i].blocked = false;
        lanes[i].front_dist = PREDICTION_HIROZON;
        lanes[i].front_v = MAX_SPEED;
    }
}



std::ostream& operator<<(std::ostream &strm, const Prediction &pred) {
    for (int i = 0; i < NUM_LANES; i++) 
    {
        const Lane& lane = pred.lanes[i];
        strm << "Lane " << i 
                        << ": blocked=" << lane.blocked 
                        << ", front_dist=" << lane.front_dist 
                        << ", front_v=" << lane.front_v << endl;
    }
    strm << "---- ALL_BLOCKED=" << pred.all_lanes_blocked << endl;
    return strm;
}
