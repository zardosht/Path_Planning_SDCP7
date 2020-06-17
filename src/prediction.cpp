#include <math.h>
#include<iostream>

#include "prediction.h"

using std::cout;
using std::endl;


void Prediction::update(vector<vector<double>>& sensor_fusion, Vehicle& egocar, Trajectory& prev_path)
{
    int prev_path_size = prev_path.size();
    int ego_lane = egocar.get_lane();
    double ego_s = egocar.s;
    if (prev_path_size > 2) {
        ego_s = prev_path.end_s;
    }  

    too_close = false;
    dist_front = LARGE_NUMBER;
    car_left = false;
    dist_front_left = (ego_lane == 0)? -1.0 : LARGE_NUMBER;
    car_right = false;
    dist_front_right = (ego_lane == 2)? -1.0 : LARGE_NUMBER;

    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
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
        double car_speed = sqrt(car.vx*car.vx + car.vy*car.vy);
        car.s += (double)prev_path_size * 0.02 * car_speed;
        int car_lane = car.get_lane();
  
        // distance from ego car to the car in front of us
        double dist = car.s - ego_s;
        int lanediff = car_lane - ego_lane; 
        if (lanediff == 0) {
            // other car in our lane.
            too_close |= dist > 0 && dist < FRONT_GAP;
            if (dist > 0 && dist_front_right > dist) {
                dist_front = dist;
            }
        } else if(lanediff == -1) {
            // other car in left lane
            car_left |= LANE_CHANGE_GAP_REAR < dist && dist < LANE_CHANGE_GAP_FRONT;
            if(dist > 0 && dist_front_left > dist) {
                dist_front_left = dist;
            }
        } else if (lanediff == 1) {
            // other car in right lane
            car_right |= LANE_CHANGE_GAP_REAR < dist && dist < LANE_CHANGE_GAP_FRONT;
            if(dist > 0 && dist_front_right > dist) {
                dist_front_right = dist;
            }
        }
    }
}



