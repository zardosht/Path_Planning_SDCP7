#### Udacity Self-driving Car Nanodegree
# Project 7: Motion Planning (Highway Driving)

#### Motion Planning

The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided by the simulator. A map file is given with a sparse map list of waypoints around the highway. The solution should meet the following criteria: 

* The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
* The car should avoid hitting other cars at all cost. 
* The car should drive inside of the marked road lanes at all times, unless going from one lane to another. 
* The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
* The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The project code is organized in different modules: 


* **main.cpp**: Contains the main program loop. It receives sensor fustion, localization data for the AV, and remaining portion of previous trajectory from the simulator over WebSockets and returns back the calculated new trajectory. 
* **Map:** Contains a sparse map list of waypoints around the highway as well as some helper methods for converting coordinate and angle values. 
* ***Predicition:** Takes the sensor fusion data from simulator and generates predictions about state of the road such as lane distance to front cars and lane blocked state. 
* ***Behavior Planning:** Given the predictions, the Behavior Planner finds a best behavior by calculating the cost of each bahvior. A behavior defines if the car should keep or change the lane and the target velocity for the AV. 
* **Trajectory Generation:** Generates a trajectory for the best behavior respecting the collisiont, speed, acceleration, and jerk constraints. 


## Map
The `Map` struct loads the map data from `../data/highway_map.csv` file. Each waypoint in the list contains  `[x,y,s,dx,dy]` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the Frenet s value, distance along the road, goes from 0 to 6945.554.

The method `get_frenet()` and `get_xy()` can be used to convert from `(x, y)` coordiantes to Frenet `(s, d)` coordinates and vice versa. 


## Prediction
The `Prediciton` struct takes the sensor fusion input and localization data of the AV from the simulator and calculated the current state of the highway. The highway consists of three lane. For each lane the `Prediction` struct maintains the state in an instance of `Lane` struct. The `Lane` struct contains id of the lane, the distance to the front vehicle in the lane and if the lane is blocked. 

```cpp
struct Lane 
{
    int id;
    bool blocked;
    double front_dist;
    double front_v;
};
```



## Behavior Planning



## Trajectory Generation 



