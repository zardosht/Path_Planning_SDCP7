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


## Main Planning Loop




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
The `update(sensor_fusion, egocar, previous_path)` method takes the `sensor_fusion` array, the AV localizatin data, and the array of remaining points form previous planned trajectory:  

* The car localization data, contained in the `egocar` instance, is an array giving car position in (x, y) and (s, d) coordinates, the heading, and the speed: `[x, y, s, d, yaw, speed]`. 
* The previous path data is an instance of `Trajectory` struct containing the x and y positions of the previous path and the `end_s` and `end_d` values of the last point of the previous path. 
* The `sensor_fustion` contains tracking data (position in x,y and Frenet coordinates and the velocity in x and y direction) for all other cars in the current scene of simulator: `[x, y, vx, vy, s, d]`. 

For each car around our AV, the `update()` method calcuates the speed of the car and its projected `s` postion at the end of current trajectory (previous path). Comparing this value with the AV's `end_s` value at the end of current trajectory gives us the predicted distance of the other car to the AV. Using this distance value we also calculate if the lane is blocked. 

The lane blocking rule differentiates between the cars in the same lane as AV and the cars in the other lanes: 
* If the car is in the same lane, we only consider the positive front distance. If this distance is less that `TOO_CLOSE_GAP = 30` meters, then the AV's lane is blocked (e.g. we have to either slow down or change lane). 
* If the car is in other lanes, the decision if the lane is blocked is used for lane changes. In this the distance behind is alos considered by compairing the distance to two threshold values `LANE_CHANGE_GAP_REAR` and `LANE_CHANGE_GAP_FRONT`. The thresholds are adopted based on the difference between the speed of ego vehicle and the other car's speed: for example if `delta_v` is positive (i.e. our car is driving faster than the other car) then the front gap is increased and the (absolute value of) rear gap is decreased

```cpp
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
```

For each lane, the prediction also updates the speed and distance to the front vehicle. This information is used for finding the beste behavior. For example when it is possible to change lanes in both left and right directeion, the `front_dist` is used to choose the lane with larger distance in front. The `front_v` is either set to the `MAX_SPEED` or to the speed of the front vehicle if the distance is less than `TOO_CLOSE_GAP`.  


## Behavior Planning

The behavior planning uses the information from prediction to decide which behavior is the best. There are three behaviors: 

* `KeepLane`: stay in the same lane, adapt speed to the front vehicle or drive at `MAX_SPEED`
* `ChangeLaneLeft`: change lane to left, adapt speed to the front vehicle or drive at `MAX_SPEED`
* `ChangeLaneRight`: change lane to right, adapt speed to the front vehicle or drive at `MAX_SPEED`

The lane rule defines the `d` value of the vehicle when generating the trajectory. The target `s` for trajectory is always the end of current trajectory (previous path). 

At each planning cycle the `BehaviorPlanner` class assigns each behavior a cost value calculated as the sum of different cost functions. The behavior with the lowest total cost is chosen as best behavior. The `best_behavior` is returned unless overriden (see below). Following cost functions are used to determine the best behavior given information from prediction: 

* `speed_cost()`: The behavior whose target lane has a higher `front_v` will get a lower `speed_cost`. The cost is determined based on the relative value `front_v / MAX_SPEED` for the target lane of the behavior. If the lane is blocked the speed cost is highest (by adding a constant cost of 100 to it). 
* `distance_cost()`: The behavior whose target lane has a larger front distance `front_dist` will get a lower `distance_cost`. The cost is determined as the inverse of the front distance. If the lane is blocked the cost is highest (by adding a constant cost of 100 to it). 
* `lane_change_cost()`: For the `KeepLane` behavior the cost is 0. For change lane behaviors (left and right) if the lane is blocked the cost is highest (100), otherwise the cost is zero. 
* `transition_cost()`: This cost function helps in smoothing the sequence of behaviors and prevents oscilating too fast between them. This cost prevents immediate chnage of behavior for lane changes in the opposite direction. Also, this cost favors a `KeepLane` behavior after lane chnages. 

The `BehaviorPlanner` sometimes ovverrides the `best_bahavior`. Following rules are used of overriding the best behavior: 

* Right at the beginning of a driving session, the `KeepLane` behavior is returned until the speed of the car reachs 10 m/s. (This rule is set based on observing how simulator starts a driving session).
* 
* 
 





## Trajectory Generation 



