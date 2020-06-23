#### Udacity Self-driving Car Nanodegree
# Project 7: Motion Planning (Highway Driving)

#### Motion Planning

The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided by the [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). A map file is given with a sparse map list of waypoints around the highway. The solution should meet the following criteria: 

* The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
* The car should avoid hitting other cars at all cost. 
* The car should drive inside of the marked road lanes at all times, unless going from one lane to another. 
* The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
* The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The project code is organized in different modules: 


* **main.cpp**: Contains the main program loop. It receives sensor fusion, localization data for the AV, and remaining portion of previous trajectory from the simulator over WebSockets and returns back the calculated new trajectory. 
* **Map:** Contains a sparse map list of waypoints around the highway as well as some helper methods for converting coordinate and angle values. 
* **Prediction:** Takes the sensor fusion data from simulator and generates predictions about state of the road such as lane distance to front cars and lane blocked state. 
* **Behavior Planning:** Given the predictions, the Behavior Planner finds a best behavior by calculating the cost of each behavior. A behavior defines if the car should keep or change the lane and the target velocity for the AV. 
* **Trajectory Generation:** Generates a trajectory for the best behavior respecting the collision, speed, acceleration, and jerk constraints. 


## Map
The `Map` struct loads the map data from `../data/highway_map.csv` file. Each waypoint in the list contains  `[x,y,s,dx,dy]` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the Frenet s value, distance along the road, goes from 0 to 6945.554.

The method `get_frenet()` and `get_xy()` can be used to convert from `(x, y)` coordinates to Frenet `(s, d)` coordinates and vice versa. 


## Prediction
The `Prediction` struct takes the sensor fusion input and localization data of the AV from the simulator and calculated the current state of the highway. The highway consists of three lane. For each lane the `Prediction` struct maintains the state in an instance of `Lane` struct. The `Lane` struct contains id of the lane, the distance to the front vehicle in the lane and if the lane is blocked. 

```cpp
struct Lane 
{
    int id;
    bool blocked;
    double front_dist;
    double front_v;
};
```
The `update(sensor_fusion, egocar, previous_path)` method takes the `sensor_fusion` array, the AV localization data, and the array of remaining points form previous planned trajectory:  

* The car localization data, contained in the `egocar` instance, is an array giving car position in (x, y) and (s, d) coordinates, the heading, and the speed: `[x, y, s, d, yaw, speed]`. 
* The previous path data is an instance of `Trajectory` struct containing the x and y positions of the previous path and the `end_s` and `end_d` values of the last point of the previous path. 
* The `sensor_fusion` contains tracking data (position in x,y and Frenet coordinates and the velocity in x and y direction) for all other cars in the current scene of simulator: `[x, y, vx, vy, s, d]`. 

For each car around our AV, the `update()` method calculates the speed of the car and its projected `s` position at the end of current trajectory (previous path). Comparing this value with the AV's `end_s` value at the end of current trajectory gives us the predicted distance of the other car to the AV. Using this distance value we also calculate if the lane is blocked. 

The lane blocking rule differentiates between the cars in the same lane as AV and the cars in the other lanes: 
* If the car is in the same lane, we only consider the positive front distance. If this distance is less that `TOO_CLOSE_GAP = 30` meters, then the AV's lane is blocked (e.g. we have to either slow down or change lane). 
* If the car is in other lanes, the decision if the lane is blocked is used for lane changes. In this the distance behind is also considered by comparing the distance to two threshold values `LANE_CHANGE_GAP_REAR` and `LANE_CHANGE_GAP_FRONT`. The thresholds are adopted based on the difference between the speed of ego vehicle and the other car's speed: for example if `delta_v` is positive (i.e. our car is driving faster than the other car) then the front gap is increased and the (absolute value of) rear gap is decreased

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

For each lane, the prediction also updates the speed and distance to the front vehicle. This information is used for finding the best behavior. For example when it is possible to change lanes in both left and right direction, the `front_dist` is used to choose the lane with larger distance in front. The `front_v` is either set to the `MAX_SPEED` or to the speed of the front vehicle if the distance is less than `TOO_CLOSE_GAP`.  


## Behavior Planning

The behavior planning uses the information from prediction to decide which behavior is the best. There are three behaviors: 

* `KeepLane`: stay in the same lane, adapt speed to the front vehicle or drive at maximum speed
* `ChangeLaneLeft`: change lane to left, adapt speed to the front vehicle or drive at maximum speed
* `ChangeLaneRight`: change lane to right, adapt speed to the front vehicle or drive at maximum speed

The lane rule defines the `d` value of the vehicle when generating the trajectory. The target `s` for trajectory is always the end of current trajectory (previous path). 

At each planning cycle the `BehaviorPlanner` class assigns each behavior a cost value calculated as the sum of different cost functions. The behavior with the lowest total cost is chosen as best behavior. The `best_behavior` is returned unless overridden (see below). Following cost functions are used to determine the best behavior given information from prediction: 

* `speed_cost()`: The behavior whose target lane has a higher `front_v` will get a lower `speed_cost`. The cost is determined based on the relative value `front_v / MAX_SPEED` for the target lane of the behavior. If the lane is blocked the speed cost is highest (by adding a constant cost of 100 to it). 
* `distance_cost()`: The behavior whose target lane has a larger front distance `front_dist` will get a lower `distance_cost`. The cost is determined as the inverse of the front distance. If the lane is blocked the cost is highest (by adding a constant cost of 100 to it). 
* `lane_change_cost()`: For the `KeepLane` behavior the cost is 0. For change lane behaviors (left and right) if the lane is blocked the cost is highest (100), otherwise the cost is zero. 
* `transition_cost()`: This cost function helps in smoothing the sequence of behaviors and prevents oscillating too fast between them. This cost prevents immediate change of behavior for lane changes in the opposite direction. Also, this cost favors a `KeepLane` behavior after lane changes. 

The `BehaviorPlanner` sometimes overrides the `best_behavior`. Following rules are used of overriding the best behavior: 

* Right at the beginning of a driving session, the `KeepLane` behavior is returned with maximum target speed, until the speed of the car reaches 10 m/s. (This rule is set based on observing how simulator starts a driving session).
* If a lane change behavior is started, it is continued till the lane change is complete.
* After a lane change, we return `KeepLane` for `KEEP_LANE_AFTER_LANE_CHANGE = 50` cycles. This is roughly about 2 seconds (see Trajectory Generation section about discussion of time). This rule is added to prevent too hasty lane changes (which could violate lateral jerk constraint).  

The combination of these cost functions and rules leads to acceptable driving behavior as can be seen in examples below: 

* Safe lane change: 
  
![Safe lane change](./readme_files/safe_lane_change.gif)

* Slowing down if lane change is not possible: the AV adapts its speed to the front vehicle if it cannot change lane. 

![Slowing down if safe lane change not possible](./readme_files/slow_down_before_lane_change.gif)

* Predictive lane change: Change to the lane with the largest free distance ahead.

![Predictive lane change](./readme_files/predictive_lane_change.gif)

* Change lane to the higher speed and double lane change: 

![Double lane change](./readme_files/double_lane_change.gif)





## Trajectory Generation 

I used the same method for generating trajectories using spline interpolation as described in the [Q&A video](https://youtu.be/7sI3VHFPP0w). This approach uses an very easy to use implementation of spline interpolation in one single header file by http://kluge.in-chemnitz.de/opensource/spline/.

Another approach would be using a [Jerk Minimizing Quintic Polynomial](https://github.com/ChenBohan/Robotics-Path-Planning-04-Quintic-Polynomial-Solver). 

### Simulator Details
1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### Generating Trajectory

At each planning cycle a set of trajectory points are generated that are added to the end for remaining points from previous trajectory (previous path) and returned as the new trajectory. The trajectory points are interpolated using spline. The knot points for the spline are determined as followed: 

* the starting points are either the current vehicles position (if previous path is shorter that 2 points) or the end of previous path.

* the end points are determined based on target `s` and terget `d` values. For target `s` fix values of 30, 60, and 90 meters are added to the end of previous path (if it has more than two points) or the vehicles position (for example right at the beginning). The target `d` is determined by the target lane of the given behavior. 

The spline knot points are in (x,y) coordinates. The trajectory points are first generated in local car coordinates, then transformed to global (x, y) coordinates. This is done to avoid errors in spline interpolation caused by the same values for x if the spline knots are in global coordinates. 

The trajectory consists of a series of (x, y) points, each reached (consumed) by the simulator in `TIMESTEP = 0.02` seconds. The planning distance in car's local x direction is set to be `PLANNING_DISTANCE_X = 50.0` meters. The target speed for planning the trajectory is given by the behavior's target speed. The number of trajectory points to return is fixed and set to `NUM_TRAJECTORY_POINTS = 50`. The threshold `PLAN_NEW_TRAJECTORY_THRESHOLD = 50` also determines how many points from the remaining points of the previous path should be integrated into the new plan. If the number of points in previous path is less than this threshold the new trajectory points are generated. The current value of 50 for this threshold means that new points are generated at every planning cycle. That is, the AV is swifter in reacting to changes of the road state. 

The speed of AV for generating the trajectory points (`ego_v`) is determined based on the target speed for the lane, given by the behavior (`target_v`). At each planning cycle the current speed of AV is increased or decreased according to the maximum acceleration (`MAX_ACC`). The value for `MAX_ACC` is set to 7 m/s^2 (to meet the max acceleration constraint of 10 m/s^2; the value could be set nearer to 10 of course): 

```cpp
    if (target_v > ego_v) {
        //accelerate
        ego_v = min(ego_v + MAX_ACC * TIMESTEP, MAX_SPEED);
    } else {
        //decelerate
        ego_v = min(ego_v - MAX_ACC * TIMESTEP, MAX_SPEED);
    }
```


The trajectory points are distributed evenly along the planning distance. The combination of the current speed (`ego_v`), the fixed number of trajectory points, and the TIMESTEP determines the distance between each consecutive points in the trajectory: 

```cpp
    double target_x = PLANNING_DISTANCE_X; //horizon
    double target_y = spl(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double num_points = target_dist / (TIMESTEP * ego_v);
    double distance_between_points = target_x / num_points;
```



## Future Work

The current state of the project meets the requirements defined in [project rubric](https://review.udacity.com/#!/rubrics/1971/view). Following can further improvements the project:

* Adding behaviors for emergency break / emergency slow down: In the current code of simulator sometimes other cars perform dangerous lane change maneuvers with a very short distance in front of our AV. If the distance is too short, a crash is not avoidable. This could be handled by adding a behavior for emergency breaks. 
* Adding behavior for double lane change: Currently the behavior planner only has the behavior for change one lane to left and right. Also, the cost functions only watch just the immediate lane to left or right. This sometimes leads to suboptimal behavior, as the car does not change the lane to a free lane if it is not immediately in its left or right. Adding two DoubleChangeLane behaviors (for left and right) could solve this problem. The cost functions should consider the immediate lane to the left and right and the lane, that is two lanes apart from current lane. The trajectory generation also should generate a smooth trajectory with spline knots in current lane, in the middle, and in the third lane. 
* Implementing another approach for trajectory generation (e.g. JMT with quintic polynomials) and adding it as a trajectory generation strategy. 
* Combining different parameters and thresholds that influence driving behavior and performance into a class for driving profile. These values could then be set and tuned for certain driving types, such as aggressive, conservative, etc. 

