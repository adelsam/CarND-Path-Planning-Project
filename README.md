# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Writeup
I chose to implement a deterministic path planner optimized for the situations encountered on this straightforward
three lane highway.  Since there are three lanes, with no on-ramps or off-ramps, the car only has to deal with
a limited number of scenarios, so it is simpler to handle them explicitly.

### Base Behavior
When the car isn't doing anything else, it just needs to keep its lane.  This implementation was mostly taken
from Aaron's project walk-through, with some tuning for performance.  The next three waypoints from the map
are converted from global cartesian coordinates to frenet coordinates relative to the car, based on the intended
lane, and then use to create a spline: (line 352)
```C++
for (int i = 1; i <= 3; i++) {
  vector<double> pos = getXY(car_s + 30 * i, (2 + 4 * lane), map_waypoints_s, map_waypoints_x,
                             map_waypoints_y);
  ptsx.push_back(pos[0]);
  ptsy.push_back(pos[1]);
}
```
...
```
tk::spline s;
s.set_points(ptsx, ptsy);
```
The [tk spline library][http://kluge.in-chemnitz.de/opensource/spline/] makes the generation of the spline really
easy.

Also, if the car cannot change lanes, it must slow down to avoid cars in front of it.  I re-use the output from
sensor fusion to check for a car in the current lane and reduce the reference velocity if required (line 315).

### Decisions
The options available to the car depend on the current lane.
## Left Lane
In the left lane (lane = 0), the car can go straight or change lanes to the right.  I was able to get acceptable
results comparing only the distance to the car ahead in the left lane versus the center lane (line 293).  If the
closest car ahead is further out in the center lane versus the left lane, then we want to change lanes.  In order
to make sure it is safe to do so, I also check that the closest car behind is at least 3 m behind the projected
frenet s distance for the ego car.  If the car ahead in the left lane is more than 30m ahead, then there's no need
to change lanes.
```
  if (ahead_status[lane][2] < 30) {
    if (lane == 0 && ahead_status[1][2] > ahead_status[0][2] &&
        behind_status[1][2] < -3) {
      lane = 1;
    ...
```
## Center Lane
The center lane is a bit more complicated, since the car can choose to go straight, change lanes to the left or
change lanes to the right.  The logic is similar to the left lane, but I compare the distance of the car(s) ahead
of the ego car in all three lanes and choose the lane with the most space.  If there is a better lane, I check
for space in the lane by looking at the cars behind using the same buffer of 3 m for both potential lane changes.

## Right Lane
Like the Left lane, there are only two choices.  Decision made to maximize the distance ahead of the car.

## Caveats
To prevent the car from quickly crossing all three lanes, I introduced a cool-off period of 5 cycles (1 second)
after a lane change initiated from the Left- or Right-most lane.  This forces the car to drive in the center lane
for at least 1 s prior to initiating a second lane change.

### Simulator.
Code runs with the Udacity Term3 Simulator [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
Safely navigate around a virtual highway with localization and sensor fusion data.  Pass slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

