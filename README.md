# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

[//]: # (Image References)
[image1]: ./examples/car1.png
[image2]: ./examples/car2.png

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Reflection
I started off with the provided code from the seed project and followed the instructions of Aaron Brown and David Silver for Trajectory generation. The code could be modularized and made more object oriented but for this project I stuck to having everything in the main.cpp.

The code is divided into three sections:

### Prediction [line 288 to line 377](./src/main.cpp#L288)

The Prediction section deals with the telemetry and sensor fusion data. This helps us understand the environment (other vehicles) around us.  There are multiple aspects we calculate which will be used later for behaviour planning. 

- Is there a car ahead of us blocking the traffic?
- Is there a car to the left/right lane ahead of our car, making lane change safe? Here we use 30 meters from our current car position.  
- Is there a car to the left/right lane behind our car, making lane change safe?  Here we use 5 meters from our current car position.  
- Is there a speeding car from behind in the left/right lane?
- What is the average speed of the cars in the left/right/current lane? Here we only consider cars 75 meters ahead of our current car position.

Before calculating these we predict where each car's position will be at the end of the last planned trajectory.
One thing to improve on would be to predict the lane the car would be. We do face occasional collision if the car performs a lane change to our car's lane. Need pointers on how to do it.

### Behaviour [line 381 to line 466](./src/main.cpp#L381)

The Behaviour section decides what course of action to take. 
- Do we increase/decrease the speed?
- Do we need to change lanes?

The Prediction section helps us understand the environment we are in. We analyze if we need to change lanes and if we do then we understand if it is safe to make a left/right lane change. We use the metrics calculated from the sensor fusion data earlier to make these decisions. 

The decision points are summarized below:
- Is there a car ahead of us?
	- If yes, can we change lanes left or right?
		- If we can change to both lanes then we decide the lane based on the lane average speed.
		- If change to only one lane is possible then do that.
		- If no lane change is possible then reduce the speed of the car.		
- If not, are we on the center lane? 
	- If yes, then continue on the same lane and accelerate until we hit the speed limit.
	- If not, then change to center lane.

Based on these the car is able to handle situations where the car ahead is braking, make intelligent decisions on which lane to change. The car also positions itself in the center lane when possible as being in the center lane gives more options for optimized path generation. 
Instead of the decision flow, we could create multiple cost functions and add weights to it and arrive at the final behaviour. Will attempt this once the code is made object oriented.

### Trajectory Generation [line 470 to line 584](./src/main.cpp#L470)

Trajectory Generation section calculates the trajectory based on the speed and lane output from the Behavior section, the car coordinates and the past path points. We use Spline library to do the polynomial fits as it easy to use and it is guaranteed to use all the points. Followed the lecture from Aaron Brown and David Silver for Trajectory generation.

The simulator provides the previous path for every iteration. There trajectory is generated starting with the last two points of the previous trajectory.  If we dont have previous trajectory then we start with the current position, heading and velocity. We then add cars current location and the points 30 m and 60 m ahead and in the target lane.  

These points are then fed to the spline for trajectory generation. To make the calculations easier, the coordinates are transformed to local car coordinates before being fed to spline. We also use all the points from the previous path that was not traversed by the car in the new trajectory to ensure continuity.

### Conclusion
The resulting path planner works well for this highway driving. It satifies the conditions in the rubic points.
It has managed to go incident free for over 30 miles multiple times with the maximum of 52 miles. It also manages a average speed of 48.71 MPH during a run of over 30 miles. Images below. 

![alt text][image1]

![alt text][image2]

There are occasional collision that happens when some other car changes lane to ego car lane. The prediction section needs to get better as it only predicts the position assuming it doesnt change lane. 

### Basic Build Instructions

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

