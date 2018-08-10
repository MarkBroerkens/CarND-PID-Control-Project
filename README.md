[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# The Project
In this project a PID controller that is used to  maneuver the vehicle around the track in the simulator.

The simulator provides the cross track error (CTE) and the velocity (mph) and current steering angle. This data is used to compute the appropriate steering angle and throttle.

[//]: # (Image References)

[image1]: ./images/simulator.png "Simaluator"

## Rubric Discussion Points

- *The effect each of the P, I, D components of the steering PID controller of the steering.*
The steering PID controller analyzes the cross track error (CTE) provided by the simulator and calculates the steering angle that tries to bring the car back to the center line (minimial magnitude of CTE).

The P, or "proportional", component had the most directly observable effect on the car's behavior. It causes the car to steer proportionally (and opposite) to the car's distance from the lane center (which is the CTE) - if the car is far to the right it steers hard to the left, if it's slightly to the left it steers slightly to the right.

The D, or "differential", component counteracts the P component's tendency to overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly.

The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift.

- *The effect each of the P, I, D components of the throttle PID controller of the steering.*
An additional PID controller was used to control the speed of the car via the throttle. The target speed is reduced if there in case of big CTE and high steering angle.

The P, or "proportional", component had the most directly observable effect on the speed of the car's behavior. High values of P resulted in quick reactions on changes of the target speed which even allowed the car to brake (negative throttle).

The D, or "differential", component counteracts the P component's tendency to overshoot the target speed.

The I, or "integral", was set to 0 since the target speed is constantly changing and I observed a tendency to not breaking fast enough.

- *Describe how the final hyperparameters were chosen.*

Hyperparameters were tuned manually at first. As an inital configuration I used the values from the lesson (p=0,2, i=0,004, d=3). These parameters worked pretty well for constant throttle of 0.3.
The I tried to increase the speed of the car by using higher constant throttle values and figured out that the car frequently left the track. In order to overcome this issue added a PID controller that minimizes the speed of the car in case of high cross track error and high steering angle.

With this setup and some additional manual experiments with regards to the Hyperparameters of the PID controllers I ended up with the following result. 

TODO: fill in the parameters
[![Initial Hyperparameters](https://img.youtube.com/vi/ihwbbllSh9A/0.jpg)](https://www.youtube.com/watch?v=ihwbbllSh9A)


I then implemented Twiddle. 
TODO

---

# Dependencies

* cmake >= 3.5
* All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
* Linux: make is installed by default on most Linux distros
* Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
* Linux: gcc / g++ is installed by default on most Linux distros
* Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
* Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* Run either `./install-mac.sh` or `./install-ubuntu.sh`.
* If you install from source, checkout to commit `e94b6e1`, i.e.
```
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
```
Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

# Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

# Code Style
I tried to stick to the [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
In order to check the guidelines I installed cpplint using 
`pip install cpplint`





