[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# The Project
In this project a PID controller that is used to  maneuver the vehicle around the track in the simulator.

The simulator provides the cross track error (CTE) and the velocity (mph) and current steering angle. This data is used to compute the appropriate steering angle and throttle.

[//]: # (Image References)

[image1]: ./images/simulator.png "Simaluator"

# Rubric Discussion Points

- *The effect each of the P, I, D components of the steering PID controller of the steering.*
The steering PID controller analyzes the cross track error (CTE) provided by the simulator and calculates the steering angle that tries to bring the car back to the center line (minimial magnitude of CTE).

The P, or "proportional", component had the most directly observable effect on the car's behavior. It causes the car to steer proportionally (and opposite) to the car's distance from the lane center (which is the CTE) - if the car is far to the right it steers hard to the left, if it's slightly to the left it steers slightly to the right. Bigger values of P result in faster reactions of the steering angle with respect to the CTE. I had to use higher values for higher speed of the car.

The D, or "differential", component counteracts the P component's tendency to overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly. Higher values of P also required higher values of D.

The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift.

- *The effect each of the P, I, D components of the throttle PID controller of the steering.*
An additional PID controller was used to control the speed of the car via the throttle. The target speed is reduced if there in case of big CTE and high steering angle.

The P, or "proportional", component had the most directly observable effect on the speed of the car's behavior. High values of P resulted in quick reactions on changes of the target speed which even allowed the car to brake (negative throttle).

The D, or "differential", component counteracts the P component's tendency to overshoot the target speed.

The I, or "integral", was set to 0 since the target speed is constantly changing and I observed a tendency to not breaking fast enough.

- *Describe how the final hyperparameters were chosen.*

Hyperparameters were tuned manually at first. As an inital configuration I used the values from the lesson (p=0,2, i=0,004, d=3). These parameters worked pretty well for constant throttle of 0.3.
The I tried to increase the speed of the car by using higher constant throttle values and figured out that the car frequently left the track. In order to overcome this issue added a PID controller that minimizes the speed of the car in case of high cross track error and high steering angle.

**Manual Experiments**

With this setup and some additional manual experiments with regards to the Hyperparameters of the PID controllers I ended up with the following result. 

steering PID parameters (p=0.14, i=0.00027, d=6).
throttle PID parameters (p=0.1, i=0.0, d=1).

[![Initial Hyperparameters](https://img.youtube.com/vi/ihwbbllSh9A/0.jpg)](https://www.youtube.com/watch?v=ihwbbllSh9A)

**Twiddle**
I then implemented the Twiddle algorithm for automated optimization of the Hyperparameters of both controllers. This step is quite time consuming since the car needs to pass all critical locations of the track for each combination of parameters. I ran the car for more than a lap in order to be sure that the critical curve after the bridge is handle correctly. 

Using a max throttle of 0.3 a few iterations of the twiddle algorithm resulted in the following hyperparameters. 

steering PID parameters (p=0.135, i=0.0015, d=8.51561).
throttle PID parameters (p=0.1, i=0.0, d=1).

[![Twiddled Hyperparameters](https://img.youtube.com/vi/8nSa8rzO8rw/0.jpg)](https://www.youtube.com/watch?v=8nSa8rzO8rw)

# Extra Material
The following links to extra material on PID controllers are from the Udacity review. Thanks.

* [A Conceptual Breakdown of PID Controllers](https://hackernoon.com/a-conceptual-breakdown-of-pid-controllers-9fa072a140a5)
* [Automating tuning of PID Controllers](https://www.researchgate.net/post/Automating_tuning_of_PID_Controllers)
* [PID Explained for Process Engineers: Part 2 - Tuning Coefficients](https://www.aiche.org/resources/publications/cep/2016/february/pid-explained-process-engineers-part-2-tuning-coefficients)
* [PID Controller](https://en.wikipedia.org/wiki/PID_controller)
* [Automatic Controller Tuning using Relay-based Model Identification](http://portal.research.lu.se/ws/files/33100749/ThesisJosefinBerner.pdf)
* [On Automation of the PID Tuning Procedure](https://pdfs.semanticscholar.org/4bdd/25ea2d463ed7626eb37d18be1687b3a4391e.pdf)
* [PID Control for self-driving](https://medium.com/@cacheop/pid-control-for-self-driving-1128b42ab2)

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





