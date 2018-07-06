# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

# PID Controller Project Report

## PID Controller

### "P" - Proportional Gain
Proportional gain affects how fast a current state reaches to a reference state. In this project, this P gain controls the steering value to bring the current position of a vehicle to the referent poistion as quickly as possible.

### "I" - Integral Gain
Integral gain helps to get rid of the steady-state error. For PD controller, there is a tendency that a system state stabilizes at a state, which is off from the reference state. For example in this project, when a vehicle with a PD controller comes to the straight track, the PD controller would bring the vehicle position a few centimeter off from the reference position and would stabilize there. So, this Integral gain helps to reduce the steady-state offset.

### "D" - Differential Gain
Differential gain manages to control the 'overshoot'. A controller with only Proportional gain can bring the current state to the reference state, but it tends to pass after the reference state rather than stabilizes at the reference state. So, it needs to oscillates quite a time to stabilize at the reference state (It is like a slightly-damped spring-mass system). This is called 'overshoot' and Differential gain can minimize this problem.

## PID Tuning
For this project, PID gains were tuned manually. First, Proportional gain was tuned so that the vehicle can be brought to the reference position at a reasonable rate. Then, Differential gain was tuned to minimize the 'overshoot' problem. Once PD gain-tuning resulted a reasonable vehicle behavior, then these PD gains were further fine-tuned (down to the third decimal point) for a better vehicle performance.

For the Integral gain, it was not that critical for this project as there was enough space for a vehicle around the reference position. It did not require a precision control, so only PD controller was enough to control the vehicle within the track. However, for a better vehicle performance, Integral gain was tuned. Also, integral error was saturated at the set minimum and maximum values (-10 and 10) to prevent the integral error value explosion.

The final PID gains were [P = 0.158; I = 0.014; D = 1.785]