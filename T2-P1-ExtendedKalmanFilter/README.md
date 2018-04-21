# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project, a kalman filter is utilized to estimate the state of a moving object of interest with noisy LIDAR and Radar measurements. The objective of this project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make` 
3. Run it: `./ExtendedKF `

## Editor Settings

Editor configuration files were purposfully kept out of this repo in order to
keep it as simple and environment agnostic as possible. However, it is recommended
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

[//]: # (Image References)

[final_result1]: ./report_images/Final_Result_Dataset1.png "final_result1"
[final_result2]: ./report_images/Final_Result_Dataset2.png "final_result2"
[lidar_only]: ./report_images/LIDAR_only_result.png "lidar_only"
[radar_only]: ./report_images/Radar_only_result.png "radar_only"

# Project Report
## Raw Measurement Data Handling
It was found that some of phi raw measurement was out of desired range from -pi to pi. Thus, in `UpdateEKF` function in `kalman_filter.cpp` file, phi raw measurement data always get monitored and predicted phi values get added or subtracted by `2*pi`.

Moreover, it was also observed that, when a vehicle crossed between second and third quadrant, vehicle estimated positions significantly deviated from the ground truth values, and RMSE value spiked. This was because the negative side of the x-axis is the transition line where an angle value sign changes between negative and postiive. For example, lets say phi_raw = -3.00 rad (third quadrant) and phi_predicted = 3.00 rad (second quadrant). Their difference is only around 0.28 rad, but if they are input to the error equation,'y = z - z_predicted' then, error_phi becomes 6. Therefore, a few lines of code was added to check whether the signs of phi raw measurement and predicted phi are different and add or subtract `2*pi` to the predicted phi to match the sign.

## Improve RMSE value
While initial x-position and y-position values were estimated by the first raw measurement values, initial x-velocity and y-velocity values
were absent. 
They were approximated by simply running the simulator with datatset1 and got the x-velocity and y-velocity values at very first few data points. 

Initial State Covariance matrix was approximated by trying large diagonal values and small diagonal values. And, it was found that small diagonal values yielded the better result.

## Sensor fusion VS. Single Sensor Type
`ExtendedKF` was modified to compare its performance by LIDAR only and by Radar only. To turn off a sensor, one of `Update` function was commented out.

1. LIDAR Sensor Only
Since LIDAR raw measurement provided the direct measurement values of px and py, RMSE values of px, py, vx, and vy were little higher than the acceptance criteria.

![LIDAR Only Result][lidar_only]

2. Radar Sensor Only
RMSE values of px, py, vx, and vy produced by `Radar-Only ExtendedKF` were much higher than the acceptance criteria. This was because of the nonlinear mapping from the Radar raw measurements value to the system state vectors. First order Taylor Series Expansion was used to linearize the nonlinearity of the measurement vector, and this was where significant gaps created. As it is shown in the picture below, the estimated points are noticeably off from the measurement points, especially at the curved path.

![Radar Only Result][radar_only]

## Final Result

`ExtendedKF` has been run against Dataset1 and RMSE values were as follow:
[px, py, vx, vy] ~ [0.09, 0.08, 0.30, 0.39]

![Final Result 1][final_result1]

For Dataset2, the initial value for vx was changed to -5 as the vehicle started toward the opposite direction to the Datatet1, and RMSE values were as follow:
[px, py, vx, vy] ~ [0.07, 0.09, 0.30, 0.42]

![Final Result 2][final_result2]

`ExtendedKF` with the sensor fusion algorithm, combining both LIDAR and Radar measurements, resulted much better performance than the one relied on a single sensor measurement. 