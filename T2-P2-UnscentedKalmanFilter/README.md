# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The objective of this project is obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


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

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Generating Additional Data

This is optional.

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

--------------------------------
[//]: # (Image References)

[NIS]: ./report_images/nis.jpg "NIS"
[Chi_Sqr_Dist]: ./report_images/Chi_square_distribution.jpg "Chi_Sqr_Dist"
[NIS_Laser]: ./report_images/NIS_Laser.png "NIS_Laser"
[NIS_Radar]: ./report_images/NIS_Radar.png "NIS_Radar"
[px_est_vs_gt]: ./report_images/px_est_vs_gt.png "px_est_vs_gt"
[py_est_vs_gt]: ./report_images/py_est_vs_gt.png "py_est_vs_gt"
[vx_est_vs_gt]: ./report_images/vx_est_vs_gt.png "vx_est_vs_gt"
[vy_est_vs_gt]: ./report_images/vy_est_vs_gt.png "vy_est_vs_gt"
[yaw_est_vs_gt]: ./report_images/yaw_est_vs_gt.png "yaw_est_vs_gt"
[EKF_Result_Dataset1]: ./report_images/EKF_Result_Dataset1.png "EKF_Result_Dataset1"
[EKF_Result_Dataset2]: ./report_images/EKF_Result_Dataset2.png "EKF_Result_Dataset2"
[UKF_Result_Dataset1]: ./report_images/UKF_Result_Dataset1.png "UKF_Result_Dataset1"
[UKF_Result_Dataset2]: ./report_images/UKF_Result_Dataset2.png "UKF_Result_Dataset2"


# Unscented Kalman Filter Project Report
## Tune logitudinal and yaw acceleration noise parameters
The longitudinal and yaw acceleration noise standard deviation values were approximated by analyzing the overall vehicle motions on the simulator; how much the vehicle accelerated in both longitudinal and angular. Then, since a big approximation on logitudinal acceleration and yaw acceleration was made by assuming a white process noise, it needs to be checked if noise parameters are correclty tuned. So, the consistency check, Normalized Innovation Squared (NIS), was run on the filter:

![NIS equation][NIS]

The NIS value follows the Chi-squared distribution

![Chi-squared Table][Chi_Sqr_Dist]

The dimension of Radar measurement space is three and the table shows that 5% of Radar NIS need to be higher than 7.815. As same for the LIDAR measurement, which has the two-dimensional measurement space, 5% of LIDAR NIS need to be higher than 5.991.

The longitudinal and yaw acceleration standard deviation values were fine-tuned based on the NIS statistics against Chi-Squared distribution.

Finally, the longitudinal acceleration and yaw acceleration standard deviations were chosen as follow:

And, Radar and LIDAR NIS plots were made as follows:

### NIS: Radar Measurement
![Radar NIS plot][NIS_Radar]
### NIS: Laser Measurement
![LIDAR NIS plot][NIS_Laser]


## Estimates vs. Ground Truth
Estimates by UKF vs. Ground Truth plots for position, velocity, and yaw angle are made to visualize the performance of UKF.

### x-position: estimates vs ground truth
![px][px_est_vs_gt]
### y-position: estimates vs ground truth
![py][py_est_vs_gt]
### x-velocity: estimates vs ground truth
![vx][vx_est_vs_gt]
### y-velocity: estimates vs ground truth
![vy][vy_est_vs_gt]
### yaw angle: estimates vs ground truth
![yaw][yaw_est_vs_gt]

Position estimates were the same to the ground truth values. Velocity and yaw angle estimates were bit noisy, but they were also very close to the ground truth values.


## Unscented Kalman Filter vs. Extended Kalman Filter

### EKF Dataset1 Result
![EKF Dataset1][EKF_Result_Dataset1]
### EKF Dataset2 Result
![EKF Dataset2][EKF_Result_Dataset2]
### UKF Dataset1 Result
![UKF Dataset1][UKF_Result_Dataset1]
### UKF Dataset2 Result
![UKF Dataset2][UKF_Result_Dataset2]

Comparing the UKF results with EKF results from the previous project, it was found that UKF yielded much better performance, especially on velocity estimates. For the EKF case, it linearized the nonlinear system at every point of interest and it can only achieve the first-order accuracy (Taylor Series Expansion). In contrast, UKF carefully chose sample points, mapped to true nonlinear system, and was able to captrue the posterior mean and covariance accurately to the third-order (Taylor Series Expansion). Moreover, the computational complexity of the UKF was the same order to that of the EKF. Therefore, UKF can achieve a better level of accuracy than the EKF at a comparable level of complexity. For more information, here is a link to a paper:
[The Unscented Kalman Filter for Nonlinear Estimation](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf)
