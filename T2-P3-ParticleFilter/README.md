# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

#### Submission
All you will submit is your completed version of `particle_filter.cpp`, which is located in the `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time.)

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria

1. **Accuracy**: particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: particle filter should complete execution within the time of 100 seconds.


[//]: # (Image References)

[flowchart]: ./report_images/flowchart.png "flowchart"
[filter_pass]: ./report_images/particle_filter_pass.png "filter_pass"

# Project Report

## Project Overview
In this Particle Filter project, a two-dimensional particel filter was used to localize a vehicle position based on GPS data and sensor observations (LIDAR) of landmarks around the vehicle. Its initial position was estimated by generating about 100 particles randomly around the current GSP position data within its position standard deviations. Then, vehicle motions (velocity and yaw/turn rate) were applied to all particles and sensor observations of landmarks were transformed into the global coordinates. These transformed sensor observations were associated with landmarks on the map, and then weights of particles were updated by calculating the likelihood that a given particle made those transformed sensor observations of landmarks around the vehicle. These updated particles, finally, got re-sampled based on their magnitude of weights, guiding the particle filter to estimate the position of the vehicle more accurately.

![Particle Filter Algorithm Flowchart][flowchart]


## Results

The particle filter was run in a given simulation environment and passed the acceptance criteria. During the entire simulation test, errors for x-position, y-position, and yaw of the vehicle stayed at around 0.110, 0.110, and 0.004, respectively.

![Particle Filter Pass][filter_pass]
