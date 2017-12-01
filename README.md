# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project we code and tune an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

## Results

The tuned UKF presented here achieves the following RMSE values:

| RMSE component | Project requirement | This UKF, dataset 1 | This UKF, dataset 2 |
|----------------|:-------------------:|:-------------------:|:-------------------:|
| px             | 0.09                | 0.0661              | 0.0654              |
| py             | 0.10                | 0.0899              | 0.0640              |
| vx             | 0.40                | 0.1551              | 0.2784              |
| vy             | 0.30                | 0.1669              | 0.2804              |


## Folders and Files

This project includes the following main files and folders:

* **root**:
    - CMakeLists.txt, cmakepatch.txt: To build the executable program.
    - install-mac.sh, install-ubuntu.sh: Setup the environment 
    - LICENSE, README: Documentation

* **src**:
    - main.cpp: Main program, receives inputs from the simulator and sends Root Square Mean Error values back to it through uWebSocketIO.
    - ukf.h, ukf.cpp: Define the UKF class and the functions used to estimate the vehicle's position based on sensor measurements and process 
    model.
    - tools.h, tools.cpp: Define tools to compute the RMSE, normalize angles to [-pi, +pi] and calculate the square root of matrices (used during the generation of augmented sigma points).
    - measurement_package.h: Defines the MeasurementPackage class. Each instance contains the raw sensor measurements, a timestamp when the measurement was made and the sensor used to measure these values (radar or lidar).
    - json.hpp: Functions used to exchange information with the simulator in a JSON format. Do not modify.
    - **Eigen**: The Eigen library is provided for convenience. This is used for vector and matrix computations.

* **build**:
    - UnscentedKF: The executable file for this program. See below for run instructions.
    - Makefile, CMakeCache.txt, cmake_install.cmake, **CMakeFiles**: Compile files. Leave alone.

* **plotting**:
    - nis_plots.R: R script that reads the NIS output files and builds a ggplot2 graph of NIS over time for both sensors. It also calculates 
    the proportion of NIS values for each sensor that exceed the chi-square 95th centile. This should ideally be close to 5% for a consistent 
    UKF.
    - lidar_nis.csv, radar_nis.csv: Output files containing the timesteps and NIS values for each sensor.


## Basic Build and Run Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`
5. Execute the Term2 Simulator and select "Project 1/2: EKF and UKF". You can switch between Dataset 1 and Dataset2 with the tickboxes.
    - Red circles represent LIDAR measurements (px and py)
    - Blue circles represent RADAR measurements (radius, bearing and radial velocity). The small arrows inside the circles show the measured
    bearing.
    - The green triangles represent the postion of the vehicle as estimated by the UKF.

**Note**: Kill your program and run it again before restarting the simulator, otherwise old values of the state vector and covariance matrix will be used as starting points.


## Requirements

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


**Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.**

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] -> the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <- kalman filter estimated position x
["estimate_y"] <- kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >- 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >- 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >- 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

