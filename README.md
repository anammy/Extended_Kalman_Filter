# Extended Kalman Filter Project

The objective of this project is to implement an extended kalman filter to estimate the state of a moving vehicle with noisy lidar and radar measurements. The performance of the kalman filter was evaluated by calculating the root mean squared error, RMSE, over the track and ensuring it is lower than the tolerance.

[//]: # (Image References)
[image1]: ./Pictures/Results-DS1
[image2]: ./Pictures/Results-DS2

This project requires the following files to run:
* Utiltiies: Run script 'install-ubuntu.sh' for Linux, 'install-mac.sh' for Mac, 'install-ubuntu.sh' in Ubuntu Bash 16.04 for Windows.
  * cmake: 3.5
  * make: 4.1 (Linux and Mac), 3.81 (Windows)
  * gcc/g++: 5.4
  * [uWebSocketIO](https://github.com/uNetworking/uWebSockets)
* Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

The starter code from [Udacity](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) was used. The code in the following files were completed to implement the EKF: src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h. The program main.cpp uses uWebSocketIO to communicate with the simulator.

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

## Important Dependencies

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
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## EKF Results

The RMSE for the vehicle position (px, py) and velocity (vx, vy) calculated over track for dataset 1 and 2 satisfied the following tolerance:
* px and py <= 0.11
* vx and vy <= 0.52

The following is a snapshot of the final simulator result for datasets 1 and 2:

![Result-Dataset1][image1]

![Result-Dataset2][image2]
