# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Overview

This project implements a Model Predictive Controller to drive a simulated vehicle along the track. The controller receives the waypoints of the lane center line and sends the driving commands of steering angle and acceleration to the simulator. The below video shows the result of the car driven in the lane. The yellow line visualizes the sensor feedback on the lane and the green line is the controller forecast of driving trajectory.

![project_preview](https://github.com/qqquan/t2p5_mpc/raw/master/mpc_simulation.gif)

## Design

### The Model

A Bicycle model is used with an assupmtion of zero yaw rate, which means that the steering makes an instant change on the car. The formula is as following:

```
      x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

`x`: vehicle state of x-axis position in meters.

`y`: vehicle state of y-axis position in meters.

`v`: vehicle state of speed in meter/second.

`psi`: vehicle state of orientation in radians.

`delta`: actuation command of steering angle in radians.

`a`: actuation command of acceleration in m^2/s.

`f()`: polyfit based on the lane waypoints.

`cte`: cross track error.

`epsi`: orientation error.

`psides`: road curvature angle that the car must turns for.

`Lf`: the distance between the front of the vehicle and its center of gravity.

The model is fed as constraints into the `ipopt` optimization library.

### Hyper-parameters (N, dt, weights)

After manually tuning the hyper-parameters, the timestep length N of 25 and the elapsed duration of 0.02 is chosen for a reference speed of 50 m/s. 

This means the total MPC prediction time is 25*0.02=0.5 seconds. 

The prediction time needs to be longer than the actuation interval. Too long a prediction time, the MPC is easily affected by the higher order term of the polyfit lane and the resulted trajectory is not smooth. Too short a prediction time, the MPC would not prepare for a incoming curve.

Additionally, an extra weight of 500 is added to penalize the steering change rate. This avoids sudden steering change and results in a smoother lane-following performance. 

### Polynomial Fitting and MPC Preprocessing

The simulator feedbacks the lane centerline waypoints. The waypoints are in the global map coordinate. MPC preprocesses the data by transforming them to a local reference frame, where the car orientation is the x-axis and the perpendicular line to x-axis is the y-axis.


This preprocessed data is further fitted as a 3rd degree polynomial to model the lane.

### Model Predictive Control with Latency

The simulation has a 100ms latency before actuation. MPC controller accommodates the issue by incorporating the latency into prediction. In other words, MPC controller returns an actuation prediction at the time of the timestep dt plus latency after the current timestep.


## Installation

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

