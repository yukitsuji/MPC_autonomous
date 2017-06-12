# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## The Model
The vehicle state has the following components:

- x: position of the vehicle in the forward direction
- y: position of the vehicle in the lateral direction
- psi: yaw angle or orientation of the vehicle
- v: speed of the vehicle

The actuators of the vehicle are:

- delta: steering angle in radians
- a: acceleration

The update equations are used to compute the state of the car at the next time step, based on the state at the
current time step:

```
x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta * dt
v_t+1 = v_t + a_t * dt
```

In addition to the state and actuator variables, they include the time step `dt` and the factor
`Lf` which is defined as the distance between the vehicle's front and its center of gravity. `Lf`, 
together with the current velocity `v_t`, determine the current turning radius of the vehicle.

---
## Polynomial Fitting and MPC Preprocessing
In a first step, the waypoints are transformed into the vehicle coordinate
system. The resulting x-direction is the forward direction of the vehicle, while the y-direction represents the lateral
displacement of waypoints relative to the center of the vehicle. This allows for easy fitting of a polynomial of the form 
`y = a*x^3 + b*x^2 + c*x + d`. When constructing the initial state vector for the solver, the values for x, y, and psi can
now be conveniently set to zero because the coordinate system is defined relative to the vehicle. The initial cross track error is 
equal to the constant term of the fitted polynomial in this coordinate system, and the initial orientation error is
calculated from the first derivative of the polynomial at x=0.


## Model Predictive Control with Latency
The additional latency introduced in `main.cpp` presents a problem: The steering value computed by the solver
for the first time-step will already be in the past, which will lead to an increase of the CTE and orientation error. 
The solver will try to compensate the resulting increase in cost in the next iteration but this compensation will
also come too late. When this process is repeated a few times, the vehicle will start to oscillate around the intended
trajectory, especially at higher velocities.

In order to mitigate this problem, I implemented an averaging procedure that takes the mean of the first three predicted
steering angles. With a time step of 100ms, this corresponds a time interval of 300ms, which is about twice of the total
expected latency. This averaging measure successfully stabilizes the control behavior.


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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.
