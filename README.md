# Model Predictive Control (MPC) with actuator delay
Project for Self-Driving Car Engineer Nanodegree Program 

---

## Purpose of the Project 
The model predictive controler has the purpose to drive a car around a track
in a simulator. The simulator provides the car's position (x,y) and orientation.
In addition the simulator provides a list of waypoints for the reference trajectory.
All data is provided in a global coordinate system.
The MPC controls steering angle and throttle to drive the car safely around the
track.

[![MPC with delay](http://img.youtube.com/vi/QcmNCa6CQHM/0.jpg)](http://www.youtube.com/watch?v=QcmNCa6CQHM "CarND MPC Project")

### Vehicle Model in MPC
The MPC uses as vehicle model a bicycle model. It is described by the following 
equations in class `FG_eval`:

```
 // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt 
 // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt 
 // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt 
 // v_[t] = v[t-1] + a[t-1] * dt 
 // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt 
 // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt 
```

The variables have the following meaning:

* `x`: x-position of the car
* `y`: y-position of the car
* `psi`: orientation of the car
* `v`: velocity of the car
* `cte`: cross track error of the car to the reference trajectory
* `epsi`: heading error of the car'S orientation to the reference trajectory's orientation 
* `Lf`: distance between front axle and center of gravity
 
### MPC Preprocessing
MPC is fed by data transformed to the vehicle coordinate system. This makes
fitting of polynomial to waypoints of reference trajectory easier.
The reference trajectory from the simulator is transformed to vehicle coordinates
in the following way:

```
const double cos_psi = cos(psi);
const double sin_psi = sin(psi);
for (int i=0; i < ptsx.size(); i++) {
  ptsx_vcs[i] =  (ptsx[i] - px) * cos_psi + (ptsy[i] - py) * sin_psi;
  ptsy_vcs[i] = -(ptsx[i] - px) * sin_psi + (ptsy[i] - py) * cos_psi;
}
``` 

Here the car position is described by its global x-position `px`, y-position `py` 
and orientation `psi`. The waypoints of the reference trajectory is given by the 
simulator in the global coordinates `ptsx` and `ptsy`. These waypoints are transfered
to the vehicle coordinates `ptsx_vcs` and `ptsy_vcs`.


Therefore, the initial state of the car for MPC is:

```
state << 0.0,0.0,0.0,v,cte,epsi;
```

### MPC algorithm (N & dt)
The MPC solves a optimal control problem in every state provided by the simulator.
The MPC trajectory is optimized for N timesteps in the future with a stepsize of dt.
In the beginning `N` was set to `25`. But this was reduced to `10` to reduce computational 
calculation time. `dt` is normally set to `100 ms`. So an optimal trajectory is 
searched for a view range of `1 s`. Sometimes the view range of the reference trajectory
is not long enough for the current velocity. Then `dt` is reduced to fit the MPC 
trajectory to the waypoints view range.

The view range of the reference trajectory is calculated in the following way:

```
double viewRange = 0.0;
for (int i=1; i < ptsx_vcs.size(); i++) {
  viewRange += std::sqrt(
                    std::pow((ptsx_vcs[i]-ptsx_vcs[i-1]),2)
                  + std::pow((ptsy_vcs[i]-ptsy_vcs[i-1]),2) );
}
```

`dt` is calculated in the following way:

```
dt = std::min(MPC_MAX_DT/1000.0, (viewRange/v)/MPC_N);
```

In addition the target speed is reduced depending on the view range, but minimum 65 and maximum 100, 
so that the distance traveled during the MPC horizon is not longer than 120% of 
the view range of the reference trajectory:

```
speedLim = 1.2*viewRange/(MPC_N*(MPC_MAX_DT/1000.0));
v_ref = std::max(std::min(100.0, speedLim), 65.0);
```

### Delay of the actuators
The projet gets more difficult because the control of the actuators (throttle and 
steering) is not performed immediately but with a delay of 100 ms. 
This delay is regarded in the MPC algorithm. For the time of the delay and in addition 
for the calculation time of the MPC the actuator values throttle and steering 
a fixed to the previous values for the delay time:

```
int latency_ind = std::lround((MPC_LATENCY+calcTime)/MPC_MAX_DT);
for (int i = delta_start; i < delta_start + latency_ind; i++) {
  vars_lowerbound[i] = delta_prev;
  vars_upperbound[i] = delta_prev;
}
```

For throttle it is done analog.

---

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

