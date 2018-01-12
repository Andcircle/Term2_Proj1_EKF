# Term2_Proj1_EKF
Extended Kalman Filter

The target of this project is to apply Extended Kalman Filter to fuse data from LIDAR and Radar of a self driving car using C++ (Eclipse IDE).

## Content of this repo
- `scr` source code directory:
  - `main.cpp` - communicate with simulation tool, call functions to run the Kalman filter and calculate RMSE
  - `FusionEKF.cpp` - initializes the Kalman filter, execute predict and update function of Kalman filter
  - `kalman_filter.cpp`- defines the predict function, the update function for lidar, and the update function for radar
  - `tools.cpp` - calculate RMSE and the Jacobian matrix
 

## Result


## How to run the code
Clone this repo and perform
```
mkdir build && cd build
cmake ../src/  && make
./ExtendedKF 
./ExtendedKF 
```



