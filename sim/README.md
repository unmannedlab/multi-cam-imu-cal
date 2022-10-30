# mutli-imu-calibration-sim
A collection of work regarding calibrating multiple IMUs using Kalman filter frameworks
This package simulates multiple IMUs in a calibration EKF framework that
produces body acceleration and angular rate errors that outperform a single IMU
or a complementary filter using multiple IMUs.

By randomly assigning calibration errors, this simulation can run a Monte-Carlo
simulation to show the error distribution and overall stability of the filter.

## Running a single simulation
Running a single instance of the filter is very easy, using the `run_sim()` 
function. A bump time can also be input to randomly change the extrinsic 
calibration of the IMUs to simulate a jerk or bump of one of the sensor mounts

## Running the Monte-Carlo simulation
An example of running a Monte-Carlo simulation can be found in `run_sim.ipynb`
which utilizes a multiprocessing pool to mapping the simulation outputs to get 
the EKF and CF results.

## Plotting and Evaluation
Plotting and statistical results can be generated using the
`plot_*()` and `evaluate_ekfs()` functions respectively. 