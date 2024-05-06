# Extended-Kalman-Filter--Particle-Filter--Vehicle-Movement--SLAM

## Overview

This project involves the <b>position estimation of vehicle and obstacles</b> using the Extended-Kalman and Particle filters.

It is implemented in <b>Matlab</b> (R2019a was used) using the:
* <b>extendedKalmanFilter</b> and 
* <b>particleFilter</b> objects 
of the <b>System Identification Toolbox</b>.

The project is broken down into three parts:

Question 1:

Question 2:

Question 3:

## Data

Sampling: A sampling rate of 10Hz is assumed for both datasets.
Noise of measuring device: A mean value of 0 and a standard deviation of 0.3 radians (angle) and 0.5 meters (distance) are assumed.

### Dataset 1

control1.csv contains the speed measurements, u and θ

radar1.csv contains the noisy measurement of the obstacles from the vehicle, d1, φ1, d2, φ2

### Dataset 2

control2.csv contains the speed measurements, u and θ

radar2.csv contains the noisy measurement of the obstacles from the vehicle, d1, φ1, d2, φ2






## Project Structure
```
| - myLikelihoodMeasurement2Fcn.m
| - myLikelihoodMeasurementFcn.m
| - myVehicleMovingObstacleStateTransitionFcn.m
| - myVehicleStateTransitionFcn.m
| - plot_error_covariance_ellipsoid.m
| - question1_plots_video.m
| - question1.m
| - question2.m
| - question3.m
| - dataset/
| - - control1.csv
| - - control2.csv
| - - radar1.csv
| - - radar2.csv
```
