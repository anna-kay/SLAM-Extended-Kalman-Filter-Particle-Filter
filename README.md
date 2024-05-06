# Kalman-Filter--Extended-Kalman-Filter--Particle-Filter--Vehicle-Movement--SLAM
matlab, kalman, extended-kalman

## Overview

This project involves the position estimation of vehicle and obstacles using the Extended-Kalman and Particle filters.
It is implemented in <b>Matlab</b> (R2019a was used) using the <b>extendedKalmanFilter</b> and <b>particleFilter</b> objects of the <b>System Identification Toolbox</b>.

Question 1:

Question 2:

Question 3:

## Data

10Hz samling is assumed for both datasets

### Dataset 1

control1.csv contains the speed measurements, u and θ
radar1.csv contains the noisy measurement of the obstacles from the vehicle, d1, φ1, d2, φ2

### Dataset 2

control2.csv contains the speed measurements, u and θ
radar2.csv contains the noisy measurement of the obstacles from the vehicle, d1, φ1, d2, φ2






## Project Structure

| - - myLikelihoodMeasurement2Fcn.m
| - - myLikelihoodMeasurementFcn.m
| - - myVehicleMovingObstacleStateTransitionFcn.m
| - - myVehicleStateTransitionFcn.m
| - - plot_error_covariance_ellipsoid.m
| - - question1_plots_video.m
| - - question1.m
| - - question2.m
| - - question3.m
| - dataset/
| - - control1.csv
| - - control2.csv
| - - radar1.csv
| - - radar2.csv
