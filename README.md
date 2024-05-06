# Extended-Kalman-Filter--Particle-Filter--Vehicle-Movement--SLAM

## Overview

This project involves the <b>position estimation of vehicle and obstacles</b> using the <b>Extended-Kalman</b> and <b>Particle filters</b>.

It is implemented in <b>Matlab</b> (R2019a) using the:
* <b>extendedKalmanFilter</b> object and 
* <b>particleFilter</b> object
  
of the <b>System Identification Toolbox</b>.

The project is broken down into three parts:

* Question 1 - Position estimation of moving vehicle & static obstacles using Extended Kalman Filter

* Question 2 - Position estimation of moving vehicle & static obstacles using Particle Filter

* Question 3 - Position estimation of moving vehicle & one moving obstacle Particle Filter

## Data

Sampling: A sampling rate of 10Hz is assumed for both datasets.

Noise of measuring device: A mean value of 0 and a standard deviation of 0.3 radians (angle) and 0.5 meters (distance) are assumed.

### Dataset 1 (used in Question 1 & Question 2)

control1.csv contains the speed measurements, u and θ

radar1.csv contains the noisy measurement of the obstacles from the vehicle, d1, φ1, d2, φ2

### Dataset 2 (used in Question 3)

control2.csv contains the speed measurements, u and θ

radar2.csv contains the noisy measurement of the obstacles from the vehicle, d1, φ1, d2, φ2


## Questions

### General Premises
A vehicle us moving on a plane (2 dimensions). 
The vehicle is aware of two static obstacles on the same plane.
The model of the movement of the vehicle is described by:

![kalman_vehicle_movement](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/3d139c30-69f6-4ab8-8132-717234f4c7a6)

While the model of the measurement of the positions of the obstacles by:

![kalman_obstacles_position](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/31d5dadf-925b-4d1e-a5d7-e5cc42453a97)


![kalman_X_o_t](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/36cbb9aa-7614-49ed-8fb3-22e1db9698c5) and ![kalman_Y_o_t](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/1f4efe6d-9cb3-4c37-98ab-b30a40063e5f) are the coordinates of the vehicle at time step t. 

The noise in the system Is Gaussian with mean vlaue 0 and standard deviation σ.


### Question 1

Q1: Estimate the seven positions using the Extended Kalman filter. Given that: the vehicle is moving with changing velocity and rotation.

Solution:

The model of the wolrd is described by:

```
f = @(x,u)[ x(1) + u(1)*cos(x(3))*dt;
            x(2) + u(1)*sin(x(3))*dt;
            x(3) + u(2)*dt;
            x(4);
            x(5);
            x(6);
            x(7)
            ];
```

x(1), x(2), x(3) describe the position of the vehicle and change according to the model of movement that was provided

x(4), x(5) are the coordinates of the first obstacle, and since it is static, they do not change

x(6), x(7) are the coordinates of the second obstacle, and, similarly to the first one, since it is static, they do not change

The process noise was modeled as follows:

```
Q = [q1, 0, 0, 0, 0, 0, 0;
      0, q2, 0, 0, 0, 0, 0;
      0, 0, q3, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 0];
```

The three first positions of the diagonal of the matrix describe the noise in the movement of the vehicle.
The rest of the positions of the diagonal stay empty as they refer to the coordinates of the static obstacles and thus there can be no noise.
It is assumed that q1=q2=q3 without loss of generality.

The model of the measurement for the obstacles is described by:

```
h= @(x) [sqrt((x(4)-x(1))^2 + (x(5) - x(2))^2);
          atan2(((x(5) - x(2)),((x(4)-x(1))) - x(3);
          sqrt((x(6)-x(1))^2 + (x(7) - x(2))^2);
          atan2((x(7) - x(2)),(x(6)-x(1))) - x(3);
          ];
```

### Question 2
Q2: Using the best estimation of the positions of the obstacles achieved with the Extended-Kalman filter (question 1), estimate from the beginning the three states of the vehicle using the Particle Filter.

Solution:

### Question 3
Q3:

Solution:

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
| - datasets/
| - - control1.csv
| - - control2.csv
| - - radar1.csv
| - - radar2.csv
```
