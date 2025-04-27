## Overview

This project involves the <b>position estimation of vehicle and obstacles</b> using <b>Extended Kalman Filter</b> and <b>Particle Filter</b>.

It is implemented in <b>Matlab</b> (R2019a) using the <b>extendedKalmanFilter</b> object and  <b>particleFilter</b> object of the <b>System Identification Toolbox</b>.


The project is organised in three questions:

- **Question 1:** Position estimation of moving vehicle & stationary obstacles using **Extended Kalman Filter**

- **Question 2:** Position estimation of moving vehicle & stationary obstacles using **Particle Filter**

- **Question 3:** Position estimation of moving vehicle & **one moving obstacle Particle Filter**


**How to use:**

* For question 1, run `question1_plots_video.m` in Matlab

* For questions 2 & 3,  run `question2.m` & `question3.m` respectively

(question1.m is used by question2.m & question3.m to first generate the estimations on which questions 2 & 3 rely)

## Data

**Dataset 1:** (used in Part 1 & 2)
- control1.csv
- radar1.csv

**Dataset 2:**  (used in Part 3)
- control2.csv
- radar2.csv

control1.csv & control2.csv contain the speed measurements, u and θ

radar1.csv & radar2.csv contain the noisy measurement of the obstacles from the vehicle, d1, φ1, d2, φ2

**Sampling:** A sampling rate of 10Hz is assumed for both datasets.

**Noise of measuring device:** A mean value of 0 and a standard deviation of 0.3 radians (angle) and 0.5 meters (distance) are assumed.

## Project Structure
```
| - src/
| - - myLikelihoodMeasurement2Fcn.m
| - - myLikelihoodMeasurementFcn.m
| - - myVehicleMovingObstacleStateTransitionFcn.m
| - - myVehicleStateTransitionFcn.m
| - - plot_error_covariance_ellipsoid.m
| - - question1_plots_video.m
| - - question1.m
| - - question2.m
| - - question3.m
| - datasets/
| - - control1.csv
| - - control2.csv
| - - radar1.csv
| - - radar2.csv
```

## Main Project

### General Premises
A vehicle us moving on a plane (2 dimensions). 
The vehicle is aware of two stationary obstacles on the same plane.
The model of the movement of the vehicle is described by:

![kalman_vehicle_movement](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/3d139c30-69f6-4ab8-8132-717234f4c7a6)

while the model of the measurement of the positions of the obstacles by:

![kalman_obstacles_position](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/31d5dadf-925b-4d1e-a5d7-e5cc42453a97)


![kalman_X_o_t](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/36cbb9aa-7614-49ed-8fb3-22e1db9698c5) and ![kalman_Y_o_t](https://github.com/anna-kay/extended-kalman-filter-particle-filter-vehicle-movement/assets/56791604/1f4efe6d-9cb3-4c37-98ab-b30a40063e5f) are the coordinates of the vehicle at time step t. 

The noise in the system is Gaussian with mean vlaue 0 and standard deviation σ.


### Part 1: Extended Kalman Filter

**Task:** Estimate the seven states using the Extended Kalman filter. The vehicle's measurements include the angle and distance from which it perceives each obstacle. The angle is calculated with respect to the longitudinal axis of the vehicle, with rotation considered counter-clockwise. Both angle and distance measurements are noisy, with Gaussian noise having a mean value of 0. The vehicle is subject to changing velocity and rotation.

**Solution:**

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

x(4), x(5) are the coordinates of the first obstacle, and since it is stationary, they do not change

x(6), x(7) are the coordinates of the second obstacle, and, similarly to the first one, since it is stationary, they do not change.

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
The rest of the positions of the diagonal stay empty as they refer to the coordinates of the stationary obstacles and thus there can be no noise.
It is assumed that q1=q2=q3 without loss of generality.

The model of the measurement for the obstacles is described by:

```
h= @(x) [sqrt((x(4)-x(1))^2 + (x(5) - x(2))^2);
          atan2(((x(5) - x(2)),((x(4)-x(1))) - x(3);
          sqrt((x(6)-x(1))^2 + (x(7) - x(2))^2);
          atan2((x(7) - x(2)),(x(6)-x(1))) - x(3);
          ];
```

**Extended Kalman Filter: Prediction & Correction Steps**

https://github.com/anna-kay/SLAM-Extended-Kalman-filter-Particle-filter/assets/56791604/0645cec2-1be4-4476-bde7-6b9f51ceacc2

**Extended Kalman Filter: Uncertainties of Positions**

https://github.com/user-attachments/assets/d715cccd-348d-4022-a5b3-10a5930c4e04


### Part 2: Particle Filter
**Task:** Utilizing the best estimations of the obstacle positions obtained with the Extended Kalman filter (from part 1), use the Particle Filter to estimate the three states of the vehicle from the outset.


### Part 3: Particle Filter & moving obstacle
**Task:** Assume that the second obstacle moves on the x-axis with an unknown stable velocity. Utilizing the best estimation from part 1 as the initial position of the second obstacle, estimate the three states of the vehicle and the position x of the moving obstacle using the Particle Filter. Utilize the provided noisy measurements (dataset 2).

## Resources 

**Kalman Filter:**

- [Kalman filter.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227552/Kalman.filter.pdf)

- [Understanding the Basis of the Kalman Filter.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227595/Understanding.the.Basis.of.the.Kalman.Filter.pdf)


**Extended Kalman Filter:**

- [06 EKF Localization - Worked out Example.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227556/06.EKF.Localization.-.Worked.out.Example.pdf)

- [extended_kalman_filterPDFNiceImages.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227561/extended_kalman_filterPDFNiceImages.pdf)

- [sheet08sol_seems_good.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227584/sheet08sol_seems_good.pdf)

- [Simulataneous_localization_and_mapping.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227586/Simulataneous_localization_and_mapping.pdf)

- [tutorialEKF_ma8imatika.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227593/tutorialEKF_ma8imatika.pdf)

- [theoria.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227597/theoria.pdf)

- [Uploading mark_wickert_247_NA_DW.pdf…]()


**Particle Filter:**

- [77002125.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227557/77002125.pdf)

- [lecture-pf.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227569/lecture-pf.pdf)

- [Particle Filter.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227574/Particle.Filter.pdf)

- [particle-filters++_v2Berckley.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227581/particle-filters%2B%2B_v2Berckley.pdf)

- [Hsaio_plinval_miller_ParticleFiltersPrint.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227639/Hsaio_plinval_miller_ParticleFiltersPrint.pdf)

- [LIKELIHOOD.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227664/LIKELIHOOD.pdf)


**SLAM:**


- [slamForDummies.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227590/slamForDummies.pdf)

- [2014_0213_EKF SLAM.pptx](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227633/2014_0213_EKF.SLAM.pptx)

- [Localization3_Slides_Good.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227667/Localization3_Slides_Good.pdf)

- [IntroductionToEstimationForNavigation_Newman06_matlabExamplesInTheEnd.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227647/IntroductionToEstimationForNavigation_Newman06_matlabExamplesInTheEnd.pdf)

- [Stachniss_slam04-ekf-slam_na_to_dw.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227625/Stachniss_slam04-ekf-slam_na_to_dw.pdf)

- [slam10-fastslam-4.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227588/slam10-fastslam-4.pdf)

- [WeightsFreiburg.pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227596/WeightsFreiburg.pdf)

- [2017-simultaneous_localization_and_mapping_a_survey_of_current_trends_in_autonomous_driving (1).pdf](https://github.com/anna-kay/SLAM-Extended-Kalman-Filter-Particle-Filter/files/15227635/2017-simultaneous_localization_and_mapping_a_survey_of_current_trends_in_autonomous_driving.1.pdf)

