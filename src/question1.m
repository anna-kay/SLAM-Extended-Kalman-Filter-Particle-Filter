%---------------Question 1 - Extended Kalman Filter-------------------------
clear;
clc;

% process noise covariance matrix            
q = 0.001;
Q = [q, 0, 0, 0, 0, 0, 0;
       0, q, 0, 0, 0, 0, 0;
       0, 0, q, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0];

%number of measurements
m = 4;

% measurement noise covariance matrix
R = eye(m);               
R(logical(eye(m))) = [0.25, 0.09, 0.25, 0.09];

% sampling frequency = 10Hz
dt = 0.1;

% Orizw ti sunartisi katastasis f kai ti sunartisi metrisis h
f = @(x,u)[ x(1) + u(1)*cos(x(3))*dt; 
            x(2) + u(1)*sin(x(3))*dt;
            x(3) + u(2)*dt;
            x(4);
            x(5);
            x(6);
            x(7)
           ];

h= @(x) [sqrt((x(4)-x(1))^2 + (x(5) - x(2))^2);
         atan2((x(5) - x(2)),(x(4)-x(1))) - x(3);
             
         sqrt((x(6)-x(1))^2 + (x(7) - x(2))^2);
         atan2((x(7) - x(2)),(x(6)-x(1))) - x(3);
        ];                               

Ax = 1;
Ay = -1;
Bx = 1;
By = 1;

% Initial state definition   
initialState = [0;
                0;
                0;
                Ax;
                Ay;
                Bx;
                By]; 

% Initilization of the Extended Kalman Filter 
x=initialState; 

myEKF = extendedKalmanFilter(f,h,x); 

myEKF.StateCovariance = [0, 0, 0, 0, 0, 0, 0;
                         0, 0, 0, 0, 0, 0, 0;
                         0, 0, 0, 0, 0, 0, 0;
                         0, 0, 0, 5, 0, 0, 0;
                         0, 0, 0, 0, 5, 0, 0;
                         0, 0, 0, 0, 0, 5, 0;
                         0, 0, 0, 0, 0, 0, 5];

myEKF.ProcessNoise = Q;
myEKF.MeasurementNoise = R;

control = csvread('datasets/control1.csv');
radar = csvread('datasets/radar1.csv');
radar(:,2) = wrapToPi(radar(:,2));
radar(:,4) = wrapToPi(radar(:,4));

PredictedState = zeros(7,100);
PredictedStateCovariance = zeros(7,7,100);
CorrectedState = zeros(7,100);
CorrectedStateCovariance = zeros(7,7,100);

for k=1:length(control)
    [PredictedState(:,k), PredictedStateCovariance(:,:,k)] = predict(myEKF, control(k,:));
    PredictedState(3,k) = wrapToPi(PredictedState(3,k));
    [CorrectedState(:,k), CorrectedStateCovariance(:,:,k)] = correct(myEKF, radar(k,:));
    CorrectedState(3,k) = wrapToPi(CorrectedState(3,k));
end

% %% OBSTACLES
% % CHECKING OBSTACLES COVARIANCE MATRIX, to define the best estimation
% vehicleCorrectedStateCovariance = CorrectedStateCovariance(1:2, 1:2, :);
% vehicleCorrectedState = CorrectedState(1:2, :);
% 
% obstacleAcovarinace = CorrectedStateCovariance(4:5, 4:5, :);
% obstacleAcoordinates = CorrectedState(4:5, :);
% 
% obstacleBcovarinace = CorrectedStateCovariance(6:7, 6:7, :);
% obstacleBcoordinates = CorrectedState(6:7, :);
% 
% temp=eye(4);
% 
% for i=1:100
%     currentMatrix = obstacleAcovarinace(i)-temp;
%     [~,p] = chol(currentMatrix);
%     if p==0
%         indexOfSmallest = i;
%     end
%     temp = obstacleAcovarinace(i);
% end
% 
% % figure(3) for i=1:100
% %     plot_error_covariance_ellipsoid(obstacleBcoordinates(:,i),
% %     obstacleBcovarinace(:,:,i)) hold on grid on legend('Obstacle B')
% % end