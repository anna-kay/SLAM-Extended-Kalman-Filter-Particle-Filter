%---------------Ergasia 1 - Extended Kalman Filter-------------------------
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

control = csvread('control1.csv');
radar = csvread('radar1.csv');
radar(:,2) = wrapToPi(radar(:,2));
radar(:,4) = wrapToPi(radar(:,4));

PredictedState = zeros(7,100);
PredictedStateCovariance = zeros(7,7,100);
CorrectedState = zeros(7,100);
CorrectedStateCovariance = zeros(7,7,100);

for k=1:length(control)
    [PredictedState(:,k), PredictedStateCovariance(:,:,k)] = predict(myEKF, control(k,:));
    [CorrectedState(:,k), CorrectedStateCovariance(:,:,k)] = correct(myEKF, radar(k,:));  
end


%%---------------------------PLOTS & VIDEOS------------------------------%%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%% ------------------------------- PLOTS----------------------------------%%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%% General plot - To plot landmarks as well:

figure(1)
plot(PredictedState(1,:), PredictedState(2,:), CorrectedState(1,:),CorrectedState(2,:), CorrectedState(4,:),CorrectedState(5,:),'bx',CorrectedState(6,:),CorrectedState(7,:), 'ro')
legend('Predicted', 'Corrected', 'Obstacle1', 'Obstacle2')
hold on
plot(Ax, Ay, 'r*')
plot(Bx, By, 'b*')
plot(CorrectedState(4,k),CorrectedState(5,k),'k*')
plot(CorrectedState(6,k),CorrectedState(7,k), 'k*')
title('Kalman filter Prediction-Correction & Obstacles Positions')
grid on
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';

%% PLOTTING ERROR COVARIANCE MATRIX ELLIPSOID

% Define VEHICLE Covariance Matrix
vehiclePredictedStateCovariance = PredictedStateCovariance(1:2, 1:2, :);
vehiclePredictedState = PredictedState(1:2, :);

%To plot error covariance ellipsoid
vehicleCorrectedStateCovariance = CorrectedStateCovariance(1:2, 1:2, :);
vehicleCorrectedState = CorrectedState(1:2, :);

figure(2)
for i=1:100
    plot_error_covariance_ellipsoid(vehicleCorrectedState(:,i), vehicleCorrectedStateCovariance(:,:,i))
    hold on
    plot(vehicleCorrectedState(1,:), vehicleCorrectedState(2,:), 'b--')
    hold on
    plot(vehicleCorrectedState(1,i), vehicleCorrectedState(2,i), 'k.')
    title('Vehicle corrected path & covariance')
end
grid on
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';

%% OBSTACLES
% CHECKING OBSTACLES COVARIANCE MATRIX, to define the best estimation
vehicleCorrectedStateCovariance = CorrectedStateCovariance(1:2, 1:2, :);
vehicleCorrectedState = CorrectedState(1:2, :);

obstacleAcovarinace = CorrectedStateCovariance(4:5, 4:5, :);
obstacleAcoordinates = CorrectedState(4:5, :);

obstacleBcovarinace = CorrectedStateCovariance(6:7, 6:7, :);
obstacleBcoordinates = CorrectedState(6:7, :);

temp=eye(4);

for i=1:100
    currentMatrix = obstacleAcovarinace(i)-temp;
    [~,p] = chol(currentMatrix);
    if p==0
        indexOfSmallest = i;
    end
    temp = obstacleAcovarinace(i);
end

% figure(3) for i=1:100
%     plot_error_covariance_ellipsoid(obstacleBcoordinates(:,i),
%     obstacleBcovarinace(:,:,i)) hold on grid on legend('Obstacle B')
% end


%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%%------------------------------- VIDEOS---------------------------------%%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
figure(4)
for i=1:100
    plot(vehiclePredictedState(1,i), vehiclePredictedState(2,i), 'k.')
    plot(vehicleCorrectedState(1,i), vehicleCorrectedState(2,i), 'r.')
    plot(CorrectedState(4,i),CorrectedState(5,i),'bx',CorrectedState(6,i),CorrectedState(7,i), 'ro')
    legend('Obstacle1', 'Obstacle2', 'Predicted', 'Corrected')
    title('Kalman filter Prediction-Correction steps')
    hold on
    grid on
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    F(i) = getframe(gcf);
    drawnow    
end

% create the video writer with 1 fps
writerObj = VideoWriter('kalmanFilterPredictionCorrection.avi');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);


%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

figure(5)
for i=1:100
    plot_error_covariance_ellipsoid(vehicleCorrectedState(:,i), vehicleCorrectedStateCovariance(:,:,i))
    plot(vehicleCorrectedState(1,i), vehicleCorrectedState(2,i), 'k.')
    plot(vehicleCorrectedState(1,:), vehicleCorrectedState(2,:), 'b--')
    title('Vehicle corrected Path and Covariance')
    grid on
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    hold off
    F(i) = getframe(gcf);
    
    drawnow
end

% create the video writer with 1 fps
writerObj = VideoWriter('vehiclePath&Covariance.avi');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%% Video - PLOTTING COVARIANCES OF VEHICLE & OBSCTACLES

figure(6)
for i=1:100
    % vehicle
    plot_error_covariance_ellipsoid(vehicleCorrectedState(:,i), vehicleCorrectedStateCovariance(:,:,i))
    hold on
    plot(vehicleCorrectedState(1,:), vehicleCorrectedState(2,:), 'b--')
    hold on
    plot(vehicleCorrectedState(1,i), vehicleCorrectedState(2,i), 'k.')
    hold on
    % obstacle A
    plot_error_covariance_ellipsoid(obstacleAcoordinates(:,i), obstacleAcovarinace(:,:,i))
    hold on
    plot(obstacleAcoordinates(1,:), obstacleAcoordinates(2,:), 'b--')
    hold on
    plot(obstacleAcoordinates(1,i), obstacleAcoordinates(2,i), 'k.')
    % obstacle B
    plot_error_covariance_ellipsoid(obstacleBcoordinates(:,i), obstacleBcovarinace(:,:,i))
    hold on
    plot(obstacleBcoordinates(1,:), obstacleBcoordinates(2,:), 'b--')
    hold on
    plot(obstacleBcoordinates(1,i), obstacleBcoordinates(2,i), 'k.')
    title('Uncertainties of corrected positions')
    hold on
    grid on
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    F(i) = getframe(gcf);
    drawnow    
end

% create the video writer with 1 fps
writerObj = VideoWriter('uncertaintiesOfPositions.avi');
writerObj.FrameRate = 10;
% set the seconds per image open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);