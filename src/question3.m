%----------------------Question 3 - Particle Filter-------------------------
clear;
clc;

run('question1.m');
rng(1234); 

% sampling frequency = 10Hz
dt = 0.1;
numParticles = 2000; 

% Define Obstacles positions
A = CorrectedState(4:5,100);
B_initial = CorrectedState(6:7,100); % Initial position of B from EKF

% Defintion of the Particle Filter & necessary functions
myPF = particleFilter(@myVehicleMovingObstacleStateTransitionFcn, @myLikelihoodMeasurement2Fcn);

initalState = [0; 0; 0; B_initial(1); B_initial(2)];
initialCov = diag([0.01, 0.01, 0.01, 0.5, 0.5]); % Inital uncertainty

initialize(myPF, numParticles, initalState, initalCov, 'CircularVariables', [0 0 1 0 0], 'StateOrientation', 'row'); 
% Initilization of the Particle Filter
% myPF.NumParticles = numPartciles;
% myPF.IsStateVariableCircular = [0 0 1];
% myPF.State = initalState;
% myPF.StateCovariance = initalCov;

myPF.ResamplingPolicy.MinEffectiveParticleRatio = 0.75;

% Particle Filter Parameters
myPF.StateEstimationMethod = 'mean';  % (maxweight, mean)
myPF.ResamplingMethod = 'stratified'; % ('multinomial', 'stratified', 'systematic'), 'systematic' more efficient
myPF.ResamplingPolicy.MinEffectiveParticleRation = 0.6; % More aggressive samlping

% Load  control and measurement data
control = csvread('../datasets/control2.csv');
radar = csvread('../datasets/radar2.csv');
radar(:,2) = wrapToPi(radar(:,2));
radar(:,4) = wrapToPi(radar(:,4));

PredictedStatePF = zeros(5, length(control));
PredictedStateCovariancePF = zeros(5, 5, length(control));
CorrectedStatePF = zeros(5, 100);
CorrectedStateCovariancePF = zeros(5, 5, length(control));

% Prediction and Correction 
for k=1:length(control)
    % Predict step
    [PredictedStatePF(:,k), PredictedStateCovariancePF(:,:,k)] = predict(myPF, dt, control(k,:));
    
    % Define obstacle's B changed position before the correction step
    B = [PredictedStatePF(4,k), PredictedStatePF(5,k)];
    [CorrectedStatePF(:,k), CorrectedStateCovariancePF(:,:,k)] = correct(myPF, radar(k,:), A, B);  
end

figure(1)
plot(PredictedStatePF(1,:),PredictedStatePF(2,:), CorrectedStatePF(1,:),CorrectedStatePF(2,:))
hold on
plot(A(1),A(2), 'x')
hold on
plot(CorrectedStatePF(4,:), CorrectedStatePF(5,:), 'ro')
legend('Predicted', 'Corrected', 'Obstacle1', 'Obstacle2')
title('Particle Filter, N = 1000, u=0.1m/s')
grid on
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
