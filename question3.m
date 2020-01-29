%----------------------Ergasia 3 - Particle Filter-------------------------
clear;
clc;
run('ergasia1.m');
rng(1234); 

% sampling frequency = 10Hz
dt = 0.1;

% Define Obstacles positions
A = CorrectedState(4:5,100);
B = CorrectedState(6:7,100);

% Defintion of the Particle Filter & necessary functions
myPF = particleFilter(@myVehicleMovingObstacleStateTransitionFcn, @myLikelihoodMeasurement2Fcn);

initialize(myPF, 1000, [0; 0; 0; B(1); B(2)], zeros(5, 5), 'CircularVariables', [0 0 1 0 0], 'StateOrientation', 'row'); 
% Initilization of the Particle Filter
% myPF.NumParticles = 1000;
% myPF.IsStateVariableCircular = [0 0 1];
% myPF.State = [0; 0; 0];
% myPF.StateCovariance = zero(3,3);

myPF.ResamplingPolicy.MinEffectiveParticleRatio = 0.75;

% Particle Filter Parameters
myPF.StateEstimationMethod = 'mean';  % (maxweight, mean)
myPF.ResamplingMethod = 'stratified'; %('multinomial', 'stratified', 'systematic')

control = csvread('control2.csv');
radar = csvread('radar2.csv');
radar(:,2) = wrapToPi(radar(:,2));
radar(:,4) = wrapToPi(radar(:,4));

PredictedStatePF = zeros(5,100);
PredictedStateCovariancePF = zeros(5,5,100);
CorrectedStatePF = zeros(5,100);
CorrectedStateCovariancePF = zeros(5,5,100);

% Prediction and Correction 
for k=1:length(control)
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
