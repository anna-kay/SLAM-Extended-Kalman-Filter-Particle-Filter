% MeasurementLikelihoodFcn

function likelihood = myLikelihoodMeasurementFcn(predictedParticles, measurement, A, B)
    
   % A and B are the obstacles
   
   %number of measurements
   m = 4;
   % measurement noise covariance matrix
   R = eye(m);               
   R(logical(eye(m))) = [0.25, 0.09, 0.25, 0.09];
   
   std1=0.5;
   std2=0.3;
  
   %  N-element measurement hypothesis vector (matrix) construction (H*x_particles)
   distanceToAx = predictedParticles(:,1) - A(1);
   distanceToAy = predictedParticles(:,2) - A(2);
    
   distanceToBx = predictedParticles(:,1) - B(1);
   distanceToBy = predictedParticles(:,2) - B(2);
   % Measurement update based on already happened state update
   % noise should be added to the measurement,  e.g. 0.5*randn(length(predictedParticles),1)
   predictedRangeToA = sqrt(distanceToAx.^2 + distanceToAy.^2) + std1*randn(length(predictedParticles), 1); % +noise std*randn(length(predictedParticles),1)
   predictedRangeToB = sqrt(distanceToBx.^2 + distanceToBy.^2) + std1*randn(length(predictedParticles), 1);
  
   predictedBearingToA = atan2(distanceToAy,distanceToAx) - predictedParticles(:,3) + std2*randn(length(predictedParticles), 1);
   predictedBearingToB = atan2(distanceToBy,distanceToBx) - predictedParticles(:,3) + std2*randn(length(predictedParticles), 1); 
   
  % *** concatenate the predicted stuff!
  % to get 4 columns for measurement & 4 columns for predictedMeasurement
  
  M = repmat(measurement, length(predictedParticles), 1);  % y(k)
  
  %  N-element measurement hypothesis vector (matrix) construction (H*x_particles)
  X = horzcat(predictedRangeToA,  predictedBearingToA, predictedRangeToB, predictedBearingToB);
  
  % the likelihood of each measurement hypothesis is calculated based 
  % on the sensor measurement and the measurement noise probability distribution.
  likelihood = mvnpdf(M, X, R);
  
  %likelihood = 1/(sqrt((2*pi).^4*det(R)))*exp(-0.5*(X-M)*(inv(R))*(X-M)');
end
