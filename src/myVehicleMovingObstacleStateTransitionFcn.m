% The state transition function for the obstacle moving along the x-axis
% with a constant unknown speed.

function particles = myVehicleMovingObstacleStateTransitionFcn(particles, dt, u)
       
       % Adding Process Noise
       q = 0.5; 
       std1 = q;
       std2 = q;
       
       % Obstacle Xvelocity & process noise
       uObstacle = 0.1;
       xAxisDistance = uObstacle*dt;
            
       for i=1:length(particles)   
           
           noise1 = std1*randn(1, 1);
           noise2 = std2*randn(1, 1);
           
           particles(i,1) = particles(i,1) + dt*(u(1) + noise1)*cos(particles(i,3));
           particles(i,2) = particles(i,2) + dt*(u(1) + noise1)*sin(particles(i,3));
           particles(i,3) = particles(i,3) + dt*(u(2)+ noise2);  
           particles(i,4) = particles(i,4) + xAxisDistance;
           particles(i,5) = particles(i,5);
       end
end