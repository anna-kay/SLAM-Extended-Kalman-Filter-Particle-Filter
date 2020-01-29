% StateTranstionFcn

function particles = myVehicleStateTranstionFcn(particles, dt, u)

    % Defining Process Noise
    q = 0.6; 
    std1 = q;
    std2 = q;
       
    % Calculating next generation particles
    for i=1:length(particles)  % to length epistrefei ti megaluteri diastasi
        noise1 = std1*randn(1, 1);
        noise2 = std2*randn(1, 1);
        particles(i,1) = particles(i,1) + dt*(u(1) + noise1)*cos(particles(i,3));
        particles(i,2) = particles(i,2) + dt*(u(1) + noise1)*sin(particles(i,3));
        particles(i,3) = particles(i,3) + dt*(u(2)+ noise2);    
    end
        
end
       