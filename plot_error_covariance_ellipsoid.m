function plot_error_covariance_ellipsoid( mu, Sigma )
    NP = 16;
    alpha  = 2*pi/NP*(0:NP);
    circle = [cos(alpha);sin(alpha)];
    ns = 3;
    x = [mu(1) ;mu(2)];
    P = [Sigma(1,1) Sigma(1,2);Sigma(2,1) Sigma(2,2)];
    C = chol(P)'; %Choleski method <-????????????
    ellip = ns*C*circle;
    X = x(1)+ellip(1,:);
    Y = x(2)+ellip(2,:);
    plot(X,Y)
    hold on
end

