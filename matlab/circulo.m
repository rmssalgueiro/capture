% Define radius and center of the circle
% radius = 5;
% center = [0, 0];

% Generate angles from 0 to 2*pi
% angles = linspace(0, 2*pi, 100);

% Calculate x and y coordinates of the circumference
% x = center(1) + radius * cos(angles);
% y = center(2) + radius * sin(angles);

% Plot the circumference




for k = 1:100
    
    x(k) = 2 * cos(k*0.1);
    y(k) = 2 * sin(k*0.1);

end

plot(x, y);