addpath('functions'); %NO TOCAR

% Steering Definition
rack_centre = v3(1630, 0, 124.4537); % Only X & Z
pinion_diameter = 35; % Doesn't matter since it's in the back
rack_centre_distance = 255; % The Y of the inner suport for the tierod
rear_pinion = true; % Doesn't matter
r_steering = steering(rack_centre, pinion_diameter, rack_centre_distance, rear_pinion);

save('car/car_variables/r_steering.mat', 'r_steering'); %NO TOCAR
clear; %NO TOCAR
fprintf("r_steering saved at car/car_variables/r_steering.mat\n"); %NO TOCAR