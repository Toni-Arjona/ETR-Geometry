addpath('functions'); %NO TOCAR

% Steering Definition
rack_centre = v3(-50, 0.00000000, 168.60000000);
pinion_diameter = 35;
rack_centre_distance = 229.50000000;
front_pinion = false;
f_steering = steering(rack_centre, pinion_diameter, rack_centre_distance, front_pinion);

save('car/car_variables/f_steering.mat', 'f_steering'); %NO TOCAR
clear; %NO TOCAR
fprintf("f_steering saved at car/car_variables/f_steering.mat\n"); %NO TOCAR