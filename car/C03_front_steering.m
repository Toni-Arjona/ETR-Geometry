
% Steering Definition
rack_centre = v3(-200, 0.00000000, 120.0);
pinion_diameter = 35;
rack_centre_distance = 250;
front_pinion = false;
max_to_side = 120;
f_steering = steering(rack_centre, pinion_diameter, rack_centre_distance, front_pinion, max_to_side);


save('car_variables/f_steering.mat', 'f_steering'); %NO TOCAR
clear; %NO TOCAR
fprintf("f_steering saved at car/car_variables/f_steering.mat\n"); %NO TOCAR