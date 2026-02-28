
% Steering Definition
rack_centre = v3(-180, 0.00000000, 149.46);
pinion_diameter = 40;
rack_centre_distance = 225;
front_pinion = false;
max_to_side = 120;
f_steering = steering(rack_centre, pinion_diameter, rack_centre_distance, front_pinion, max_to_side);


save('car_variables/f_steering.mat', 'f_steering'); %NO TOCAR
clear; %NO TOCAR
fprintf("f_steering saved at car/car_variables/f_steering.mat\n"); %NO TOCAR