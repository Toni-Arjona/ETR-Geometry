clc;
clear;
load('car/car_variables/car.mat')

car.centre_steering();
car.set_front_dampers(180);
car.set_rear_dampers(180);
car.set_toe_rear(2);
car.set_toe_front(2);
car.print()

car.set_toe_front(0);
car.set_steering_wheel(0);
car.set_rear_dampers(180);
car.set_front_dampers(180);
p = car.rl_susp.rocker.coord(3);
q = car.rl_susp.rocker.coord(4);
car.set_rear_dampers(180);
p = p - car.rl_susp.rocker.coord(3);
q = q - car.rl_susp.rocker.coord(4);
fprintf("Rocker Ratio: %4.3f\n", (p.')/(q.'));

car.print_extended();
car.plot3d();