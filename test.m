clc;
clear;
load('car/car_variables/car.mat')

car.centre_steering();
car.set_front_dampers(180);
car.set_rear_dampers(180);
car.set_steering_wheel(120);
car.set_toe_front(2);
car.print()
