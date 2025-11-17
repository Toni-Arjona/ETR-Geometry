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
car.set_steering_wheel(45);
car.set_rear_dampers(190);
car.print_extended();