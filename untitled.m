clear;
clc;
addpath('functions');
format long

load("car\car_variables\fl_suspension.mat")
load("car\car_variables\fr_suspension.mat")

fl_suspension.set_damper_distance(180);
base_steering_angle = fl_suspension.steering_angle()*180/pi;
base_camber_angle = fl_suspension.camber_angle()*180/pi;
fprintf("LEFT Steering Angle: %10.5f\n", base_steering_angle);
fprintf("LEFT Camber Angle: %10.5f\n", base_camber_angle);
fl_suspension.knuckle.coord(3).print()

fr_suspension.set_damper_distance(180);
base_steering_angle = fr_suspension.steering_angle()*180/pi;
base_camber_angle = fr_suspension.camber_angle()*180/pi;
fprintf("RIGHT Steering Angle: %10.5f\n", base_steering_angle);
fprintf("RIGHT Camber Angle: %10.5f\n", base_camber_angle);

