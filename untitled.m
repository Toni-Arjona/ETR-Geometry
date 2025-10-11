clear;
clc;
addpath('functions');
format long

load("car\car_variables\fl_suspension.mat")

fl_suspension.set_damper_distance(180)
base_steering_angle = fl_suspension.steering_angle()*180/pi;
base_camber_angle = fl_suspension.camber_angle()*180/pi;
fprintf("Steering Angle: %10.5f\n", base_steering_angle);
fprintf("Camber Angle: %10.5f\n", base_camber_angle);
