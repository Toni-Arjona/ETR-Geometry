clear;
clc;
addpath('functions');
format long

load("car\car_variables\fl_suspension.mat")

fl_suspension.set_damper_distance(200)
fl_suspension.steering_angle()
fl_suspension.knuckle.coord(3).print()


