clear;
clc;
addpath('functions');
format long

load("car\car_variables\fl_suspension.mat")

fl_suspension.set_damper_distance(180)
fl_suspension.steering_angle()
fl_suspension.unprojected_steering_angle()
fl_suspension.knuckle.coord(3).print()
fl_suspension.knuckle.coord(1).print()
fl_suspension.knuckle.coord(2).print()
fl_suspension.knuckle_rotation_centre().print()


