%% 
clc;
clear;
addpath(genpath("car/"));
addpath(genpath("functions/"));
addpath(genpath("functions teresa/"));

run("car\C01_front_suspension.m");
run("car\C02_rear_suspension.m");
run("car\C03_front_steering.m");
run("car\C04_rear_steering.m");
run("car\C05_car.m");

load('car/car_variables/car.mat')

car.ui_plot3d();

