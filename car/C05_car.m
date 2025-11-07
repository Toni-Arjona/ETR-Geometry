addpath('functions'); %NO TOCAR
load('car\car_variables\fl_suspension.mat');
load('car\car_variables\fr_suspension.mat');
load('car\car_variables\rl_suspension.mat');
load('car\car_variables\rr_suspension.mat');
load('car\car_variables\f_steering.mat');
load('car\car_variables\r_steering.mat');


car = car(fl_suspension, fr_suspension, rl_suspension, rr_suspension, f_steering, r_steering);
save('car/car_variables/car.mat', 'car'); %NO TOCAR
clear; %NO TOCAR
fprintf("car saved at car/car_variables/car.mat\n"); %NO TOCAR
