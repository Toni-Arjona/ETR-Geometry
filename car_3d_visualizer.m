%% Boiler plate
clc;
clear;
load("car\car_variables\car.mat");

figure
hold on

%% Actual points drawn

car.set_front_dampers(160);
car.set_steering_wheel(90);
car.plot3dpoints();

%% End boilerplate
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
view(3);