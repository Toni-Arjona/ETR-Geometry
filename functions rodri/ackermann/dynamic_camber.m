close all
idx = 1;
camber_FL = zeros(1, 23); % Preallocate arrays for camber values
camber_FR = zeros(1, 23);
jack_FR = zeros(1, 23);
jack_FL = zeros(1, 23);

for steering_wheel_angle = 0:5:150
    [FL_steer, FR_steer, FL_camber, FR_camber, FL_jack, FR_jack] = ackermann_function_3D(steering_wheel_angle);
    camber_FL(idx) = FL_camber;
    camber_FR(idx) = FR_camber;
    jack_FL(idx) = FL_jack;
    jack_FR(idx) = FR_jack;
    idx = idx + 1
end

figure(1)
plot(0:5:150, camber_FL)
hold on
grid on
plot(0:5:150, camber_FR)

figure(2)
plot(0:5:150, jack_FL)
hold on
grid on
plot(0:5:150, jack_FR)