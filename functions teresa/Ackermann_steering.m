
function [left_ackermann_percentage, right_ackermann_percentage] = Ackermann_steering(car, steering_wheel_angle)
    %{
    steer_angle_max= 120; %in deg
    steer_angle_min= -120; %in deg
    L= 1535; %in mm
    tf=1250; %in mm
    tr=1150; %in mm
    %}
    arguments
    car car
    end 
    
    %From steering wheel angle to inner and outer wheel angle
    delta_i_actual= left_wheel_angle(steering_wheel_angle, car); %in deg
    delta_o_actual= right_wheel_angle(steering_wheel_angle, car);%in deg

    %Ackermann calculated for a left turn
    Ri= car.wheelbase/atan(delta_i_actual*pi/180)-(car.trackwidth_rear-car.trackwidth_front)/2;
    Ro= car.wheelbase/atan(delta_o_actual*pi/180)-(car.trackwidth_rear+car.trackwidth_front)/2;
    R=(Ri+Ro)/2;
    delta_i_ackermann= atan(car.wheelbase/(R+(car.trackwidth_rear-car.trackwidth_front)/2));
    delta_o_ackermann= atan(car.wheelbase/(R+(car.trackwidth_rear+car.trackwidth_front)/2));

    left_ackermann_percentage= (delta_i_actual-delta_i_ackermann)/delta_i_ackermann*100;
    right_ackermann_percentage=(delta_o_actual-delta_o_ackermann)/delta_i_ackermann*100;

