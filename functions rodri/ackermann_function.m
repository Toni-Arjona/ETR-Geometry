function [intwheel_deg, extwheel_deg] = ackermann_function(steering_wheel_angle)
% DATOS (distancias todas en mm)
% dimensiones coche
wheelbase = 1600;
front_track = 1250;
rear_track = 1250; 
r_pinion = 35/2; % radio pinion

% INPUTS geometría steering. Centro de coordenadas en KingPin rueda
% izquierda
l_rack = 600; %
x_rack = 60; % posición en eje x rack
y_tie_mangueta = 25;
x_tie_mangueta = 60;

% cálculo puntos y ángulos para steer_angle = 0
ax = x_rack;
ay = front_track/2 - l_rack/2;
bx = x_tie_mangueta;
by = y_tie_mangueta;

l_brazo = sqrt(bx^2 + by^2);
l_tierod = sqrt((ay - by)^2 + (ax - bx)^2);
l_ac_ini = sqrt(ay^2 + ax^2);
ang_ac_ini = atan2d(ax, ay);
alpha_ini = acosd((l_ac_ini^2 + l_brazo^2 - l_tierod^2)/(2*l_brazo*l_ac_ini));


%rueda exterior
    rack_disp = deg2rad(steering_wheel_angle)*r_pinion;
                                                                
    l_ac = sqrt((ay + rack_disp)^2 + ax^2);
    alpha_delta = acosd((l_ac^2 + l_brazo^2 - l_tierod^2)/(2*l_brazo*l_ac)) - alpha_ini;
    ang_ac_delta = atan2d(ax, rack_disp + ay) - ang_ac_ini;

    extwheel_deg = (ang_ac_delta - alpha_delta);

%rueda interior
    rack_disp = -deg2rad(steering_wheel_angle)*r_pinion;
                                                                
    l_ac = sqrt((ay + rack_disp)^2 + ax^2);
    alpha_delta = acosd((l_ac^2 + l_brazo^2 - l_tierod^2)/(2*l_brazo*l_ac)) - alpha_ini;
    ang_ac_delta = atan2d(ax, rack_disp + ay) - ang_ac_ini;

    intwheel_deg = (ang_ac_delta + alpha_delta);



if steering_wheel_angle == 0
    intwheel_deg = 0; % No steering input results in no wheel angle change
    extwheel_deg = 0; % No steering input results in no wheel angle change
end

end
