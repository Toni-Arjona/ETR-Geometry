function [FL_steer, FR_steer] = ackermann_function(steering_wheel_angle)
% Punto c, kingpin a la altura del tie, rueda izquierda.  Es el origen de coordenadas para el estudio. 
% Punto b, unión del tie con la mangueta.
% Punto a, unión del tie con el rack.
% y eje a lo ancho del coche, positiva hacia la derecha.
% x eje a lo largo del coche, positiva hacia delante del coche


% DATOS (distancias todas en mm)
% dimensiones coche
wheelbase = 1535;
front_track = 1150; %realmente no es el front track, sino la distancia entre kingpin izquierdo y derecho a la altura del tie.
rear_track = 1250;
r_pinion = 35/2; % radio pinion
rack_disp = deg2rad(steering_wheel_angle)*r_pinion;

% INPUTS geometría steering. Centro de coordenadas en KingPin rueda
% izquierda
l_rack = 450; %
x_rack = 0; % posición en eje x rack
y_tie_mangueta = 20;
x_tie_mangueta = 80;

% cálculo puntos y ángulos para steer_angle = 0
ax = x_rack;
ay = front_track/2 - l_rack/2;
bx = x_tie_mangueta;
by = y_tie_mangueta;

%% CÁLCULOS FUNCIÓN 
% longitud del brazo
l_brazo = @(rack_disp, ax, ay, bx, by) sqrt(bx^2 + by^2);

% longitud del tie
l_tierod = @(rack_disp, ax, ay, bx, by) sqrt((ay - by)^2 + (ax - bx)^2);

% longitud inicial del segmento AC
l_ac_ini = @(rack_disp, ax, ay, bx, by) sqrt(ay^2 + ax^2); 

% ángulo beta inicial (ángulo con la horizontal del segmento AC)
beta_ini =@(rack_disp, ax, ay, bx, by) atan2d(ax, ay); 

% ángulo alpha inicial (ángulo entre brazo y segmento AC)
alpha_ini = @(rack_disp, ax, ay, bx, by) acosd((l_ac_ini(rack_disp, ax, ay, bx, by)^2 + l_brazo(rack_disp, ax, ay, bx, by)^2 - l_tierod(rack_disp, ax, ay, bx, by)^2)/(2*l_brazo(rack_disp, ax, ay, bx, by)*l_ac_ini(rack_disp, ax, ay, bx, by)));; 
                                                              
% longitud del segmento ac en función del desplazamiento del rack
l_ac = @(rack_disp, ax, ay, bx, by) sqrt((ay + rack_disp)^2 + ax^2); 

% variación total del ángulo alpha
alpha_delta = @(rack_disp, ax, ay, bx, by) alpha_ini(rack_disp, ax, ay, bx, by) - acosd((l_ac(rack_disp, ax, ay, bx, by)^2 + l_brazo(rack_disp, ax, ay, bx, by)^2 - l_tierod(rack_disp, ax, ay, bx, by)^2)/(2*l_brazo(rack_disp, ax, ay, bx, by)*l_ac(rack_disp, ax, ay, bx, by)));

% variación total del ángulo beta
beta_delta = @(rack_disp, ax, ay, bx, by) beta_ini(rack_disp, ax, ay, bx, by) - atan2d(ax, rack_disp + ay);

% variación total del ángulo phi de la rueda. Rueda exterior para rack_disp > 0. Rueda interior para rack_dips < 0.
phi_delta = @(rack_disp, ax, ay, bx, by) abs(beta_delta(rack_disp, ax, ay, bx, by) + alpha_delta(rack_disp, ax, ay, bx, by));

if steering_wheel_angle > 0

    FL_steer = phi_delta(rack_disp, ax, ay, bx, by);
    FR_steer = phi_delta(-rack_disp, ax, ay, bx, by);

else 
    FL_steer = -phi_delta(rack_disp, ax, ay, bx, by); 
    FR_steer = -phi_delta(-rack_disp, ax, ay, bx, by);
end


end
