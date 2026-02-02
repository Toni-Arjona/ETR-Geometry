close all
clear all
clc

% DATOS (distancias todas en mm)
% dimensiones coche
wheelbase = 1600;
front_track = 1150;
rear_track = 1250; 
r_pinion = 35/2; % radio pinion

% INPUTS geometría steering. Centro de coordenadas en KingPin rueda
% izquierda
l_rack = 450; %
x_rack = -67.969; % posición en eje x rack
y_tie_mangueta = 40;
x_tie_mangueta = 70;

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


intwheel_deg = [zeros(1, 121)];
extwheel_deg = [zeros(1, 121)];
extwheel_ack_deg = [zeros(1, 121)];
idx = 2;

for swheel_angle = 1:120 %rueda exterior
    rack_disp = deg2rad(swheel_angle)*r_pinion;
                                                                
    l_ac = sqrt((ay + rack_disp)^2 + ax^2);
    alpha_delta = alpha_ini- acosd((l_ac^2 + l_brazo^2 - l_tierod^2)/(2*l_brazo*l_ac));
    ang_ac_delta = ang_ac_ini - atan2d(ax, rack_disp + ay);

    extwheel_deg(idx) = abs(ang_ac_delta + alpha_delta);

    idx = idx + 1;

end



idx = 2;
for swheel_angle = 1:120 %rueda interior
    rack_disp = -deg2rad(swheel_angle)*r_pinion;
                                                                
    l_ac = sqrt((ay + rack_disp)^2 + ax^2);
    alpha_delta = alpha_ini- acosd((l_ac^2 + l_brazo^2 - l_tierod^2)/(2*l_brazo*l_ac));
    ang_ac_delta = ang_ac_ini - atan2d(ax, rack_disp + ay);

    intwheel_deg(idx) = abs(ang_ac_delta + alpha_delta);
    extwheel_ack_deg(idx) = atand(wheelbase/(wheelbase/tan(deg2rad(intwheel_deg(idx))) + rear_track));


    idx = idx + 1;

end

dynamic_toe_ack = intwheel_deg - extwheel_ack_deg ;
dynamic_toe =  intwheel_deg - extwheel_deg;
ack_pctge = dynamic_toe./dynamic_toe_ack;


figure(1)
plot(0:120, intwheel_deg);
hold on
plot(0:120, extwheel_deg);
legend('rueda interior', 'rueda exterior')
xlabel('Steering wheel angle [deg]')
ylabel('Wheel steer angle [deg]')

figure(2)
plot(0:120, ack_pctge*100);
xlabel('Steering wheel angle [deg]')
ylabel('Porcentaje de ackermann [%]')
ylim([-200, 200])
grid on

figure(3)
plot(0:120, dynamic_toe)
hold on
plot(0:120, dynamic_toe_ack)
grid on
legend('Dynamic toe', 'Dynamic toe Ackermann')
xlabel('Steering wheel angle [deg]')
ylabel('Toe [deg]')


