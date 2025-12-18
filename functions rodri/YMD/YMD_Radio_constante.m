clear all
clc
close all

car = struct(); % Definiciones del coche

g=9.81; %m/s^2

car.l=1.6; % Wheelbase [m]
car.a = 0.5*car.l;% Distancia CG a front axle [m]
car.b = car.l-car.a;% Distancia Cg a rear axle [m]
car.Tf = 1.25;% Trackwith front [m]
car.Tr = 1.25;% Trackwith rear [m]
car.m = 205+70; % Pes coche + piloto [kg]
car.Zrf=0.07; % Roll center height front [m]
car.Zrr=0.08; % Roll center height rear [m]
car.H=0.225; % distancia CG a roll axis [m]
car.Kf= 18370.86641; % roll stiffnes front [Nm/rad]
car.Kr= 18370.86641; % roll stiffnes rear [Nm/rad]
car.h= 0.26; 
car.copx=475/1600; % centre de pressions en x (porcentaje de wheelbase desde eje delantero)
car.copz = 0.595; % altura centro de presiones [m]

rolltocamber_ratio = 0.58984220783; % ratio rads camber gain por rad de roll
gammaF0= deg2rad(2); % Front Camber deg abs value (real negativo) [rad]
gammaR0= deg2rad(1.5); % Rear Camber deg abs value (real negativo) [rad]

% AERO
df_coeff = 4.18; % coeficiente downforce
dr_coeff = 1.3; % coeficiente drag
air_d = 1.225; % densidad del aire [kg/m^3]
area = 0.563; % área frontal del coche [m^2]



% Inputs
R = 10; % Radio de giro [m]

steer_int= 10:5:50; % Intervalo Steering wheel angle [rad]
yaw_slip_int = deg2rad(0:0.5:12); % Intervalo Slip de CG en [rad]

Wf = g*car.m*car.b/(car.l); % peso sobre eje delantero [N]
Wr = g*car.m*car.a/(car.l); % peso sobre eje trasero [N]

idx_steer = 1;
idx_yawslip = 1;
yaw_moment_matrix = zeros(length(steer_int), length(yaw_slip_int));
ay_matrix = zeros(length(steer_int), length(yaw_slip_int));

v = 0.0001; % v inicial
ay = sqrt(v*R)/g; % lat acc
yaw_vel = v/R;

ay_maximum=0;
yaw_moment_ay_max=0;
ay_minimum=0;
yaw_moment_ay_min=0;

for steer = steer_int

    for yaw_slip = yaw_slip_int
    [FL_steer, FR_steer] = ackermann_function(steer); % llamada función ackermann, devuelve steer angle rueda iquierda y derecha
    FL_steer = deg2rad(FL_steer);
    FR_steer = deg2rad(FR_steer);
        for k = 1:10
            
            Vx = cos(yaw_slip)*v; % Velocidad x
            Vy = sin(yaw_slip)*v; % Velocidad y
            yaw_vel = v/R;

            % DOWNFORCE Y DRAG SOBRE CADA EJE
            dForce = 0.5*air_d*area*df_coeff*v^2; % downforce total [N]
            dForce_front = dForce*car.a*car.copx; % downforce eje delantero [N]
            dForce_rear = dForce*car.b*(1-car.copx); % downforce eje trasero [N]
            drag = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
            drag_front = - drag*car.copz/car.l; % carga que aporta el drag del eje delantero [N] (es negativa, por lo que realmente es carga que quita)
            drag_rear = - drag_front; % carga que aporta el drag sobre el eje trasero [N] (es positiva, por lo que aporta carga, exactamente la carga que se quita del eje delantero)

            % SLIPS NEUMÁTICO             
            FL_slip =  min(atan2((Vy + yaw_vel*car.a), (Vx + (yaw_vel*car.Tf/2))) - FL_steer, 15/57); % front left slip angle [rad]
            FR_slip =  min(atan2((Vy + yaw_vel*car.a), (Vx - (yaw_vel*car.Tf/2)))- FR_steer, 15/57); % front right slip angle [rad]
            RL_slip =  min(atan2((Vy - yaw_vel*car.b), (Vx + (yaw_vel*car.Tr/2))), 15/57); % rear left slip angle [rad]
            RR_slip =  min(atan2((Vy - yaw_vel*car.b), (Vx - (yaw_vel*car.Tr/2))), 15/57); % rear right slip angle [rad]
        

            % CAMBER GAIN
            roll = (car.h*car.m*ay)/(car.Kf + car.Kr); % roll [rads]
            camber_FL = gammaF0 - rolltocamber_ratio*roll ; % camber total Front Left [rads]
            camber_FR = gammaF0 + rolltocamber_ratio*roll ; % camber total Front Right [rads]
            camber_RL = gammaR0 - rolltocamber_ratio*roll ; % camber total Rear Left [rads]
            camber_RR = gammaR0 + rolltocamber_ratio*roll ; % camber total Rear Right [rads]
           
            %WEIGHT TRASNFER
            [FZ_FR, FZ_FL, FZ_RR, FZ_RL]= normal_load_per_tire_complete(car.m, ay, car.Tf, car.Tf, car.Zrf, car.Zrr, car.Kf, car.Kr, car.H, car.l, car.a); % llamada función laod transfer, devuelve carga sobre cada rueda, sin contar downforce

            % TOTAL LOADS
            FZ_FR = FZ_FR + (dForce_front + drag_front)/2 ;% total load on front right [N]
            FZ_FL = FZ_FL + (dForce_front + drag_front)/2; % total load on front left [N]
            FZ_RR = FZ_RR + (dForce_rear + drag_rear)/2; % total load on rear right [N]
            FZ_RL = FZ_RL + (dForce_rear + drag_rear)/2; % total load on rear left [N]

            % EACH TIRE'S FY AND MZ
            [FY_FL, MZ_FL] = tire_model_function(FL_slip, -camber_FL, FZ_FL); % Front left Fy and Mz
            [FY_FR, MZ_FR] = tire_model_function(FR_slip, camber_FR, FZ_FR);% Front right Fy and Mz
            [FY_RL, MZ_RL] = tire_model_function(RL_slip, -camber_RL, FZ_RL); % Rear left Fy and Mz
            [FY_RR, MZ_RR] = tire_model_function(RR_slip, camber_RR, FZ_RR); % Rear right Fy and Mz

            % YAW AND AY CALCULATIONS
            ay = (FY_RR + FY_RL + FY_FR*cos(FR_steer) + FY_FL*cos(FL_steer))/(car.m *g); % aceleración lateral resultante [g]
            yaw_moment = ((FY_FR*cos(FR_steer) + FY_FL*cos(FL_steer))*car.a - (FY_RR + FY_RL)*car.b - (MZ_RR + MZ_RL + MZ_FL + MZ_FR));
            v = sqrt(abs(ay*g*R));
            
            if ay> ay_maximum
                ay_maximum = ay; % Update maximum lateral acceleration
                yaw_moment_ay_max=yaw_moment;
            elseif ay< ay_minimum
                ay_minimum=ay;
                yaw_moment_ay_min=yaw_moment;
            end

        end
       
        yaw_moment_matrix(idx_steer, idx_yawslip) = yaw_moment ;
        ay_matrix(idx_steer, idx_yawslip) = ay;
        idx_yawslip = idx_yawslip + 1;
      
    end
    idx_yawslip = 1;
    idx_steer = idx_steer + 1;
end
ay_maximum 
yaw_moment_ay_max
ay_minimum 
yaw_moment_ay_min

figure(1)
plot(ay_matrix, yaw_moment_matrix, '.')
hold on
plot(ay_matrix', yaw_moment_matrix', '.')
grid on



% --- Configuración de Colores y Gráfico ---

n_steer = length(steer_int);
n_slip = length(yaw_slip_int);

% Mapas de color personalizados (similares a la foto 2)
cmap_steer = [zeros(n_steer,1), linspace(1,0.5,n_steer)', linspace(0.5,1,n_steer)']; 
cmap_slip = [ones(n_slip,1), linspace(0,1,n_slip)', zeros(n_slip,1)];

fig = figure('Color', [0.1 0.1 0.1]); % Fondo oscuro
ax1 = axes('Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
hold on; grid on;
set(ax1, 'GridColor', [0.3 0.3 0.3], 'MinorGridColor', [0.2 0.2 0.2]);

% Plot isolíneas de Steer (Sólidas)
for i = 1:n_steer
    plot(ax1, ay_matrix(i, :), yaw_moment_matrix(i, :), 'Color', cmap_steer(i,:), 'LineWidth', 1.5);
end

% Plot isolíneas de Slip (Discontinuas)
for j = 1:n_slip
    plot(ax1, ay_matrix(:, j), yaw_moment_matrix(:, j), '--', 'Color', cmap_slip(j,:), 'LineWidth', 1);
end

xlabel('Aceleración Lateral (a_y) [g]');
ylabel('Momento de Guiñada [N·m]');
title('YMD con Mapas de Color', 'Color', 'w');

% --- Gestión de las dos Colorbars ---

% Barra 1: Steer
colormap(ax1, cmap_steer);
cb1 = colorbar(ax1, 'Location', 'none');
cb1.Label.String = 'Ángulo Volante (Steer) [deg]';
cb1.Label.Color = 'w'; cb1.Color = 'w';
cb1.Ticks = linspace(0, 1, 8);
cb1.TickLabels = string(round(linspace(min(steer_int), max(steer_int), 8)));

% Barra 2: Slip (usando un eje invisible para el segundo colormap)
ax2 = axes('Position', ax1.Position, 'Visible', 'off');
colormap(ax2, cmap_slip);
cb2 = colorbar(ax2, 'Location', 'none');
cb2.Label.String = 'Ángulo Deriva (Slip) [deg]';
cb2.Label.Color = 'w'; cb2.Color = 'w';
cb2.Ticks = linspace(0, 1, 7);
cb2.TickLabels = string(round(rad2deg(linspace(min(yaw_slip_int), max(yaw_slip_int), 7)), 1));

% --- Ajuste de Posiciones (Layout final) ---
ax1.Position = [0.10 0.15 0.65 0.75]; % Espacio para el gráfico
cb1.Position = [0.82 0.15 0.025 0.75]; % Posición barra Steer
cb2.Position = [0.91 0.15 0.025 0.75]; % Posición barra Slip