clear all
clc
close all

car = struct(); % Definiciones del coche

g=9.81; %m/s^2

car.l=1.6; % Wheelbase [m]
car.a = 0.8;% Distancia CG a front axle [m]
car.b = car.l-car.a;% Distancia Cg a rear axle [m]
car.Tf = 1.25;% Trackwith front [m]
car.Tr = 1.25;% Trackwith rear [m]
car.m = 205+70; % Pes coche + piloto [kg]
car.Zrf=0.07; % Roll center height front [m]
car.Zrr=0.08; % Roll center height rear [m]
car.H=0.225; % distancia CG a roll axis [m]
car.Kf= 18370.86641; % roll stiffnes front [Nm/rad]
car.Kr= 18370.86641; % roll stiffnes rear [Nm/rad]
car.h= 0.3; 
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

steer_int= 0:5:80; % Intervalo Steering wheel angle [rad]
yaw_slip_int = deg2rad(0:0.5:9); % Intervalo Slip de CG en [rad]

Wf = g*car.m*car.b/(car.l); % peso sobre eje delantero [N]
Wr = g*car.m*car.a/(car.l); % peso sobre eje trasero [N]

idx_steer = 1;
idx_yawslip = 1;
yaw_moment_matrix = zeros(length(steer_int), length(yaw_slip_int));
ay_matrix = zeros(length(steer_int), length(yaw_slip_int));

v = 0.1; % v inicial
ay = sqrt(v*R)/g; % lat acc
yaw_vel = v/R


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
            FL_slip =  (Vy + yaw_vel*car.a)/(Vx - (yaw_vel*car.Tf/2)) - FL_steer; % front left slip angle [rad]
            FR_slip =  (Vy + yaw_vel*car.a)/(Vx + (yaw_vel*car.Tf/2)) - FR_steer; % front right slip angle [rad]
            RL_slip =  (Vy - yaw_vel*car.b)/(Vx - (yaw_vel*car.Tr/2)); % rear left slip angle [rad]
            RR_slip =  (Vy - yaw_vel*car.b)/(Vx + (yaw_vel*car.Tr/2)); % rear right slip angle [rad]
 
             % CAMBER GAIN
            roll = (car.h*car.m*ay)/(car.Kf + car.Kr); % roll [rads]
            camber_FL = gammaF0 + rolltocamber_ratio*roll ; % camber total Front Left [rads]
            camber_FR = gammaF0 - rolltocamber_ratio*roll ; % camber total Front Right [rads]
            camber_RL = gammaR0 + rolltocamber_ratio*roll ; % camber total Rear Left [rads]
            camber_RR = gammaR0 - rolltocamber_ratio*roll ; % camber total Rear Right [rads]
           
            %WEIGHT TRASNFER
            [FZ_FR, FZ_FL, FZ_RR, FZ_RL]= normal_load_per_tire_complete(car.m, ay, car.Tf, car.Tf, car.Zrf, car.Zrr, car.Kf, car.Kr, car.H, car.l, car.a); % llamada función laod transfer, devuelve carga sobre cada rueda, sin contar downforce

            % TOTAL LOADS
            FZ_FR = FZ_FR + (dForce_front + drag_front)/2 ;% total load on front right [N]
            FZ_FL = FZ_FL + (dForce_front + drag_front)/2; % total load on front left [N]
            FZ_RR = FZ_RR + (dForce_rear + drag_rear)/2; % total load on rear right [N]
            FZ_RL = FZ_RL + (dForce_rear + drag_rear)/2; % total load on rear left [N]

            % EACH TIRE'S FY AND MZ
            [FY_FL, MZ_FL] = tire_model_function(FL_slip, camber_FL, FZ_FL); % Front left Fy and Mz
            [FY_FR, MZ_FR] = tire_model_function(FR_slip, camber_FR, FZ_FR);% Front right Fy and Mz
            [FY_RL, MZ_RL] = tire_model_function(RL_slip, camber_RL, FZ_RL); % Rear left Fy and Mz
            [FY_RR, MZ_RR] = tire_model_function(RR_slip, camber_RR, FZ_RR); % Rear right Fy and Mz

            % YAW AND AY CALCULATIONS
            ay = (FY_RR + FY_RL + FY_FR*cos(FR_steer) + FY_FL*cos(FL_steer))/(car.m *g); % aceleración lateral resultante [g]
            yaw_moment = ((FY_FR*cos(FR_steer) + FY_FL*cos(FL_steer))*car.a - (FY_RR + FY_RL)*car.b - (MZ_RR + MZ_RL + MZ_FL + MZ_FR));
            v = sqrt(abs(ay*g*R));

        end
       
        yaw_moment_matrix(idx_steer, idx_yawslip) = yaw_moment ;
        ay_matrix(idx_steer, idx_yawslip) = ay;
        idx_yawslip = idx_yawslip + 1;
      
    end
    idx_yawslip = 1;
    idx_steer = idx_steer + 1;
end



plot(ay_matrix, yaw_moment_matrix)
hold on
plot(ay_matrix', yaw_moment_matrix')





%{
%% 1. Preparación de datos
steer_vec_deg = rad2deg(steer_int);      
slip_vec_deg  = rad2deg(yaw_slip_int);   

[n_slip, n_steer] = size(ay_matrix);

%% 2. Configuración de la Figura (MODO OSCURO)
figure('Color', 'k'); 
ax = gca;
set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
set(ax, 'GridColor', 'w', 'GridAlpha', 0.3);
hold on; grid on;

title('Diagrama de Manejo: Mz vs Ay', 'Color', 'w', 'FontSize', 12);
xlabel('Aceleración Lateral A_y [m/s^2]', 'Color', 'w');
ylabel('Momento de Guiñada M_z [Nm]', 'Color', 'w');

%% 3. Graficar Líneas de STEER (Colores)
colormap(turbo(n_steer)); 
colores = colormap;

for i = 1:n_steer
    plot(ay_matrix(:, i), yaw_moment_matrix(:, i), '-', ...
         'LineWidth', 1.5, 'Color', colores(i, :));
end

% Barra de colores
c = colorbar;
c.Color = 'w'; 
c.Label.String = 'Ángulo de Volante [deg]';
c.Label.Color = 'w';
clim([min(steer_vec_deg), max(steer_vec_deg)]); 

%% 4. Graficar Isolíneas de YAW SLIP con etiquetas divididas
% Color gris muy claro para las líneas transversales
color_slip = [0.8 0.8 0.8]; 

for j = 1:n_slip
    % Graficar la línea completa
    plot(ay_matrix(j, :), yaw_moment_matrix(j, :), '--', ...
         'Color', color_slip, 'LineWidth', 1.0); 
     
    % --- LÓGICA DE POSICIÓN DE ETIQUETAS ---
    % Calculamos la media de Mz para saber si la línea está arriba o abajo
    mz_promedio = mean(yaw_moment_matrix(j, :));
    
    if mz_promedio >= 0
        % PARTE SUPERIOR -> ETIQUETA A LA IZQUIERDA (Columna 1)
        idx = 1; 
        alineacion = 'right'; % El texto acaba en el punto (se expande hacia la izquierda)
        offset_x = -0.2; % Un pequeño margen extra a la izquierda
    else
        % PARTE INFERIOR -> ETIQUETA A LA DERECHA (Columna final)
        idx = n_steer;
        alineacion = 'left'; % El texto empieza en el punto (se expande hacia la derecha)
        offset_x = 0.2; % Un pequeño margen extra a la derecha
    end
    
    % Coordenadas del punto elegido
    x_pos = ay_matrix(j, idx); 
    y_pos = yaw_moment_matrix(j, idx);
    
    % Escribir etiqueta
    txt = sprintf('%.1f°', slip_vec_deg(j));
    text(x_pos, y_pos, txt, 'FontSize', 8, 'Color', 'w', ...
         'HorizontalAlignment', alineacion, 'VerticalAlignment', 'middle');
end

axis tight;
hold off;
           
%}
           



