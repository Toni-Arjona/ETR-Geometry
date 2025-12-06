clc
clear all

load("datos_circuito.mat")

%PARÁMETROS COCHE (datos cogido del FSAE con aero que hay en OptimumLap)
m = 200 + 70; % peso coche + piloto [kg]
g = 9.81; % gravedad [m/s^2]
df_coeff = 1.2; % coeficiente downforce
dr_coeff = 0.8; % coeficiente drag
air_d = 1.2; % densidad del aire [kg/m^3]
area = 1.1; % área frontal del coche [m^2]
tire_d = 2*effective_rolling_radius(m*g/4, 0.827); % diámetro de la rueda [m]
lat_mu = 1.6; % coeficiente lateral neumático
long_mu = 1.7; % coeficiente longitudinal neumático
rolling_res = 0.01035; % coeficiente rolling resistance
gear_ratio = 12; % ratio reducción rpms transmi

%PARÁMETROS NEUMÁTICO
coeff_long_max = 2.1;
coeff_lat_max = 1.9;

%PARÁMETROS SIMU
v = 23.6875; % velocidad inicial [m/s] (se ha cogido velocidad final de vuelta de una simu forward cualquiera)
d = 0.01; % intervalo para evaluación splines (como el vector radio se define a partir de esta evaluación, y los bucles iteran sobre el radio, acaba siendo el intervalo de simulación, en metros)

effcy = 0.95; %eficiencia transmi (ni puta idea, eso ponía predeterminado en el optimum)

% vectores velocidad angular y par motor
par_vector = curva_motor.P;
v_ang_vector = curva_motor.W;
par_vector(diff(v_ang_vector)==0) = []; % limpiar valores de par correspondientes a valores de v_ang repetidos
v_ang_vector(diff(v_ang_vector)==0) = []; % limpiar valores de v_ang repetidos

% DEFINICIÓN TRAYECTO CIRCUITO CON SPLINES
%distancia recorrida total e intervalo de evaluación para splines
d_total = [0; cumsum(sqrt(diff(circuito.X).^2 + diff(circuito.Y).^2))];
d_interval = 0: d : max(d_total);


% definir exprsión splines circuito x e y, y evalurar para intervalo simu
x_spline = csaps(d_total, circuito.X, 1); x_track = ppval(x_spline, d_interval);
y_spline = csaps(d_total, circuito.Y, 1); y_track = ppval(y_spline, d_interval);


% DEFINICIÓN RADIO INSTANTÁNEO DE CURVATURA
% derivadas primeras y segundas trayectoria y evaluación de expresiones
dx =  fnder(x_spline, 1); dx_eval =  ppval(dx,  d_interval); 
dy =  fnder(y_spline, 1); dy_eval =  ppval(dy,  d_interval);
ddx = fnder(x_spline, 2); ddx_eval = ppval(ddx, d_interval);
ddy = fnder(y_spline, 2); ddy_eval = ppval(ddy, d_interval);


% Calcular la curvatura y radio del circuito en cada instante
curvatura = (dx_eval .* ddy_eval - dy_eval .* ddx_eval) ./ (dx_eval.^2 + dy_eval.^2).^(3/2);
radio = 1./curvatura; % vector radio [m]

%FORWARD LOOP
% vector velocidad resultant forward + índice idx
v_resultante_forward = zeros(1, length(radio));
idx = 1;

for r = radio
    % CÁLCULO POTENCIA DISPONIBLE
    v_ang= v/(tire_d/2)*gear_ratio; % revoluciones motor a velocidad dada [rad/s]
    par = interp1(v_ang_vector, par_vector, v_ang); % encontrar par motor a velocidad dada [Nm]
    pwr = effcy*par*v_ang/1000; % potencia motor a velocidad dada [kW]
    
    if pwr > 40
        pwr = 40;
    end

    % CÁLCULO VELOCIDAD MÁXIMA SEGÚN RADIO DE CURVA 
    v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));
    
   
    % CÁLCULO CARGA VERTICAL TOTAL
    dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
    z_load = dForce + m*g; % cálculo fuerza vertical total [N]
    
    % CÁLCULO RESISTENCIA A AVANCE 
    dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
    rollingForce = -z_load*rolling_resistance_coeff(m, -2, 0.827, v, tire_d/2, 1.08); % cálculo resistencia rodadura [N]

    % CÁLCULO GRIP LONGITUDINAL Y FUERZA LONGITUDINAL ACTUAL
    long_grip = z_load*long_mu/m; % cálculo grip longitudinal [m/s^2]
    motor_acc = 2*pwr*1000/(m*v); % cálculo aceleración (longitudinal) actual proveniente de motores [m/s^2]
    
    % CÁLCULO DE FUERZA NETA Y ACELERACIÓN
    net_acc = motor_acc - (dragForce + rollingForce)/m; % aceleración neta


    if v_max_curva < v
        v = v_max_curva; 


    elseif v < v_max_curva
        v = sqrt(v^2 + 2*min(0.5*long_grip, net_acc)*d);
    end

    v_resultante_forward(idx) = v;
    idx = idx + 1;

end


%BACKWARD LOOP
% vector velocidad backward + índice
v_resultante_backward = zeros(1, length(radio));
idx_b = 1;
radio_backward = flip(radio);
v = v_resultante_forward(end);

for r = radio_backward

    % CÁLCULO VELOCIDAD MÁXIMA SEGÚN RADIO DE CURVA (filtrar valores recta)
    v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));
   
    % CÁLCULO CARGA VERTICAL TOTAL
    dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
    z_load = dForce + m*g; % cálculo fuerza vertical total [N]
    
    % CÁLCULO RESISTENCIA A AVANCE 
    dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
    rollingForce = -z_load*rolling_resistance_coeff(m, -2, 0.827, v, tire_d/2, 1.08); % cálculo resistencia rodadura [N]
    resistant_acc = (dragForce + rollingForce)/m; % fuerza resistente al avance total [N]
        
    % CÁLCULO GRIP LONGITUDINAL Y FUERZA LONGITUDINAL ACTUAL
    long_grip = z_load*long_mu/m; % cálculo grip longitudinal [m/s^2]
    
    if v_max_curva < v
        v = v_max_curva; 

    elseif v < v_max_curva
        v = sqrt(v^2 + 2*(0.5*long_grip  + resistant_acc)*d);
    end

    v_resultante_backward(idx_b) = v;
    idx_b = idx_b + 1;

end


        
v_resultante = min(v_resultante_forward, flip(v_resultante_backward));
lat_acc = v_resultante.^2./radio;


dt = d ./ v_resultante; 
tiempo_total = sum(dt)
g_long = [diff(v_resultante),0]./dt;


figure(1)
% Crear vector Z de ceros para engañar a la función surface (pintar en 2D)
z = zeros(size(x_track)); 

% Pintar la línea usando la velocidad como mapa de color
surface([x_track; x_track], [y_track; y_track], [z; z], [v_resultante*3.6; v_resultante*3.6], ...
        'facecol', 'no', ...
        'edgecol', 'interp', ...
        'linew', 2); % Grosor de línea

colorbar; % Añade la barra lateral de leyenda
c = colorbar;
c.Label.String = 'Velocidad [Km/h]';
colormap(turbo); % 'turbo', 'jet' o 'parula' son buenas paletas
axis equal
xlabel('x [m]')
ylabel('y [m]')
title('Mapa de Velocidad')

figure(2)
% Crear vector Z de ceros para engañar a la función surface (pintar en 2D)
z = zeros(size(x_track)); 

%Pintar la línea usando la velocidad como mapa de color
surface([x_track; x_track], [y_track; y_track], [z; z], [lat_acc/g; lat_acc/g], ...
        'facecol', 'no', ...
        'edgecol', 'interp', ...
        'linew', 2); % Grosor de línea

colorbar; % Añade la barra lateral de leyenda
c = colorbar;
c.Label.String = 'Lateral acceleration [g]';
colormap(turbo); % 'turbo', 'jet' o 'parula' son buenas paletas
axis equal
xlabel('x [m]')
ylabel('y [m]')
title('Mapa de aceleración lateral')

figure(3)
% Crear vector Z de ceros para engañar a la función surface (pintar en 2D)
z = zeros(size(x_track)); 

%Pintar la línea usando la velocidad como mapa de color
surface([x_track; x_track], [y_track; y_track], [z; z], [g_long; g_long], ...
        'facecol', 'no', ...
        'edgecol', 'interp', ...
        'linew', 2); % Grosor de línea

colorbar; % Añade la barra lateral de leyenda
c = colorbar;
c.Label.String = 'Longitudinal acceleration [g]';
colormap(turbo); % 'turbo', 'jet' o 'parula' son buenas paletas
axis equal
xlabel('x [m]')
ylabel('y [m]')
title('Mapa de aceleración longitudinal')


figure(4)
plot(d_interval, v_resultante)
xlabel('elapsed distance [m]')
ylabel('v_resultante')


figure(5)
plot(d_interval, lat_acc)
xlabel('elapsed distance [m]')
ylabel('lateral acceleration [m/s^2]')


figure(6)
plot(d_interval, g_long/g)
xlabel('elapsed distance [m]')
ylabel('longitudinal acceleration [g]')
ylim([-2.5, 1.5])


figure(7)
plot(d_interval, lat_acc/g)
xlabel('elapsed distance [m]')
ylabel('lateral acceleration [g]')

figure(8)
plot(d_interval, radio)
ylim([-200, 200])

