clc
clear all

load("datos_circuito.mat")

%PARÁMETROS COCHE (datos cogido del FSAE con aero que hay en OptimumLap)
m = 200 + 70; % peso coche + piloto [kg]
df_coeff = 1.2; % coeficiente downforce
dr_coeff = 0.8; % coeficiente drag
air_d = 1.2; % densidad del aire [kg/m^3]
area = 1.1; % área frontal del coche [m^2]
tire_d = 16*0.0254; % diámetro de la rueda [m]
lat_mu = 1.5; % coeficiente lateral neumático
long_mu = 1.4; % coeficiente longitudinal neumático
rolling_res = 0.03; % coeficiente rolling resistance
gear_ratio = 12; % ratio reducción rpms transmi

%PARÁMETROS SIMU
v = 25; % velocidad inicial [m/s] (se ha cogido velocidad final de vuelta de una simu forward cualquiera)
d = 0.01; % intervalo para evaluación splines (como el vector radio se define a partir de esta evaluación, y los bucles iteran sobre el radio, acaba siendo el intervalo de simulación, en metros)
g = 9.81; % gravedad [m/s^2]
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

    % CÁLCULO VELOCIDAD MÁXIMA SEGÚN RADIO DE CURVA 
    % (filtrar valores recta)
    
    denom_vmax= m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff;
    
    if denom_vmax > 0
        v_max_curva = sqrt(lat_mu*m*g / denom_vmax);
    
    else
        v_max_curva = 1000; 
    end
    
    % CÁLCULO POTENCIA DISPONIBLE
    v_ang= v/(tire_d/2)*gear_ratio; % revoluciones motor a velocidad dada [rad/s]
    par = interp1(v_ang_vector, par_vector, v_ang); % encontrar par motor a velocidad dada [Nm]
    pwr = effcy*par*v_ang/1000; % potencia motor a velocidad dada [kW]
    
    % CÁLCULO CARGA VERTICAL TOTAL
    dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
    z_load = dForce + m*g; % cálculo fuerza vertical total [N]
    
    % CÁLCULO RESISTENCIA A AVANCE 
    dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
    rollingForce = z_load*rolling_res; % cálculo resistencia rodadura [N]
        
    % CÁLCULO GRIP LONGITUDINAL Y FUERZA LONGITUDINAL ACTUAL
    long_grip = z_load*long_mu/m; % cálculo grip longitudinal [m/s^2]
    long_force = pwr*1000/v; % cálculo fuerza longitudinal actual proveniente de motores [N]
    
    % CÁLCULO DE FUERZA NETA Y ACELERACIÓN
    netForce = long_force - dragForce - rollingForce; % fuerza neta [N]
    acceleration = netForce / m; % aceleración del coche [m/s^2]

    if v_max_curva < v
        v = v_max_curva; 

    elseif v < v_max_curva
        v = sqrt(v^2 + 2*min(long_grip, acceleration)*d);
    end

    v_resultante_forward(idx) = v;
    idx = idx + 1;

end


%BACKWARD LOOP
% vector velocidad backward + índice
v_resultante_backward = zeros(1, length(radio));
idx_b = 1;
radio_backward = flip(radio);
v = 25;

for r = radio_backward

    % CÁLCULO VELOCIDAD MÁXIMA SEGÚN RADIO DE CURVA (filtrar valores recta)
    denom_vmax = m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff;
    
    if denom_vmax > 0
        v_max_curva = sqrt(lat_mu*m*g / denom_vmax);
    
    else
        v_max_curva = 1000; 
    end
   
    % CÁLCULO CARGA VERTICAL TOTAL
    dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
    z_load = dForce + m*g; % cálculo fuerza vertical total [N]
    
    % CÁLCULO RESISTENCIA A AVANCE 
    dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
    rollingForce = z_load*rolling_res; % cálculo resistencia rodadura [N]
    resistant_force = dragForce + rollingForce; % fuerza resistente al avance total [N]
        
    % CÁLCULO GRIP LONGITUDINAL Y FUERZA LONGITUDINAL ACTUAL
    long_grip = z_load*long_mu/m; % cálculo grip longitudinal [m/s^2]
    
    if v_max_curva < v
        v = v_max_curva; 

    elseif v < v_max_curva
        v = sqrt(v^2 + 2*(long_grip + resistant_force/m)*d);
    end

    v_resultante_backward(idx_b) = v;
    idx_b = idx_b + 1;

end


        
v_resultante = min(v_resultante_forward, flip(v_resultante_backward));
lat_acc = v_resultante.^2./radio;
max(v_resultante)


figure(1)
plot(x_track, y_track)
axis equal
xlabel('x [m]')
ylabel('y [m]')

figure(2)
plot(d_interval, v_resultante)
xlabel('elapsed distance [m]')
ylabel('v_resultante')

figure(3)
plot(d_interval, lat_acc)
xlabel('elapsed distance [m]')
ylabel('lateral acceleration [m/s^2]')

figure(4)
plot(radio, lat_acc)
xlim([-50, 50])
xlabel('radio de curva')
ylabel('lateral acceleration [m/s^2]')


