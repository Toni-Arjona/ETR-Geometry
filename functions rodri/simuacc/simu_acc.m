close all
clear all
clc
%function tiempo_acc = simu_acc(gear_ratio)
curva_pot = readtable("C:\Users\rodri\OneDrive\Escritorio\ETECH\GitHub\ETR-Geometry\functions rodri\datos\PAR RPM.xlsx");


par_vector = curva_pot.P;
v_ang_vector = curva_pot.W;
par_vector(diff(v_ang_vector)==0) = []; % limpiar valores de par correspondientes a valores de v_ang repetidos
v_ang_vector(diff(v_ang_vector)==0) = []; % limpiar valores de v_ang repetidos


load('datos_circuito.mat')
g = 9.81; % gravedad [m/s^2]

%PARÁMETROS COCHE 
m = 200 + 70; % peso coche + piloto [kg]


% AERO
df_coeff = 4.18; % coeficiente downforce
dr_coeff = 1.3; % coeficiente drag
air_d = 1.225; % densidad del aire [kg/m^3]
area = 0.563; % área frontal del coche [m^2]

%TRANSMI
gear_ratio =  10.8;% ratio reducción rpms transmisión
effcy = 0.9; % eficiencia transmi

% NEUMÁTICO
tire_radius = effective_rolling_radius(m*g/4, 0.827); % radio efectivo de la rueda [m]    
camber_deg = -2;
slip_ratio = 1.08;
v = 0.01; % v inicial muy pequeña, para v = 0 peta código


d_interval = 0.01; % intervalo simulación
d_total = 0:d_interval:75;  % distancia total acc dividida en intervalo simu
v_resultante = zeros(1, length(d_total)); % creación vector v resultante
idx = 1;


for x = d_total

    v_ang= v/(tire_radius)*gear_ratio; % revoluciones motor a velocidad dada [rad/s]
    par = interp1(v_ang_vector, par_vector, v_ang); % encontrar par motor a velocidad dada [Nm]
    pwr = effcy*par*v_ang/1000; % potencia motor a velocidad dada [kW]
    
    % Limitación motor a 40kW (máximo total con dos motores 80kW)
        if pwr > 20*effcy
            pwr = 20*effcy;
        end
    
    pwrv(idx) = pwr/effcy; % entrega de potencia total

    % CÁLCULO CARGA VERTICAL TOTAL
    dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
    z_load = dForce + m*g; % cálculo fuerza vertical total [N]

    % CÁLCULO GRIP LONGITUDINAL MÁXIMO Y FUERZA MOTOR 
    Long_force_coef = longitudinal_force(m, z_load/g, effective_rolling_radius(z_load/(4*g), 0.827), camber_deg, slip_ratio, v, 0.827);
    long_grip = z_load*Long_force_coef/m; % cálculo grip longitudinal máx [m/s^2]
    motor_acc = 4*pwr*1000/(m*v); % cálculo aceleración (longitudinal) actual proveniente de motores [m/s^2]
   

    % CÁLCULO RESISTENCIA A AVANCE (drag + rolling resistance)
    dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
    rollingForce = -z_load*rolling_resistance_coeff(z_load/g, camber_deg, 0.827, v, tire_radius, 1.08); % cálculo resistencia rodadura [N]

    % CÁLCULO ACELERACIÓN LONGITUDINAL NETA (MOTOR - PÉRDIDAS)
    net_acc = min(motor_acc, long_grip) - (dragForce + rollingForce)/(m); % aceleración neta [m/s^2]
    net_accv(idx) = net_acc; % vector que almacena aceleración neta en cada instante


    v = sqrt(v^2 + 2*net_acc*d_interval);

    v_resultante(idx) = v;

    idx = idx + 1;

end

dt = d_interval ./ v_resultante;
tiempo_acc = sum(dt)



%end


figure(1)
plot(d_total, net_accv/g)


figure(2)
plot(d_total, v_resultante)

figure(3)
plot(d_total, 4*pwrv)





            
