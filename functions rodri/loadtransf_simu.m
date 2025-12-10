
%PARÁMETROS COCHE (datos cogido del FSAE con aero que hay en OptimumLap)
    m = 200 + 70; % peso coche + piloto [kg]
    g = 9.81; % gravedad [m/s^2]
    df_coeff = 4.18; % coeficiente downforce
    dr_coeff = 1.3; % coeficiente drag
    air_d = 1.225; % densidad del aire [kg/m^3]
    area = 0.563; % área frontal del coche [m^2]


%PARÁMETROS NEUMÁTICO
    coeff_long_max = 2.1; % coeficiente longitudinal máximo 
    coeff_lat_max = 1.9; % coeficiente lateral máximo
    sf= 1; % porcentaje del agarre máximo al que se llega
    lat_mu = sf*coeff_lat_max; % coeficiente lateral neumático
    long_mu = sf*coeff_long_max; % coeficiente longitudinal neumático




R = [3:1:25];
RL = zeros(1, length(R));
RR = zeros(1, length(R));
FL = zeros(1, length(R));
FR = zeros(1, length(R));
AccY = zeros(1, length(R));
Vel = zeros(1, length(R));
idx = 1;

for k = R
    
    v = sqrt(lat_mu*m*g/ abs(m/k - 0.5*lat_mu*air_d*area*df_coeff));
    ay = ((v)^2/k)/g;
    
    [fz_FR, fz_FL, fz_RR, fz_RL] = normal_load_per_tire_complete(270, ay, 1.25, 1.25, 0.07, ...
        0.08, 18370.86641/57.2958, 18370.86641/57.2958, 0.225, 1.6, 0.8, v);

    RL(idx) = fz_RL;
    RR(idx) = fz_RR;
    FL(idx) = fz_FL;
    FR(idx) = fz_FR;
    AccY(idx) = ay;
    Vel(idx) = v;
    idx = idx + 1;


end

RL 
RR 
FL
FR
AccY
Vel