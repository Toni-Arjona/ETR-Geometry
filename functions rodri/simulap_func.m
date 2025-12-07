function tiempo_total = simulap_func(gear_ratio)

    load("datos_circuito.mat")
    
    %PARÁMETROS COCHE (datos cogido del FSAE con aero que hay en OptimumLap)
    m = 200 + 70; % peso coche + piloto [kg]
    g = 9.81; % gravedad [m/s^2]
    df_coeff = 4.18; % coeficiente downforce
    dr_coeff = 1.3; % coeficiente drag
    air_d = 1.2; % densidad del aire [kg/m^3]
    area = 0.563; % área frontal del coche [m^2]
    tire_d = 2*effective_rolling_radius(m*g/4, 0.827); % diámetro de la rueda [m]
    rolling_res = 0.01035; % coeficiente rolling resistance
     % ratio reducción rpms transmi
    
    %PARÁMETROS NEUMÁTICO
    coeff_long_max = 2.1;
    coeff_lat_max = 1.9;
    sf= 0.9;
    lat_mu = sf*coeff_lat_max; % coeficiente lateral neumático
    long_mu = sf*coeff_long_max; % coeficiente longitudinal neumático
    %Parametros iniciales
    lat_mu_available= lat_mu;
    long_mu_available= long_mu;
    
    %PARÁMETROS SIMU
    v = 23.6875; % velocidad inicial [m/s] (se ha cogido velocidad final de vuelta de una simu forward cualquiera)
    d = 0.01; % intervalo para evaluación splines (como el vector radio se define a partir de esta evaluación, y los bucles iteran sobre el radio, acaba siendo el intervalo de simulación, en metros)
    
    effcy = 0.9; %eficiencia transmi 
    
    % vectores velocidad angular y par motor
    par_vector = curva_motor.P;
    v_ang_vector = curva_motor.W;
    par_vector(diff(v_ang_vector)==0) = []; % limpiar valores de par correspondientes a valores de v_ang repetidos
    v_ang_vector(diff(v_ang_vector)==0) = []; % limpiar valores de v_ang repetidos
    
    % DEFINICIÓN TRAYECTO CIRCUITO CON SPLINES
    %distancia recorrida total e intervalo de evaluación para splines
    d_total = [0; cumsum(sqrt(diff(0.8*circuito.X).^2 + diff(0.8*circuito.Y).^2))];
    d_interval = 0: d : max(d_total);
    
    
    % definir exprsión splines circuito x e y, y evalurar para intervalo simu
    x_spline = csaps(d_total, 0.8*circuito.X, 1); x_track = ppval(x_spline, d_interval);
    y_spline = csaps(d_total, 0.8*circuito.Y, 1); y_track = ppval(y_spline, d_interval);
    
    
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
        
        % CÁLCULO CARGA VERTICAL TOTAL
        dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
        z_load = dForce + m*g; % cálculo fuerza vertical total [N]
       
        % CÁLCULO GRIP LONGITUDINAL Y FUERZA LONGITUDINAL ACTUAL
        long_grip = z_load*long_mu/m; % cálculo grip longitudinal máx [m/s^2]
        motor_acc = 2*pwr*1000/(m*v); % cálculo aceleración (longitudinal) actual proveniente de motores [m/s^2]
       
        % LIMITAR FUERZA LONGITUDINAL (elipse rozamiento)  [m/s^2]
        if long_grip < 4618/m
            long_grip= long_grip;
        else
            long_grip= 4618/m;
        end 
    
        % CÁLCULO VELOCIDAD MÁXIMA SEGÚN RADIO DE CURVA 
        v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));
        
        % CÁLCULO RESISTENCIA A AVANCE 
        dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
        rollingForce = -z_load*rolling_resistance_coeff(m, -2, 0.827, v, tire_d/2, 1.08); % cálculo resistencia rodadura [N]
    
        
        % CÁLCULO DE FUERZA NETA Y ACELERACIÓN
        net_acc = motor_acc - (dragForce + rollingForce)/m; % aceleración neta
    
        % LIMITAR VELOCIDAD
        if v_max_curva < v
            v = v_max_curva; 
    
    
        elseif v < v_max_curva
            long_grip_available= sqrt(abs(1-((v^2./abs(r))./(lat_mu*z_load./m)).^2))*long_grip;
            v = min(sqrt(v^2 + 2*min(0.5*long_grip_available, net_acc)*d), v_max_curva);
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
        
        % CÁLCULO CARGA VERTICAL TOTAL
        dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
        z_load = dForce + m*g; % cálculo fuerza vertical total [N]
    
        % CÁLCULO GRIP LATERAL Y LONGITUDINAL
        long_grip = z_load*long_mu/m; % cálculo grip longitudinal max [m/s^2]
    
        % LIMITAR FUERZA LONGITUDINAL (sauración neumático) [m/s^2]
        if long_grip < 4618/m
            long_grip= long_grip;
        else
            long_grip= 4618/m;
        end 
    
        % CÁLCULO VELOCIDAD MÁXIMA SEGÚN RADIO DE CURVA 
        v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));
        
        % CÁLCULO RESISTENCIA A AVANCE 
        dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
        rollingForce = -z_load*rolling_resistance_coeff(m, -2, 0.827, v, tire_d/2, 1.08); % cálculo resistencia rodadura [N]
        resistant_acc = (dragForce + rollingForce)/m; % aceleración resistente al avance total [m/s^2]
           
        %LIMITAR VELOCIDAD
        if v_max_curva < v
            v = v_max_curva; 
    
        elseif v < v_max_curva
            long_grip_available= sqrt(abs(1-((v^2./abs(r))./(lat_mu*z_load./m)).^2))*long_grip;
            v = min(sqrt(v^2 + 2*(long_grip_available  + resistant_acc)*d), v_max_curva);
        end
    
        v_resultante_backward(idx_b) = v;
        idx_b = idx_b + 1;
    
    end
    
    
    v_resultante = min(v_resultante_forward, flip(v_resultante_backward));
    lat_acc = v_resultante.^2./radio;
    
    
    dt = d ./ v_resultante; 
    tiempo_total = sum(dt)
    g_long = [diff(v_resultante),0]./dt;
