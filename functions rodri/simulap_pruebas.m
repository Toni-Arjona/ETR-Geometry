clear all
clc
close all



%PARÁMETROS COCHE (datos cogido del FSAE con aero que hay en OptimumLap)
    m = 200 + 70; % peso coche + piloto [kg]
    g = 9.81; % gravedad [m/s^2]
    df_coeff = 4.18; % coeficiente downforce
    dr_coeff = 1.3; % coeficiente drag
    air_d = 1.225; % densidad del aire [kg/m^3]
    area = 0.563; % área frontal del coche [m^2]
    tire_radius = effective_rolling_radius(m*g/4, 0.827); % radio efectivo de la rueda [m]    
    gear_ratio = 10.85; % ratio reducción rpms transmi
    effcy = 0.9; %eficiencia transmi 

%PARÁMETROS NEUMÁTICO
    coeff_long_max = 2.1; % coeficiente longitudinal máximo 
    coeff_lat_max = 1.9; % coeficiente lateral máximo
    sf= 0.9; % porcentaje del agarre máximo al que se llega
    lat_mu = sf*coeff_lat_max; % coeficiente lateral neumático
    long_mu = sf*coeff_long_max; % coeficiente longitudinal neumático

%PARÁMETROS NEUMÁTICO INICIALES
    lat_mu_available = lat_mu;
    long_mu_available = long_mu;

%PARÁMETROS SIMU
    v = 19.56; % velocidad inicial [m/s] (se ha cogido velocidad final de vuelta de una simu forward cualquiera)
    d = 0.01; % intervalo para evaluación splines (como el vector radio se define a partir de esta evaluación, y los bucles iteran sobre el radio, acaba siendo el intervalo de simulación, en metros)

% CARGA DATOS CIRCUITO Y MOTOR
    load("datos_circuito.mat")

% DATOS CURVAS PAR/POTENCIA MOTOR
    par_vector = curva_motor.P;
    v_ang_vector = curva_motor.W;
    par_vector(diff(v_ang_vector)==0) = []; % limpiar valores de par correspondientes a valores de v_ang repetidos
    v_ang_vector(diff(v_ang_vector)==0) = []; % limpiar valores de v_ang repetidos

% DEFINICIÓN TRAYECTO CIRCUITO CON SPLINES
    % 1. Distancia recorrida total e intervalo de evaluación para splines
    d_total = [0; cumsum(sqrt(diff(0.8*circuito.X).^2 + diff(0.8*circuito.Y).^2))];
    d_interval = 0: d : max(d_total);
    
    
    % 2. Definir exprsión splines circuito x e y, y evalurar para intervalo simu
    x_spline = csaps(d_total, 0.8*circuito.X, 1); x_track = ppval(x_spline, d_interval);
    y_spline = csaps(d_total, 0.8*circuito.Y, 1); y_track = ppval(y_spline, d_interval);
    
    
    % 3. Cálculo curvatura circuito
        % 3.1 derivadas primeras y segundas trayectoria y evaluación de expresiones
        dx =  fnder(x_spline, 1); dx_eval =  ppval(dx,  d_interval); 
        dy =  fnder(y_spline, 1); dy_eval =  ppval(dy,  d_interval);
        ddx = fnder(x_spline, 2); ddx_eval = ppval(ddx, d_interval);
        ddy = fnder(y_spline, 2); ddy_eval = ppval(ddy, d_interval);
    
    
        % 3.2 Calcular la curvatura y radio del circuito en cada instante
        curvatura = (dx_eval .* ddy_eval - dy_eval .* ddx_eval) ./ (dx_eval.^2 + dy_eval.^2).^(3/2);
        radio = 1./curvatura; % vector radio [m]




%FORWARD LOOP
    % Creación vector velocidad resultante forward + índice idx
        v_resultante_forward = zeros(1, length(radio));
        idx = 1;
    
    for r = radio
        % CÁLCULO POTENCIA DISPONIBLE
            v_ang= v/(tire_radius)*gear_ratio; % revoluciones motor a velocidad dada [rad/s]
            par = interp1(v_ang_vector, par_vector, v_ang); % encontrar par motor a velocidad dada [Nm]
            pwr = effcy*par*v_ang/1000; % potencia motor a velocidad dada [kW]
            
            % Limitación motor a 40kW (máximo total con dos motores 80kW)
                if pwr > 40
                    pwr = 40;
                end
        
        % CÁLCULO CARGA VERTICAL TOTAL
            dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
            z_load = dForce + m*g; % cálculo fuerza vertical total [N]
       
        % CÁLCULO GRIP LONGITUDINAL MÁXIMO Y FUERZA MOTOR 
            long_grip = z_load*long_mu/m; % cálculo grip longitudinal máx [m/s^2]
            motor_acc = 2*pwr*1000/(m*v); % cálculo aceleración (longitudinal) actual proveniente de motores [m/s^2]
       
            % Limitar agarre longitudinal por saturación neumático [m/s^2]
                if long_grip < 4618/m
                    long_grip= long_grip;
                else
                    long_grip= 4618/m;
                end 
        
        % CÁLCULO RESISTENCIA A AVANCE (drag + rolling resistance)
            dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
            rollingForce = -z_load*rolling_resistance_coeff(m, -2, 0.827, v, tire_radius, 1.08); % cálculo resistencia rodadura [N]
        
        % CÁLCULO ACELERACIÓN LONGITUDINAL NETA (MOTOR - PÉRDIDAS)
            net_acc = motor_acc - (dragForce + rollingForce)/m; % aceleración neta [m/s^2]
    
        % DEFINIR FACTOR LIMITANTE (GRIP LATERAL / GRIP LONGITUDINAL / POTENCIA)
            % Cálculo velocidad máxima según radio de curva
                v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));
    
            if v_max_curva < v
                v = v_max_curva; 
        
        
            elseif v < v_max_curva
                long_grip_available= sqrt(abs(1-((v^2./abs(r))./(lat_mu*z_load./m)).^2))*long_grip;
                v = min(sqrt(v^2 + 2*min(0.5*long_grip_available, net_acc)*d), v_max_curva);
            end
    
        % ALMACENAR VELOCIDAD FINAL DE BUCLE EN VECTOR
            v_resultante_forward(idx) = v;
            idx = idx + 1;
        
    end






%BACKWARD LOOP
    % vector velocidad backward + índice
        v_resultante_backward = zeros(1, length(radio));
        idx_b = 1;
        v = v_resultante_forward(end);
    
    for r = flip(radio)
        
        % CÁLCULO CARGA VERTICAL TOTAL
            dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
            z_load = dForce + m*g; % cálculo fuerza vertical total [N]
    
        % CÁLCULO GRIP LONGITUDINAL MÁXIMO
            long_grip = z_load*long_mu/m; % cálculo grip longitudinal max [m/s^2]
    
            % Limitar agarre longitudinal por saturación neumático [m/s^2]
                if long_grip < 4618/m
                    long_grip= long_grip;
                else
                    long_grip= 4618/m;
                end    
        
        % CÁLCULO RESISTENCIA A AVANCE (DRAG + ROLLING RESISTANCE)
            dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
            rollingForce = -z_load*rolling_resistance_coeff(m, -2, 0.827, v, tire_radius, 1.08); % cálculo resistencia rodadura [N]
            resistant_acc = (dragForce + rollingForce)/m; % aceleración resistente al avance total [m/s^2]
           
        % DEFINIR FACTOR LIMITANTE (GRIP LATERAL / GRIP LONGITUDINAL / POTENCIA)
            % Cálculo velocidad máxima según radio de curva
                v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));

            % Valoración grip para determinar nueva velocidad
                if v_max_curva < v
                    v = v_max_curva; 
    
                elseif v < v_max_curva
                    long_grip_available= sqrt(abs(1-((v^2./abs(r))./(lat_mu*z_load./m)).^2))*long_grip;
                    v = min(sqrt(v^2 + 2*(long_grip_available  + resistant_acc)*d), v_max_curva);
                end
    
        % ALMACENAR VELOCIDAD FINAL DE BUCLE EN VECTOR
            v_resultante_backward(idx_b) = v;
            idx_b = idx_b + 1;
    
    end



 
% COMBINACIÓN BUCLE FORWARD Y BACKWARD
    v_resultante = min(v_resultante_forward, flip(v_resultante_backward));

% CÁLCULOS RESULTADOS
    lat_acc = v_resultante.^2./radio; % Cálculo aceleración lateral para cada punto de simu
    dt = d ./ v_resultante; % Cálculo tiempo transcurrido en cada intervalo de simu
    g_long = [diff(v_resultante),0]./dt; % cálculo aceleración longitudinal para cada punto de simu
    
    
    tiempo_total = sum(dt) % cálculo tiempo de vuelta
    
        

% PORCENTAJES RADIOS DE CURVA
p = abs(radio);

    % 1. Definir la recta (lo que queda fuera del rango 0 a 100)
        idx_recta = p <= 0 | p >= 100;
        recta = radio(idx_recta);

    % 2. Denominador para los porcentajes (% de secciones con radio > 100m)
        total_curva = length(radio) - length(recta);

    % 3. Extraer vectores usando indexación lógica (corrige tu error de índices)
        radio_0_5   = radio(p > 0  & p < 5);
        radio_5_10  = radio(p >= 5 & p < 10);
        radio_10_15 = radio(p >= 10 & p < 15);
        radio_15_20 = radio(p >= 15 & p < 20);
        radio_20_25 = radio(p >= 20 & p < 25);
        radio_25_30 = radio(p >= 25 & p < 30);
        radio_30_35 = radio(p >= 30 & p < 35);
        radio_35_40 = radio(p >= 35 & p < 40);
        radio_40_45 = radio(p >= 40 & p < 45);
        radio_45_50 = radio(p >= 45 & p < 50);
        radio_50_55 = radio(p >= 50 & p < 55);
        radio_55_60 = radio(p >= 55 & p < 60);
        radio_60_65 = radio(p >= 60 & p < 65);
        radio_65_70 = radio(p >= 65 & p < 70);
        radio_70_75 = radio(p >= 70 & p < 75);
        radio_75_80 = radio(p >= 75 & p < 80);
        radio_80_85 = radio(p >= 80 & p < 85);
        radio_85_90 = radio(p >= 85 & p < 90);
        radio_90_95 = radio(p >= 90 & p < 95);
        radio_95_100= radio(p >= 95 & p < 100);

    % 4. Calcular porcentajes
        porcentaje_0_5   = length(radio_0_5) / total_curva;
        porcentaje_5_10  = length(radio_5_10) / total_curva;
        porcentaje_10_15 = length(radio_10_15) / total_curva;
        porcentaje_15_20 = length(radio_15_20) / total_curva;
        porcentaje_20_25 = length(radio_20_25) / total_curva;
        porcentaje_25_30 = length(radio_25_30) / total_curva;
        porcentaje_30_35 = length(radio_30_35) / total_curva;
        porcentaje_35_40 = length(radio_35_40) / total_curva;
        porcentaje_40_45 = length(radio_40_45) / total_curva;
        porcentaje_45_50 = length(radio_45_50) / total_curva;
        porcentaje_50_55 = length(radio_50_55) / total_curva;
        porcentaje_55_60 = length(radio_55_60) / total_curva;
        porcentaje_60_65 = length(radio_60_65) / total_curva;
        porcentaje_65_70 = length(radio_65_70) / total_curva;
        porcentaje_70_75 = length(radio_70_75) / total_curva;
        porcentaje_75_80 = length(radio_75_80) / total_curva;
        porcentaje_80_85 = length(radio_80_85) / total_curva;
        porcentaje_85_90 = length(radio_85_90) / total_curva;
        porcentaje_90_95 = length(radio_90_95) / total_curva;
        porcentaje_95_100= length(radio_95_100)/ total_curva;
        
    % 5. Almacenar porcentajes en un mimso vector
        vector_porcentajes = [porcentaje_0_5, porcentaje_5_10, porcentaje_10_15, porcentaje_15_20, ...
                              porcentaje_20_25, porcentaje_25_30, porcentaje_30_35, porcentaje_35_40, ...
                              porcentaje_40_45, porcentaje_45_50, porcentaje_50_55, porcentaje_55_60, ...
                              porcentaje_60_65, porcentaje_65_70, porcentaje_70_75, porcentaje_75_80, ...
                              porcentaje_80_85, porcentaje_85_90, porcentaje_90_95, porcentaje_95_100];







       
% PLOTS

    % 1. MAPA VELOCIDAD CON ESCALA DE COLORES       
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
        colormap(turbo); % turbo es una paleta de colores
        axis equal
        xlabel('x [m]')
        ylabel('y [m]')
        title('Mapa de Velocidad')


    % 2. MAPA ACELERACIÓN LATERAL CON ESCALA DE COLORES  
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
        colormap(turbo); % % turbo es una paleta de colores
        axis equal
        xlabel('x [m]')
        ylabel('y [m]')
        title('Mapa de aceleración lateral')


    % 3. MAPA ACELERACIÓN LONGITUDINAL CON ESCALA DE COLORES 

        figure(3)
        % Crear vector Z de ceros para engañar a la función surface (pintar en 2D)
        z = zeros(size(x_track)); 
        
        %Pintar la línea usando la velocidad como mapa de color
        surface([x_track; x_track], [y_track; y_track], [z; z], [g_long/g; g_long/g], ...
                'facecol', 'no', ...
                'edgecol', 'interp', ...
                'linew', 2); % Grosor de línea
        
        colorbar; % Añade la barra lateral de leyenda
        c = colorbar;
        c.Label.String = 'Longitudinal acceleration [g]';
        colormap(turbo); % turbo es una paleta de colores
        axis equal
        xlabel('x [m]')
        ylabel('y [m]')
        title('Mapa de aceleración longitudinal')

    % 4. MAPA RADIOS DE CURVA CON ESCALA DE COLORES 
        curvas = zeros(1, length(radio));
        idx_c = 1;

        for k = abs(radio)
            if k > 50
                curvas(idx_c) = 50;

            else
                curvas(idx_c) = k;
            
            idx_c = idx_c + 1; 
            end
        end
        

        figure(4)
        % Crear vector Z de ceros para engañar a la función surface (pintar en 2D)
        z = zeros(size(x_track)); 
        
        %Pintar la línea usando la velocidad como mapa de color
        surface([x_track; x_track], [y_track; y_track], [z; z], [curvas; curvas], ...
                'facecol', 'no', ...
                'edgecol', 'interp', ...
                'linew', 2); % Grosor de línea
        
        colorbar; % Añade la barra lateral de leyenda
        c = colorbar;
        c.Label.String = 'radio de curva [m]';
        colormap(turbo); % turbo es una paleta de colores
        axis equal
        xlabel('x [m]')
        ylabel('y [m]')
        title('Mapa de radio de curva')


    % 5. GRÁFICA VELOCIDAD VS DISTANCIA 
        figure(5)
        plot(d_interval, v_resultante)
        xlabel('elapsed distance [m]')
        ylabel('v_resultante')

    
    % 6. GRÁFICA ACELERACIÓN LONGITUDINAL (G) VS DISTANCIAL
        figure(6)
        plot(d_interval, g_long/g)
        xlabel('elapsed distance [m]')
        ylabel('longitudinal acceleration [g]')
        ylim([-2.5, 1.5])

    % 7. GRÁFICA ACELERACIÓN LATERAL (G) VS DISTANCIA
        figure(7)
        plot(d_interval, lat_acc/g)
        xlabel('elapsed distance [m]')
        ylabel('lateral acceleration [g]')


    % 8. DISTRIBUCIÓN RADIOS DE CURVA
        figure(8)
        centros = 2.5:5:97.5;
        bar(centros, vector_porcentajes * 100, 1); 
        xlabel('Radio de giro [m]');
        ylabel('Porcentaje [%]');
        title('Distribución de Radios de Giro');
        grid on;