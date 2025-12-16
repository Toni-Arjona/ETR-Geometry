clc
clear all
close all


load('datos_simulap.mat')
g = 9.81; % gravedad [m/s^2]


%% PARÁMETROS SIMU
    v = 0.001; % velocidad inicial [m/s] (se ha cogido velocidad final de vuelta de una simu forward cualquiera)
    d = 0.1; % intervalo para evaluación splines (como el vector radio se define a partir de esta evaluación, y los bucles iteran sobre el radio, acaba siendo el intervalo de simulación, en metros)
    sf_lateral= 1 ; % porcentaje del agarre máximo lateral al que se llega
    sf_braking = 1; % porcentaje de agarre de frenada máxima al que se llega
    sf_accel = 1; % porcentaje de agarre de aceleración máxima al que se llega

%% PARÁMETROS COCHE 
        m = 200 + 70; % peso coche + piloto [kg]
    

    % AERO
        df_coeff = 4.18; % coeficiente downforce
        dr_coeff = 1.3; % coeficiente drag
        air_d = 1.225; % densidad del aire [kg/m^3]
        area = 0.563; % área frontal del coche [m^2]

    % TRANSMI
        gear_ratio =  10.2;% ratio reducción rpms transmisión
        effcy = 0.9; % eficiencia transmi

%% NEUMÁTICO
    tire_radius = effective_rolling_radius(m*g/4, 0.827); % radio efectivo de la rueda [m]    
    
    coeff_long_max = 2.1; % coeficiente longitudinal máximo 
    coeff_lat_max = 1.9; % coeficiente lateral máximo        
    lat_mu = sf_lateral*coeff_lat_max; % coeficiente lateral neumático
    long_mu = sf_accel*coeff_long_max; % coeficiente longitudinal neumático
    camber_deg = -2;
    slip_ratio = 1.12;

    % Parámetros iniciales grip disponible
    lat_mu_available = lat_mu;
    long_mu_available = long_mu;

%% CARGA DATOS CIRCUITO Y MOTOR
% DATOS CURVAS PAR/POTENCIA MOTOR
    par_vector = motor.P;
    v_ang_vector = motor.W;
    par_vector(diff(v_ang_vector)==0) = []; % limpiar valores de par correspondientes a valores de v_ang repetidos
    v_ang_vector(diff(v_ang_vector)==0) = []; % limpiar valores de v_ang repetidos

% DEFINICIÓN TRAYECTO CIRCUITO CON SPLINES
    % 1. Distancia recorrida total e intervalo de evaluación para splines
    d_total = [0; cumsum(sqrt(diff(circuito.X).^2 + diff(circuito.Y).^2))];
    d_interval = 0: d : max(d_total);
    
    
    % 2. Definir exprsión splines circuito x e y, y evalurar para intervalo simu
    x_spline = spline(d_total, circuito.X); x_track = ppval(x_spline, d_interval);
    y_spline = spline(d_total, circuito.Y); y_track = ppval(y_spline, d_interval);
    
    
    % 3. Cálculo curvatura circuito
        % 3.1 derivadas primeras y segundas trayectoria y evaluación de expresiones
        dx =  fnder(x_spline, 1); dx_eval =  ppval(dx,  d_interval); 
        dy =  fnder(y_spline, 1); dy_eval =  ppval(dy,  d_interval);
        ddx = fnder(x_spline, 2); ddx_eval = ppval(ddx, d_interval);
        ddy = fnder(y_spline, 2); ddy_eval = ppval(ddy, d_interval);
    
    
        % 3.2 Calcular la curvatura y radio del circuito en cada instante
        curvatura = (dx_eval .* ddy_eval - dy_eval .* ddx_eval) ./ (dx_eval.^2 + dy_eval.^2).^(3/2);
        radio = 1./curvatura; % vector radio [m]




%% FORWARD LOOP
    % Creación vector velocidad resultante forward + índice idx
        v_resultante_forward = zeros(1, length(radio));
        idx = 1;
    
    for r = radio
        % CÁLCULO POTENCIA DISPONIBLE
            v_ang= v/(tire_radius)*gear_ratio; % revoluciones motor a velocidad dada [rad/s]
            par = interp1(v_ang_vector, par_vector, v_ang); % encontrar par motor a velocidad dada [Nm]
            pwr = effcy*par*v_ang/1000; % potencia motor a velocidad dada [kW]
            
            % Limitación motor a 40kW (máximo total con dos motores 80kW)
                if pwr > 20*effcy
                    pwr = 20*effcy;
                end
        
        % CÁLCULO CARGA VERTICAL TOTAL
            dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
            z_load = dForce + m*g; % cálculo fuerza vertical total [N]
       
        % CÁLCULO GRIP LONGITUDINAL MÁXIMO Y FUERZA MOTOR 
            Long_force_coef = longitudinal_force(m, z_load/g, effective_rolling_radius(z_load/(4*g), 0.827), camber_deg, slip_ratio, v, 0.827);
            long_grip = sf_accel*z_load*Long_force_coef/m; % cálculo grip longitudinal máx [m/s^2]
            motor_acc = 4*pwr*1000/(m*v); % cálculo aceleración (longitudinal) actual proveniente de motores [m/s^2]
                
        % CÁLCULO RESISTENCIA A AVANCE (drag + rolling resistance)
            dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
            rollingForce = -z_load*rolling_resistance_coeff(z_load/g, camber_deg, 0.827, v, tire_radius, slip_ratio); % cálculo resistencia rodadura [N]
        
        % CÁLCULO ACELERACIÓN LONGITUDINAL NETA (MOTOR - PÉRDIDAS)
            net_acc = motor_acc - (dragForce + rollingForce)/m; % aceleración neta [m/s^2]
    
        % DEFINIR FACTOR LIMITANTE (GRIP LATERAL / GRIP LONGITUDINAL / POTENCIA)
            % Cálculo velocidad máxima según radio de curva
                v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));
    
            if v_max_curva < v
                v = v_max_curva; 
        
        
            elseif v < v_max_curva
                long_grip_available= sqrt(abs(1-((v^2./abs(r))./(lat_mu*z_load./m)).^2))*long_grip;
                v = min(sqrt(v^2 + 2*min(long_grip_available, net_acc)*d), v_max_curva);
            end
    
        % ALMACENAR VELOCIDAD FINAL DE BUCLE EN VECTOR
            v_resultante_forward(idx) = v;
            idx = idx + 1;
        
    end






%% BACKWARD LOOP
    % vector velocidad backward + índice
        v_resultante_backward = zeros(1, length(radio));
        idx_b = 1;
        v = v_resultante_forward(end);
    
    for r = flip(radio)
        
        % CÁLCULO CARGA VERTICAL TOTAL
            dForce = 0.5*air_d*area*df_coeff*v^2; % cálculo downforce [N]
            z_load = dForce + m*g; % cálculo fuerza vertical total [N]
    
        % CÁLCULO GRIP LONGITUDINAL MÁXIMO
            Long_force_coef = longitudinal_force(m, z_load/g, effective_rolling_radius(z_load/(4*g), 0.827), camber_deg, slip_ratio, v, 0.827);
            long_grip = z_load*Long_force_coef/m; % cálculo grip longitudinal máx [m/s^2]
       
        
        % CÁLCULO RESISTENCIA A AVANCE (DRAG + ROLLING RESISTANCE)
            dragForce = 0.5*air_d*area*dr_coeff*v^2; % cálculo drag [N]
            rollingForce = -z_load*rolling_resistance_coeff(z_load/g, camber_deg, 0.827, v, tire_radius, slip_ratio); % cálculo resistencia rodadura [N]
            resistant_acc = (dragForce + rollingForce)/m; % aceleración resistente al avance total [m/s^2]
           
        % DEFINIR FACTOR LIMITANTE (GRIP LATERAL / GRIP LONGITUDINAL / POTENCIA)
            % Cálculo velocidad máxima según radio de curva
                v_max_curva = sqrt(lat_mu*m*g / abs(m/abs(r) - 0.5*lat_mu*air_d*area*df_coeff));

            % Valoración grip para determinar nueva velocidad
                if v_max_curva < v
                    v = v_max_curva; 
    
                elseif v < v_max_curva
                    long_grip_available= sqrt(abs(1-((v^2./abs(r))./(lat_mu*z_load./m)).^2))*long_grip;
                    v = min(sqrt(v^2 + 2*(sf_braking*long_grip_available  + resistant_acc)*d), v_max_curva);
                end
    
        % ALMACENAR VELOCIDAD FINAL DE BUCLE EN VECTOR
            v_resultante_backward(idx_b) = v;
            idx_b = idx_b + 1;
    
    end


    
 
%% COMBINACIÓN BUCLE FORWARD Y BACKWARD
    v_resultante = min(v_resultante_forward, flip(v_resultante_backward));

%% CÁLCULOS RESULTADOS
    lat_acc = v_resultante.^2./radio; % Cálculo aceleración lateral para cada punto de simu
    dt = d ./ v_resultante; % Cálculo tiempo transcurrido en cada intervalo de simu
    g_long =    [diff(v_resultante),0]./dt; % cálculo aceleración longitudinal para cada punto de simu
    dForce = 0.5*air_d*area*df_coeff*v_resultante.^2;
    
    
    tiempo_total = sum(dt) % cálculo tiempo de vuelta

%% LOAD TRASNFER 
    for lt = 1:length(abs(radio))
        [fz_FR, fz_FL, fz_RR, fz_RL] = normal_load_per_tire_complete(m, abs(lat_acc(lt)/g), 1.25, 1.25, 0.07, ...
        0.08, 18370.86641/57.2958, 18370.86641/57.2958, 0.225, 1.6, 0.8);

        RL(lt) = fz_RL; % FALTA AÑADIR CARGA DE DOWNFORCE SOBRE CADA NEUMÁTICO
        RR(lt) = fz_RR;
        F_ext(lt) = fz_FL;
        F_int(lt) = fz_FR;
        
    end

    
        

%% PORCENTAJES RADIOS DE CURVA
    % 1. filtro valores rectas (radios mayores a 50m)
    curvas = radio(abs(radio) < 50); 
    
    % 2. Creación cell arrays
    radio_curva = cell(1, 50);
    pctaje_curva = cell(1, 50);
    
    % 3. Bucle para dividir según radio
    for interval = 1:50
        radio_curva{interval} = radio((abs(radio) > (interval - 1)) & (abs(radio) < interval));
        pctaje_curva{interval} = 100*length(radio_curva{interval})/length(curvas);
    
    end

       
%% PLOTS

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
        surface([x_track; x_track], [y_track; y_track], [z; z], [abs(lat_acc/g); abs(lat_acc/g)], ...
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
        curvas = abs(radio);
        curvas(curvas > 50) = 50;

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
        bar(0.5:1:49.5, cell2mat(pctaje_curva), 1)
        xlabel('Radio de curva [m]')
        ylabel('Porcentaje del total de curvas [%]')
        grid on

        figure(9)
       
        plot(d_interval, F_ext)
        hold on
        plot (d_interval, F_int)
        plot (d_interval, (F_int + F_ext))
        xlabel(['distancia [m]'])
        ylabel(['Carga sobre neumáticos delanteros [N]'])
        legend("rueda exterior", "rueda interior", "carga total")

        figure(10)
        plot(d_interval, (gear_ratio*v_resultante/tire_radius)*30/pi)

       

       
        
        
        