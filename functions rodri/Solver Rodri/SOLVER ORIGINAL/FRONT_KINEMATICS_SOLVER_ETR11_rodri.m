function[FL, FR] = FRONT_KINEMATICS_SOLVER_ETR11_rodri(steering_wheel_angle, FL_DPR_COMPRESSION, FR_DPR_COMPRESSION)
    % Front left suspension and steering geometry
    % [wheel]_[componente 1]_[componente 2]
    % F - Front
    % R - Rear/Right
    % U - Upper
    % L - Lower
    % W - Wishbone
    % MC - Monocoque
    % KN - Knuckle
    % RKR - Rocker
    % PUSH - Pushrod
    % DPR - Damper
    % UPRIGHT - Upright (mangueta)
    % TR - Tie rod
    % RACK - Rack
    % KP - Kingpin
    
    %% FRONT LEFT WHEEL
    %% COORDINATES DEFINITIONS. POINTS
        % Wheel spindle 
        FL_SPINDLE_CENTER = [-100, -625, 203]; 
        FL_SPINDLE_INNER = [-100, -561.5, 200.5];

        % FL Knuckles
        FL_UW_KN = [-90, -540, 294];
        FL_LW_KN = [-103.5 ,-580, 106]; 
        
        % FL wishbones joints with monocoque
        FL_URW_MC = [75, -225, 267];
        FL_UFW_MC = [-210, -225, 281];
        FL_LRW_MC = [80, -225, 135];
        FL_LFW_MC = [-210, -225, 123];
        
        % FL Push rod
        FL_PUSH_UW = [-90, -489, 319];
        FL_PUSH_RKR = [-90, -193.5, 558];
        
        % FL Rocker axis points
        FL_RKR_1 = [-18, -186.5, 587];
        FL_RKR_2 = [-18, -164, 552];
        
        % FL Damper
        FL_DPR_RKR = [-43, -112, 610];
        FL_DPR_MC = [167, -112, 610];

        % FL Tie rod
        FL_TR_UPRIGHT = [-185, -560, 140];
        FL_TR_RACK = [-175, -225, 150];
        
    %% INPUTS STEERING
        D_PINION = 35;
        RACK_DISPLACEMENT = deg2rad(steering_wheel_angle)*D_PINION/2;
        FL_TR_RACK_FINAL_POINT = FL_TR_RACK + [0, RACK_DISPLACEMENT, 0];
      
    %% FIXED DISTANCES DEFINITIONS
        % Push rod
        F_PUSH_LENGTH = norm(FL_PUSH_UW - FL_PUSH_RKR);
    
        % Tie rod
        F_TR_LENGTH = norm(FL_TR_UPRIGHT - FL_TR_RACK);
        
        % Upright
        F_KP_LENGTH   = norm(FL_UW_KN - FL_LW_KN);

        % Damper initial length
        DPR_LENGTH = norm(FL_DPR_MC - FL_DPR_RKR);
    
    
    %% AXIS DEFINITIONS. NORMALIZED VECTORS
        % FL Kingpin axis. Definido hacia abajo, para tener giro a derechas positivo.
        FL_KP = (FL_LW_KN - FL_UW_KN)/norm((FL_LW_KN - FL_UW_KN));
        
        % FL Wishbone axis. Definido para valores de rotación positivos en compresión
        FL_UW_AXIS = (FL_UFW_MC - FL_URW_MC)/norm(FL_UFW_MC - FL_URW_MC);
        FL_LW_AXIS = (FL_LFW_MC - FL_LRW_MC)/norm(FL_LFW_MC - FL_LRW_MC);
        
        % FL rocker axis. Definido para valores positivos de rotación en compresión.
        FL_RKR_AXIS = (FL_RKR_2 - FL_RKR_1)/norm(FL_RKR_2 - FL_RKR_1);
                
    %% SOLVER
    %% Definición de la compresión del rocker y la rotación del rocker asociada. Formula de Rodrigues. 
    % - Incógnita(s): ángulo rotación rocker.
    % - Restricción(es): longitud damper (distancia entre FL_DPR_RKR y FL_DPR_MC)
        % vectores a rotar del rocker. Punto referencia FL_RKR_2 para Rodrigues.
        FL_RKR_AXIS_2_to_DPR = FL_DPR_RKR - FL_RKR_2 ;
        FL_RKR_AXIS_2_to_PUSH = FL_PUSH_RKR - FL_RKR_2 ;
        
        % Fórmula de Rodrigues
        rodrigues_rotation = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 
        
        % Función a solucionar. Busca el ángulo que hace que se consiga la longitud total deseada del damper
        FL_RKR_SOLVE_NEW_FUNCTION = @(FL_RKR_ANGLE) norm((FL_RKR_2 + rodrigues_rotation(FL_RKR_AXIS_2_to_DPR, FL_RKR_AXIS, FL_RKR_ANGLE)) - FL_DPR_MC) - (DPR_LENGTH - FL_DPR_COMPRESSION);
        
        % Solucionar ecuación anterior igualada a 0.
        FL_RKR_ANGLE = fzero(FL_RKR_SOLVE_NEW_FUNCTION, 0, optimset('Display', 'off'));

        % Definir nueva posición de los extremos del rocker.
        FL_DPR_RKR_FINAL_POINT = FL_RKR_2 + rodrigues_rotation(FL_RKR_AXIS_2_to_DPR, FL_RKR_AXIS, FL_RKR_ANGLE);
        FL_PUSH_RKR_FINAL_POINT = FL_RKR_2 + rodrigues_rotation(FL_RKR_AXIS_2_to_PUSH, FL_RKR_AXIS, FL_RKR_ANGLE);
    
    
    %% Definición de la nueva posición de Kingpin, con el cálculo de ángulos de wishbones asociado. Fórmula de RODRIGUEEEESSSSS
    % - Incógnita(s): Ángulo rotación upper wishbone, ángulo de rotación lower wishbone y ángulo de rotación mangueta alrededor del kingpin.
    % - Restricción(es): Distancia entre knuckles, longitud pushrod y longitud tie.
        % Ecuación 1. Distancia entre kunckles.
            % Vectores a rotar.
            FL_URW_MC_to_UW_KN = FL_UW_KN - FL_URW_MC; % vector a rotar upper wishbone. Referencia Rodrigues rear mc joint.
            FL_LRW_MC_to_LW_KN = FL_LW_KN - FL_LRW_MC; % vector a rota lower wishbone- Referencia Rodrigues rear mc joint.
            
            % Rotaciones
            FL_UW_KN_NEW_FUNCTION = @(x) FL_URW_MC + rodrigues_rotation(FL_URW_MC_to_UW_KN, FL_UW_AXIS, x(1));
            FL_LW_KN_NEW_FUNCTION = @(x) FL_LRW_MC + rodrigues_rotation(FL_LRW_MC_to_LW_KN, FL_LW_AXIS, x(2));

            % Función a solucionar. Distancia entre kunckles.
            FL_KNS_SOLVE_NEW_FUNCTION = @(x) norm(FL_UW_KN_NEW_FUNCTION(x) - FL_LW_KN_NEW_FUNCTION(x)) - F_KP_LENGTH; % x(1) es el ángulo del UW, x(2) es el ángulo de LW
            FL_KP_NEW_FUNCTION = @(x) (FL_LW_KN_NEW_FUNCTION(x) - FL_UW_KN_NEW_FUNCTION(x))/norm(FL_LW_KN_NEW_FUNCTION(x) - FL_UW_KN_NEW_FUNCTION(x));
       
        % Ecuación 2. Longitud tie. 
            FL_STEER_ARM = FL_TR_UPRIGHT - FL_LW_KN;
           
            h_TR = dot(FL_STEER_ARM, FL_KP); % Cuánto sube el brazo por el eje (Escrito por gemini, me da palo)
            v_rad_TR = FL_STEER_ARM - h_TR * FL_KP; % El resto del brazo (perpendicular) (Escrito por gemini, me da palo)
    
            % 2. Rotación acoplada: inclina con el eje y luego gira x(3)
            FL_TR_UPRIGHT_NEW_FUNCTION = @(x) FL_LW_KN_NEW_FUNCTION(x) +  h_TR * FL_KP_NEW_FUNCTION(x) + rodrigues_rotation(v_rad_TR, FL_KP_NEW_FUNCTION(x), x(3)); %(Escrito por gemini, me da palo)
    
            % Ecuación a resolver
            FL_TR_SOLVE_NEW_FUNCTION = @(x) norm(FL_TR_UPRIGHT_NEW_FUNCTION(x) - FL_TR_RACK_FINAL_POINT) - F_TR_LENGTH; %(Escrito por gemini, me da palo)
        
        % Ecuación 3. Longitud push.
            % Vector a rotar. (mc hasta push-uptight)
            FL_PUSH_MC = FL_PUSH_UW - FL_URW_MC;
    
            % Rotación.
            FL_PUSH_UW_NEW_FUNCTION = @(x) FL_URW_MC + rodrigues_rotation(FL_PUSH_MC, FL_UW_AXIS, x(1));
    
            % Ecuación a solucionar
            FL_PUSH_SOLVE_NEW_FUNCTION = @(x) norm(FL_PUSH_UW_NEW_FUNCTION(x) - FL_PUSH_RKR_FINAL_POINT) - F_PUSH_LENGTH;
    
        % RESOLUCIÓN FINAL
        FL_TOTAL_SOLVE_NEW_FUNCTION = @(x) [FL_KNS_SOLVE_NEW_FUNCTION(x); FL_TR_SOLVE_NEW_FUNCTION(x); FL_PUSH_SOLVE_NEW_FUNCTION(x)];
        x = fsolve(FL_TOTAL_SOLVE_NEW_FUNCTION, [0, 0, 0], optimoptions('fsolve', 'Display', 'off'));
    
    %% PUNTOS ACTUALIZADOS
         % KNUCKLES
         FL_UW_KN_FINAL_POINT = FL_UW_KN_NEW_FUNCTION(x);
         FL_LW_KN_FINAL_POINT = FL_LW_KN_NEW_FUNCTION(x);

         % Kingpin nuevo
         FL_KP_FINAL = FL_KP_NEW_FUNCTION(x);
    
         % TIE ROD
         FL_TR_UPRIGHT_FINAL_POINT = FL_TR_UPRIGHT_NEW_FUNCTION(x);
         
         % PUSH
         FL_PUSH_UW_FINAL_POINT = FL_PUSH_UW_NEW_FUNCTION(x);
       
    
    % ROTACIÓN SPINDLE
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT CENTER
            FL_LW_KN_to_SPINDLE_CENTER = FL_SPINDLE_CENTER - FL_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            FL_LW_KN_to_SPINDLE_CENTER_axial_KP = dot(FL_LW_KN_to_SPINDLE_CENTER, FL_KP); % comoponente axial
            FL_LW_KN_to_SPINDLE_CENTER_radial_KP = FL_LW_KN_to_SPINDLE_CENTER - FL_LW_KN_to_SPINDLE_CENTER_axial_KP*FL_KP; % componente radial
        
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT INNER.
            FL_LW_KN_to_SPINDLE_INNER = FL_SPINDLE_INNER - FL_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            FL_LW_KN_to_SPINDLE_INNER_axial_KP = dot(FL_LW_KN_to_SPINDLE_INNER, FL_KP); % comoponente axial
            FL_LW_KN_to_SPINDLE_INNER_radial_KP = FL_LW_KN_to_SPINDLE_INNER - FL_LW_KN_to_SPINDLE_INNER_axial_KP*FL_KP; % componente radial
        
        % Nuevo spindle center
        FL_SPINDLE_CENTER_FINAL_POINT = FL_LW_KN_FINAL_POINT + FL_LW_KN_to_SPINDLE_CENTER_axial_KP * FL_KP_FINAL + rodrigues_rotation(FL_LW_KN_to_SPINDLE_CENTER_radial_KP, FL_KP_FINAL, x(3));
        
        % Nuevo spindle inner
        FL_SPINDLE_INNER_FINAL_POINT = FL_LW_KN_FINAL_POINT + FL_LW_KN_to_SPINDLE_INNER_axial_KP * FL_KP_FINAL + rodrigues_rotation(FL_LW_KN_to_SPINDLE_INNER_radial_KP, FL_KP_FINAL, x(3));
        
        % Nuevo spindle
        FL_SPINDLE_FINAL = (FL_SPINDLE_INNER_FINAL_POINT - FL_SPINDLE_CENTER_FINAL_POINT)/norm(FL_SPINDLE_INNER_FINAL_POINT - FL_SPINDLE_CENTER_FINAL_POINT); % nuevo vector spindle
    

    [~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,FL.LOADED_RADIUS ] = mfeval_function(270*9.81/4, 0, 0, asind(FL_SPINDLE_FINAL(3)), 0);

    FL.URW_MC        = FL_URW_MC ;
    FL.UFW_MC        = FL_UFW_MC ;
    FL.LRW_MC        = FL_LRW_MC ;
    FL.LFW_MC        = FL_LFW_MC ;
    FL.UW_KN         = FL_UW_KN_FINAL_POINT;
    FL.LW_KN         = FL_LW_KN_FINAL_POINT;
    FL.KP            = FL_KP_FINAL;
    FL.TR_RACK       = FL_TR_RACK_FINAL_POINT;
    FL.TR_UPRIGHT    = FL_TR_UPRIGHT_FINAL_POINT;
    FL.DPR_RKR       = FL_DPR_RKR_FINAL_POINT;
    FL.PUSH_UW       = FL_PUSH_UW_FINAL_POINT;
    FL.PUSH_RKR      = FL_PUSH_RKR_FINAL_POINT;
    FL.SPINDLE_CENTER= FL_SPINDLE_CENTER_FINAL_POINT;
    FL.SPINDLE_INNER = FL_SPINDLE_INNER_FINAL_POINT;
    FL.SPINDLE       = FL_SPINDLE_FINAL;
    FL.DPR_MC        = FL_DPR_MC ;
    FL.RKR_AXIS_1    = FL_RKR_1 ;
    FL.RKR_AXIS_2    = FL_RKR_2 ;

    % Cálculo contact patch
    FL_RADIAL_VECTOR = cross(cross(FL.SPINDLE, [0, 0, -1]), FL.SPINDLE);
    FL_RADIAL_VECTOR = FL_RADIAL_VECTOR/norm(FL_RADIAL_VECTOR);
    
    FL.CONTACT_PATCH = FL.SPINDLE_CENTER + (FL.LOADED_RADIUS * 1000) * FL_RADIAL_VECTOR;
    
    % Cálculo plano del suelo en la rueda, respecto a monocasco fijo
    FLOOR_PARELLEL_PLANE = cross([1, 0, 0], [0, 1, 0]);

    FL.FLOOR_PLANE = [FLOOR_PARELLEL_PLANE, -dot(FLOOR_PARELLEL_PLANE, FL.CONTACT_PATCH)];
   
        
    %% FRONT RIGHT WHEEL
    %% COORDINATES DEFINITIONS. POINTS
        % Wheel spindle 
        FR_SPINDLE_CENTER = FL_SPINDLE_CENTER .* [1, -1, 1]; 
        FR_SPINDLE_INNER  = FL_SPINDLE_INNER .* [1, -1, 1];
      
        % FR Knuckles
        FR_UW_KN = FL_UW_KN .* [1, -1, 1]; 
        FR_LW_KN = FL_LW_KN .* [1, -1, 1];
        
        % FR wishbones joints with monocoque
        FR_URW_MC = FL_URW_MC .* [1, -1, 1];
        FR_UFW_MC = FL_UFW_MC .* [1, -1, 1];
        FR_LRW_MC = FL_LRW_MC .* [1, -1, 1];
        FR_LFW_MC = FL_LFW_MC .* [1, -1, 1];
        
        % FR Push rod
        FR_PUSH_UW  = FL_PUSH_UW .* [1, -1, 1];
        FR_PUSH_RKR = FL_PUSH_RKR .* [1, -1, 1];
        
        % FR Rocker axis points
        FR_RKR_1 = FL_RKR_1 .* [1, -1, 1];
        FR_RKR_2 = FL_RKR_2 .* [1, -1, 1];
        
        % FR Damper
        FR_DPR_RKR = FL_DPR_RKR .* [1, -1, 1];
        FR_DPR_MC  = FL_DPR_MC .* [1, -1, 1];
        
        % FR Tie rod
        FR_TR_UPRIGHT = FL_TR_UPRIGHT .* [1, -1, 1];
        FR_TR_RACK    = FL_TR_RACK .* [1, -1, 1];
        
    %% INPUTS STEERING
        FR_TR_RACK_FINAL_POINT = FR_TR_RACK + [0, RACK_DISPLACEMENT, 0];
    
    
    %% AXIS DEFINITIONS. NORMALIZED VECTORS
        % FR Kingpin axis. Definido hacia abajo, para tener giro a derechas positivo.
        FR_KP = (FR_LW_KN - FR_UW_KN)/norm((FR_LW_KN - FR_UW_KN));
        
        % FR Wishbone axis. Definido para valores de rotación positivos en compresión
        FR_UW_AXIS = (FR_UFW_MC - FR_URW_MC)/norm(FR_UFW_MC - FR_URW_MC);
        FR_LW_AXIS = (FR_LFW_MC - FR_LRW_MC)/norm(FR_LFW_MC - FR_LRW_MC);
        
        % FR rocker axis. Definido para valores positivos de rotación en compresión.
        FR_RKR_AXIS = (FR_RKR_2 - FR_RKR_1)/norm(FR_RKR_2 - FR_RKR_1);
            
    %% SOLVER
    %% Definición de la compresión del rocker y la rotación del rocker asociada. Formula de Rodrigues. 
    % - Incógnita(s): ángulo rotación rocker.
    % - Restricción(es): longitud damper (distancia entre FR_DPR_RKR y FR_DPR_MC)
        % vectores a rotar del rocker. Punto referencia FR_RKR_2 para Rodrigues.
        FR_RKR_AXIS_2_to_DPR = FR_DPR_RKR - FR_RKR_2 ;
        FR_RKR_AXIS_2_to_PUSH = FR_PUSH_RKR - FR_RKR_2 ;
        
        % Fórmula de Rodrigues
        rodrigues_rotation = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 
        
        % Función a solucionar. Busca el ángulo que hace que se consiga la longitud total deseada del damper
        FR_RKR_SOLVE_NEW_FUNCTION = @(FR_RKR_ANGLE) norm((FR_RKR_2 + rodrigues_rotation(FR_RKR_AXIS_2_to_DPR, FR_RKR_AXIS, FR_RKR_ANGLE)) - FR_DPR_MC) - (DPR_LENGTH - FR_DPR_COMPRESSION);
        
        % Solucionar ecuación anterior igualada a 0.
        FR_RKR_ANGLE = fzero(FR_RKR_SOLVE_NEW_FUNCTION, 0, optimset('Display', 'off'));

        % Definir nueva posición de los extremos del rocker.
        FR_DPR_RKR_FINAL_POINT = FR_RKR_2 + rodrigues_rotation(FR_RKR_AXIS_2_to_DPR, FR_RKR_AXIS, FR_RKR_ANGLE);
        FR_PUSH_RKR_FINAL_POINT = FR_RKR_2 + rodrigues_rotation(FR_RKR_AXIS_2_to_PUSH, FR_RKR_AXIS, FR_RKR_ANGLE);
    
    
    %% Definición de la nueva posición de Kingpin, con el cálculo de ángulos de wishbones asociado. Fórmula de RODRIGUEEEESSSSS
    % - Incógnita(s): Ángulo rotación upper wishbone, ángulo de rotación lower wishbone y ángulo de rotación mangueta alrededor del kingpin.
    % - Restricción(es): Distancia entre knuckles, longitud pushrod y longitud tie.
        % Ecuación 1. Distancia entre kunckles.
            % Vectores a rotar.
            FR_URW_MC_to_UW_KN = FR_UW_KN - FR_URW_MC; % vector a rotar upper wishbone. Referencia Rodrigues rear mc joint.
            FR_LRW_MC_to_LW_KN = FR_LW_KN - FR_LRW_MC; % vector a rota lower wishbone- Referencia Rodrigues rear mc joint.
            
            % Rotaciones
            FR_UW_KN_NEW_FUNCTION = @(x) FR_URW_MC + rodrigues_rotation(FR_URW_MC_to_UW_KN, FR_UW_AXIS, x(1));
            FR_LW_KN_NEW_FUNCTION = @(x) FR_LRW_MC + rodrigues_rotation(FR_LRW_MC_to_LW_KN, FR_LW_AXIS, x(2));
            
            % Función a solucionar. Distancia entre kunckles.
            FR_KNS_SOLVE_NEW_FUNCTION = @(x) norm(FR_UW_KN_NEW_FUNCTION(x) - FR_LW_KN_NEW_FUNCTION(x)) - F_KP_LENGTH; % x(1) es el ángulo del UW, x(2) es el ángulo de LW
            FR_KP_NEW_FUNCTION = @(x) (FR_LW_KN_NEW_FUNCTION(x) - FR_UW_KN_NEW_FUNCTION(x))/norm(FR_LW_KN_NEW_FUNCTION(x) - FR_UW_KN_NEW_FUNCTION(x));
       
        % Ecuación 2. Longitud tie.
            FR_STEER_ARM = FR_TR_UPRIGHT - FR_LW_KN;
           
            h_TR = dot(FR_STEER_ARM, FR_KP); % Cuánto sube el brazo por el eje
            v_rad_TR = FR_STEER_ARM - h_TR * FR_KP; % El resto del brazo (perpendicular)
    
            % 2. Rotación acoplada: inclina con el eje y luego gira x(3)
            FR_TR_UPRIGHT_NEW_FUNCTION = @(x) FR_LW_KN_NEW_FUNCTION(x) +  h_TR * FR_KP_NEW_FUNCTION(x) + rodrigues_rotation(v_rad_TR, FR_KP_NEW_FUNCTION(x), x(3));
    
            % Ecuación a resolver
            FR_TR_SOLVE_NEW_FUNCTION = @(x) norm(FR_TR_UPRIGHT_NEW_FUNCTION(x) - FR_TR_RACK_FINAL_POINT) - F_TR_LENGTH;
        
        % Ecuación 3. Longitud push.
            % Vector a rotar. (mc hasta push-uptight)
            FR_PUSH_MC = FR_PUSH_UW - FR_URW_MC;
    
            % Rotación.
            FR_PUSH_UW_NEW_FUNCTION = @(x) FR_URW_MC + rodrigues_rotation(FR_PUSH_MC, FR_UW_AXIS, x(1));
    
            % Ecuación a solucionar
            FR_PUSH_SOLVE_NEW_FUNCTION = @(x) norm(FR_PUSH_UW_NEW_FUNCTION(x) - FR_PUSH_RKR_FINAL_POINT) - F_PUSH_LENGTH;
    
        % RESOLUCIÓN FINAL
        FR_TOTAL_SOLVE_NEW_FUNCTION = @(x) [FR_KNS_SOLVE_NEW_FUNCTION(x); FR_TR_SOLVE_NEW_FUNCTION(x); FR_PUSH_SOLVE_NEW_FUNCTION(x)];
        x = fsolve(FR_TOTAL_SOLVE_NEW_FUNCTION, [0, 0, 0], optimoptions('fsolve', 'Display', 'off'));
        
    
    %% PUNTOS ACTUALIZADOS
         % KNUCKLES
         FR_UW_KN_FINAL_POINT = FR_UW_KN_NEW_FUNCTION(x);
         FR_LW_KN_FINAL_POINT = FR_LW_KN_NEW_FUNCTION(x);

         % Kingpin nuevo
         FR_KP_FINAL = FR_KP_NEW_FUNCTION(x);
        
         % TIE ROD
         FR_TR_UPRIGHT_FINAL_POINT = FR_TR_UPRIGHT_NEW_FUNCTION(x); 

         % PUSH
         FR_PUSH_UW_FINAL_POINT = FR_PUSH_UW_NEW_FUNCTION(x);
    
    
    % ROTACIÓN SPINDLE
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT CENTER
            FR_LW_KN_to_SPINDLE_CENTER = FR_SPINDLE_CENTER - FR_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            FR_LW_KN_to_SPINDLE_CENTER_axial_KP = dot(FR_LW_KN_to_SPINDLE_CENTER, FR_KP); % comoponente axial
            FR_LW_KN_to_SPINDLE_CENTER_radial_KP = FR_LW_KN_to_SPINDLE_CENTER - FR_LW_KN_to_SPINDLE_CENTER_axial_KP*FR_KP; % componente radial
        
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT INNER.
            FR_LW_KN_to_SPINDLE_INNER = FR_SPINDLE_INNER - FR_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            FR_LW_KN_to_SPINDLE_INNER_axial_KP = dot(FR_LW_KN_to_SPINDLE_INNER, FR_KP); % comoponente axial
            FR_LW_KN_to_SPINDLE_INNER_radial_KP = FR_LW_KN_to_SPINDLE_INNER - FR_LW_KN_to_SPINDLE_INNER_axial_KP*FR_KP; % componente radial
        
        % Nuevo spindle center
        FR_SPINDLE_CENTER_FINAL_POINT = FR_LW_KN_FINAL_POINT + FR_LW_KN_to_SPINDLE_CENTER_axial_KP * FR_KP_FINAL + rodrigues_rotation(FR_LW_KN_to_SPINDLE_CENTER_radial_KP, FR_KP_FINAL, x(3));
        
        % Nuevo spindle inner
        FR_SPINDLE_INNER_FINAL_POINT = FR_LW_KN_FINAL_POINT + FR_LW_KN_to_SPINDLE_INNER_axial_KP * FR_KP_FINAL + rodrigues_rotation(FR_LW_KN_to_SPINDLE_INNER_radial_KP, FR_KP_FINAL, x(3));
        
        % Nuevo spindle
        FR_SPINDLE_FINAL = (FR_SPINDLE_INNER_FINAL_POINT - FR_SPINDLE_CENTER_FINAL_POINT)/norm(FR_SPINDLE_INNER_FINAL_POINT - FR_SPINDLE_CENTER_FINAL_POINT); % nuevo vector spindle
    
    [~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,FR.LOADED_RADIUS] = mfeval_function(270*9.81/4, 0, 0, asind(FR_SPINDLE_FINAL(3)), 0);

    FR.URW_MC        = FR_URW_MC;
    FR.UFW_MC        = FR_UFW_MC;
    FR.LRW_MC        = FR_LRW_MC;
    FR.LFW_MC        = FR_LFW_MC;
    FR.UW_KN         = FR_UW_KN_FINAL_POINT;
    FR.LW_KN         = FR_LW_KN_FINAL_POINT;
    FR.KP            = FR_KP_FINAL;
    FR.TR_RACK       = FR_TR_RACK_FINAL_POINT;
    FR.TR_UPRIGHT    = FR_TR_UPRIGHT_FINAL_POINT;
    FR.DPR_RKR       = FR_DPR_RKR_FINAL_POINT;
    FR.PUSH_UW       = FR_PUSH_UW_FINAL_POINT;
    FR.PUSH_RKR      = FR_PUSH_RKR_FINAL_POINT;
    FR.SPINDLE_CENTER= FR_SPINDLE_CENTER_FINAL_POINT;
    FR.SPINDLE_INNER = FR_SPINDLE_INNER_FINAL_POINT;
    FR.SPINDLE       = FR_SPINDLE_FINAL;
    FR.DPR_MC        = FR_DPR_MC;
    FR.RKR_AXIS_1    = FR_RKR_1;
    FR.RKR_AXIS_2    = FR_RKR_2;
    
    % Cálculo contact patch
    FR_RADIAL_VECTOR = cross(cross(FR.SPINDLE, [0, 0, -1]), FR.SPINDLE);
    FR_RADIAL_VECTOR = FR_RADIAL_VECTOR/norm(FR_RADIAL_VECTOR);
    
    FR.CONTACT_PATCH = FR.SPINDLE_CENTER + (FR.LOADED_RADIUS * 1000) * FR_RADIAL_VECTOR;

    % Cálculo plano suelo
    FR.FLOOR_PLANE = [FLOOR_PARELLEL_PLANE, -dot(FLOOR_PARELLEL_PLANE, FR.CONTACT_PATCH)];
    
end



    












   


