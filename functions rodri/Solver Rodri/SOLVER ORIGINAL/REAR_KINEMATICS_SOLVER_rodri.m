function[RL, RR] = REAR_KINEMATICS_SOLVER_ETR11_rodri(RL_DPR_COMPRESSION, RR_DPR_COMPRESSION)
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
    
    %% REAR LEFT WHEEL
    %% COORDINATES DEFINITIONS. POINTS
        % Wheel spindle 
        RL_SPINDLE_CENTER = [1435, -625, 203]; 
        RL_SPINDLE_INNER = [1435, -561.5, 200.5];
               
        % RL Knuckles
        RL_UW_KN = [1435, -551.5, 294]; 
        RL_LW_KN = [1435 ,-576, 111];
        
        % RL wishbones joints with monocoque
        RL_URW_MC = [1545, -255, 281];
        RL_UFW_MC = [1265, -255, 294];
        RL_LRW_MC = [1545, -255, 124];
        RL_LFW_MC = [1250, -255, 151];
        
        % RL Push rod
        RL_PUSH_UW = [1435, -517, 306.5];
        RL_PUSH_RKR = [1435, -206, 487];
        
        % RL Rocker axis points
        RL_RKR_1 = [1373, -228, 497];
        RL_RKR_2 = [1373, -205, 463];
        
        % RL Damper
        RL_DPR_RKR = [1359, -161, 518];
        RL_DPR_MC = [1151, -161, 518];

        % RL Tie rod
        RL_TR_UPRIGHT = [1517, -560, 140];
        RL_TR_RACK = [1528, -255, 155];

      
    %% FIXED DISTANCES DEFINITIONS
        % Push rod
        R_PUSH_LENGTH = norm(RL_PUSH_UW - RL_PUSH_RKR);
    
        % Tie rod
        R_TR_LENGTH = norm(RL_TR_UPRIGHT - RL_TR_RACK);
        
        % Upright
        R_KP_LENGTH   = norm(RL_UW_KN - RL_LW_KN);
        
        % Damper initial length
        DPR_LENGTH = norm(RL_DPR_MC - RL_DPR_RKR);
    
    
    %% AXIS DEFINITIONS. NORMALIZED VECTORS
        % RL Kingpin axis. Definido hacia abajo, para tener giro a derechas positivo.
        RL_KP = (RL_LW_KN - RL_UW_KN)/norm((RL_LW_KN - RL_UW_KN));
        
        % RL Wishbone axis. Definido para valores de rotación positivos en compresión
        RL_UW_AXIS = (RL_UFW_MC - RL_URW_MC)/norm(RL_UFW_MC - RL_URW_MC);
        RL_LW_AXIS = (RL_LFW_MC - RL_LRW_MC)/norm(RL_LFW_MC - RL_LRW_MC);
        
        % RL rocker axis. Definido para valores positivos de rotación en compresión.
        RL_RKR_AXIS = (RL_RKR_2 - RL_RKR_1)/norm(RL_RKR_2 - RL_RKR_1);
             
    %% SOLVER
    %% Definición de la compresión del rocker y la rotación del rocker asociada. Formula de Rodrigues. 
    % - Incógnita(s): ángulo rotación rocker.
    % - Restricción(es): longitud damper (distancia entre RL_DPR_RKR y RL_DPR_MC)
        % vectores a rotar del rocker. Punto referencia RL_RKR_2 para Rodrigues.
        RL_RKR_AXIS_2_to_DPR = RL_DPR_RKR - RL_RKR_2 ;
        RL_RKR_AXIS_2_to_PUSH = RL_PUSH_RKR - RL_RKR_2 ;
        
        % Fórmula de Rodrigues
        rodrigues_rotation = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 
        
        % Función a solucionar. Busca el ángulo que hace que se consiga la longitud total deseada del damper
        RL_RKR_SOLVE_NEW_FUNCTION = @(RL_RKR_ANGLE) norm((RL_RKR_2 + rodrigues_rotation(RL_RKR_AXIS_2_to_DPR, RL_RKR_AXIS, RL_RKR_ANGLE)) - RL_DPR_MC) - (DPR_LENGTH - RL_DPR_COMPRESSION);
        
        % Solucionar ecuación anterior igualada a 0.
        RL_RKR_ANGLE = fzero(RL_RKR_SOLVE_NEW_FUNCTION, 0, optimset('Display', 'off'));

        % Definir nueva posición de los extremos del rocker.
        RL_DPR_RKR_FINAL_POINT = RL_RKR_2 + rodrigues_rotation(RL_RKR_AXIS_2_to_DPR, RL_RKR_AXIS, RL_RKR_ANGLE);
        RL_PUSH_RKR_FINAL_POINT = RL_RKR_2 + rodrigues_rotation(RL_RKR_AXIS_2_to_PUSH, RL_RKR_AXIS, RL_RKR_ANGLE);
    
    
    %% Definición de la nueva posición de Kingpin, con el cálculo de ángulos de wishbones asociado. Fórmula de RODRIGUEEEESSSSS
    % - Incógnita(s): Ángulo rotación upper wishbone, ángulo de rotación lower wishbone y ángulo de rotación mangueta alrededor del kingpin.
    % - Restricción(es): Distancia entre knuckles, longitud pushrod y longitud tie.
        % Ecuación 1. Distancia entre kunckles.
            % Vectores a rotar.
            RL_URW_MC_to_UW_KN = RL_UW_KN - RL_URW_MC; % vector a rotar upper wishbone. Referencia Rodrigues rear mc joint.
            RL_LRW_MC_to_LW_KN = RL_LW_KN - RL_LRW_MC; % vector a rota lower wishbone- Referencia Rodrigues rear mc joint.
            
            % Rotaciones
            RL_UW_KN_NEW_FUNCTION = @(x) RL_URW_MC + rodrigues_rotation(RL_URW_MC_to_UW_KN, RL_UW_AXIS, x(1));
            RL_LW_KN_NEW_FUNCTION = @(x) RL_LRW_MC + rodrigues_rotation(RL_LRW_MC_to_LW_KN, RL_LW_AXIS, x(2));

            % Función a solucionar. Distancia entre kunckles.
            RL_KNS_SOLVE_NEW_FUNCTION = @(x) norm(RL_UW_KN_NEW_FUNCTION(x) - RL_LW_KN_NEW_FUNCTION(x)) - R_KP_LENGTH; % x(1) es el ángulo del UW, x(2) es el ángulo de LW
            RL_KP_NEW_FUNCTION = @(x) (RL_LW_KN_NEW_FUNCTION(x) - RL_UW_KN_NEW_FUNCTION(x))/norm(RL_LW_KN_NEW_FUNCTION(x) - RL_UW_KN_NEW_FUNCTION(x));
       
        % Ecuación 2. Longitud tie. 
            RL_STEER_ARM = RL_TR_UPRIGHT - RL_LW_KN;
           
            h_TR = dot(RL_STEER_ARM, RL_KP); % Cuánto sube el brazo por el eje (Escrito por gemini, me da palo)
            v_rad_TR = RL_STEER_ARM - h_TR * RL_KP; % El resto del brazo (perpendicular) (Escrito por gemini, me da palo)
    
            % 2. Rotación acoplada: inclina con el eje y luego gira x(3)
            RL_TR_UPRIGHT_NEW_FUNCTION = @(x) RL_LW_KN_NEW_FUNCTION(x) +  h_TR * RL_KP_NEW_FUNCTION(x) + rodrigues_rotation(v_rad_TR, RL_KP_NEW_FUNCTION(x), x(3)); %(Escrito por gemini, me da palo)
    
            % Ecuación a resolver
            RL_TR_SOLVE_NEW_FUNCTION = @(x) norm(RL_TR_UPRIGHT_NEW_FUNCTION(x) - RL_TR_RACK) - R_TR_LENGTH; %(Escrito por gemini, me da palo)
        
        % Ecuación 3. Longitud push.
            % Vector a rotar. (mc hasta push-uptight)
            RL_PUSH_MC = RL_PUSH_UW - RL_URW_MC;
    
            % Rotación.
            RL_PUSH_UW_NEW_FUNCTION = @(x) RL_URW_MC + rodrigues_rotation(RL_PUSH_MC, RL_UW_AXIS, x(1));
    
            % Ecuación a solucionar
            RL_PUSH_SOLVE_NEW_FUNCTION = @(x) norm(RL_PUSH_UW_NEW_FUNCTION(x) - RL_PUSH_RKR_FINAL_POINT) - R_PUSH_LENGTH;
    
        % RESOLUCIÓN FINAL
        RL_TOTAL_SOLVE_NEW_FUNCTION = @(x) [RL_KNS_SOLVE_NEW_FUNCTION(x); RL_TR_SOLVE_NEW_FUNCTION(x); RL_PUSH_SOLVE_NEW_FUNCTION(x)];
        x = fsolve(RL_TOTAL_SOLVE_NEW_FUNCTION, [0, 0, 0], optimoptions('fsolve', 'Display', 'off'));
        
    
    %% PUNTOS ACTUALIZADOS
         % KNUCKLES
         RL_UW_KN_FINAL_POINT = RL_UW_KN_NEW_FUNCTION(x);
         RL_LW_KN_FINAL_POINT = RL_LW_KN_NEW_FUNCTION(x);

         % Kingpin nuevo
         RL_KP_FINAL = RL_KP_NEW_FUNCTION(x);
    
         % TIE ROD
         RL_TR_UPRIGHT_FINAL_POINT = RL_TR_UPRIGHT_NEW_FUNCTION(x); 

         % PUSH
         RL_PUSH_UW_FINAL_POINT = RL_PUSH_UW_NEW_FUNCTION(x);
    
    
    % ROTACIÓN SPINDLE
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT CENTER
            RL_LW_KN_to_SPINDLE_CENTER = RL_SPINDLE_CENTER - RL_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            RL_LW_KN_to_SPINDLE_CENTER_axial_KP = dot(RL_LW_KN_to_SPINDLE_CENTER, RL_KP); % comoponente axial
            RL_LW_KN_to_SPINDLE_CENTER_radial_KP = RL_LW_KN_to_SPINDLE_CENTER - RL_LW_KN_to_SPINDLE_CENTER_axial_KP*RL_KP; % componente radial
        
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT INNER.
            RL_LW_KN_to_SPINDLE_INNER = RL_SPINDLE_INNER - RL_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            RL_LW_KN_to_SPINDLE_INNER_axial_KP = dot(RL_LW_KN_to_SPINDLE_INNER, RL_KP); % comoponente axial
            RL_LW_KN_to_SPINDLE_INNER_radial_KP = RL_LW_KN_to_SPINDLE_INNER - RL_LW_KN_to_SPINDLE_INNER_axial_KP*RL_KP; % componente radial
        
        % Nuevo spindle center
        RL_SPINDLE_CENTER_FINAL_POINT = RL_LW_KN_FINAL_POINT + RL_LW_KN_to_SPINDLE_CENTER_axial_KP * RL_KP_FINAL + rodrigues_rotation(RL_LW_KN_to_SPINDLE_CENTER_radial_KP, RL_KP_FINAL, x(3));
        
        % Nuevo spindle inner
        RL_SPINDLE_INNER_FINAL_POINT = RL_LW_KN_FINAL_POINT + RL_LW_KN_to_SPINDLE_INNER_axial_KP * RL_KP_FINAL + rodrigues_rotation(RL_LW_KN_to_SPINDLE_INNER_radial_KP, RL_KP_FINAL, x(3));
        
        % Nuevo spindle
        RL_SPINDLE_FINAL = (RL_SPINDLE_INNER_FINAL_POINT - RL_SPINDLE_CENTER_FINAL_POINT)/norm(RL_SPINDLE_INNER_FINAL_POINT - RL_SPINDLE_CENTER_FINAL_POINT); % nuevo vector spindle
    
    [~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,RL.LOADED_RADIUS ] = mfeval_function(270*9.81/4, 0, 0, asind(RL_SPINDLE_FINAL(3)), 0);

    RL.URW_MC        = RL_URW_MC ;
    RL.UFW_MC        = RL_UFW_MC ;
    RL.LRW_MC        = RL_LRW_MC ;
    RL.LFW_MC        = RL_LFW_MC ;
    RL.UW_KN         = RL_UW_KN_FINAL_POINT;
    RL.LW_KN         = RL_LW_KN_FINAL_POINT;
    RL.KP            = RL_KP_FINAL;
    RL.TR_RACK       = RL_TR_RACK;
    RL.TR_UPRIGHT    = RL_TR_UPRIGHT_FINAL_POINT;
    RL.DPR_RKR       = RL_DPR_RKR_FINAL_POINT;
    RL.PUSH_UW       = RL_PUSH_UW_FINAL_POINT;
    RL.PUSH_RKR      = RL_PUSH_RKR_FINAL_POINT;
    RL.SPINDLE_CENTER= RL_SPINDLE_CENTER_FINAL_POINT;
    RL.SPINDLE_INNER = RL_SPINDLE_INNER_FINAL_POINT;
    RL.SPINDLE       = RL_SPINDLE_FINAL;
    RL.DPR_MC        = RL_DPR_MC ;
    RL.RKR_AXIS_1    = RL_RKR_1 ;
    RL.RKR_AXIS_2    = RL_RKR_2 ;

    % Cálculo contact patch
    RL_RADIAL_VECTOR = cross(cross(RL.SPINDLE, [0, 0, -1]), RL.SPINDLE);
    RL_RADIAL_VECTOR = RL_RADIAL_VECTOR/norm(RL_RADIAL_VECTOR);
    
    RL.CONTACT_PATCH = RL.SPINDLE_CENTER + (RL.LOADED_RADIUS * 1000) * RL_RADIAL_VECTOR;

    % Cálculo plano del suelo en la rueda, respecto a monocasco fijo
    FLOOR_PARELLEL_PLANE = cross([1, 0, 0], [0, 1, 0]);

    RL.FLOOR_PLANE = [FLOOR_PARELLEL_PLANE, -dot(FLOOR_PARELLEL_PLANE, RL.CONTACT_PATCH)];
   
        
    %% REAR RIGHT WHEEL
    %% COORDINATES DEFINITIONS. POINTS
        % Wheel spindle 
        RR_SPINDLE_CENTER = RL_SPINDLE_CENTER .* [1, -1, 1]; 
        RR_SPINDLE_INNER  = RL_SPINDLE_INNER .* [1, -1, 1];

        % RR Knuckles
        RR_UW_KN = RL_UW_KN .* [1, -1, 1]; 
        RR_LW_KN = RL_LW_KN .* [1, -1, 1];
        
        % RR wishbones joints with monocoque
        RR_URW_MC = RL_URW_MC .* [1, -1, 1];
        RR_UFW_MC = RL_UFW_MC .* [1, -1, 1];
        RR_LRW_MC = RL_LRW_MC .* [1, -1, 1];
        RR_LFW_MC = RL_LFW_MC .* [1, -1, 1];
        
        % RR Push rod
        RR_PUSH_UW  = RL_PUSH_UW .* [1, -1, 1];
        RR_PUSH_RKR = RL_PUSH_RKR .* [1, -1, 1];
        
        % RR Rocker axis points
        RR_RKR_1 = RL_RKR_1 .* [1, -1, 1];
        RR_RKR_2 = RL_RKR_2 .* [1, -1, 1];
        
        % RR Damper
        RR_DPR_RKR = RL_DPR_RKR .* [1, -1, 1];
        RR_DPR_MC  = RL_DPR_MC .* [1, -1, 1];
        
        % RR Tie rod
        RR_TR_UPRIGHT = RL_TR_UPRIGHT .* [1, -1, 1];
        RR_TR_RACK    = RL_TR_RACK .* [1, -1, 1];
    
    
    %% AXIS DEFINITIONS. NORMALIZED VECTORS
        % RR Kingpin axis. Definido hacia abajo, para tener giro a derechas positivo.
        RR_KP = (RR_LW_KN - RR_UW_KN)/norm((RR_LW_KN - RR_UW_KN));
        
        % RR Wishbone axis. Definido para valores de rotación positivos en compresión
        RR_UW_AXIS = (RR_UFW_MC - RR_URW_MC)/norm(RR_UFW_MC - RR_URW_MC);
        RR_LW_AXIS = (RR_LFW_MC - RR_LRW_MC)/norm(RR_LFW_MC - RR_LRW_MC);
        
        % RR rocker axis. Definido para valores positivos de rotación en compresión.
        RR_RKR_AXIS = (RR_RKR_2 - RR_RKR_1)/norm(RR_RKR_2 - RR_RKR_1);
        
    
    %% SOLVER
    %% Definición de la compresión del rocker y la rotación del rocker asociada. Formula de Rodrigues. 
    % - Incógnita(s): ángulo rotación rocker.
    % - Restricción(es): longitud damper (distancia entre RR_DPR_RKR y RR_DPR_MC)
        % vectores a rotar del rocker. Punto referencia RR_RKR_2 para Rodrigues.
        RR_RKR_AXIS_2_to_DPR = RR_DPR_RKR - RR_RKR_2 ;
        RR_RKR_AXIS_2_to_PUSH = RR_PUSH_RKR - RR_RKR_2 ;
        
        % Fórmula de Rodrigues
        rodrigues_rotation = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 
        
        % Función a solucionar. Busca el ángulo que hace que se consiga la longitud total deseada del damper
        RR_RKR_SOLVE_NEW_FUNCTION = @(RR_RKR_ANGLE) norm((RR_RKR_2 + rodrigues_rotation(RR_RKR_AXIS_2_to_DPR, RR_RKR_AXIS, RR_RKR_ANGLE)) - RR_DPR_MC) - (DPR_LENGTH - RR_DPR_COMPRESSION);
        
        % Solucionar ecuación anterior igualada a 0.
        RR_RKR_ANGLE = fzero(RR_RKR_SOLVE_NEW_FUNCTION, 0, optimset('Display', 'off'));

        % Definir nueva posición de los extremos del rocker.
        RR_DPR_RKR_FINAL_POINT = RR_RKR_2 + rodrigues_rotation(RR_RKR_AXIS_2_to_DPR, RR_RKR_AXIS, RR_RKR_ANGLE);
        RR_PUSH_RKR_FINAL_POINT = RR_RKR_2 + rodrigues_rotation(RR_RKR_AXIS_2_to_PUSH, RR_RKR_AXIS, RR_RKR_ANGLE);
    
    
    %% Definición de la nueva posición de Kingpin, con el cálculo de ángulos de wishbones asociado. Fórmula de RODRIGUEEEESSSSS
    % - Incógnita(s): Ángulo rotación upper wishbone, ángulo de rotación lower wishbone y ángulo de rotación mangueta alrededor del kingpin.
    % - Restricción(es): Distancia entre knuckles, longitud pushrod y longitud tie.
        % Ecuación 1. Distancia entre kunckles.
            % Vectores a rotar.
            RR_URW_MC_to_UW_KN = RR_UW_KN - RR_URW_MC; % vector a rotar upper wishbone. Referencia Rodrigues rear mc joint.
            RR_LRW_MC_to_LW_KN = RR_LW_KN - RR_LRW_MC; % vector a rota lower wishbone- Referencia Rodrigues rear mc joint.
            
            % Rotaciones
            RR_UW_KN_NEW_FUNCTION = @(x) RR_URW_MC + rodrigues_rotation(RR_URW_MC_to_UW_KN, RR_UW_AXIS, x(1));
            RR_LW_KN_NEW_FUNCTION = @(x) RR_LRW_MC + rodrigues_rotation(RR_LRW_MC_to_LW_KN, RR_LW_AXIS, x(2));
            
            % Función a solucionar. Distancia entre kunckles.
            RR_KNS_SOLVE_NEW_FUNCTION = @(x) norm(RR_UW_KN_NEW_FUNCTION(x) - RR_LW_KN_NEW_FUNCTION(x)) - R_KP_LENGTH; % x(1) es el ángulo del UW, x(2) es el ángulo de LW
            RR_KP_NEW_FUNCTION = @(x) (RR_LW_KN_NEW_FUNCTION(x) - RR_UW_KN_NEW_FUNCTION(x))/norm(RR_LW_KN_NEW_FUNCTION(x) - RR_UW_KN_NEW_FUNCTION(x));
       
        % Ecuación 2. Longitud tie.
            RR_STEER_ARM = RR_TR_UPRIGHT - RR_LW_KN;
           
            h_TR = dot(RR_STEER_ARM, RR_KP); % Cuánto sube el brazo por el eje
            v_rad_TR = RR_STEER_ARM - h_TR * RR_KP; % El resto del brazo (perpendicular)
    
            % 2. Rotación acoplada: inclina con el eje y luego gira x(3)
            RR_TR_UPRIGHT_NEW_FUNCTION = @(x) RR_LW_KN_NEW_FUNCTION(x) +  h_TR * RR_KP_NEW_FUNCTION(x) + rodrigues_rotation(v_rad_TR, RR_KP_NEW_FUNCTION(x), x(3));
    
            % Ecuación a resolver
            RR_TR_SOLVE_NEW_FUNCTION = @(x) norm(RR_TR_UPRIGHT_NEW_FUNCTION(x) - RR_TR_RACK) - R_TR_LENGTH;
        
        % Ecuación 3. Longitud push.
            % Vector a rotar. (mc hasta push-uptight)
            RR_PUSH_MC = RR_PUSH_UW - RR_URW_MC;
    
            % Rotación.
            RR_PUSH_UW_NEW_FUNCTION = @(x) RR_URW_MC + rodrigues_rotation(RR_PUSH_MC, RR_UW_AXIS, x(1));
    
            % Ecuación a solucionar
            RR_PUSH_SOLVE_NEW_FUNCTION = @(x) norm(RR_PUSH_UW_NEW_FUNCTION(x) - RR_PUSH_RKR_FINAL_POINT) - R_PUSH_LENGTH;
    
        % RESOLUCIÓN FINAL
        RR_TOTAL_SOLVE_NEW_FUNCTION = @(x) [RR_KNS_SOLVE_NEW_FUNCTION(x); RR_TR_SOLVE_NEW_FUNCTION(x); RR_PUSH_SOLVE_NEW_FUNCTION(x)];
        x = fsolve(RR_TOTAL_SOLVE_NEW_FUNCTION, [0, 0, 0], optimoptions('fsolve', 'Display', 'off'));

    %% PUNTOS ACTUALIZADOS
         % KNUCKLES
         RR_UW_KN_FINAL_POINT = RR_UW_KN_NEW_FUNCTION(x);
         RR_LW_KN_FINAL_POINT = RR_LW_KN_NEW_FUNCTION(x);
         
         % Kingpin nuevo
         RR_KP_FINAL = RR_KP_NEW_FUNCTION(x);
        
         % TIE ROD
         RR_TR_UPRIGHT_FINAL_POINT = RR_TR_UPRIGHT_NEW_FUNCTION(x); 

         % PUSH
         RR_PUSH_UW_FINAL_POINT = RR_PUSH_UW_NEW_FUNCTION(x);
    
    
    % ROTACIÓN SPINDLE
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT CENTER
            RR_LW_KN_to_SPINDLE_CENTER = RR_SPINDLE_CENTER - RR_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            RR_LW_KN_to_SPINDLE_CENTER_axial_KP = dot(RR_LW_KN_to_SPINDLE_CENTER, RR_KP); % comoponente axial
            RR_LW_KN_to_SPINDLE_CENTER_radial_KP = RR_LW_KN_to_SPINDLE_CENTER - RR_LW_KN_to_SPINDLE_CENTER_axial_KP*RR_KP; % componente radial
        
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT INNER.
            RR_LW_KN_to_SPINDLE_INNER = RR_SPINDLE_INNER - RR_LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            RR_LW_KN_to_SPINDLE_INNER_axial_KP = dot(RR_LW_KN_to_SPINDLE_INNER, RR_KP); % comoponente axial
            RR_LW_KN_to_SPINDLE_INNER_radial_KP = RR_LW_KN_to_SPINDLE_INNER - RR_LW_KN_to_SPINDLE_INNER_axial_KP*RR_KP; % componente radial
        
        % Nuevo spindle center
        RR_SPINDLE_CENTER_FINAL_POINT = RR_LW_KN_FINAL_POINT + RR_LW_KN_to_SPINDLE_CENTER_axial_KP * RR_KP_FINAL + rodrigues_rotation(RR_LW_KN_to_SPINDLE_CENTER_radial_KP, RR_KP_FINAL, x(3));
        
        % Nuevo spindle inner
        RR_SPINDLE_INNER_FINAL_POINT = RR_LW_KN_FINAL_POINT + RR_LW_KN_to_SPINDLE_INNER_axial_KP * RR_KP_FINAL + rodrigues_rotation(RR_LW_KN_to_SPINDLE_INNER_radial_KP, RR_KP_FINAL, x(3));
        
        % Nuevo spindle
        RR_SPINDLE_FINAL = (RR_SPINDLE_INNER_FINAL_POINT - RR_SPINDLE_CENTER_FINAL_POINT)/norm(RR_SPINDLE_INNER_FINAL_POINT - RR_SPINDLE_CENTER_FINAL_POINT); % nuevo vector spindle
    
    [~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,RR.LOADED_RADIUS] = mfeval_function(270*9.81/4, 0, 0, asind(RR_SPINDLE_FINAL(3)), 0);

    RR.URW_MC        = RR_URW_MC;
    RR.UFW_MC        = RR_UFW_MC;
    RR.LRW_MC        = RR_LRW_MC;
    RR.LFW_MC        = RR_LFW_MC;
    RR.UW_KN         = RR_UW_KN_FINAL_POINT;
    RR.LW_KN         = RR_LW_KN_FINAL_POINT;
    RR.KP            = RR_KP_FINAL;
    RR.TR_RACK       = RR_TR_RACK;
    RR.TR_UPRIGHT    = RR_TR_UPRIGHT_FINAL_POINT;
    RR.DPR_RKR       = RR_DPR_RKR_FINAL_POINT;
    RR.PUSH_UW       = RR_PUSH_UW_FINAL_POINT;
    RR.PUSH_RKR      = RR_PUSH_RKR_FINAL_POINT;
    RR.SPINDLE_CENTER= RR_SPINDLE_CENTER_FINAL_POINT;
    RR.SPINDLE_INNER = RR_SPINDLE_INNER_FINAL_POINT;
    RR.SPINDLE       = RR_SPINDLE_FINAL;
    RR.DPR_MC        = RR_DPR_MC;
    RR.RKR_AXIS_1    = RR_RKR_1;
    RR.RKR_AXIS_2    = RR_RKR_2;
    
    % Cálculo contact patch
    RR_RADIAL_VECTOR = cross(cross(RR.SPINDLE, [0, 0, -1]), RR.SPINDLE);
    RR_RADIAL_VECTOR = RR_RADIAL_VECTOR/norm(RR_RADIAL_VECTOR);
    
    RR.CONTACT_PATCH = RR.SPINDLE_CENTER + (RR.LOADED_RADIUS * 1000) * RR_RADIAL_VECTOR;

    % Cálculo plano del suelo en la rueda, respecto a monocasco fijo
    RR.FLOOR_PLANE = [FLOOR_PARELLEL_PLANE, -dot(FLOOR_PARELLEL_PLANE, RR.CONTACT_PATCH)];


    
end