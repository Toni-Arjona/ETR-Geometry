function Out = ETR11_KINEMATICS_SOLVER(HARDPOINTS, steering_wheel_angle, DPR_COMPRESSION)
    %% MAPEO DE HARDPOINTS A VARIABLES LOCALES GENÉRICAS
        SPINDLE_CENTER = HARDPOINTS.SPINDLE_CENTER; 
        SPINDLE_INNER  = HARDPOINTS.SPINDLE_INNER;
        UW_KN          = HARDPOINTS.UW_KN; 
        LW_KN          = HARDPOINTS.LW_KN;
        URW_MC         = HARDPOINTS.URW_MC;
        UFW_MC         = HARDPOINTS.UFW_MC;
        LRW_MC         = HARDPOINTS.LRW_MC;
        LFW_MC         = HARDPOINTS.LFW_MC;
        PUSH_UW        = HARDPOINTS.PUSH_UW;
        PUSH_RKR       = HARDPOINTS.PUSH_RKR;
        RKR_1          = HARDPOINTS.RKR_1;
        RKR_2          = HARDPOINTS.RKR_2;
        DPR_RKR        = HARDPOINTS.DPR_RKR;
        DPR_MC         = HARDPOINTS.DPR_MC;
        TR_UPRIGHT     = HARDPOINTS.TR_UPRIGHT;
        TR_RACK        = HARDPOINTS.TR_RACK;
        D_PINION       = HARDPOINTS.D_PINION;

    %% INPUTS STEERING
        RACK_DISPLACEMENT = deg2rad(steering_wheel_angle)*D_PINION/2;
        TR_RACK_FINAL_POINT = TR_RACK + [0, RACK_DISPLACEMENT, 0];
      
    %% FIXED DISTANCES DEFINITIONS
        % Push rod
        PUSH_LENGTH = norm(PUSH_UW - PUSH_RKR);
    
        % Tie rod
        TR_LENGTH = norm(TR_UPRIGHT - TR_RACK);
        
        % Upright
        KP_LENGTH   = norm(UW_KN - LW_KN);
    
        % Damper initial length
        DPR_LENGTH = norm(DPR_MC - DPR_RKR);
    
    %% AXIS DEFINITIONS. NORMALIZED VECTORS
        % Kingpin axis. Definido hacia abajo, para tener giro a derechas positivo.
        KP = (LW_KN - UW_KN)/norm((LW_KN - UW_KN));
        
        % Wishbone axis. Definido para valores de rotación positivos en compresión
        UW_AXIS = (UFW_MC - URW_MC)/norm(UFW_MC - URW_MC);
        LW_AXIS = (LFW_MC - LRW_MC)/norm(LFW_MC - LRW_MC);
        
        % Rocker axis. Definido para valores positivos de rotación en compresión.
        RKR_AXIS = (RKR_2 - RKR_1)/norm(RKR_2 - RKR_1);
                
    %% SOLVER
    %% Definición de la compresión del rocker y la rotación del rocker asociada. Formula de Rodrigues. 
        % - Incógnita(s): ángulo rotación rocker.
        % - Restricción(es): longitud damper (distancia entre DPR_RKR y DPR_MC)
        % vectores a rotar del rocker. Punto referencia RKR_2 para Rodrigues.
        RKR_AXIS_2_to_DPR = DPR_RKR - RKR_2 ;
        RKR_AXIS_2_to_PUSH = PUSH_RKR - RKR_2 ;
        
        % Fórmula de Rodrigues
        rodrigues_rotation = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 
        
        % Función a solucionar. Busca el ángulo que hace que se consiga la longitud total deseada del damper
        RKR_SOLVE_NEW_FUNCTION = @(RKR_ANGLE) norm((RKR_2 + rodrigues_rotation(RKR_AXIS_2_to_DPR, RKR_AXIS, RKR_ANGLE)) - DPR_MC) - (DPR_LENGTH - DPR_COMPRESSION);
        
        % Solucionar ecuación anterior igualada a 0.
        RKR_ANGLE = fzero(RKR_SOLVE_NEW_FUNCTION, 0, optimset('Display', 'off'));
    
        % Definir nueva posición de los extremos del rocker.
        DPR_RKR_FINAL_POINT = RKR_2 + rodrigues_rotation(RKR_AXIS_2_to_DPR, RKR_AXIS, RKR_ANGLE);
        PUSH_RKR_FINAL_POINT = RKR_2 + rodrigues_rotation(RKR_AXIS_2_to_PUSH, RKR_AXIS, RKR_ANGLE);
    
    %% Definición de la nueva posición de Kingpin, con el cálculo de ángulos de wishbones asociado. Fórmula de RODRIGUEEEESSSSS
    % - Incógnita(s): Ángulo rotación upper wishbone, ángulo de rotación lower wishbone y ángulo de rotación mangueta alrededor del kingpin.
    % - Restricción(es): Distancia entre knuckles, longitud pushrod y longitud tie.
        % Ecuación 1. Distancia entre kunckles.
            % Vectores a rotar.
            URW_MC_to_UW_KN = UW_KN - URW_MC; % vector a rotar upper wishbone. Referencia Rodrigues rear mc joint.
            LRW_MC_to_LW_KN = LW_KN - LRW_MC; % vector a rota lower wishbone- Referencia Rodrigues rear mc joint.
            
            % Rotaciones
            UW_KN_NEW_FUNCTION = @(x) URW_MC + rodrigues_rotation(URW_MC_to_UW_KN, UW_AXIS, x(1));
            LW_KN_NEW_FUNCTION = @(x) LRW_MC + rodrigues_rotation(LRW_MC_to_LW_KN, LW_AXIS, x(2));
        
            % Función a solucionar. Distancia entre kunckles.
            KNS_SOLVE_NEW_FUNCTION = @(x) norm(UW_KN_NEW_FUNCTION(x) - LW_KN_NEW_FUNCTION(x)) - KP_LENGTH; % x(1) es el ángulo del UW, x(2) es el ángulo de LW
            KP_NEW_FUNCTION = @(x) (LW_KN_NEW_FUNCTION(x) - UW_KN_NEW_FUNCTION(x))/norm(LW_KN_NEW_FUNCTION(x) - UW_KN_NEW_FUNCTION(x));
           
       % Ecuación 2. Longitud tie. 
            STEER_ARM = TR_UPRIGHT - LW_KN;
           
            h_TR = dot(STEER_ARM, KP); % Cuánto sube el brazo por el eje (Escrito por gemini, me da palo)
            v_rad_TR = STEER_ARM - h_TR * KP; % El resto del brazo (perpendicular) (Escrito por gemini, me da palo)
    
            % 2. Rotación acoplada: inclina con el eje y luego gira x(3)
            TR_UPRIGHT_NEW_FUNCTION = @(x) LW_KN_NEW_FUNCTION(x) +  h_TR * KP_NEW_FUNCTION(x) + rodrigues_rotation(v_rad_TR, KP_NEW_FUNCTION(x), x(3)); %(Escrito por gemini, me da palo)
        
            % Ecuación a resolver
            TR_SOLVE_NEW_FUNCTION = @(x) norm(TR_UPRIGHT_NEW_FUNCTION(x) - TR_RACK_FINAL_POINT) - TR_LENGTH; %(Escrito por gemini, me da palo)
        
        % Ecuación 3. Longitud push.
            % Vector a rotar. (mc hasta push-uptight)
            PUSH_MC_VEC = PUSH_UW - URW_MC;
        
            % Rotación.
            PUSH_UW_NEW_FUNCTION = @(x) URW_MC + rodrigues_rotation(PUSH_MC_VEC, UW_AXIS, x(1));
        
            % Ecuación a solucionar
            PUSH_SOLVE_NEW_FUNCTION = @(x) norm(PUSH_UW_NEW_FUNCTION(x) - PUSH_RKR_FINAL_POINT) - PUSH_LENGTH;
    
        % RESOLUCIÓN FINAL
        TOTAL_SOLVE_NEW_FUNCTION = @(x) [KNS_SOLVE_NEW_FUNCTION(x); TR_SOLVE_NEW_FUNCTION(x); PUSH_SOLVE_NEW_FUNCTION(x)];
        x = fsolve(TOTAL_SOLVE_NEW_FUNCTION, [0, 0, 0], optimoptions('fsolve', 'Display', 'off'));
    
    %% PUNTOS ACTUALIZADOS
        % KNUCKLES
        UW_KN_FINAL_POINT = UW_KN_NEW_FUNCTION(x);
        LW_KN_FINAL_POINT = LW_KN_NEW_FUNCTION(x);
    
        % Kingpin nuevo
        KP_FINAL = KP_NEW_FUNCTION(x);
    
        % TIE ROD
        TR_UPRIGHT_FINAL_POINT = TR_UPRIGHT_NEW_FUNCTION(x);
         
        % PUSH
        PUSH_UW_FINAL_POINT = PUSH_UW_NEW_FUNCTION(x);
       
    %% ROTACIÓN SPINDLE
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT CENTER
            LW_KN_to_SPINDLE_CENTER = SPINDLE_CENTER - LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            LW_KN_to_SPINDLE_CENTER_axial_KP = dot(LW_KN_to_SPINDLE_CENTER, KP); % comoponente axial
            LW_KN_to_SPINDLE_CENTER_radial_KP = LW_KN_to_SPINDLE_CENTER - LW_KN_to_SPINDLE_CENTER_axial_KP*KP; % componente radial
    
        % DESCOMPOSICIÓN AXIAL Y RADIAL RESPECTO A KINGPIN SPINDLE POINT INNER.
            LW_KN_to_SPINDLE_INNER = SPINDLE_INNER - LW_KN; % Vector inicial de lower knuckle a spindle center
        
            % Descomposición vector en componente axial y radial sobre kingpin
            LW_KN_to_SPINDLE_INNER_axial_KP = dot(LW_KN_to_SPINDLE_INNER, KP); % comoponente axial
            LW_KN_to_SPINDLE_INNER_radial_KP = LW_KN_to_SPINDLE_INNER - LW_KN_to_SPINDLE_INNER_axial_KP*KP; % componente radial

        % Nuevo spindle center
        SPINDLE_CENTER_FINAL_POINT = LW_KN_FINAL_POINT + LW_KN_to_SPINDLE_CENTER_axial_KP * KP_FINAL + rodrigues_rotation(LW_KN_to_SPINDLE_CENTER_radial_KP, KP_FINAL, x(3));
    
        % Nuevo spindle inner
        SPINDLE_INNER_FINAL_POINT = LW_KN_FINAL_POINT + LW_KN_to_SPINDLE_INNER_axial_KP * KP_FINAL + rodrigues_rotation(LW_KN_to_SPINDLE_INNER_radial_KP, KP_FINAL, x(3));
    
        % Nuevo spindle
        SPINDLE_FINAL = (SPINDLE_INNER_FINAL_POINT - SPINDLE_CENTER_FINAL_POINT)/norm(SPINDLE_INNER_FINAL_POINT - SPINDLE_CENTER_FINAL_POINT); % nuevo vector spindle

    [~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,Out.LOADED_RADIUS ] = mfeval_function(270*9.81/4, 0, 0, asind(SPINDLE_FINAL(3)), 0);

    Out.URW_MC        = URW_MC ;
    Out.UFW_MC        = UFW_MC ;
    Out.LRW_MC        = LRW_MC ;
    Out.LFW_MC        = LFW_MC ;
    Out.UW_KN         = UW_KN_FINAL_POINT;
    Out.LW_KN         = LW_KN_FINAL_POINT;
    Out.KP            = KP_FINAL;
    Out.TR_RACK       = TR_RACK_FINAL_POINT;
    Out.TR_UPRIGHT    = TR_UPRIGHT_FINAL_POINT;
    Out.DPR_RKR       = DPR_RKR_FINAL_POINT;
    Out.PUSH_UW       = PUSH_UW_FINAL_POINT;
    Out.PUSH_RKR      = PUSH_RKR_FINAL_POINT;
    Out.SPINDLE_CENTER= SPINDLE_CENTER_FINAL_POINT;
    Out.SPINDLE_INNER = SPINDLE_INNER_FINAL_POINT;
    Out.SPINDLE       = SPINDLE_FINAL;
    Out.DPR_MC        = DPR_MC ;
    Out.RKR_AXIS_1    = RKR_1 ;
    Out.RKR_AXIS_2    = RKR_2 ;

    % Cálculo contact patch
    RADIAL_VECTOR = cross(cross(Out.SPINDLE, [0, 0, -1]), Out.SPINDLE);
    RADIAL_VECTOR = RADIAL_VECTOR/norm(RADIAL_VECTOR);
    
    Out.CONTACT_PATCH = Out.SPINDLE_CENTER + (Out.LOADED_RADIUS * 1000) * RADIAL_VECTOR;
    
    % Cálculo plano del suelo en la rueda, respecto a monocasco fijo
    FLOOR_PARELLEL_PLANE = cross([1, 0, 0], [0, 1, 0]);

    Out.FLOOR_PLANE = [FLOOR_PARELLEL_PLANE, -dot(FLOOR_PARELLEL_PLANE, Out.CONTACT_PATCH)];
end