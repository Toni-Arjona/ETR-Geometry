function[FL_steer, FR_steer, FL_camber, FR_camber, FL_jack, FR_jack] = ackermann_function_3D(steering_wheel_angle)
    
    pinion_diameter = 35;
    
    upper_knuckle = [-80, -520, 294]; 
    lower_knuckle = [-96.5,-571, 112]; 
    spindle_center_ini = [-100, -625, 203]; 
    spindle_inner_point = [-100, -561.5, 200.5];
    tie_upright_joint_ini = [-180, -560, 140];
    tie_rack_joint_ini = [-170, -225, 149.39];
    l_tie = norm(tie_upright_joint_ini - tie_rack_joint_ini);
   
    % Puntos geometría. Todos los puntos corresponden a rueda delantera izquierda
    % Spindle. Esto es lo que determina toe y camber.
    spindle_vec_ini = (spindle_inner_point - spindle_center_ini)/ norm(spindle_inner_point - spindle_center_ini); % Vector unitario del spindle, inicial.
    
    % Knuckle
    kingpin_vector = (upper_knuckle - lower_knuckle)/norm(lower_knuckle - upper_knuckle); % vector unitario kingpin.
    

    % Vector brazo de dirección (para trackear rotación total)
    steerarm_vector_ini = tie_upright_joint_ini - lower_knuckle;
    
    % Vector radio desde el punto de pivote (lower knuckle) al centro de rueda. Se usa para el cálculo del jacking geométrico
    spindle_radius_vector_ini = spindle_center_ini - lower_knuckle; 
    
    % Formula Rodrigues: rotar un vector v (steerarm_vec), alrededor de un eje k (kingpin_vec), un ángulo th (total steer, contando variaciones en todos los ejes).
    rotated_vec = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 
    

    % --- CÁLCULO DE SCRUB RADIUS Y MECHANICAL TRAIL ---
    
    % 1. Definir el vector Kingpin (Dirección del eje de giro)
    kp_vec = upper_knuckle - lower_knuckle;
    
    % 2. Encontrar el punto de intersección con el suelo (Z=0)
    % Ecuación de la recta: P = LK + t * vector
    % Queremos encontrar 't' tal que la componente Z sea 0:
    % lower_knuckle(3) + t * kp_vec(3) = 0  =>  t = -lower_knuckle(3) / kp_vec(3)
    t_ground = -lower_knuckle(3) / kp_vec(3);
    
    % Kingpin Strike Point (KSP): Punto donde el eje toca el suelo
    ksp_ground = lower_knuckle + t_ground * kp_vec;
    
    % 3. Definir el centro de contacto del neumático (Contact Patch)
    % Proyectamos el centro del spindle al suelo (asumiendo Z=0 y sin deformación)
    tire_contact_patch = [spindle_center_ini(1), spindle_center_ini(2), 0];
    
    %% RUEDA EXTERIOR
    rack_disp = deg2rad(steering_wheel_angle)*(pinion_diameter/2);    
   
    tie_rack_joint = tie_rack_joint_ini + [0, rack_disp, 0]; % nueva posición unión tie-rack.
    
    cost_func = @(th) norm((lower_knuckle + rotated_vec(steerarm_vector_ini, kingpin_vector, th)) - tie_rack_joint) - l_tie; % función a resolver. Saca el ángulo que hacer que se mantenga la longitud del tie.
    
    theta_axis = fzero(cost_func, 0);
    
    spindle_final = rotated_vec(spindle_vec_ini, kingpin_vector, theta_axis); % posición final del vector spindle después de la rotación.
    
    steer_angle = atan2d(spindle_final(1), spindle_final(2)); % proyección del spindle sobre plano XY para obtener steer
    camber_angle = asind(spindle_final(3)); % proyección del spindle sobre plano perpendicular al suelo y dirección de la rueda
    
    ext_steer = abs(steer_angle);
    ext_camber = camber_angle;
   
    spindle_radius_final = rotated_vec(spindle_radius_vector_ini, kingpin_vector, theta_axis); % cálculo vector para jacking rotado   
    spindle_center_final = lower_knuckle + spindle_radius_final; % cálculo nueva posición centro de la rueda    
   
    ext_jacking = spindle_center_ini(3) - spindle_center_final(3); % jacking. Diferencia de altura entre 

    FL_steer = ext_steer;
    FL_camber = ext_camber;
    FL_jack = ext_jacking;
    
    %% RUEDA INTERIOR
   
    % Puntos geometría. Todos los puntos corresponden a rueda delantera izquierda
    % Spindle. Esto es lo que determina toe y camber.
    spindle_vec_ini = (spindle_inner_point - spindle_center_ini)/ norm(spindle_inner_point - spindle_center_ini); % Vector unitario del spindle, inicial.
    
    % Knuckle
    kingpin_vector = (upper_knuckle - lower_knuckle)/norm(lower_knuckle - upper_knuckle); % vector unitario kingpin.
    

    % Vector brazo de dirección (para trackear rotación total)
    steerarm_vector_ini = tie_upright_joint_ini - lower_knuckle;
    
    % Vector radio desde el punto de pivote (lower knuckle) al centro de rueda. Se usa para el cálculo del jacking geométrico
    spindle_radius_vector_ini = spindle_center_ini - lower_knuckle; 
    
    % Formula Rodrigues: rotar un vector v (steerarm_vec), alrededor de un eje k (kingpin_vec), un ángulo th (total steer, contando variaciones en todos los ejes).
    rotated_vec = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 

    rack_disp = -deg2rad(steering_wheel_angle)*(pinion_diameter/2);
            
    tie_rack_joint = tie_rack_joint_ini + [0, rack_disp, 0];
    
    cost_func = @(th) norm((lower_knuckle + rotated_vec(steerarm_vector_ini, kingpin_vector, th)) - tie_rack_joint) - l_tie;
    
    theta_axis = fzero(cost_func, 0);
    
    spindle_final = rotated_vec(spindle_vec_ini, kingpin_vector, theta_axis);
    
    steer_angle = atan2d(spindle_final(1), spindle_final(2));
    camber_angle = asind(spindle_final(3));
    
    int_steer = abs(steer_angle);
    int_camber = camber_angle;
   
    spindle_radius_final = rotated_vec(spindle_radius_vector_ini, kingpin_vector, theta_axis);
    spindle_center_final = lower_knuckle + spindle_radius_final;
    int_jacking= spindle_center_ini(3) - spindle_center_final(3);
          
    FR_steer = int_steer;
    FR_camber = int_camber;
    FR_jack = int_jacking;

    if steering_wheel_angle < 0
        FL_steer = -FL_steer;
        FR_steer = -FR_steer;
        
end

