function [upper_knuckle, lower_knuckle, spindle_center, spindle_inner, tie_upright, tie_rack, l_tie_design] = get_roll_geometry(roll)
    
    % --- 1. DATOS DE DISEÑO ESTÁTICO (TUS PUNTOS ORIGINALES) ---
    % Mangueta (Outboard)
    uk_design = [-80, -520, 294]; 
    lk_design = [-96.5,-571, 112]; 
    
    % Spindle & Tie (Solidarios a la mangueta)
    sp_c_design = [-100, -625, 203]; 
    sp_i_design = [-100, -561.5, 200.5];
    tie_u_design = [-180, -560, 140];
    
    % Chasis (Inboard) - Puntos de anclaje trapecios
    uf_design = [-210, -225, 277]; ur_design = [80, -225, 268];
    lf_design = [-210, -225, 123]; lr_design = [85, -225, 136];
    
    % Rack (Cremallera)
    tie_r_design = [-170, -225, 149.39];

    % Longitudes Rígidas (Invariantes físicas)
    L_uf = norm(uk_design - uf_design);
    L_ur = norm(uk_design - ur_design);
    L_lf = norm(lk_design - lf_design);
    L_lr = norm(lk_design - lr_design);
    L_kp = norm(uk_design - lk_design); % Distancia Kingpin
    
    % Devolvemos la longitud de diseño de la barra de dirección para el solver principal
    l_tie_design = norm(tie_u_design - tie_r_design);

    % --- 2. SOLVER CINEMÁTICO ---
    
    if abs(roll) < 0.001
        % Si no hay roll, devolvemos los puntos de diseño tal cual
        upper_knuckle = uk_design; lower_knuckle = lk_design;
        spindle_center = sp_c_design; spindle_inner = sp_i_design;
        tie_upright = tie_u_design; tie_rack = tie_r_design;
        return;
    end

    % A. Rotar Puntos Inboard (Chasis) + Rack
    % Roll positivo = Lado izq comprime (Chasis baja Z -> Rotación positiva X)
    th = deg2rad(roll);
    Rx = [1 0 0; 0 cos(th) -sin(th); 0 sin(th) cos(th)];
    
    P_uf = (Rx * uf_design')'; P_ur = (Rx * ur_design')';
    P_lf = (Rx * lf_design')'; P_lr = (Rx * lr_design')';
    tie_rack = (Rx * tie_r_design')'; % El rack se mueve con el chasis
    
    % B. Resolver Lower Knuckle (lk)
    % Asumimos que la rueda mantiene su altura relativa al suelo (Z constante)
    % para simplificar la convergencia del solver.
    z_floor = lk_design(3); 
    
    fun_lo = @(xy) [norm([xy(1), xy(2), z_floor] - P_lf) - L_lf;
                    norm([xy(1), xy(2), z_floor] - P_lr) - L_lr];
    
    opts = optimset('Display','off', 'TolFun', 1e-6);
    xy_new = fsolve(fun_lo, lk_design(1:2), opts);
    lower_knuckle = [xy_new, z_floor];
    
    % C. Resolver Upper Knuckle (uk)
    % Intersección de 3 esferas: Wishbones Front/Rear y distancia Kingpin
    fun_up = @(xyz) [norm(xyz - P_uf) - L_uf;
                     norm(xyz - P_ur) - L_ur;
                     norm(xyz - lower_knuckle) - L_kp];
                 
    upper_knuckle = fsolve(fun_up, uk_design, opts);
    
    % D. Mover Spindle y Tie Upright (Transformación de Cuerpo Rígido)
    % Calculamos la rotación local de la mangueta (Old KP -> New KP)
    vec_old = (uk_design - lk_design)/L_kp;
    vec_new = (upper_knuckle - lower_knuckle)/L_kp;
    
    rot_axis = cross(vec_old, vec_new);
    sin_a = norm(rot_axis); cos_a = dot(vec_old, vec_new);
    
    if sin_a > 1e-7
        k = [0 -rot_axis(3) rot_axis(2); rot_axis(3) 0 -rot_axis(1); -rot_axis(2) rot_axis(1) 0] / sin_a;
        R_rigid = eye(3) + sin_a*k + (1-cos_a)*(k*k);
    else
        R_rigid = eye(3);
    end
    
    % Función para mover puntos solidarios
    transf = @(p) (R_rigid * (p - lk_design)')' + lower_knuckle;
    
    spindle_center = transf(sp_c_design);
    spindle_inner = transf(sp_i_design);
    tie_upright = transf(tie_u_design);
end