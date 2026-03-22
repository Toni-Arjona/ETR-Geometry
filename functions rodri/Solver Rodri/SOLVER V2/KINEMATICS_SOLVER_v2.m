function Out = KINEMATICS_SOLVER_v2(HP, steering_wheel_angle, DPR_COMPRESSION, wheel_ID)

%% DEFINICIÓN DE CONSTANTES GEOMÉTRICAS y SOLUCIONES PREVIAS
persistent C prev_th

if isempty(C)
    C = struct();
    prev_th = struct('FL', zeros(1,5), 'FR', zeros(1,5), 'RL', zeros(1,5), 'RR', zeros(1,5));
end


if ~isfield(C, wheel_ID)
    % Longitudes fijas (restricciones)
    constant.PUSH_LENGTH        = norm(HP.PUSH_UW - HP.PUSH_RKR);
    constant.TR_LENGTH          = norm(HP.TR_UPRIGHT - HP.TR_RACK);
    constant.KP_LENGTH          = norm(HP.UW_KN - HP.LW_KN);
    constant.DPR_LENGTH         = norm(HP.DPR_MC - HP.DPR_RKR);
    constant.ARB_LINK_LENGTH    = norm(HP.AR_LINK_RKR - HP.AR_LINK_ARB);

    % Ejes normalizados
    constant.KP0                = (HP.LW_KN - HP.UW_KN)/norm((HP.LW_KN - HP.UW_KN));
    constant.UW_AXIS            = (HP.UFW_MC - HP.URW_MC)/norm(HP.UFW_MC - HP.URW_MC);
    constant.LW_AXIS            = (HP.LFW_MC - HP.LRW_MC)/norm(HP.LFW_MC - HP.LRW_MC);
    constant.RKR_AXIS           = (HP.RKR_2 - HP.RKR_1)/norm(HP.RKR_2 - HP.RKR_1);
    constant.ARB_AXIS           = [0, 1, 0];

    % Vectores a rotar
    % Rocker
    constant.RKR_2_to_DPR       = HP.DPR_RKR    - HP.RKR_2;
    constant.RKR_2_to_PUSH      = HP.PUSH_RKR   - HP.RKR_2;
    constant.RKR_2_to_AR        = HP.AR_LINK_RKR - HP.RKR_2;

    % ARB
    constant.AR_AXIS_to_ARB     = HP.AR_LINK_ARB - HP.AR_AXIS;

    % Wishbones
    constant.PUSH_MC_VEC        = HP.PUSH_UW - HP.URW_MC;
    constant.URW_to_UW_KN       = HP.UW_KN   - HP.URW_MC;
    constant.LRW_to_LW_KN       = HP.LW_KN   - HP.LRW_MC;

    % Steer arm — descomposición axial/radial sobre KP inicial
    SA                          = HP.TR_UPRIGHT - HP.LW_KN;
    constant.SA_axial           = dot(SA, constant.KP0);
    constant.SA_radial          = SA - constant.SA_axial * constant.KP0;

    % Spindle center — descomposición axial/radial sobre KP inicial
    SC                          = HP.SPINDLE_CENTER - HP.LW_KN;
    constant.SC_axial           = dot(SC, constant.KP0);
    constant.SC_radial          = SC - constant.SC_axial * constant.KP0;

    % Spindle inner — descomposición axial/radial sobre KP inicial
    SI                          = HP.SPINDLE_INNER - HP.LW_KN;
    constant.SI_axial           = dot(SI, constant.KP0);
    constant.SI_radial          = SI - constant.SI_axial * constant.KP0;

    % Hardpoints necesarios para cálculos (offsets y targets + hp modificados por inputs (steering/damper))
    constant.D_PINION           = HP.D_PINION;
    constant.TR_RACK            = HP.TR_RACK;   
    constant.RKR_2              = HP.RKR_2;
    constant.DPR_MC             = HP.DPR_MC;
    constant.AR_AXIS            = HP.AR_AXIS;
    constant.URW_MC             = HP.URW_MC;
    constant.LRW_MC             = HP.LRW_MC;

    C.(wheel_ID) = constant;
end



% Asignaciones de memoria correspondientes al wheel ID
constant   = C.(wheel_ID);
th0 = prev_th.(wheel_ID);   % ángulos de la última resolución, con identificador por rueda, anterior [rkr, arb, x1, x2, x3]

% Fórmula de Rodrigues
rod = @(v, k , th) v*cos(th) + cross(k,v)*sin(th) + k*dot(k,v)*(1-cos(th));

% INPUTS STEERING + DAMPERS
    % STEERING
    RACK_DISP           = deg2rad(steering_wheel_angle) * constant.D_PINION / 2;
    TR_RACK_FINAL       = constant.TR_RACK + [0, RACK_DISP, 0];

    % DAMPER 
    DAMPER_LENGTH_FINAL = constant.DPR_LENGTH - DPR_COMPRESSION;
    

%% ECUACIONES PARA RESOLVER
%% ECUACIÓN 1. ÁNGULO ROCKER. 
    RKR_ANGLE = th_solver(constant.RKR_2_to_DPR, constant.RKR_AXIS, constant.RKR_2, constant.DPR_MC, DAMPER_LENGTH_FINAL, th0(1));

    DPR_RKR_FINAL   = constant.RKR_2 + rod(constant.RKR_2_to_DPR,  constant.RKR_AXIS, RKR_ANGLE);
    PUSH_RKR_FINAL  = constant .RKR_2 + rod(constant.RKR_2_to_PUSH, constant.RKR_AXIS, RKR_ANGLE);
    AR_RKR_FINAL    = constant.RKR_2 + rod(constant.RKR_2_to_AR,   constant.RKR_AXIS, RKR_ANGLE);

%% ECUACIÓN 2. ÁNGULO ARB
    ARB_ANGLE = th_solver(constant.AR_AXIS_to_ARB, constant.ARB_AXIS, constant.AR_AXIS, AR_RKR_FINAL, constant.ARB_LINK_LENGTH, th0(2));

    AR_ARB_FINAL = constant.AR_AXIS + rod(constant.AR_AXIS_to_ARB, [0,1,0], ARB_ANGLE);

%% ECUACIÓN 3. ÁNGULO UPPER WISHBONE
    UW_ANGLE = th_solver(constant.PUSH_MC_VEC, constant.UW_AXIS, constant.URW_MC, PUSH_RKR_FINAL, constant.PUSH_LENGTH, th0(3));

    UW_KN_FINAL  = constant.URW_MC + rod(constant.URW_to_UW_KN, constant.UW_AXIS, UW_ANGLE);
    PUSH_UW_FINAL = constant.URW_MC + rod(constant.PUSH_MC_VEC,  constant.UW_AXIS, UW_ANGLE);

%% ECUACIÓN 4. ÁNGULO LOWER WISHBONE
    LW_ANGLE = th_solver(constant.LRW_to_LW_KN, constant.LW_AXIS, constant.LRW_MC, UW_KN_FINAL, constant.KP_LENGTH, th0(4));

    LW_KN_FINAL  = constant.LRW_MC + rod(constant.LRW_to_LW_KN, constant.LW_AXIS, LW_ANGLE);
    KP_FINAL     = (LW_KN_FINAL - UW_KN_FINAL) / norm(LW_KN_FINAL - UW_KN_FINAL);

    % CORRECCIÓN NUEVO KINGPIN
        tilt_axis   = @(k_old, k_new) cross(k_old, k_new) / (norm(cross(k_old, k_new)) + eps);
        tilt_angle  = @(k_old, k_new) acos(max(-1, min(1, dot(k_old, k_new))));
        tilt_vector = @(v, k_old, k_new) rod(v, tilt_axis(k_old, k_new), tilt_angle(k_old, k_new));
    
    % CORRECCIÓN COMPONENTES AXIALES SPINDLE Y STEERARM
        SA_tilted = tilt_vector(constant.SA_radial, constant.KP0, KP_FINAL);
        SC_tilted = tilt_vector(constant.SC_radial, constant.KP0, KP_FINAL);
        SI_tilted = tilt_vector(constant.SI_radial, constant.KP0, KP_FINAL);

%% ECUACIÓN 5. TOE.
    TR_offset = LW_KN_FINAL + constant.SA_axial * KP_FINAL;
    toe_angle = th_solver(SA_tilted, KP_FINAL, TR_offset, TR_RACK_FINAL, constant.TR_LENGTH, th0(5));

    TR_UP_FINAL = TR_offset + rod(SA_tilted, KP_FINAL, toe_angle);

prev_th.(wheel_ID) = [RKR_ANGLE, ARB_ANGLE, UW_ANGLE, LW_ANGLE, toe_angle];
    
    
% ROTACIÓN SPINDLE
    SPINDLE_CENTER_FINAL = LW_KN_FINAL + constant.SC_axial * KP_FINAL + rod(SC_tilted, KP_FINAL, toe_angle);
    SPINDLE_INNER_FINAL  = LW_KN_FINAL + constant.SI_axial * KP_FINAL + rod(SI_tilted, KP_FINAL, toe_angle);
    SPINDLE_FINAL        = (SPINDLE_INNER_FINAL - SPINDLE_CENTER_FINAL) / norm(SPINDLE_INNER_FINAL - SPINDLE_CENTER_FINAL);

    %% ── OUTPUTS (idénticos a ETR11_KINEMATICS_SOLVER) ────────────────────────
    Out.LRW_MC         = constant.LRW_MC;
    Out.LFW_MC         = HP.LFW_MC;
    Out.LW_KN          = LW_KN_FINAL;
    Out.URW_MC         = constant.URW_MC;
    Out.UFW_MC         = HP.UFW_MC;
    Out.UW_KN          = UW_KN_FINAL;
    Out.PUSH_UW        = PUSH_UW_FINAL;
    Out.PUSH_RKR       = PUSH_RKR_FINAL;
    Out.TR_UPRIGHT     = TR_UP_FINAL;
    Out.TR_RACK        = TR_RACK_FINAL;
    Out.DPR_MC         = constant.DPR_MC;
    Out.DPR_RKR        = DPR_RKR_FINAL;
    Out.SPINDLE_INNER  = SPINDLE_INNER_FINAL;
    Out.SPINDLE_CENTER = SPINDLE_CENTER_FINAL;
    Out.RKR_AXIS_1     = HP.RKR_1;
    Out.RKR_AXIS_2     = constant.RKR_2;
    Out.SPINDLE        = SPINDLE_FINAL;
    Out.RKR_ANGLE      = RKR_ANGLE;
    Out.KP             = KP_FINAL;
    Out.AR_RKR_LINK    = AR_RKR_FINAL;
    Out.AR_LINK_ARB    = AR_ARB_FINAL;
    Out.AR_AXIS        = constant.AR_AXIS;
    Out.ARB_ANGLE      = ARB_ANGLE;

    %% ── CONTACT PATCH ────────────────────────────────────────────────────────
    RADIAL = cross(cross(Out.SPINDLE, [0,0,-1]), Out.SPINDLE);
    RADIAL = RADIAL / norm(RADIAL);
    Out.CONTACT_PATCH = Out.SPINDLE_CENTER + (HP.SPINDLE_CENTER(3)) * RADIAL;

    %% ── PLANO DEL SUELO Y KP_FLOOR ───────────────────────────────────────────
    Out.FLOOR_PLANE = [0, 0, 1, -Out.CONTACT_PATCH(3)];
    lambda = (dot(Out.FLOOR_PLANE(1:3), Out.LW_KN) + Out.FLOOR_PLANE(4)) / dot(Out.FLOOR_PLANE(1:3), Out.KP);
    Out.KP_FLOOR = Out.LW_KN - lambda * Out.KP;

end    
    






    

