function Out = ETR11_KINEMATICS_SOLVER(HP, steering_wheel_angle, DPR_COMPRESSION, LOADED_RADIUS, wheel_ID)

    %% CACHÉ DE CONSTANTES POR RUEDA (solo se computan la primera vez)
    persistent C prev_x prev_rkr prev_arb
    if isempty(C)
        C        = struct();
        prev_x   = struct('FL', [0,0,0], 'FR', [0,0,0], 'RL', [0,0,0], 'RR', [0,0,0]);
        prev_rkr = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
        prev_arb = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
    end

    if ~isfield(C, wheel_ID)
        c.PUSH_LENGTH = norm(HP.PUSH_UW  - HP.PUSH_RKR);
        c.TR_LENGTH   = norm(HP.TR_UPRIGHT - HP.TR_RACK);
        c.KP_LENGTH   = norm(HP.UW_KN    - HP.LW_KN);
        c.DPR_LENGTH  = norm(HP.DPR_MC   - HP.DPR_RKR);

        c.KP       = (HP.LW_KN   - HP.UW_KN)   / norm(HP.LW_KN   - HP.UW_KN);
        c.UW_AXIS  = (HP.UFW_MC  - HP.URW_MC)  / norm(HP.UFW_MC  - HP.URW_MC);
        c.LW_AXIS  = (HP.LFW_MC  - HP.LRW_MC)  / norm(HP.LFW_MC  - HP.LRW_MC);
        c.RKR_AXIS = (HP.RKR_2   - HP.RKR_1)   / norm(HP.RKR_2   - HP.RKR_1);

        c.RKR_2_to_DPR  = HP.DPR_RKR    - HP.RKR_2;
        c.RKR_2_to_PUSH = HP.PUSH_RKR   - HP.RKR_2;
        c.RKR_2_to_AR   = HP.AR_LINK_RKR - HP.RKR_2;

        c.ARB_LINK_LENGTH = norm(HP.AR_LINK_RKR - HP.AR_LINK_ARB);
        c.AR_AXIS_to_ARB  = HP.AR_LINK_ARB - HP.AR_AXIS;

        c.PUSH_MC_VEC  = HP.PUSH_UW - HP.URW_MC;
        c.URW_to_UW_KN = HP.UW_KN   - HP.URW_MC;
        c.LRW_to_LW_KN = HP.LW_KN   - HP.LRW_MC;

        STEER_ARM_INI  = HP.TR_UPRIGHT - HP.LW_KN;
        c.SA_axial     = dot(STEER_ARM_INI, c.KP);
        c.SA_radial    = STEER_ARM_INI - c.SA_axial * c.KP;

        LW_to_SC    = HP.SPINDLE_CENTER - HP.LW_KN;
        c.SC_axial  = dot(LW_to_SC, c.KP);
        c.SC_radial = LW_to_SC - c.SC_axial * c.KP;

        LW_to_SI    = HP.SPINDLE_INNER - HP.LW_KN;
        c.SI_axial  = dot(LW_to_SI, c.KP);
        c.SI_radial = LW_to_SI - c.SI_axial * c.KP;

        % Campos de HP necesarios fuera del bloque de caché
        c.D_PINION    = HP.D_PINION;
        c.TR_RACK     = HP.TR_RACK;
        c.RKR_2       = HP.RKR_2;
        c.DPR_MC      = HP.DPR_MC;
        c.AR_AXIS     = HP.AR_AXIS;
        c.AR_LINK_ARB = HP.AR_LINK_ARB;
        c.URW_MC      = HP.URW_MC;
        c.LRW_MC      = HP.LRW_MC;
        c.UFW_MC      = HP.UFW_MC;
        c.LFW_MC      = HP.LFW_MC;
        c.RKR_1       = HP.RKR_1;

        C.(wheel_ID) = c;
    end
    c = C.(wheel_ID);

    %% FÓRMULA DE RODRIGUES
    rodrigues_rotation = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th));

    %% INPUTS STEERING
    RACK_DISPLACEMENT   = deg2rad(steering_wheel_angle) * c.D_PINION / 2;
    TR_RACK_FINAL_POINT = c.TR_RACK + [0, RACK_DISPLACEMENT, 0];

    %% SOLVER ROCKER
    RKR_SOLVE = @(a) norm(c.RKR_2 + rodrigues_rotation(c.RKR_2_to_DPR, c.RKR_AXIS, a) - c.DPR_MC) - (c.DPR_LENGTH - DPR_COMPRESSION);
    RKR_ANGLE = fzero(RKR_SOLVE, prev_rkr.(wheel_ID), optimset('Display', 'off'));
    prev_rkr.(wheel_ID) = RKR_ANGLE;

    DPR_RKR_FINAL_POINT     = c.RKR_2 + rodrigues_rotation(c.RKR_2_to_DPR,  c.RKR_AXIS, RKR_ANGLE);
    PUSH_RKR_FINAL_POINT    = c.RKR_2 + rodrigues_rotation(c.RKR_2_to_PUSH, c.RKR_AXIS, RKR_ANGLE);
    AR_LINK_RKR_FINAL_POINT = c.RKR_2 + rodrigues_rotation(c.RKR_2_to_AR,   c.RKR_AXIS, RKR_ANGLE);

    %% ANTI-ROLL
    AR_NEW    = @(a) c.AR_AXIS + rodrigues_rotation(c.AR_AXIS_to_ARB, [0, 1, 0], a);
    ARB_ANGLE = fzero(@(a) norm(AR_NEW(a) - AR_LINK_RKR_FINAL_POINT) - c.ARB_LINK_LENGTH, prev_arb.(wheel_ID), optimset('Display', 'off'));
    prev_arb.(wheel_ID) = ARB_ANGLE;
    AR_LINK_ARB_FINAL_POINT = AR_NEW(ARB_ANGLE);

    %% SOLVER CINEMÁTICA (wishbones + tie rod)
    tilt_axis   = @(k_old, k_new) cross(k_old, k_new) / (norm(cross(k_old, k_new)) + eps);
    tilt_angle  = @(k_old, k_new) acos(max(-1, min(1, dot(k_old, k_new))));
    tilt_vector = @(v, k_old, k_new) rodrigues_rotation(v, tilt_axis(k_old, k_new), tilt_angle(k_old, k_new));

    UW_KN_F   = @(x) c.URW_MC + rodrigues_rotation(c.URW_to_UW_KN, c.UW_AXIS, x(1));
    LW_KN_F   = @(x) c.LRW_MC + rodrigues_rotation(c.LRW_to_LW_KN, c.LW_AXIS, x(2));
    KP_F      = @(x) (LW_KN_F(x) - UW_KN_F(x)) / norm(LW_KN_F(x) - UW_KN_F(x));
    PUSH_UW_F = @(x) c.URW_MC + rodrigues_rotation(c.PUSH_MC_VEC,   c.UW_AXIS, x(1));
    TR_UP_F   = @(x) LW_KN_F(x) + ...
                     c.SA_axial * KP_F(x) + ...
                     rodrigues_rotation(tilt_vector(c.SA_radial, c.KP, KP_F(x)), KP_F(x), x(3));

    TOTAL_F = @(x) [norm(UW_KN_F(x)  - LW_KN_F(x))            - c.KP_LENGTH;   ...
                    norm(TR_UP_F(x)   - TR_RACK_FINAL_POINT)   - c.TR_LENGTH;   ...
                    norm(PUSH_UW_F(x) - PUSH_RKR_FINAL_POINT)  - c.PUSH_LENGTH];

    x = fsolve(TOTAL_F, prev_x.(wheel_ID), optimoptions('fsolve', 'Display', 'off'));
    prev_x.(wheel_ID) = x;

    %% PUNTOS ACTUALIZADOS
    UW_KN_FINAL   = UW_KN_F(x);
    LW_KN_FINAL   = LW_KN_F(x);
    KP_FINAL      = KP_F(x);
    TR_UP_FINAL   = TR_UP_F(x);
    PUSH_UW_FINAL = PUSH_UW_F(x);

    %% ROTACIÓN SPINDLE
    SPINDLE_CENTER_FINAL = LW_KN_FINAL + ...
        c.SC_axial * KP_FINAL + ...
        rodrigues_rotation(tilt_vector(c.SC_radial, c.KP, KP_FINAL), KP_FINAL, x(3));

    SPINDLE_INNER_FINAL = LW_KN_FINAL + ...
        c.SI_axial * KP_FINAL + ...
        rodrigues_rotation(tilt_vector(c.SI_radial, c.KP, KP_FINAL), KP_FINAL, x(3));

    SPINDLE_FINAL = (SPINDLE_INNER_FINAL - SPINDLE_CENTER_FINAL) / norm(SPINDLE_INNER_FINAL - SPINDLE_CENTER_FINAL);

    %% OUTPUTS
    Out.LRW_MC         = c.LRW_MC;
    Out.LFW_MC         = c.LFW_MC;
    Out.LW_KN          = LW_KN_FINAL;
    Out.URW_MC         = c.URW_MC;
    Out.UFW_MC         = c.UFW_MC;
    Out.UW_KN          = UW_KN_FINAL;
    Out.PUSH_UW        = PUSH_UW_FINAL;
    Out.PUSH_RKR       = PUSH_RKR_FINAL_POINT;
    Out.TR_UPRIGHT     = TR_UP_FINAL;
    Out.TR_RACK        = TR_RACK_FINAL_POINT;
    Out.DPR_MC         = c.DPR_MC;
    Out.DPR_RKR        = DPR_RKR_FINAL_POINT;
    Out.SPINDLE_INNER  = SPINDLE_INNER_FINAL;
    Out.SPINDLE_CENTER = SPINDLE_CENTER_FINAL;
    Out.RKR_AXIS_1     = c.RKR_1;
    Out.RKR_AXIS_2     = c.RKR_2;
    Out.SPINDLE        = SPINDLE_FINAL;
    Out.RKR_ANGLE      = RKR_ANGLE;
    Out.KP             = KP_FINAL;
    Out.AR_RKR_LINK    = AR_LINK_RKR_FINAL_POINT;
    Out.AR_LINK_ARB    = AR_LINK_ARB_FINAL_POINT;
    Out.AR_AXIS        = c.AR_AXIS;
    Out.ARB_ANGLE      = ARB_ANGLE;

    %% CONTACT PATCH
    RADIAL_VECTOR = cross(cross(Out.SPINDLE, [0, 0, -1]), Out.SPINDLE);
    RADIAL_VECTOR = RADIAL_VECTOR / norm(RADIAL_VECTOR);
    Out.CONTACT_PATCH = Out.SPINDLE_CENTER + (LOADED_RADIUS * 1000) * RADIAL_VECTOR;

    %% PLANO DEL SUELO Y KINGPIN FLOOR
    Out.FLOOR_PLANE = [0, 0, 1, -Out.CONTACT_PATCH(3)];
    lambda_KP_FLOOR = (dot(Out.FLOOR_PLANE(1:3), Out.LW_KN) + Out.FLOOR_PLANE(4)) / dot(Out.FLOOR_PLANE(1:3), Out.KP);
    Out.KP_FLOOR = Out.LW_KN - lambda_KP_FLOOR * Out.KP;

end
