function Out = ETR11_KINEMATICS_ANALYTICAL(HP, steering_wheel_angle, DPR_COMPRESSION, LOADED_RADIUS, wheel_ID)
% ETR11_KINEMATICS_ANALYTICAL
% Versión analítica pura de ETR11_KINEMATICS_SOLVER. Misma interfaz, mismo output.
%
% Reemplaza los 3 solvers numéricos (2x fzero + 1x fsolve) por 5 soluciones
% analíticas en cascada. Cero iteraciones, cero tolerancias, cero callbacks.
%
% FUNDAMENTO MATEMÁTICO
% ─────────────────────
% Todas las restricciones tienen la forma:
%
%   ‖ offset + rodrigues(v, k, θ) - target ‖ = L
%
% Descomponiendo rodrigues en componentes axial/radial respecto a k:
%   rodrigues(v, k, θ) = a + r·cos(θ) + s·sin(θ)
%   con  a = dot(k,v)·k,   r = v − a,   s = cross(k,v)
%
% Sustituyendo y expandiendo (dot(r,s)=0, |r|=|s|):
%
%   B·cos(θ) + C·sin(θ) = D
%
%   B = dot(q, r),   C = dot(q, s),   D = (|r|² + |q|² − L²) / 2
%   q = target − offset − a
%
% Solución cerrada:   θ = atan2(C, B) ± acos(D / √(B²+C²))
%
% Selección de raíz: la más próxima al ángulo del frame anterior (equivalente
% al rol que cumplía la semilla caliente en los solvers numéricos).
%
% ORDEN DE RESOLUCIÓN (cascada)
% ──────────────────────────────
%   1. RKR_ANGLE  — restricción longitud amortiguador
%   2. ARB_ANGLE  — restricción longitud biela ARB
%   3. x1 (UW)    — restricción longitud pushrod       [solo depende de x1]
%   4. x2 (LW)    — restricción distancia entre knuckles [depende de x1, x2]
%   5. x3 (steer) — restricción longitud tirante        [depende de x1, x2, x3]
%
% USO
% ───
% Sustituye ETR11_KINEMATICS_SOLVER en ETR11_GET_POINTS cambiando las 4
% llamadas a ETR11_KINEMATICS_ANALYTICAL. Para resetear la caché:
%   >> clear ETR11_KINEMATICS_ANALYTICAL


    %% ── CACHÉ DE CONSTANTES (solo se computan la primera vez por rueda) ──────
    persistent C prev_th
    if isempty(C)
        C       = struct();
        prev_th = struct('FL', zeros(1,5), 'FR', zeros(1,5), ...
                         'RL', zeros(1,5), 'RR', zeros(1,5));
    end

    if ~isfield(C, wheel_ID)
        % Longitudes fijas
        c.PUSH_LENGTH = norm(HP.PUSH_UW   - HP.PUSH_RKR);
        c.TR_LENGTH   = norm(HP.TR_UPRIGHT - HP.TR_RACK);
        c.KP_LENGTH   = norm(HP.UW_KN     - HP.LW_KN);
        c.DPR_LENGTH  = norm(HP.DPR_MC    - HP.DPR_RKR);

        % Ejes normalizados
        c.KP0      = (HP.LW_KN  - HP.UW_KN)  / norm(HP.LW_KN  - HP.UW_KN);
        c.UW_AXIS  = (HP.UFW_MC - HP.URW_MC) / norm(HP.UFW_MC - HP.URW_MC);
        c.LW_AXIS  = (HP.LFW_MC - HP.LRW_MC) / norm(HP.LFW_MC - HP.LRW_MC);
        c.RKR_AXIS = (HP.RKR_2  - HP.RKR_1)  / norm(HP.RKR_2  - HP.RKR_1);

        % Vectores Rodrigues del rocker (referencia en RKR_2)
        c.RKR_2_to_DPR  = HP.DPR_RKR    - HP.RKR_2;
        c.RKR_2_to_PUSH = HP.PUSH_RKR   - HP.RKR_2;
        c.RKR_2_to_AR   = HP.AR_LINK_RKR - HP.RKR_2;

        % ARB
        c.ARB_LINK_LENGTH = norm(HP.AR_LINK_RKR - HP.AR_LINK_ARB);
        c.AR_AXIS_to_ARB  = HP.AR_LINK_ARB - HP.AR_AXIS;

        % Wishbones
        c.PUSH_MC_VEC  = HP.PUSH_UW - HP.URW_MC;
        c.URW_to_UW_KN = HP.UW_KN   - HP.URW_MC;
        c.LRW_to_LW_KN = HP.LW_KN   - HP.LRW_MC;

        % Steer arm — descomposición axial/radial sobre KP inicial
        SA               = HP.TR_UPRIGHT - HP.LW_KN;
        c.SA_axial       = dot(SA, c.KP0);
        c.SA_radial      = SA - c.SA_axial * c.KP0;

        % Spindle center — descomposición axial/radial sobre KP inicial
        SC               = HP.SPINDLE_CENTER - HP.LW_KN;
        c.SC_axial       = dot(SC, c.KP0);
        c.SC_radial      = SC - c.SC_axial * c.KP0;

        % Spindle inner — descomposición axial/radial sobre KP inicial
        SI               = HP.SPINDLE_INNER - HP.LW_KN;
        c.SI_axial       = dot(SI, c.KP0);
        c.SI_radial      = SI - c.SI_axial * c.KP0;

        % Campos HP necesarios tras el bloque de caché
        c.D_PINION    = HP.D_PINION;
        c.TR_RACK     = HP.TR_RACK;
        c.RKR_2       = HP.RKR_2;
        c.RKR_1       = HP.RKR_1;
        c.DPR_MC      = HP.DPR_MC;
        c.AR_AXIS     = HP.AR_AXIS;
        c.URW_MC      = HP.URW_MC;
        c.LRW_MC      = HP.LRW_MC;
        c.UFW_MC      = HP.UFW_MC;
        c.LFW_MC      = HP.LFW_MC;

        C.(wheel_ID) = c;
    end

    c   = C.(wheel_ID);
    th0 = prev_th.(wheel_ID);   % ángulos del frame anterior [rkr, arb, x1, x2, x3]

    %% ── RODRIGUES (inline, sin overhead de feval) ────────────────────────────
    rod = @(v, k, th) v*cos(th) + cross(k,v)*sin(th) + k*dot(k,v)*(1-cos(th));

    %% ── STEERING ─────────────────────────────────────────────────────────────
    RACK_DISP           = deg2rad(steering_wheel_angle) * c.D_PINION / 2;
    TR_RACK_FINAL       = c.TR_RACK + [0, RACK_DISP, 0];

    %% ── 1. ROCKER ANGLE ──────────────────────────────────────────────────────
    % Restricción: ‖ RKR_2 + rod(RKR_2_to_DPR, RKR_AXIS, θ) − DPR_MC ‖ = DPR_LENGTH − compression
    RKR_ANGLE = solve_rod_angle(c.RKR_2_to_DPR, c.RKR_AXIS, c.RKR_2, ...
                                c.DPR_MC, c.DPR_LENGTH - DPR_COMPRESSION, th0(1));

    DPR_RKR_FINAL   = c.RKR_2 + rod(c.RKR_2_to_DPR,  c.RKR_AXIS, RKR_ANGLE);
    PUSH_RKR_FINAL  = c.RKR_2 + rod(c.RKR_2_to_PUSH, c.RKR_AXIS, RKR_ANGLE);
    AR_RKR_FINAL    = c.RKR_2 + rod(c.RKR_2_to_AR,   c.RKR_AXIS, RKR_ANGLE);

    %% ── 2. ARB ANGLE ─────────────────────────────────────────────────────────
    % Restricción: ‖ AR_AXIS + rod(AR_AXIS_to_ARB, [0,1,0], θ) − AR_RKR_FINAL ‖ = ARB_LINK_LENGTH
    ARB_ANGLE = solve_rod_angle(c.AR_AXIS_to_ARB, [0,1,0], c.AR_AXIS, ...
                                AR_RKR_FINAL, c.ARB_LINK_LENGTH, th0(2));

    AR_ARB_FINAL = c.AR_AXIS + rod(c.AR_AXIS_to_ARB, [0,1,0], ARB_ANGLE);

    %% ── 3. UW ANGLE (x1) — restricción pushrod ───────────────────────────────
    % Restricción: ‖ URW_MC + rod(PUSH_MC_VEC, UW_AXIS, x1) − PUSH_RKR_FINAL ‖ = PUSH_LENGTH
    x1 = solve_rod_angle(c.PUSH_MC_VEC, c.UW_AXIS, c.URW_MC, ...
                         PUSH_RKR_FINAL, c.PUSH_LENGTH, th0(3));

    UW_KN_FINAL  = c.URW_MC + rod(c.URW_to_UW_KN, c.UW_AXIS, x1);
    PUSH_UW_FINAL = c.URW_MC + rod(c.PUSH_MC_VEC,  c.UW_AXIS, x1);

    %% ── 4. LW ANGLE (x2) — restricción distancia entre knuckles ─────────────
    % Restricción: ‖ LRW_MC + rod(LRW_to_LW_KN, LW_AXIS, x2) − UW_KN_FINAL ‖ = KP_LENGTH
    x2 = solve_rod_angle(c.LRW_to_LW_KN, c.LW_AXIS, c.LRW_MC, ...
                         UW_KN_FINAL, c.KP_LENGTH, th0(4));

    LW_KN_FINAL  = c.LRW_MC + rod(c.LRW_to_LW_KN, c.LW_AXIS, x2);
    KP_FINAL     = (LW_KN_FINAL - UW_KN_FINAL) / norm(LW_KN_FINAL - UW_KN_FINAL);

    %% ── ROTACIÓN DE INCLINACIÓN KP0 → KP_FINAL (se reutiliza 3 veces) ───────
    % Rota cualquier vector que vivía en el plano ⊥ KP0 al plano ⊥ KP_FINAL
    tilt_cross = cross(c.KP0, KP_FINAL);
    tilt_s     = norm(tilt_cross);
    if tilt_s > 1e-12
        tilt_k   = tilt_cross / tilt_s;
        tilt_ang = acos(max(-1, min(1, dot(c.KP0, KP_FINAL))));
        do_tilt  = @(v) rod(v, tilt_k, tilt_ang);
    else
        do_tilt  = @(v) v;   % KP prácticamente sin cambio
    end

    SA_tilted = do_tilt(c.SA_radial);
    SC_tilted = do_tilt(c.SC_radial);
    SI_tilted = do_tilt(c.SI_radial);

    %% ── 5. STEER ANGLE (x3) — restricción tirante ───────────────────────────
    % TR_UPRIGHT = TR_offset + rod(SA_tilted, KP_FINAL, x3)
    % Restricción: ‖ TR_UPRIGHT − TR_RACK_FINAL ‖ = TR_LENGTH
    TR_offset = LW_KN_FINAL + c.SA_axial * KP_FINAL;
    x3 = solve_rod_angle(SA_tilted, KP_FINAL, TR_offset, ...
                         TR_RACK_FINAL, c.TR_LENGTH, th0(5));

    TR_UP_FINAL = TR_offset + rod(SA_tilted, KP_FINAL, x3);

    %% ── GUARDAR ÁNGULOS PARA EL SIGUIENTE FRAME ─────────────────────────────
    prev_th.(wheel_ID) = [RKR_ANGLE, ARB_ANGLE, x1, x2, x3];

    %% ── SPINDLE ──────────────────────────────────────────────────────────────
    SPINDLE_CENTER_FINAL = LW_KN_FINAL + c.SC_axial * KP_FINAL + rod(SC_tilted, KP_FINAL, x3);
    SPINDLE_INNER_FINAL  = LW_KN_FINAL + c.SI_axial * KP_FINAL + rod(SI_tilted, KP_FINAL, x3);
    SPINDLE_FINAL        = (SPINDLE_INNER_FINAL - SPINDLE_CENTER_FINAL) / ...
                            norm(SPINDLE_INNER_FINAL - SPINDLE_CENTER_FINAL);

    %% ── OUTPUTS (idénticos a ETR11_KINEMATICS_SOLVER) ────────────────────────
    Out.LRW_MC         = c.LRW_MC;
    Out.LFW_MC         = c.LFW_MC;
    Out.LW_KN          = LW_KN_FINAL;
    Out.URW_MC         = c.URW_MC;
    Out.UFW_MC         = c.UFW_MC;
    Out.UW_KN          = UW_KN_FINAL;
    Out.PUSH_UW        = PUSH_UW_FINAL;
    Out.PUSH_RKR       = PUSH_RKR_FINAL;
    Out.TR_UPRIGHT     = TR_UP_FINAL;
    Out.TR_RACK        = TR_RACK_FINAL;
    Out.DPR_MC         = c.DPR_MC;
    Out.DPR_RKR        = DPR_RKR_FINAL;
    Out.SPINDLE_INNER  = SPINDLE_INNER_FINAL;
    Out.SPINDLE_CENTER = SPINDLE_CENTER_FINAL;
    Out.RKR_AXIS_1     = c.RKR_1;
    Out.RKR_AXIS_2     = c.RKR_2;
    Out.SPINDLE        = SPINDLE_FINAL;
    Out.RKR_ANGLE      = RKR_ANGLE;
    Out.KP             = KP_FINAL;
    Out.AR_RKR_LINK    = AR_RKR_FINAL;
    Out.AR_LINK_ARB    = AR_ARB_FINAL;
    Out.AR_AXIS        = c.AR_AXIS;
    Out.ARB_ANGLE      = ARB_ANGLE;

    %% ── CONTACT PATCH ────────────────────────────────────────────────────────
    RADIAL = cross(cross(Out.SPINDLE, [0,0,-1]), Out.SPINDLE);
    RADIAL = RADIAL / norm(RADIAL);
    Out.CONTACT_PATCH = Out.SPINDLE_CENTER + (LOADED_RADIUS * 1000) * RADIAL;

    %% ── PLANO DEL SUELO Y KP_FLOOR ───────────────────────────────────────────
    Out.FLOOR_PLANE = [0, 0, 1, -Out.CONTACT_PATCH(3)];
    lambda = (dot(Out.FLOOR_PLANE(1:3), Out.LW_KN) + Out.FLOOR_PLANE(4)) / ...
              dot(Out.FLOOR_PLANE(1:3), Out.KP);
    Out.KP_FLOOR = Out.LW_KN - lambda * Out.KP;

end


%% ═══════════════════════════════════════════════════════════════════════════
function theta = solve_rod_angle(v, k, offset, target, L, theta_prev)
% Resuelve analíticamente: ‖ offset + rodrigues(v, k, θ) − target ‖ = L
%
% Derivación:
%   rodrigues(v,k,θ) = a + r·cos(θ) + s·sin(θ)
%   con  a = dot(k,v)·k  (axial),  r = v−a  (radial),  s = cross(k,v)
%   |r| = |s|,  dot(r,s) = 0  → se cancela el término cruzado
%
%   Sea q = target − offset − a:
%   dot(q,r)·cos(θ) + dot(q,s)·sin(θ) = (|r|² + |q|² − L²) / 2
%           B·cos(θ)  +      C·sin(θ)  = D
%
%   θ = atan2(C, B) ± acos(D / √(B²+C²))
%   Se elige la raíz más próxima a theta_prev.

    a = dot(k, v) * k;
    r = v - a;
    s = cross(k, v);          % = cross(k, r) ya que cross(k, a) = 0

    q = target - offset - a;

    B = dot(q, r);
    C = dot(q, s);
    D = (dot(r,r) + dot(q,q) - L^2) / 2;

    R = sqrt(B*B + C*C);

    if R < 1e-12
        % Punto sobre el eje de rotación: cualquier θ cumple (o ninguno).
        % Devolvemos el ángulo previo para mantener continuidad.
        theta = theta_prev;
        return
    end

    ratio = D / R;
    ratio = max(-1.0, min(1.0, ratio));   % clamp numérico (geometría límite)

    phi   = atan2(C, B);
    delta = acos(ratio);

    th_pos = phi + delta;
    th_neg = phi - delta;

    % Raíz más cercana al frame anterior → continuidad garantizada
    if abs(th_pos - theta_prev) <= abs(th_neg - theta_prev)
        theta = th_pos;
    else
        theta = th_neg;
    end

end
