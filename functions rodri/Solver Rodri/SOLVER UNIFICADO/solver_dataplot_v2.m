clc
close all
clear ETR11_KINEMATICS_ANALYTICAL ETR11_GET_POINTS

%% ═══════════════════════════════════════════════════════════════════════════
%%  VEHICLE PARAMETERS
%% ═══════════════════════════════════════════════════════════════════════════
LR            = 0.203;       % Loaded radius [m]
WB            = 1535;        % Wheelbase [mm]
TF            = 1250;        % Front track [mm]
TR_veh        = 1250;        % Rear track [mm]

m_total       = 270;         % Total mass [kg]
m_sprung      = 220;         % Sprung mass [kg]
m_corner_F    = m_sprung * 0.45 / 2;   % Sprung mass per front corner [kg]
m_corner_R    = m_sprung * 0.55 / 2;   % Sprung mass per rear corner [kg]

cg_balance    = 0.55;        % CG longitudinal balance (from front axle / WB)
cg_h          = 260;         % CG height [mm]
a_wb          = WB * cg_balance;       % distance CG to front axle [mm]
b_wb          = WB * (1 - cg_balance); % distance CG to rear axle [mm]

k_spring_F    = 35000;       % Front spring rate [N/m]
k_spring_R    = 35000;       % Rear spring rate [N/m]

brake_bias_F  = 0.65;        % Front brake bias

k_roll_F      = 18370.87;    % Front roll stiffness [N/m]  (same as original dataplot)
k_roll_R      = 18370.87;    % Rear  roll stiffness [N/m]
h_RC_F        = 0.07;        % Front roll centre height reference [m]
h_RC_R        = 0.08;        % Rear  roll centre height reference [m]

g             = 9.81;
mu_tire       = 1.6;

df_coeff      = 8;
drag_coeff    = 1.3;
aero_area     = 0.56;
rho_air       = 1.225;
aero_balance  = 0.4;
cop_z         = 600;         % Centre of pressure height [mm]

ay_cases      = [1.5, 2.0, 2.5] * g;  % [m/s^2]
v_cases       = [10, 20, 30];          % [m/s]

%% ═══════════════════════════════════════════════════════════════════════════
%%  SWEEP 1 — HEAVE  (symmetric: all 4 wheels)
%% ═══════════════════════════════════════════════════════════════════════════
heave_dpr  = (-25:0.2:32)';
N_h        = length(heave_dpr);
i_static   = find(heave_dpr == 0, 1);
if isempty(i_static), [~,i_static] = min(abs(heave_dpr)); end

FL_cp_z    = zeros(N_h,1);  FR_cp_z = zeros(N_h,1);
RL_cp_z    = zeros(N_h,1);  RR_cp_z = zeros(N_h,1);
FL_cp_y    = zeros(N_h,1);  FR_cp_y = zeros(N_h,1);
RL_cp_y    = zeros(N_h,1);  RR_cp_y = zeros(N_h,1);

FL_bs_h    = zeros(N_h,1);  RL_bs_h    = zeros(N_h,1);
FL_camb_h  = zeros(N_h,1);  RL_camb_h  = zeros(N_h,1);
FL_kpi_h   = zeros(N_h,1);  RL_kpi_h   = zeros(N_h,1);
FL_cast_h  = zeros(N_h,1);  RL_cast_h  = zeros(N_h,1);
FL_trail_h = zeros(N_h,1);  RL_trail_h = zeros(N_h,1);
FL_scrub_h = zeros(N_h,1);  RL_scrub_h = zeros(N_h,1);

F_RC_h     = zeros(N_h,1);  R_RC_h = zeros(N_h,1);
F_RC_y     = zeros(N_h,1);  R_RC_y = zeros(N_h,1);
F_push_dot = zeros(N_h,1);  R_push_dot = zeros(N_h,1);
F_rkr_ang  = zeros(N_h,1);  R_rkr_ang  = zeros(N_h,1);
F_arb_ang  = zeros(N_h,1);  R_arb_ang  = zeros(N_h,1);

F_antidive  = zeros(N_h,1);
R_antisquat = zeros(N_h,1);

fprintf('Computing heave sweep...\n');
for i = 1:N_h
    d = heave_dpr(i);
    [FL, FR, RL, RR, F_RC, R_RC, FL_K, ~, RL_K] = ETR11_GET_POINTS(0, d, d, d, d, LR);

    FL_cp_z(i) = FL.CONTACT_PATCH(3);   FR_cp_z(i) = FR.CONTACT_PATCH(3);
    RL_cp_z(i) = RL.CONTACT_PATCH(3);   RR_cp_z(i) = RR.CONTACT_PATCH(3);
    FL_cp_y(i) = FL.CONTACT_PATCH(2);   FR_cp_y(i) = FR.CONTACT_PATCH(2);
    RL_cp_y(i) = RL.CONTACT_PATCH(2);   RR_cp_y(i) = RR.CONTACT_PATCH(2);

    FL_bs_h(i)   = FL_K.STEER;     RL_bs_h(i)   = RL_K.STEER;
    FL_camb_h(i) = FL_K.CAMBER;    RL_camb_h(i) = RL_K.CAMBER;
    FL_kpi_h(i)  = FL_K.KPI;       RL_kpi_h(i)  = RL_K.KPI;
    FL_cast_h(i) = FL_K.CASTER;    RL_cast_h(i) = RL_K.CASTER;
    FL_trail_h(i)= FL_K.TRAIL;     RL_trail_h(i)= RL_K.TRAIL;
    FL_scrub_h(i)= FL_K.SCRUB;     RL_scrub_h(i)= RL_K.SCRUB;

    F_RC_h(i) = F_RC(3);   R_RC_h(i) = R_RC(3);
    F_RC_y(i) = F_RC(2);   R_RC_y(i) = R_RC(2);

    F_push_dot(i) = FL_K.PUSH_RKR_DOT;  R_push_dot(i) = RL_K.PUSH_RKR_DOT;
    F_rkr_ang(i)  = FL.RKR_ANGLE;       R_rkr_ang(i)  = RL.RKR_ANGLE;
    F_arb_ang(i)  = FL.ARB_ANGLE;       R_arb_ang(i)  = RL.ARB_ANGLE;

    % Side-view instant centre (XZ plane) → anti-geometry
    UF_mid = (FL.URW_MC + FL.UFW_MC) / 2;
    LF_mid = (FL.LRW_MC + FL.LFW_MC) / 2;
    [svic_xF, svic_zF] = side_view_IC(UF_mid, FL.UW_KN, LF_mid, FL.LW_KN);
    dx_F = svic_xF - FL.CONTACT_PATCH(1);
    dz_F = svic_zF - FL.CONTACT_PATCH(3);
    if abs(dx_F) > 10
        val = (dz_F / dx_F) * WB / cg_h / brake_bias_F * 100;
        F_antidive(i) = max(-200, min(200, val));
    end

    UR_mid = (RL.URW_MC + RL.UFW_MC) / 2;
    LR_mid = (RL.LRW_MC + RL.LFW_MC) / 2;
    [svic_xR, svic_zR] = side_view_IC(UR_mid, RL.UW_KN, LR_mid, RL.LW_KN);
    dx_R = svic_xR - RL.CONTACT_PATCH(1);
    dz_R = svic_zR - RL.CONTACT_PATCH(3);
    if abs(dx_R) > 10
        val = -(dz_R / dx_R) * WB / cg_h / (1 - brake_bias_F) * 100;
        R_antisquat(i) = max(-200, min(200, val));
    end
end

% Wheel travel relative to static
FL_wt = FL_cp_z - FL_cp_z(i_static);
RL_wt = RL_cp_z - RL_cp_z(i_static);

% Motion ratio  (damper stroke / wheel travel)
F_MR = gradient(heave_dpr) ./ gradient(FL_cp_z);
R_MR = gradient(heave_dpr) ./ gradient(RL_cp_z);

% Wheel rate [N/m]
F_WR = k_spring_F .* F_MR.^2;
R_WR = k_spring_R .* R_MR.^2;

% Ride frequency [Hz]
F_RF = (1/(2*pi)) * sqrt(F_WR / m_corner_F);
R_RF = (1/(2*pi)) * sqrt(R_WR / m_corner_R);

% Bump steer rate [deg/mm] — derivative w.r.t. wheel travel
dz_F = gradient(FL_wt);  dz_F(abs(dz_F)<1e-9) = nan;
dz_R = gradient(RL_wt);  dz_R(abs(dz_R)<1e-9) = nan;
FL_bsr = gradient(FL_bs_h) ./ dz_F;
RL_bsr = gradient(RL_bs_h) ./ dz_R;

% Track width change relative to static [mm]
F_track_delta = (FL_cp_y - FR_cp_y) - (FL_cp_y(i_static) - FR_cp_y(i_static));
R_track_delta = (RL_cp_y - RR_cp_y) - (RL_cp_y(i_static) - RR_cp_y(i_static));

%% ═══════════════════════════════════════════════════════════════════════════
%%  SWEEP 2 — STEER  (static ride height)
%% ═══════════════════════════════════════════════════════════════════════════
steer_sw   = (0:1:145)';
N_s        = length(steer_sw);

FL_steer   = zeros(N_s,1);  FR_steer   = zeros(N_s,1);
FL_camb_s  = zeros(N_s,1);  FR_camb_s  = zeros(N_s,1);
FL_kpi_s   = zeros(N_s,1);  FR_kpi_s   = zeros(N_s,1);
FL_cast_s  = zeros(N_s,1);  FR_cast_s  = zeros(N_s,1);
FL_trail_s = zeros(N_s,1);  FR_trail_s = zeros(N_s,1);
FL_scrub_s = zeros(N_s,1);  FR_scrub_s = zeros(N_s,1);
FL_sc_z    = zeros(N_s,1);  FR_sc_z    = zeros(N_s,1);
% Geometry needed for tie rod force method
FL_LW_KN = zeros(N_s,3);  FR_LW_KN = zeros(N_s,3);
FL_UW_KN = zeros(N_s,3);  FR_UW_KN = zeros(N_s,3);
FL_TR_UP = zeros(N_s,3);  FR_TR_UP = zeros(N_s,3);
FL_TR_RK = zeros(N_s,3);  FR_TR_RK = zeros(N_s,3);
FL_CP_s    = zeros(N_s,3);  FR_CP_s    = zeros(N_s,3);
FL_SPNDL   = zeros(N_s,3);  FR_SPNDL   = zeros(N_s,3);  % wheel lateral axis (spindle unit vec)
FL_PUSH_UW = zeros(N_s,3);  FR_PUSH_UW = zeros(N_s,3);  % needed by dynamics solver
FL_PUSH_RK = zeros(N_s,3);  FR_PUSH_RK = zeros(N_s,3);

% Constant monocoque points (read once from static position)
[FL0, FR0] = ETR11_GET_POINTS(0, 0, 0, 0, 0, LR);
FL_MC = struct('URW',FL0.URW_MC,'UFW',FL0.UFW_MC,'LRW',FL0.LRW_MC,'LFW',FL0.LFW_MC);
FR_MC = struct('URW',FR0.URW_MC,'UFW',FR0.UFW_MC,'LRW',FR0.LRW_MC,'LFW',FR0.LFW_MC);

fprintf('Computing steer sweep...\n');
for i = 1:N_s
    [FL, FR, ~, ~, ~, ~, FL_K, FR_K] = ETR11_GET_POINTS(steer_sw(i), 0, 0, 0, 0, LR);
    FL_steer(i)  = FL_K.STEER;    FR_steer(i)  = FR_K.STEER;
    FL_camb_s(i) = FL_K.CAMBER;   FR_camb_s(i) = FR_K.CAMBER;
    FL_kpi_s(i)  = FL_K.KPI;      FR_kpi_s(i)  = FR_K.KPI;
    FL_cast_s(i) = FL_K.CASTER;   FR_cast_s(i) = FR_K.CASTER;
    FL_trail_s(i)= FL_K.TRAIL;    FR_trail_s(i)= FR_K.TRAIL;
    FL_scrub_s(i)= FL_K.SCRUB;    FR_scrub_s(i)= FR_K.SCRUB;
    FL_sc_z(i)   = FL.SPINDLE_CENTER(3);
    FR_sc_z(i)   = FR.SPINDLE_CENTER(3);
    FL_LW_KN(i,:)  = FL.LW_KN;        FR_LW_KN(i,:)  = FR.LW_KN;
    FL_UW_KN(i,:)  = FL.UW_KN;        FR_UW_KN(i,:)  = FR.UW_KN;
    FL_TR_UP(i,:)  = FL.TR_UPRIGHT;    FR_TR_UP(i,:)  = FR.TR_UPRIGHT;
    FL_TR_RK(i,:)  = FL.TR_RACK;       FR_TR_RK(i,:)  = FR.TR_RACK;
    FL_CP_s(i,:)   = FL.CONTACT_PATCH; FR_CP_s(i,:)   = FR.CONTACT_PATCH;
    FL_SPNDL(i,:)  = FL.SPINDLE;       FR_SPNDL(i,:)  = FR.SPINDLE;
    FL_PUSH_UW(i,:)= FL.PUSH_UW;       FR_PUSH_UW(i,:)= FR.PUSH_UW;
    FL_PUSH_RK(i,:)= FL.PUSH_RKR;      FR_PUSH_RK(i,:)= FR.PUSH_RKR;
end

% Steer ratio: degrees of steering wheel per degree of wheel angle
d_sw     = gradient(steer_sw);
steer_ratio_int = d_sw ./ gradient(FR_steer);
steer_ratio_ext = d_sw ./ gradient(FL_steer);

% Ackermann: ideal outer angle for a given inner angle (same formula as original dataplot)
% inner = FR (right turn → right is inner), outer = FL
dynamic_toe      = FR_steer - FL_steer;
dynamic_toe_ack  = FR_steer - atand(WB ./ (WB ./ (tand(FR_steer) + 1e-9) + TF));
denom_ack        = dynamic_toe_ack;
ack_pctge        = dynamic_toe ./ (denom_ack + sign(denom_ack+eps)*1e-6);
ack_pctge(abs(FR_steer) < 1) = NaN;   % mask near-zero steer (numerical noise)

% Jack rate [mm/deg_wheel]
FL_jack_rate = -gradient(FL_sc_z) ./ (gradient(FL_steer) + 1e-9);
FR_jack_rate = -gradient(FR_sc_z) ./ (gradient(FR_steer) + 1e-9);

% Steering torques at the steering wheel for 3 ay × v cases  (same method as original dataplot)
front_load_stat = m_total * g * (1 - cg_balance);
rear_load_stat  = m_total * g * cg_balance;

h_cg_m = cg_h / 1000;                        % CG height [m]
h_roll  = h_cg_m - (h_RC_F + h_RC_R) / 2;   % roll arm height [m]

FL_jack_Nm = zeros(3, N_s);  FR_jack_Nm = zeros(3, N_s);
FL_mech_Nm = zeros(3, N_s);  FR_mech_Nm = zeros(3, N_s);
FZ_FL_cases = zeros(3,1);    FZ_FR_cases = zeros(3,1);

for ic = 1:3
    ay_i = ay_cases(ic);
    vi   = v_cases(ic);

    dF     = 0.5 * rho_air * aero_area * df_coeff   * vi^2;
    drag   = 0.5 * rho_air * aero_area * drag_coeff * vi^2;
    dF_F   = dF * (1 - aero_balance);
    drag_F = -drag * (cop_z/1000) / (WB/1000);

    F_wt = ay_i * m_sprung / (TF/1000) * ...
           ((h_roll * k_roll_F)/(k_roll_F + k_roll_R) + (b_wb/WB) * h_RC_F);

    FZ_FL = front_load_stat/2 + F_wt + (dF_F + drag_F)/2;
    FZ_FR = front_load_stat/2 - F_wt + (dF_F + drag_F)/2;
    FZ_FL_cases(ic) = FZ_FL;
    FZ_FR_cases(ic) = FZ_FR;

    FL_jack_Nm(ic,:) = (FZ_FL .* (FL_jack_rate' * 180/pi) / 1000) ./ steer_ratio_ext';
    FR_jack_Nm(ic,:) = (FZ_FR .* (FR_jack_rate' * 180/pi) / 1000) ./ steer_ratio_int';
    FL_mech_Nm(ic,:) = (FZ_FL .* mu_tire .* (FL_trail_s' / 1000)) ./ steer_ratio_ext';
    FR_mech_Nm(ic,:) = (FZ_FR .* mu_tire .* (FR_trail_s' / 1000)) ./ steer_ratio_int';
end

%% ─────────────────────────────────────────────────────────────────────────
%%  TIE ROD FORCE METHOD
%%  Physics: moment balance about KP axis → tie rod force → rack Y-force → SW torque
%%  Positive T_SW = driver effort (resistance to rightward steer)
%%  FY = +mu*FZ toward +Y (centripetal, right turn); FX = 0 (pure cornering)
%% ─────────────────────────────────────────────────────────────────────────
D_PINION_M   = 29 / 2000;           % pinion radius [m]
T_SW_TR_tot  = zeros(3, N_s);       % total: FY + FZ
T_SW_TR_vert = zeros(3, N_s);       % FZ only  (compare with jack rate method)
T_SW_TR_lat  = zeros(3, N_s);       % FY only  (compare with mechanical trail method)

for ic = 1:3
    FZ_fl = FZ_FL_cases(ic);
    FZ_fr = FZ_FR_cases(ic);
    FY_fl =  mu_tire * FZ_fl;       % toward +Y (right turn, centripetal)
    FY_fr =  mu_tire * FZ_fr;

    for i = 1:N_s
        % ── FY direction: along wheel spindle projected to road plane ────
        spndl_fl = FL_SPNDL(i,:); spndl_fl(3) = 0;
        spndl_fl = sign(spndl_fl(2)) * spndl_fl / norm(spndl_fl);
        spndl_fr = FR_SPNDL(i,:); spndl_fr(3) = 0;
        spndl_fr = sign(spndl_fr(2)) * spndl_fr / norm(spndl_fr);
        FY_vec_fl = FY_fl * spndl_fl;
        FY_vec_fr = FY_fr * spndl_fr;
        F_fl_total = FY_vec_fl + [0, 0, FZ_fl];   % [Fx, Fy, Fz] in global coords
        F_fr_total = FY_vec_fr + [0, 0, FZ_fr];

        % ── Build minimal HP struct for dynamics solver ───────────────
        HP_fl.CONTACT_PATCH = FL_CP_s(i,:);
        HP_fl.TR_RACK       = FL_TR_RK(i,:);
        HP_fl.TR_UPRIGHT    = FL_TR_UP(i,:);
        HP_fl.PUSH_RKR      = FL_PUSH_RK(i,:);
        HP_fl.PUSH_UW       = FL_PUSH_UW(i,:);
        HP_fl.LW_KN         = FL_LW_KN(i,:);
        HP_fl.UW_KN         = FL_UW_KN(i,:);
        HP_fl.URW_MC        = FL_MC.URW;
        HP_fl.UFW_MC        = FL_MC.UFW;
        HP_fl.LRW_MC        = FL_MC.LRW;
        HP_fl.LFW_MC        = FL_MC.LFW;

        HP_fr.CONTACT_PATCH = FR_CP_s(i,:);
        HP_fr.TR_RACK       = FR_TR_RK(i,:);
        HP_fr.TR_UPRIGHT    = FR_TR_UP(i,:);
        HP_fr.PUSH_RKR      = FR_PUSH_RK(i,:);
        HP_fr.PUSH_UW       = FR_PUSH_UW(i,:);
        HP_fr.LW_KN         = FR_LW_KN(i,:);
        HP_fr.UW_KN         = FR_UW_KN(i,:);
        HP_fr.URW_MC        = FR_MC.URW;
        HP_fr.UFW_MC        = FR_MC.UFW;
        HP_fr.LRW_MC        = FR_MC.LRW;
        HP_fr.LFW_MC        = FR_MC.LFW;

        % ── ETR11_DYNAMICS_SOLVER: full 6×6 statics equilibrium ──────
        D_fl = ETR11_DYNAMICS_SOLVER(HP_fl, F_fl_total);
        D_fr = ETR11_DYNAMICS_SOLVER(HP_fr, F_fr_total);

        % ── Tie rod axial force → rack Y-force → SW torque ───────────
        e_fl = (FL_TR_UP(i,:) - FL_TR_RK(i,:)) / norm(FL_TR_UP(i,:) - FL_TR_RK(i,:));
        e_fr = (FR_TR_UP(i,:) - FR_TR_RK(i,:)) / norm(FR_TR_UP(i,:) - FR_TR_RK(i,:));

        % FZ-only contribution (call solver with [0,0,FZ] for vert/lat split)
        D_fl_z = ETR11_DYNAMICS_SOLVER(HP_fl, [0, 0, FZ_fl]);
        D_fr_z = ETR11_DYNAMICS_SOLVER(HP_fr, [0, 0, FZ_fr]);
        D_fl_y = ETR11_DYNAMICS_SOLVER(HP_fl, FY_vec_fl);
        D_fr_y = ETR11_DYNAMICS_SOLVER(HP_fr, FY_vec_fr);

        rk_z_fl = -D_fl_z.TieRod * e_fl(2);   rk_z_fr = -D_fr_z.TieRod * e_fr(2);
        rk_y_fl = -D_fl_y.TieRod * e_fl(2);   rk_y_fr = -D_fr_y.TieRod * e_fr(2);
        rk_tot_fl = -D_fl.TieRod * e_fl(2);   rk_tot_fr = -D_fr.TieRod * e_fr(2);

        T_SW_TR_vert(ic,i) = -(rk_z_fl + rk_z_fr) * D_PINION_M;
        T_SW_TR_lat(ic,i)  = -(rk_y_fl + rk_y_fr) * D_PINION_M;
        T_SW_TR_tot(ic,i)  = -(rk_tot_fl + rk_tot_fr) * D_PINION_M;
    end
end

%% ═══════════════════════════════════════════════════════════════════════════
%%  SWEEP 3 — ROLL  (anti-phase: FL↑ FR↓)
%% ═══════════════════════════════════════════════════════════════════════════
roll_dpr   = (0:0.5:min(abs(-25), 32))';  % positive = compress left / extend right
N_r        = length(roll_dpr);

roll_ang_F = zeros(N_r,1);  roll_ang_R = zeros(N_r,1);
FL_cp_r    = zeros(N_r,3);  FR_cp_r = zeros(N_r,3);
RL_cp_r    = zeros(N_r,3);  RR_cp_r = zeros(N_r,3);
FL_camb_r  = zeros(N_r,1);  FR_camb_r = zeros(N_r,1);
RL_camb_r  = zeros(N_r,1);  RR_camb_r = zeros(N_r,1);
FL_bs_r    = zeros(N_r,1);  FR_bs_r   = zeros(N_r,1);
RL_bs_r    = zeros(N_r,1);  RR_bs_r   = zeros(N_r,1);
F_RC_r     = zeros(N_r,3);  R_RC_r = zeros(N_r,3);
FL_arb_r   = zeros(N_r,1);  FR_arb_r  = zeros(N_r,1);
RL_arb_r   = zeros(N_r,1);  RR_arb_r  = zeros(N_r,1);
FL_rkr_r   = zeros(N_r,1);  FL_arb_ang_r = zeros(N_r,1);

fprintf('Computing roll sweep...\n');
for i = 1:N_r
    d_L =  roll_dpr(i);
    d_R = -roll_dpr(i);
    [FL, FR, RL, RR, F_RC, R_RC, FL_K, FR_K, RL_K, RR_K] = ETR11_GET_POINTS(0, d_L, d_R, d_L, d_R, LR);

    FL_cp_r(i,:) = FL.CONTACT_PATCH;
    FR_cp_r(i,:) = FR.CONTACT_PATCH;
    RL_cp_r(i,:) = RL.CONTACT_PATCH;
    RR_cp_r(i,:) = RR.CONTACT_PATCH;

    roll_ang_F(i) = atand((FL_cp_r(i,3) - FR_cp_r(i,3)) / (FL_cp_r(i,2) - FR_cp_r(i,2)));
    roll_ang_R(i) = atand((RL_cp_r(i,3) - RR_cp_r(i,3)) / (RL_cp_r(i,2) - RR_cp_r(i,2)));

    FL_camb_r(i) = FL_K.CAMBER;   FR_camb_r(i) = FR_K.CAMBER;
    RL_camb_r(i) = RL_K.CAMBER;   RR_camb_r(i) = RR_K.CAMBER;
    FL_bs_r(i)   = FL_K.STEER;    FR_bs_r(i)   = FR_K.STEER;
    RL_bs_r(i)   = RL_K.STEER;    RR_bs_r(i)   = RR_K.STEER;

    F_RC_r(i,:) = F_RC;            R_RC_r(i,:) = R_RC;

    FL_arb_r(i) = FL.ARB_ANGLE;   FR_arb_r(i) = FR.ARB_ANGLE;
    RL_arb_r(i) = RL.ARB_ANGLE;   RR_arb_r(i) = RR.ARB_ANGLE;
    FL_rkr_r(i) = FL.RKR_ANGLE;   FL_arb_ang_r(i) = FL.ARB_ANGLE;
end

% ARB installation ratio  (d ARB_twist / d body_roll)  [deg/deg]
F_arb_twist   = rad2deg(FL_arb_r - FR_arb_r);
R_arb_twist   = rad2deg(RL_arb_r - RR_arb_r);
F_arb_ratio   = gradient(F_arb_twist) ./ (gradient(roll_ang_F) + 1e-9);
R_arb_ratio   = gradient(R_arb_twist) ./ (gradient(roll_ang_R) + 1e-9);

% RC migration
F_RC_h_r = F_RC_r(:,3);  R_RC_h_r = R_RC_r(:,3);
F_RC_y_r = F_RC_r(:,2);  R_RC_y_r = R_RC_r(:,2);

% Track width change in roll
F_track_r = (FL_cp_r(:,2) - FR_cp_r(:,2)) - (FL_cp_r(1,2) - FR_cp_r(1,2));
R_track_r = (RL_cp_r(:,2) - RR_cp_r(:,2)) - (RL_cp_r(1,2) - RR_cp_r(1,2));

%% ═══════════════════════════════════════════════════════════════════════════
%%  FIGURES
%% ═══════════════════════════════════════════════════════════════════════════
lw = 1.5;

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 1 — HEAVE: Motion Ratio, Wheel Rate, Ride Frequency, Push-Rocker
%% ────────────────────────────────────────────────────────────────────────
figure('Name','HEAVE — Spring & Damper Kinematics','NumberTitle','off')

subplot(2,2,1)
plot(heave_dpr, F_MR, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(heave_dpr, R_MR, 'LineWidth',lw, 'DisplayName','Rear')
legend; xlabel('Damper compression [mm]'); ylabel('Motion Ratio [-]')
title('Motion Ratio  (Δdamper / Δwheel)')

subplot(2,2,2)
plot(heave_dpr, F_WR/1000, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(heave_dpr, R_WR/1000, 'LineWidth',lw, 'DisplayName','Rear')
legend; xlabel('Damper compression [mm]'); ylabel('Wheel rate [N/mm]')
title('Wheel Rate  (k_{spring} \cdot MR^2)')

subplot(2,2,3)
plot(heave_dpr, F_RF, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(heave_dpr, R_RF, 'LineWidth',lw, 'DisplayName','Rear')
legend; xlabel('Damper compression [mm]'); ylabel('Ride frequency [Hz]')
title('Ride Frequency  (1/2\pi)\cdot\surd(WR/m_{corner})')

subplot(2,2,4)
plot(heave_dpr, F_push_dot, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(heave_dpr, R_push_dot, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Damper compression [mm]'); ylabel('cos\theta [-]')
title('Pushrod–Rocker Axis Perpendicularity  (ideal = 0)')

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 2 — HEAVE: Kinematics (bump steer, camber gain, KPI, caster)
%% ────────────────────────────────────────────────────────────────────────
figure('Name','HEAVE — Kinematics','NumberTitle','off')

subplot(2,2,1)
plot(FL_wt, FL_bs_h, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(RL_wt, RL_bs_h, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Wheel travel [mm]'); ylabel('Toe [deg]  (+ = toe-in)')
title('Bump Steer')

subplot(2,2,2)
plot(FL_wt, FL_bsr, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(RL_wt, RL_bsr, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Wheel travel [mm]'); ylabel('Bump steer rate [deg/mm]')
title('Bump Steer Rate')

subplot(2,2,3)
plot(FL_wt, FL_camb_h, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(RL_wt, RL_camb_h, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Wheel travel [mm]'); ylabel('Camber [deg]')
title('Camber Gain vs Wheel Travel')

subplot(2,2,4)
yyaxis left
plot(FL_wt, FL_kpi_h,  'b-',  'LineWidth',lw, 'DisplayName','KPI Front')
hold on; grid on
plot(RL_wt, RL_kpi_h, '-', 'LineWidth',lw, 'DisplayName','KPI Rear')
ylabel('KPI [deg]')
yyaxis right
plot(FL_wt, FL_cast_h, '--', 'LineWidth',lw, 'DisplayName','Caster Front')
plot(RL_wt, RL_cast_h, '--', 'LineWidth',lw, 'DisplayName','Caster Rear')
ylabel('Caster [deg]')
legend; xlabel('Wheel travel [mm]')
title('KPI and Caster vs Wheel Travel')

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 3 — HEAVE: Roll centres, track, scrub/trail, anti-geometry
%% ────────────────────────────────────────────────────────────────────────
figure('Name','HEAVE — Geometry & Roll Centres','NumberTitle','off')

subplot(2,2,1)
plot(heave_dpr, F_RC_h, 'LineWidth',lw, 'DisplayName','Front RC')
hold on; grid on
plot(heave_dpr, R_RC_h, 'LineWidth',lw, 'DisplayName','Rear RC')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Damper compression [mm]'); ylabel('RC height [mm]')
title('Roll Centre Height vs Heave')

subplot(2,2,2)
F_RC_h_delta = F_RC_h - F_RC_h(i_static);
R_RC_h_delta = R_RC_h - R_RC_h(i_static);
plot(heave_dpr, F_RC_h_delta, 'LineWidth',lw, 'DisplayName','Front RC')
hold on; grid on
plot(heave_dpr, R_RC_h_delta, 'LineWidth',lw, 'DisplayName','Rear RC')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Damper compression [mm]'); ylabel('\DeltaRC height [mm]')
title('Roll Centre Height Change vs Heave  (\Delta from static)')

subplot(2,2,3)
plot(heave_dpr, F_track_delta, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(heave_dpr, R_track_delta, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Damper compression [mm]'); ylabel('\Deltatrack [mm]')
title('Track Width Change vs Heave')

subplot(2,2,4)
plot(heave_dpr, F_antidive,  'b', 'LineWidth',lw, 'DisplayName','Anti-dive (F)')
hold on; grid on
plot(heave_dpr, R_antisquat, 'LineWidth',lw, 'DisplayName','Anti-squat (R)')
yline(100,'k--','LineWidth',0.8,'Label','100%')
yline(0,  'k:',  'LineWidth',0.8)
legend; xlabel('Damper compression [mm]'); ylabel('Anti-geometry [%]')
title('Anti-dive (Front) & Anti-squat (Rear) %')

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 4 — STEER: Geometry (angles, Ackermann, ratio, toe diff)
%% ────────────────────────────────────────────────────────────────────────
figure('Name','STEER — Geometry','NumberTitle','off')

subplot(2,2,1)
plot(steer_sw, FL_steer, 'LineWidth',lw, 'DisplayName','FL (outer)')
hold on; grid on
plot(steer_sw, FR_steer, 'LineWidth',lw, 'DisplayName','FR (inner)')
legend; xlabel('Steering wheel angle [deg]'); ylabel('Wheel steer angle [deg]')
title('Wheel Steer Angles')

subplot(2,2,2)
plot(steer_sw, ack_pctge*100, 'LineWidth',lw)
hold on; grid on
yline(100,'b--','LineWidth',0.8,'Label','100 % Ackermann')
yline(0,  'r--','LineWidth',0.8,'Label','Parallel')
xlabel('Steering wheel angle [deg]'); ylabel('Ackermann [%]')
ylim([-50 200])
title('Ackermann Percentage')

subplot(2,2,3)
plot(steer_sw, steer_ratio_int, 'LineWidth',lw, 'DisplayName','Inner (FR)')
hold on; grid on
plot(steer_sw, steer_ratio_ext, 'LineWidth',lw, 'DisplayName','Outer (FL)')
legend; xlabel('Steering wheel angle [deg]'); ylabel('Overall steer ratio [-]')
title('Overall Steering Ratio  (\delta_{SW}/\delta_{wheel})')

subplot(2,2,4)
toe_diff = FR_steer - FL_steer;
plot(FR_steer, toe_diff, 'k', 'LineWidth',lw, 'DisplayName','Actual')
hold on; grid on
ack_ideal_diff = FR_steer - atand(WB ./ (WB ./ (tand(FR_steer) + 1e-9) + TF));
plot(FR_steer, ack_ideal_diff, '--', 'LineWidth',lw, 'DisplayName','Ackermann ideal')
legend; xlabel('Inner wheel steer [deg]'); ylabel('Toe differential [deg]')
title('Toe Differential  (inner - outer)')

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 5 — STEER: KP & Tire parameters
%% ────────────────────────────────────────────────────────────────────────
figure('Name','STEER — KP & Tire Parameters','NumberTitle','off')

subplot(2,2,1)
plot(steer_sw, FL_trail_s, 'LineWidth',lw, 'DisplayName','FL (outer)')
hold on; grid on
plot(steer_sw, FR_trail_s, 'LineWidth',lw, 'DisplayName','FR (inner)')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Steering wheel angle [deg]'); ylabel('Mechanical trail [mm]')
title('Mechanical Trail vs Steer')

subplot(2,2,2)
plot(steer_sw, FL_scrub_s, 'LineWidth',lw, 'DisplayName','FL')
hold on; grid on
plot(steer_sw, FR_scrub_s, 'LineWidth',lw, 'DisplayName','FR')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Steering wheel angle [deg]'); ylabel('Scrub radius [mm]')
title('Scrub Radius vs Steer')

subplot(2,2,3)
plot(steer_sw, FL_camb_s, 'LineWidth',lw, 'DisplayName','FL')
hold on; grid on
plot(steer_sw, FR_camb_s, 'LineWidth',lw, 'DisplayName','FR')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Steering wheel angle [deg]'); ylabel('Camber [deg]')
title('Camber Induced by Steering')

subplot(2,2,4)
yyaxis left
plot(steer_sw, FL_kpi_s, 'b-', 'LineWidth',lw, 'DisplayName','KPI FL')
hold on; grid on
plot(steer_sw, FR_kpi_s, 'r-', 'LineWidth',lw, 'DisplayName','KPI FR')
ylabel('KPI [deg]')
yyaxis right
plot(steer_sw, FL_cast_s, '--', 'LineWidth',lw, 'DisplayName','Caster FL')
plot(steer_sw, FR_cast_s, '--', 'LineWidth',lw, 'DisplayName','Caster FR')
ylabel('Caster [deg]')
legend; xlabel('Steering wheel angle [deg]')
title('KPI & Caster vs Steer')

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 6 — STEER: Torques — Jack/Trail method vs Tie Rod method
%%  Solid lines = jack rate + mechanical trail (simplified)
%%  Dashed lines = full tie rod force from KP moment balance (3D exact)
%% ────────────────────────────────────────────────────────────────────────
labels3 = {'1.5g / 10 m/s', '2.0g / 20 m/s', '2.5g / 30 m/s'};

figure('Name','STEER — Torques & Tie Rod Comparison','NumberTitle','off')

subplot(2,2,1)
plot(steer_sw, FL_jack_rate, 'LineWidth',lw, 'DisplayName','FL (outer)')
hold on; grid on
plot(steer_sw, FR_jack_rate, 'LineWidth',lw, 'DisplayName','FR (inner)')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Steering wheel angle [deg]'); ylabel('Jack rate [mm/deg_{wheel}]')
title('Spindle Jack Rate vs Steer')

subplot(2,2,2)
ax2 = gca; hold on; grid on
for ic = 1:3
    h1 = plot(steer_sw, FL_jack_Nm(ic,:)+FR_jack_Nm(ic,:), '-',  'LineWidth',lw, 'DisplayName',[labels3{ic} ' (jack/trail)']);
    plot(steer_sw, T_SW_TR_vert(ic,:), '--', 'LineWidth',lw, 'Color',h1.Color, 'DisplayName',[labels3{ic} ' (tie rod FZ)']);
end
yline(0,'k:','LineWidth',0.8)
legend('Location','best'); xlabel('Steering wheel angle [deg]'); ylabel('[Nm] at SW')
title('FZ contribution: jack rate method vs tie rod (solid = jack, dashed = TR FZ)')

subplot(2,2,3)
hold on; grid on
for ic = 1:3
    h1 = plot(steer_sw, FL_mech_Nm(ic,:)+FR_mech_Nm(ic,:), '-',  'LineWidth',lw, 'DisplayName',[labels3{ic} ' (mech trail)']);
    plot(steer_sw, T_SW_TR_lat(ic,:),  '--', 'LineWidth',lw, 'Color',h1.Color, 'DisplayName',[labels3{ic} ' (tie rod FY)']);
end
yline(0,'k:','LineWidth',0.8)
legend('Location','best'); xlabel('Steering wheel angle [deg]'); ylabel('[Nm] at SW')
title('FY contribution: mechanical trail vs tie rod (solid = trail, dashed = TR FY)')

subplot(2,2,4)
hold on; grid on
for ic = 1:3
    total_classic = FL_jack_Nm(ic,:)+FR_jack_Nm(ic,:)+FL_mech_Nm(ic,:)+FR_mech_Nm(ic,:);
    h1 = plot(steer_sw, total_classic,      '-',  'LineWidth',lw, 'DisplayName',[labels3{ic} ' (classic)']);
    plot(steer_sw, T_SW_TR_tot(ic,:), '--', 'LineWidth',lw, 'Color',h1.Color, 'DisplayName',[labels3{ic} ' (tie rod)']);
end
yline(0,'k:','LineWidth',0.8)
legend('Location','best'); xlabel('Steering wheel angle [deg]'); ylabel('[Nm] at SW')
title('Total SW torque: classic (solid) vs tie rod method (dashed)')

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 7 — ROLL: Roll centres & Camber
%% ────────────────────────────────────────────────────────────────────────
figure('Name','ROLL — Roll Centres & Camber','NumberTitle','off')

subplot(2,2,1)
plot(roll_ang_F, F_RC_h_r, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(roll_ang_R, R_RC_h_r, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Body roll [deg]'); ylabel('RC height [mm]')
title('Roll Centre Height Migration in Roll')

subplot(2,2,2)
plot(roll_ang_F, F_RC_y_r, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(roll_ang_R, R_RC_y_r, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Body roll [deg]'); ylabel('RC lateral position [mm]')
title('Roll Centre Lateral Migration in Roll')

subplot(2,2,3)
plot(roll_ang_F, FL_camb_r, '-', 'LineWidth',lw, 'DisplayName','FL')
hold on; grid on
plot(roll_ang_F, FR_camb_r, '--', 'LineWidth',lw, 'DisplayName','FR')
plot(roll_ang_R, RL_camb_r, '-', 'LineWidth',lw, 'DisplayName','RL')
plot(roll_ang_R, RR_camb_r, '--', 'LineWidth',lw, 'DisplayName','RR')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Body roll [deg]'); ylabel('Camber [deg]')
title('Camber in Roll')

subplot(2,2,4)
plot(roll_ang_F, F_arb_ratio, 'LineWidth',lw, 'DisplayName','Front ARB')
hold on; grid on
plot(roll_ang_R, R_arb_ratio, 'LineWidth',lw, 'DisplayName','Rear ARB')
legend; xlabel('Body roll [deg]'); ylabel('ARB twist / body roll  [deg/deg]')
title('ARB Installation Ratio in Roll')

%% ────────────────────────────────────────────────────────────────────────
%%  FIGURE 8 — ROLL: Track change, bump steer, rocker angle, ARB twist
%% ────────────────────────────────────────────────────────────────────────
figure('Name','ROLL — Track, Bump Steer & Rocker','NumberTitle','off')

subplot(2,2,1)
plot(roll_ang_F, F_track_r, 'LineWidth',lw, 'DisplayName','Front')
hold on; grid on
plot(roll_ang_R, R_track_r, 'LineWidth',lw, 'DisplayName','Rear')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Body roll [deg]'); ylabel('\Deltatrack [mm]')
title('Track Width Change in Roll')

subplot(2,2,2)
plot(roll_ang_F, FL_bs_r, '-', 'LineWidth',lw, 'DisplayName','FL')
hold on; grid on
plot(roll_ang_F, FR_bs_r, '--', 'LineWidth',lw, 'DisplayName','FR')
plot(roll_ang_R, RL_bs_r, '-', 'LineWidth',lw, 'DisplayName','RL')
plot(roll_ang_R, RR_bs_r, '--', 'LineWidth',lw, 'DisplayName','RR')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Body roll [deg]'); ylabel('Toe [deg]  (+ = toe-in)')
title('Bump Steer in Roll')

subplot(2,2,3)
plot(roll_ang_F, F_arb_twist, 'LineWidth',lw, 'DisplayName','Front ARB')
hold on; grid on
plot(roll_ang_R, R_arb_twist, 'LineWidth',lw, 'DisplayName','Rear ARB')
yline(0,'k--','LineWidth',0.8)
legend; xlabel('Body roll [deg]'); ylabel('ARB twist [deg]')
title('ARB Torsion Angle vs Body Roll')

subplot(2,2,4)
yyaxis left
plot(roll_ang_F, rad2deg(FL_rkr_r), 'LineWidth',lw, 'DisplayName','Front rocker')
hold on; grid on
ylabel('Rocker angle [deg]')
yyaxis right
plot(roll_ang_F, rad2deg(FL_arb_ang_r), '--', 'LineWidth',lw, 'DisplayName','Front ARB arm')
ylabel('ARB arm angle [deg]')
xlabel('Body roll [deg]')
title('Rocker & ARB Arm Angles (Front) in Roll')
legend

fprintf('Done. All figures generated.\n');

%% ═══════════════════════════════════════════════════════════════════════════
%%  LOCAL HELPERS
%% ═══════════════════════════════════════════════════════════════════════════

function [x_ic, z_ic] = side_view_IC(P1, P2, P3, P4)
    %SIDE_VIEW_IC  Intersection of two lines in the XZ plane.
    %  Line A: P1 → P2,  Line B: P3 → P4.  Returns x and z of intersection.
    p1 = [P1(1); P1(3)];   d1 = [P2(1)-P1(1); P2(3)-P1(3)];
    p2 = [P3(1); P3(3)];   d2 = [P4(1)-P3(1); P4(3)-P3(3)];
    A  = [d1, -d2];
    if abs(det(A)) < 1e-8
        x_ic = inf;  z_ic = inf;
        return
    end
    t  = A \ (p2 - p1);
    pt = p1 + t(1)*d1;
    x_ic = pt(1);
    z_ic = pt(2);
end
