function SS = ETR11_STEADYSTATE_SOLVER(ay, Vx, VPARAMS, KIN)
% ETR11_STEADYSTATE_SOLVER  Steady-state cornering with Magic Formula tyre model.
%
%   Given (ay, Vx), finds [beta, SW_deg] satisfying lateral + yaw balance:
%     Eq1 (rear):  FY_RL + FY_RR  = m·ay·a_wb/WB   → solved for beta
%     Eq2 (front): FY_FL + FY_FR  = m·ay·b_wb/WB   → solved for SW_deg
%
%   Performance: delta(SW) and gamma(SW) are pre-computed as a lookup table
%   before fzero runs, so each fzero iteration costs only interp1 + mfeval,
%   not a full ETR11_GET_POINTS call.
%
% ── Sign conventions ──────────────────────────────────────────────────────
%   ay > 0   = right turn  (+Y = RIGHT in vehicle frame)
%   SW > 0   = right turn
%   beta > 0 = CG velocity toward +Y (right)
%   comp > 0 = damper compressed (bump)
%   FL = front-left (OUTER in right turn)
%
% ── mfeval / ISO-W → vehicle frame ───────────────────────────────────────
%   Empirical (Hoosier TYRESIDE=LEFT): sign(Fy_mfeval) = -sign(alpha_mfeval)
%   Both alpha_FL and alpha_RL are > 0 for a right turn with these formulas.
%   Fy_vehicle = -Fy_mfeval  (ISO-W +Y=left, vehicle +Y=right)
%   Mz_vehicle = -Mz_mfeval  (Y-flip reverses rotation sense about Z)
%   Mx_vehicle = -Mx_mfeval  (Y-flip reverses rotation sense about X)

    %% ── Unpack parameters ────────────────────────────────────────────────
    m      = VPARAMS.m_total;
    m_spr  = VPARAMS.m_sprung;
    WB_m   = VPARAMS.WB    / 1000;
    TF_m   = VPARAMS.TF    / 1000;
    TR_m   = VPARAMS.TR_veh / 1000;
    a_wb   = WB_m *  VPARAMS.cg_balance;      % CG → front axle [m]
    b_wb   = WB_m * (1 - VPARAMS.cg_balance); % CG → rear axle  [m]
    LR     = VPARAMS.LR;
    g_acc  = 9.81;
    kf     = VPARAMS.k_roll_F;
    kr     = VPARAMS.k_roll_R;
    h_RC_F = VPARAMS.h_RC_F;
    h_RC_R = VPARAMS.h_RC_R;
    h_roll = VPARAMS.cg_h/1000 - (h_RC_F + h_RC_R)/2;
    useMode = 111;
    if isfield(VPARAMS,'mfeval_useMode'), useMode = VPARAMS.mfeval_useMode; end
    tir = VPARAMS.tir_file;

    %% ── Aero & vertical loads ────────────────────────────────────────────
    dF_total = 0.5 * VPARAMS.rho_air * VPARAMS.aero_area * VPARAMS.df_coeff * Vx^2;
    dF_F = dF_total * (1 - VPARAMS.aero_balance);
    dF_R = dF_total *      VPARAMS.aero_balance;

    W         = m * g_acc;
    FZ_stat_F = W * b_wb / WB_m / 2 + dF_F / 2;
    FZ_stat_R = W * a_wb / WB_m / 2 + dF_R / 2;

    dFZ_F = ay * m_spr / TF_m * (h_roll * kf/(kf+kr) + b_wb/WB_m * h_RC_F);
    dFZ_R = ay * m_spr / TR_m * (h_roll * kr/(kf+kr) + a_wb/WB_m * h_RC_R);

    FZ_FL = max(FZ_stat_F + dFZ_F, 50);
    FZ_FR = max(FZ_stat_F - dFZ_F, 50);
    FZ_RL = max(FZ_stat_R + dFZ_R, 50);
    FZ_RR = max(FZ_stat_R - dFZ_R, 50);

    %% ── Damper compressions ──────────────────────────────────────────────
    i0 = find(KIN.heave_dpr == 0, 1);
    if isempty(i0), [~,i0] = min(abs(KIN.heave_dpr)); end
    MR_F = KIN.F_MR(i0);  WR_F = KIN.F_WR(i0);
    MR_R = KIN.R_MR(i0);  WR_R = KIN.R_WR(i0);

    comp_FL =  (dF_F/2/WR_F + dFZ_F/WR_F) * MR_F * 1000;
    comp_FR =  (dF_F/2/WR_F - dFZ_F/WR_F) * MR_F * 1000;
    comp_RL =  (dF_R/2/WR_R + dFZ_R/WR_R) * MR_R * 1000;
    comp_RR =  (dF_R/2/WR_R - dFZ_R/WR_R) * MR_R * 1000;

    r = ay / Vx;   % yaw rate [rad/s], > 0 for right turn

    %% ── Rear kinematics (independent of SW) ──────────────────────────────
    [~, ~, ~, ~, ~, ~, ~, ~, RL_K0, RR_K0] = ...
        ETR11_GET_POINTS(0, comp_FL, comp_FR, comp_RL, comp_RR, LR);
    gamma_RL = deg2rad(RL_K0.CAMBER);
    gamma_RR = deg2rad(RR_K0.CAMBER);

    %% ── Pre-compute front steer LUT: delta(SW), gamma(SW) ────────────────
    % This is the key optimisation: ETR11_GET_POINTS is called N_lut times
    % here, ONCE, and then fzero uses interp1 — no kinematics inside the loop.
    SW_max  = 145;   % [deg] mechanical lock of ETR11
    N_lut   = 83;    % ~5 deg steps; odd so SW=0 is included
    SW_lut  = linspace(-SW_max, SW_max, N_lut);
    dFL_lut = zeros(1, N_lut);
    dFR_lut = zeros(1, N_lut);
    gFL_lut = zeros(1, N_lut);
    gFR_lut = zeros(1, N_lut);
    for k = 1:N_lut
        [~,~,~,~,~,~,FLk,FRk] = ETR11_GET_POINTS( ...
            SW_lut(k), comp_FL, comp_FR, comp_RL, comp_RR, LR);
        dFL_lut(k) = deg2rad(FLk.STEER);
        dFR_lut(k) = deg2rad(FRk.STEER);
        gFL_lut(k) = deg2rad(FLk.CAMBER);
        gFR_lut(k) = deg2rad(FRk.CAMBER);
    end

    %% ── Eq1: solve for beta (rear balance) ───────────────────────────────
    FY_rear_tgt  = m * ay * a_wb / WB_m;
    FY_front_tgt = m * ay * b_wb / WB_m;

    res_rear = @(b) rear_fy(b, Vx, r, b_wb, TR_m, ...
                             FZ_RL, FZ_RR, gamma_RL, gamma_RR, tir, useMode) ...
                   - FY_rear_tgt;

    beta = bounded_fzero(res_rear, 0, -0.26, 0.26, 'beta');

    %% ── Eq2: solve for SW_deg (front balance) using LUT ──────────────────
    res_front = @(sw) front_fy_lut(sw, beta, Vx, r, a_wb, TF_m, ...
                                   FZ_FL, FZ_FR, ...
                                   SW_lut, dFL_lut, dFR_lut, gFL_lut, gFR_lut, ...
                                   tir, useMode) ...
                     - FY_front_tgt;

    % Ackermann estimate × steering ratio as initial guess
    SR = 4.5;
    SW0 = sign(ay) * max(abs(ay) * WB_m / Vx^2 * (180/pi) * SR, 2);
    SW_deg = bounded_fzero(res_front, SW0, 0.5, SW_max, 'SW');

    % Reset analytical solver seed to SW=0 after the solve so subsequent
    % calls (e.g. from visualizer) are not affected by extreme intermediate values
    clear ETR11_KINEMATICS_ANALYTICAL

    %% ── Final kinematics at converged [beta, SW] ─────────────────────────
    [FL, FR, RL, RR, F_RC, R_RC, FL_K, FR_K, RL_K, RR_K] = ...
        ETR11_GET_POINTS(SW_deg, comp_FL, comp_FR, comp_RL, comp_RR, LR);

    Vy       = Vx * beta;
    delta_FL = deg2rad(FL_K.STEER);
    delta_FR = deg2rad(FR_K.STEER);
    gamma_FL = deg2rad(FL_K.CAMBER);
    gamma_FR = deg2rad(FR_K.CAMBER);
    gamma_RL = deg2rad(RL_K.CAMBER);
    gamma_RR = deg2rad(RR_K.CAMBER);

    alpha_FL = delta_FL - atan2(Vy + a_wb*r, Vx - TF_m/2*r);
    alpha_FR = delta_FR - atan2(Vy + a_wb*r, Vx + TF_m/2*r);
    alpha_RL =          - atan2(Vy - b_wb*r, Vx - TR_m/2*r);
    alpha_RR =          - atan2(Vy - b_wb*r, Vx + TR_m/2*r);

    %% ── Magic Formula at converged state ─────────────────────────────────
    ws = warning('off','all');
    out_FL = mfeval(tir, [FZ_FL, 0, alpha_FL, gamma_FL, 0, Vx], useMode);
    out_FR = mfeval(tir, [FZ_FR, 0, alpha_FR, gamma_FR, 0, Vx], useMode);
    out_RL = mfeval(tir, [FZ_RL, 0, alpha_RL, gamma_RL, 0, Vx], useMode);
    out_RR = mfeval(tir, [FZ_RR, 0, alpha_RR, gamma_RR, 0, Vx], useMode);
    warning(ws);

    % ISO-W → vehicle frame: negate Fy, Mz, Mx
    FY_FL = -out_FL(2);  Mx_FL = -out_FL(4);  Mz_FL = -out_FL(6);
    FY_FR = -out_FR(2);  Mx_FR = -out_FR(4);  Mz_FR = -out_FR(6);

    %% ── Contact patch force vectors ──────────────────────────────────────
    spndl_fl = FL.SPINDLE; spndl_fl(3) = 0;
    spndl_fl = sign(spndl_fl(2)) * spndl_fl / norm(spndl_fl);
    spndl_fr = FR.SPINDLE; spndl_fr(3) = 0;
    spndl_fr = sign(spndl_fr(2)) * spndl_fr / norm(spndl_fr);

    F_FL = FY_FL * spndl_fl + [0, 0, FZ_FL];
    F_FR = FY_FR * spndl_fr + [0, 0, FZ_FR];

    %% ── Dynamics solver → T_SW ───────────────────────────────────────────
    HP_fl = hp_from_wheel(FL);
    HP_fr = hp_from_wheel(FR);

    D_FL = ETR11_DYNAMICS_SOLVER(HP_fl, F_FL, [Mx_FL, 0, Mz_FL]);
    D_FR = ETR11_DYNAMICS_SOLVER(HP_fr, F_FR, [Mx_FR, 0, Mz_FR]);

    D_PINION_M = VPARAMS.D_PINION / 2000;
    e_fl = (FL.TR_UPRIGHT - FL.TR_RACK) / norm(FL.TR_UPRIGHT - FL.TR_RACK);
    e_fr = (FR.TR_UPRIGHT - FR.TR_RACK) / norm(FR.TR_UPRIGHT - FR.TR_RACK);

    rk_fl = -D_FL.TieRod * e_fl(2);
    rk_fr = -D_FR.TieRod * e_fr(2);
    T_SW  = -(rk_fl + rk_fr) * D_PINION_M;

    %% ── Pack outputs ──────────────────────────────────────────────────────
    SS.SW_deg     = SW_deg;
    SS.beta_deg   = rad2deg(beta);
    SS.T_SW       = T_SW;

    SS.FZ         = [FZ_FL, FZ_FR, FZ_RL, FZ_RR];
    SS.FY_vehicle = [FY_FL, FY_FR, -out_RL(2), -out_RR(2)];
    SS.FY_mfeval  = [out_FL(2), out_FR(2), out_RL(2), out_RR(2)];
    SS.FX         = [out_FL(1), out_FR(1), out_RL(1), out_RR(1)];
    SS.Mz_vehicle = [Mz_FL, Mz_FR, -out_RL(6), -out_RR(6)];
    SS.Mx_vehicle = [Mx_FL, Mx_FR, -out_RL(4), -out_RR(4)];

    SS.alpha_deg  = rad2deg([alpha_FL, alpha_FR, alpha_RL, alpha_RR]);
    SS.gamma_deg  = [FL_K.CAMBER, FR_K.CAMBER, RL_K.CAMBER, RR_K.CAMBER];
    SS.delta_deg  = [FL_K.STEER, FR_K.STEER];
    SS.trail_mm   = [FL_K.TRAIL, FR_K.TRAIL, RL_K.TRAIL, RR_K.TRAIL];
    SS.comp_mm    = [comp_FL, comp_FR, comp_RL, comp_RR];
    SS.dFZ        = [dFZ_F, dFZ_R];
    SS.TieRod_FL  = D_FL.TieRod;
    SS.TieRod_FR  = D_FR.TieRod;
    SS.rack_F     = [rk_fl, rk_fr];
    SS.F_RC       = F_RC;  SS.R_RC = R_RC;
    SS.FL_K = FL_K;  SS.FR_K = FR_K;
    SS.RL_K = RL_K;  SS.RR_K = RR_K;
end

%% ── Rear lateral force (vehicle frame) for a given beta ──────────────────
function fy = rear_fy(beta, Vx, r, b_wb, TR_m, ...
                      FZ_RL, FZ_RR, gamma_RL, gamma_RR, tir, useMode)
    Vy   = Vx * beta;
    a_RL = clamp_alpha(-atan2(Vy - b_wb*r, Vx - TR_m/2*r));
    a_RR = clamp_alpha(-atan2(Vy - b_wb*r, Vx + TR_m/2*r));
    ws = warning('off','all');
    o_RL = mfeval(tir, [FZ_RL, 0, a_RL, gamma_RL, 0, Vx], useMode);
    o_RR = mfeval(tir, [FZ_RR, 0, a_RR, gamma_RR, 0, Vx], useMode);
    warning(ws);
    fy = -(o_RL(2) + o_RR(2));
end

%% ── Front lateral force using pre-computed LUT (fast: no kinematics) ─────
function fy = front_fy_lut(SW_deg, beta, Vx, r, a_wb, TF_m, ...
                            FZ_FL, FZ_FR, SW_lut, dFL, dFR, gFL, gFR, ...
                            tir, useMode)
    SW_deg   = max(min(SW_deg, SW_lut(end)), SW_lut(1));   % clamp to LUT range
    delta_FL = interp1(SW_lut, dFL, SW_deg, 'linear');
    delta_FR = interp1(SW_lut, dFR, SW_deg, 'linear');
    gamma_FL = interp1(SW_lut, gFL, SW_deg, 'linear');
    gamma_FR = interp1(SW_lut, gFR, SW_deg, 'linear');
    Vy   = Vx * beta;
    a_FL = clamp_alpha(delta_FL - atan2(Vy + a_wb*r, Vx - TF_m/2*r));
    a_FR = clamp_alpha(delta_FR - atan2(Vy + a_wb*r, Vx + TF_m/2*r));
    ws = warning('off','all');
    o_FL = mfeval(tir, [FZ_FL, 0, a_FL, gamma_FL, 0, Vx], useMode);
    o_FR = mfeval(tir, [FZ_FR, 0, a_FR, gamma_FR, 0, Vx], useMode);
    warning(ws);
    fy = -(o_FL(2) + o_FR(2));
end

%% ── Clamp slip angle to mfeval valid range ────────────────────────────────
function a = clamp_alpha(a)
    a = max(min(a, deg2rad(25)), deg2rad(-25));
end

%% ── Bounded fzero: scan [lo,hi] for sign change, then bisect ─────────────
% Never calls f outside [lo, hi].  Terminates in O(N_scan + fzero iters).
function x = bounded_fzero(f, x0, lo, hi, name)
    opts = optimset('Display','off','TolX',1e-5,'MaxFunEvals',200);

    % Make sure x0 is inside bounds
    x0 = max(min(x0, hi), lo);

    % Quick check: does bracket [lo,hi] already bracket a zero?
    ws = warning('off','all');
    flo = f(lo);  fhi = f(hi);
    warning(ws);

    if sign(flo) ~= sign(fhi)
        % Good bracket — fzero converges in a few iterations
        ws = warning('off','all');
        try
            x = fzero(f, [lo, hi], opts);
            warning(ws); return
        catch
            warning(ws);
        end
    end

    % No sign change at the extremes — scan with N_scan points to find one
    N_scan = 40;
    pts = unique([x0, linspace(lo, hi, N_scan)]);
    ws  = warning('off','all');
    fv  = arrayfun(f, pts);
    warning(ws);

    idx = find(diff(sign(fv)) ~= 0, 1);
    if ~isempty(idx)
        ws = warning('off','all');
        try
            x = fzero(f, [pts(idx), pts(idx+1)], opts);
            warning(ws); return
        catch
            warning(ws);
        end
    end

    % No solution found in [lo,hi] → return x0 and warn once
    x = x0;
    warning('ETR11_SS: no equilibrium for %s in [%.1f, %.1f] — returning x0=%.3g', ...
            name, lo, hi, x0);
end

%% ── Build HP struct from kinematics wheel struct ──────────────────────────
function HP = hp_from_wheel(W)
    HP.CONTACT_PATCH = W.CONTACT_PATCH;
    HP.TR_RACK       = W.TR_RACK;
    HP.TR_UPRIGHT    = W.TR_UPRIGHT;
    HP.PUSH_RKR      = W.PUSH_RKR;
    HP.PUSH_UW       = W.PUSH_UW;
    HP.LW_KN         = W.LW_KN;
    HP.UW_KN         = W.UW_KN;
    HP.URW_MC        = W.URW_MC;
    HP.UFW_MC        = W.UFW_MC;
    HP.LRW_MC        = W.LRW_MC;
    HP.LFW_MC        = W.LFW_MC;
end
