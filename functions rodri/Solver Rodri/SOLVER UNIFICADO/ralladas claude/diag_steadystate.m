% diag_steadystate.m  — diagnóstico paso a paso del solver steady-state
% Ejecutar desde SOLVER UNIFICADO después de haber corrido solver_dataplot_v2
% (necesita VPARAMS y KIN en el workspace, o los define aquí manualmente).
%
% Pega el output completo al chat para diagnóstico.

clc
fprintf('=== DIAGNÓSTICO STEADY-STATE SOLVER ===\n\n');

%% ── 1. Caso de prueba ────────────────────────────────────────────────────
ay  = 1.5 * 9.81;   % [m/s²]  1.5g derecha
Vx  = 20;           % [m/s]
g   = 9.81;

% Si VPARAMS/KIN no están en workspace los definimos mínimos aquí:
if ~exist('VPARAMS','var') || ~exist('KIN','var')
    error('Corre solver_dataplot_v2 primero para tener VPARAMS y KIN en workspace.');
end

fprintf('Caso: ay=%.2f m/s² (%.2fg)   Vx=%.1f m/s\n\n', ay, ay/g, Vx);

%% ── 2. Cargas verticales y compresiones ─────────────────────────────────
m      = VPARAMS.m_total;
m_spr  = VPARAMS.m_sprung;
WB_m   = VPARAMS.WB / 1000;
TF_m   = VPARAMS.TF / 1000;
TR_m   = VPARAMS.TR_veh / 1000;
a_wb   = WB_m * VPARAMS.cg_balance;
b_wb   = WB_m * (1 - VPARAMS.cg_balance);
kf     = VPARAMS.k_roll_F;
kr     = VPARAMS.k_roll_R;
h_RC_F = VPARAMS.h_RC_F;
h_RC_R = VPARAMS.h_RC_R;
h_roll = VPARAMS.cg_h/1000 - (h_RC_F + h_RC_R)/2;
LR     = VPARAMS.LR;

dF_total = 0.5 * VPARAMS.rho_air * VPARAMS.aero_area * VPARAMS.df_coeff * Vx^2;
dF_F = dF_total * (1 - VPARAMS.aero_balance);
dF_R = dF_total *      VPARAMS.aero_balance;

W         = m * g;
FZ_stat_F = W * b_wb / WB_m / 2 + dF_F / 2;
FZ_stat_R = W * a_wb / WB_m / 2 + dF_R / 2;
dFZ_F = ay * m_spr / TF_m * (h_roll * kf/(kf+kr) + b_wb/WB_m * h_RC_F);
dFZ_R = ay * m_spr / TR_m * (h_roll * kr/(kf+kr) + a_wb/WB_m * h_RC_R);

FZ_FL = max(FZ_stat_F + dFZ_F, 50);
FZ_FR = max(FZ_stat_F - dFZ_F, 50);
FZ_RL = max(FZ_stat_R + dFZ_R, 50);
FZ_RR = max(FZ_stat_R - dFZ_R, 50);

i0 = find(KIN.heave_dpr == 0, 1);
if isempty(i0), [~,i0] = min(abs(KIN.heave_dpr)); end
MR_F = KIN.F_MR(i0);  WR_F = KIN.F_WR(i0);
MR_R = KIN.R_MR(i0);  WR_R = KIN.R_WR(i0);

comp_FL =  (dF_F/2/WR_F + dFZ_F/WR_F) * MR_F * 1000;
comp_FR =  (dF_F/2/WR_F - dFZ_F/WR_F) * MR_F * 1000;
comp_RL =  (dF_R/2/WR_R + dFZ_R/WR_R) * MR_R * 1000;
comp_RR =  (dF_R/2/WR_R - dFZ_R/WR_R) * MR_R * 1000;

fprintf('--- Cargas verticales ---\n');
fprintf('  FZ_FL=%.1f N   FZ_FR=%.1f N\n', FZ_FL, FZ_FR);
fprintf('  FZ_RL=%.1f N   FZ_RR=%.1f N\n', FZ_RL, FZ_RR);
fprintf('  dFZ_F=%.1f N   dFZ_R=%.1f N\n', dFZ_F, dFZ_R);
fprintf('  comp_FL=%.2f mm  comp_FR=%.2f mm\n', comp_FL, comp_FR);
fprintf('  comp_RL=%.2f mm  comp_RR=%.2f mm\n\n', comp_RL, comp_RR);

%% ── 3. Balance de fuerzas esperado ──────────────────────────────────────
FY_rear_tgt  = m * ay * a_wb / WB_m;
FY_front_tgt = m * ay * b_wb / WB_m;
fprintf('--- Objetivos de fuerza lateral ---\n');
fprintf('  FY_rear_tgt  = %.1f N  (= m·ay·a_wb/WB)\n', FY_rear_tgt);
fprintf('  FY_front_tgt = %.1f N  (= m·ay·b_wb/WB)\n', FY_front_tgt);
fprintf('  Suma          = %.1f N  (debe = m·ay = %.1f N)\n\n', ...
        FY_rear_tgt + FY_front_tgt, m*ay);

%% ── 4. Llamada al solver y resultado ─────────────────────────────────────
fprintf('--- Llamando ETR11_STEADYSTATE_SOLVER...\n');
SS = ETR11_STEADYSTATE_SOLVER(ay, Vx, VPARAMS, KIN);
fprintf('  beta    = %.3f deg\n', SS.beta_deg);
fprintf('  SW_deg  = %.2f deg\n', SS.SW_deg);
fprintf('  T_SW    = %.3f N·m\n\n', SS.T_SW);

%% ── 5. Verificar balance de fuerzas en la solución ───────────────────────
FY_rear_actual  = SS.FY_vehicle(3) + SS.FY_vehicle(4);
FY_front_actual = SS.FY_vehicle(1) + SS.FY_vehicle(2);
FY_total        = sum(SS.FY_vehicle);

fprintf('--- Verificación del balance de fuerzas ---\n');
fprintf('  FY_FL=%.1f N   FY_FR=%.1f N\n', SS.FY_vehicle(1), SS.FY_vehicle(2));
fprintf('  FY_RL=%.1f N   FY_RR=%.1f N\n', SS.FY_vehicle(3), SS.FY_vehicle(4));
fprintf('  FY_front_actual = %.1f N   (objetivo: %.1f N,  error: %.2f N)\n', ...
        FY_front_actual, FY_front_tgt, FY_front_actual - FY_front_tgt);
fprintf('  FY_rear_actual  = %.1f N   (objetivo: %.1f N,  error: %.2f N)\n', ...
        FY_rear_actual, FY_rear_tgt, FY_rear_actual - FY_rear_tgt);
fprintf('  FY_total        = %.1f N   (m·ay = %.1f N,  error: %.2f N)\n\n', ...
        FY_total, m*ay, FY_total - m*ay);

%% ── 6. Ángulos de deslizamiento ─────────────────────────────────────────
fprintf('--- Slip angles en la solución ---\n');
fprintf('  alpha_FL=%.3f°  alpha_FR=%.3f°\n', SS.alpha_deg(1), SS.alpha_deg(2));
fprintf('  alpha_RL=%.3f°  alpha_RR=%.3f°\n', SS.alpha_deg(3), SS.alpha_deg(4));
fprintf('  [todos > 0 para giro derecha → mfeval FY < 0 → FY_vehicle = -FY_mfeval > 0]\n\n');

%% ── 7. Salidas brutas de mfeval ──────────────────────────────────────────
fprintf('--- mfeval output (raw, ISO-W) ---\n');
fprintf('  FY_mfeval:  FL=%.1f N   FR=%.1f N   RL=%.1f N   RR=%.1f N\n', ...
        SS.FY_mfeval(1), SS.FY_mfeval(2), SS.FY_mfeval(3), SS.FY_mfeval(4));
fprintf('  [debe ser < 0 para giro derecha]\n');
fprintf('  Mz_vehicle: FL=%.2f Nm  FR=%.2f Nm\n', SS.Mz_vehicle(1), SS.Mz_vehicle(2));
fprintf('  Mx_vehicle: FL=%.2f Nm  FR=%.2f Nm\n\n', SS.Mx_vehicle(1), SS.Mx_vehicle(2));

%% ── 8. Cinemática de dirección ───────────────────────────────────────────
fprintf('--- Cinemática en la solución ---\n');
fprintf('  Steer wheel:  delta_FL=%.3f°   delta_FR=%.3f°\n', ...
        SS.delta_deg(1), SS.delta_deg(2));
fprintf('  Camber:       gamma_FL=%.3f°   gamma_FR=%.3f°\n', ...
        SS.gamma_deg(1), SS.gamma_deg(2));
fprintf('  Trail:        trail_FL=%.2f mm  trail_FR=%.2f mm\n\n', ...
        SS.trail_mm(1), SS.trail_mm(2));

%% ── 9. Steering torque breakdown ────────────────────────────────────────
fprintf('--- Steering torque ---\n');
fprintf('  TieRod_FL = %.1f N   TieRod_FR = %.1f N\n', SS.TieRod_FL, SS.TieRod_FR);
fprintf('  rack_FL   = %.2f N   rack_FR   = %.2f N\n', SS.rack_F(1), SS.rack_F(2));
fprintf('  T_SW      = %.3f N·m  (positivo = resistencia giro derecha)\n\n', SS.T_SW);

%% ── 10. Test mfeval sign convention directo ──────────────────────────────
fprintf('--- Test mfeval signo (control: alpha=+4 deg, FZ=800 N, gamma=0) ---\n');
tir = VPARAMS.tir_file;
useMode = 111;
ws = warning('off','all');
o_pos = mfeval(tir, [800, 0,  deg2rad(4), 0, 0, Vx], useMode);
o_neg = mfeval(tir, [800, 0, -deg2rad(4), 0, 0, Vx], useMode);
warning(ws);
fprintf('  alpha=+4°: Fy=%.1f N   Mz=%.2f Nm   Mx=%.2f Nm\n', ...
        o_pos(2), o_pos(6), o_pos(4));
fprintf('  alpha=-4°: Fy=%.1f N   Mz=%.2f Nm   Mx=%.2f Nm\n', ...
        o_neg(2), o_neg(6), o_neg(4));
fprintf('  [esperado: alpha+4 → Fy<0,  alpha-4 → Fy>0]\n\n');

fprintf('=== FIN DIAGNÓSTICO ===\n');
