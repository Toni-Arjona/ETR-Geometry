%% ETR11_YMD.m  —  Yaw Moment Diagram  (Milliken Moment Method)
%
%  Variables de barrido:
%    β  (body sideslip)   de −βmax  a +βmax          → familia de curvas azules
%    SW (steering wheel)  de −SW_max a +SW_max        → familia de curvas rojas
%
%  Para cada punto (β, SW):
%    1. Cinemática → ETR11_GET_POINTS con compresión estática (sólo aero-heave).
%       El YMD usa FZ ESTÁTICO:  FZ = peso_estático + aerodinámica.
%       Sin transferencia lateral (esa entraría en el loop; la carga transferida
%       neta no cambia la física del rombo, solo desplaza ligeramente el límite).
%    2. Ángulos de deslizamiento con r = FY_est / (m·Vx)  (una iteración).
%    3. Fuerzas de neumático → mfeval (ISO-W → vehículo).
%    4. FY_total y Mz_CG → punto del YMD.
%
%  El par en volante se calcula sólo en los puntos clave (no en todo el grid)
%  para no multiplicar las llamadas al solver de dinámica.
%
%  Requiere en workspace:  VPARAMS, KIN  (ejecutar solver_dataplot_v2 antes).

clc
clear ETR11_KINEMATICS_ANALYTICAL

if ~exist('VPARAMS','var') || ~exist('KIN','var')
    error('Ejecuta solver_dataplot_v2 primero para tener VPARAMS y KIN.');
end

%% ── Parámetros ───────────────────────────────────────────────────────────
Vx      = 20;     % velocidad [m/s]
beta_max = 14;    % rango de sideslip [deg]
N_beta   = 29;    % impar para incluir β=0
N_SW     = 25;    % impar para incluir SW=0  (se barren ±SW_max)
SW_max   = 135;   % [deg]  (evitar extrapolación cinemática en topes)

beta_vec = linspace(-beta_max, beta_max, N_beta);   % [deg]
SW_vec   = linspace(-SW_max,   SW_max,   N_SW);      % [deg]

%% ── Desempaquetar parámetros de vehículo ──────────────────────────────────
m      = VPARAMS.m_total;
m_spr  = VPARAMS.m_sprung;
WB_m   = VPARAMS.WB / 1000;
TF_m   = VPARAMS.TF / 1000;
TR_m   = VPARAMS.TR_veh / 1000;
a_wb   = WB_m *  VPARAMS.cg_balance;
b_wb   = WB_m * (1 - VPARAMS.cg_balance);
LR     = VPARAMS.LR;
g_acc  = 9.81;
tir    = VPARAMS.tir_file;
uMode  = 111;
Iz     = m * (WB_m/2)^2;  % estimación Iz [kg·m²] (no afecta el steady-state YMD)

% FZ estático (+ aero, sin roll) — constante en todo el grid
dF_total  = 0.5 * VPARAMS.rho_air * VPARAMS.aero_area * VPARAMS.df_coeff * Vx^2;
dF_F      = dF_total * (1 - VPARAMS.aero_balance);
dF_R      = dF_total *      VPARAMS.aero_balance;

W         = m * g_acc;
FZ_F_stat = W * b_wb / WB_m / 2 + dF_F / 2;   % por rueda delantera [N]
FZ_R_stat = W * a_wb / WB_m / 2 + dF_R / 2;   % por rueda trasera   [N]
FZ_stat   = [FZ_F_stat, FZ_F_stat, FZ_R_stat, FZ_R_stat];   % [FL FR RL RR]

% Compresión de heave (aero, constante)
i0 = find(KIN.heave_dpr == 0, 1);
if isempty(i0), [~,i0] = min(abs(KIN.heave_dpr)); end
MR_F = KIN.F_MR(i0);  WR_F = KIN.F_WR(i0);
MR_R = KIN.R_MR(i0);  WR_R = KIN.R_WR(i0);
comp_h_F = (dF_F/2 / WR_F) * MR_F * 1000;   % [mm]
comp_h_R = (dF_R/2 / WR_R) * MR_R * 1000;

%% ── Pre-calcular LUT de cinemática en función de SW ──────────────────────
% Con FZ estático, la cinemática no depende de β → LUT indexada sólo por SW.
fprintf('Pre-calculando LUT de cinemática (%d puntos SW)...', N_SW);
dFL_lut = zeros(1,N_SW);  dFR_lut = zeros(1,N_SW);
gFL_lut = zeros(1,N_SW);  gFR_lut = zeros(1,N_SW);
gRL_lut = zeros(1,N_SW);  gRR_lut = zeros(1,N_SW);
% Guardamos también el struct completo para poder calcular T_SW en puntos clave
FL_lut = cell(1,N_SW);  FR_lut = cell(1,N_SW);

ws_save = warning('off','all');
for k = 1:N_SW
    [FLk,FRk,~,~,~,~,FLKk,FRKk,RLKk,RRKk] = ...
        ETR11_GET_POINTS(SW_vec(k), comp_h_F, comp_h_F, comp_h_R, comp_h_R, LR);
    dFL_lut(k) = deg2rad(FLKk.STEER);
    dFR_lut(k) = deg2rad(FRKk.STEER);
    gFL_lut(k) = deg2rad(FLKk.CAMBER);
    gFR_lut(k) = deg2rad(FRKk.CAMBER);
    gRL_lut(k) = deg2rad(RLKk.CAMBER);
    gRR_lut(k) = deg2rad(RRKk.CAMBER);
    FL_lut{k}  = FLk;
    FR_lut{k}  = FRk;
end
warning(ws_save);
fprintf(' hecho.\n');

% Cambers traseros (no dependen de SW, usar valor central)
gRL_0 = gRL_lut(ceil(N_SW/2));
gRR_0 = gRR_lut(ceil(N_SW/2));

%% ── Barrido principal ────────────────────────────────────────────────────
FY_arr  = zeros(N_beta, N_SW);
Mz_arr  = zeros(N_beta, N_SW);

fprintf('Barrido YMD (%d × %d = %d puntos)...\n', N_beta, N_SW, N_beta*N_SW);
ws_save = warning('off','all');

for ib = 1:N_beta
    beta  = beta_vec(ib);
    Vy    = Vx * deg2rad(beta);

    for is = 1:N_SW
        % Cinemática desde LUT (interp1 en dirección para SW intermedios)
        % Dado que usamos exactamente los puntos de la LUT: índice directo
        dFL = dFL_lut(is);  dFR = dFR_lut(is);
        gFL = gFL_lut(is);  gFR = gFR_lut(is);

        % ── Paso 1: r=0 → FY inicial ─────────────────────────────────────
        aFL0 = ca(dFL - atan2(Vy, Vx));
        aFR0 = ca(dFR - atan2(Vy, Vx));
        aRL0 = ca(     - atan2(Vy, Vx));
        aRR0 = ca(     - atan2(Vy, Vx));

        o0 = mfeval(tir, [FZ_stat(1),0,aFL0,gFL,0,Vx; ...
                          FZ_stat(2),0,aFR0,gFR,0,Vx; ...
                          FZ_stat(3),0,aRL0,gRL_0,0,Vx; ...
                          FZ_stat(4),0,aRR0,gRR_0,0,Vx], uMode);
        FY0 = -sum(o0(:,2));   % ISO-W → vehículo
        r   = FY0 / (m * Vx);  % yaw rate [rad/s]

        % ── Paso 2: ángulos con r ─────────────────────────────────────────
        aFL = ca(dFL - atan2(Vy + a_wb*r, Vx - TF_m/2*r));
        aFR = ca(dFR - atan2(Vy + a_wb*r, Vx + TF_m/2*r));
        aRL = ca(     - atan2(Vy - b_wb*r, Vx - TR_m/2*r));
        aRR = ca(     - atan2(Vy - b_wb*r, Vx + TR_m/2*r));

        o = mfeval(tir, [FZ_stat(1),0,aFL,gFL,  0,Vx; ...
                         FZ_stat(2),0,aFR,gFR,  0,Vx; ...
                         FZ_stat(3),0,aRL,gRL_0,0,Vx; ...
                         FZ_stat(4),0,aRR,gRR_0,0,Vx], uMode);

        FY_FL = -o(1,2);  FY_FR = -o(2,2);
        FY_RL = -o(3,2);  FY_RR = -o(4,2);

        % Mz de neumático (self-aligning, ISO-W → vehículo: negar)
        Mz_tyre = -(o(1,6)+o(2,6)+o(3,6)+o(4,6));

        FY_total = FY_FL + FY_FR + FY_RL + FY_RR;
        Mz_inert = (FY_FL+FY_FR)*a_wb - (FY_RL+FY_RR)*b_wb;
        Mz_total = Mz_inert + Mz_tyre;

        FY_arr(ib,is) = FY_total;
        Mz_arr(ib,is) = Mz_total;
    end
end

warning(ws_save);
clear ETR11_KINEMATICS_ANALYTICAL
fprintf('Hecho.\n\n');

%% ── Normalización ────────────────────────────────────────────────────────
ay_g = FY_arr / (m * g_acc);         % [g]
N_n  = Mz_arr / (m * g_acc * WB_m);  % normalizado Milliken N/(W·L)

%% ══════════════════════════════════════════════════════════════════════════
%%  FIGURA — YMD
%% ══════════════════════════════════════════════════════════════════════════
fig = figure('Name','YMD — Milliken Moment Method — ETR11', ...
    'NumberTitle','off','Color','k','Position',[80 80 1000 750]);
ax = axes(fig,'Color','k','XColor','w','YColor','w', ...
    'GridColor',[0.35 0.35 0.35],'GridAlpha',0.6);
hold(ax,'on'); grid(ax,'on');

% Paleta: β → azul-cian, SW → naranja-rojo
c_beta = parula(N_beta);
c_sw   = autumn(N_SW);

% Curvas β = constante
lw_b = 1.2;
for ib = 1:N_beta
    plot(ax, ay_g(ib,:), N_n(ib,:), '-', 'Color', c_beta(ib,:), 'LineWidth', lw_b);
end
% Etiquetas β
for ib = 1:4:N_beta
    [~,ic] = max(abs(ay_g(ib,:)));
    text(ax, ay_g(ib,ic), N_n(ib,ic), sprintf('β=%.0f°',beta_vec(ib)), ...
        'Color',c_beta(ib,:),'FontSize',7,'Clipping','on');
end

% Curvas SW = constante
lw_s = 1.0;
for is = 1:N_SW
    plot(ax, ay_g(:,is), N_n(:,is), '--', 'Color', c_sw(is,:), 'LineWidth', lw_s);
end
% Etiquetas SW
for is = 1:4:N_SW
    [~,ib_m] = max(abs(ay_g(:,is)));
    text(ax, ay_g(ib_m,is), N_n(ib_m,is), sprintf('SW=%.0f°',SW_vec(is)), ...
        'Color',c_sw(is,:),'FontSize',7,'Clipping','on');
end

% Línea Mz=0 (equilibrio de guiñada)
xline(ax, 0,  'w:', 'LineWidth', 0.8);
yline(ax, 0, 'w-', 'LineWidth', 1.8, 'DisplayName','Mz = 0  (equilibrio)');

%% ── Frontera de equilibrio: corte Mz=0 para cada SW ─────────────────────
eq_ay = NaN(1,N_SW);
eq_b  = NaN(1,N_SW);
for is = 1:N_SW
    col_N  = N_n(:,is);
    col_ay = ay_g(:,is);
    idx = find(diff(sign(col_N)) ~= 0);
    if isempty(idx), continue; end
    for ix = idx'
        t = -col_N(ix)/(col_N(ix+1)-col_N(ix));
        ay_cross = col_ay(ix) + t*(col_ay(ix+1)-col_ay(ix));
        b_cross  = beta_vec(ix) + t*(beta_vec(ix+1)-beta_vec(ix));
        % guardar el cruce con mayor |ay|
        if isnan(eq_ay(is)) || abs(ay_cross) > abs(eq_ay(is))
            eq_ay(is) = ay_cross;
            eq_b(is)  = b_cross;
        end
    end
end
valid = ~isnan(eq_ay);
plot(ax, eq_ay(valid), zeros(1,sum(valid)), 'wo', ...
    'MarkerSize',5,'MarkerFaceColor','w','HandleVisibility','off');

% Máxima ay en equilibrio (derecha)
eq_pos = eq_ay(valid & eq_ay > 0);
SW_pos = SW_vec(valid & eq_ay > 0);
b_pos  = eq_b( valid & eq_ay > 0);
[ay_max_g, imx] = max(eq_pos);
SW_at_max   = SW_pos(imx);
beta_at_max = b_pos(imx);

plot(ax, ay_max_g, 0, 'r*','MarkerSize',14,'LineWidth',2,'DisplayName', ...
    sprintf('Max a_y = %.2fg  (SW=%.0f°, β=%.1f°)', ay_max_g, SW_at_max, beta_at_max));

% Máxima ay (izquierda, simetría)
eq_neg  = eq_ay(valid & eq_ay < 0);
[ay_min_g, imn] = min(eq_neg);
plot(ax, ay_min_g, 0, 'r*','MarkerSize',14,'LineWidth',2,'HandleVisibility','off');

xlabel(ax,'Lateral acceleration  a_y  [g]','Color','w','FontSize',12)
ylabel(ax,'Yaw moment  N / (W·L)  [ – ]','Color','w','FontSize',12)
title(ax, sprintf('YMD  –  ETR11  |  V_x = %.0f m/s  |  β (azul)  ×  SW (naranja–rojo)', Vx), ...
    'Color','w','FontSize',12)
legend(ax,'show','Location','northwest','TextColor','w','Color','none')
ax.XLim = [min(ay_g(:))-0.1, max(ay_g(:))+0.1];

%% ══════════════════════════════════════════════════════════════════════════
%%  PUNTOS CLAVE — T_SW y fuerzas en el punto de máxima ay
%% ══════════════════════════════════════════════════════════════════════════
fprintf('══════════════════════════════════════════════════\n');
fprintf('  YMD  —  ETR11   Vx = %.0f m/s\n', Vx);
fprintf('══════════════════════════════════════════════════\n\n');
fprintf('[ MÁXIMA AY  (Mz = 0, equilibrio de guiñada) ]\n');
fprintf('  ay_max = %.3f g   SW = %.1f°   β = %.2f°\n\n', ...
        ay_max_g, SW_at_max, beta_at_max);

% Localizar índice en el grid más cercano al punto de max ay
[~, is_key] = min(abs(SW_vec - SW_at_max));
[~, ib_key] = min(abs(beta_vec - beta_at_max));
key_SW = SW_vec(is_key);
key_b  = beta_vec(ib_key);

% Recalcular punto clave con cinemática completa (con roll para mayor precisión)
kf = VPARAMS.k_roll_F;  kr = VPARAMS.k_roll_R;
h_RC_F = VPARAMS.h_RC_F;  h_RC_R = VPARAMS.h_RC_R;
h_roll = VPARAMS.cg_h/1000 - (h_RC_F+h_RC_R)/2;

ay_key = ay_max_g * g_acc;
dFZ_F  = ay_key * m_spr / TF_m * (h_roll*kf/(kf+kr) + b_wb/WB_m*h_RC_F);
dFZ_R  = ay_key * m_spr / TR_m * (h_roll*kr/(kf+kr) + a_wb/WB_m*h_RC_R);
comp_FL_k = comp_h_F + (dFZ_F/WR_F)*MR_F*1000;
comp_FR_k = comp_h_F - (dFZ_F/WR_F)*MR_F*1000;
comp_RL_k = comp_h_R + (dFZ_R/WR_R)*MR_R*1000;
comp_RR_k = comp_h_R - (dFZ_R/WR_R)*MR_R*1000;
FZ_FL_k = max(FZ_F_stat + dFZ_F, 50);
FZ_FR_k = max(FZ_F_stat - dFZ_F, 50);
FZ_RL_k = max(FZ_R_stat + dFZ_R, 50);
FZ_RR_k = max(FZ_R_stat - dFZ_R, 50);

ws_save = warning('off','all');
[FL_k,FR_k,~,~,~,~,FL_Kk,FR_Kk,RL_Kk,RR_Kk] = ...
    ETR11_GET_POINTS(key_SW, comp_FL_k, comp_FR_k, comp_RL_k, comp_RR_k, LR);
warning(ws_save);

Vy_k = Vx * deg2rad(key_b);
r_k  = ay_key / Vx;
dFL_k = deg2rad(FL_Kk.STEER);  dFR_k = deg2rad(FR_Kk.STEER);
gFL_k = deg2rad(FL_Kk.CAMBER); gFR_k = deg2rad(FR_Kk.CAMBER);
gRL_k = deg2rad(RL_Kk.CAMBER); gRR_k = deg2rad(RR_Kk.CAMBER);

aFL_k = ca(dFL_k - atan2(Vy_k+a_wb*r_k, Vx-TF_m/2*r_k));
aFR_k = ca(dFR_k - atan2(Vy_k+a_wb*r_k, Vx+TF_m/2*r_k));
aRL_k = ca(       - atan2(Vy_k-b_wb*r_k, Vx-TR_m/2*r_k));
aRR_k = ca(       - atan2(Vy_k-b_wb*r_k, Vx+TR_m/2*r_k));

ws_save = warning('off','all');
o_FL_k = mfeval(tir,[FZ_FL_k,0,aFL_k,gFL_k,0,Vx],uMode);
o_FR_k = mfeval(tir,[FZ_FR_k,0,aFR_k,gFR_k,0,Vx],uMode);
o_RL_k = mfeval(tir,[FZ_RL_k,0,aRL_k,gRL_k,0,Vx],uMode);
o_RR_k = mfeval(tir,[FZ_RR_k,0,aRR_k,gRR_k,0,Vx],uMode);
warning(ws_save);

FY_FL_k=-o_FL_k(2); FY_FR_k=-o_FR_k(2);
Mx_FL_k=-o_FL_k(4); Mx_FR_k=-o_FR_k(4);
Mz_FL_k=-o_FL_k(6); Mz_FR_k=-o_FR_k(6);

% Dinámicas de suspensión FL
spFL = FL_k.SPINDLE; spFL(3)=0; spFL = sign(spFL(2))*spFL/norm(spFL);
spFR = FR_k.SPINDLE; spFR(3)=0; spFR = sign(spFR(2))*spFR/norm(spFR);
F_FL_v = FY_FL_k*spFL + [0,0,FZ_FL_k];
F_FR_v = FY_FR_k*spFR + [0,0,FZ_FR_k];
HP_fl = hp_from_wheel(FL_k);
HP_fr = hp_from_wheel(FR_k);
D_FL = ETR11_DYNAMICS_SOLVER(HP_fl, F_FL_v, [Mx_FL_k,0,Mz_FL_k]);
D_FR = ETR11_DYNAMICS_SOLVER(HP_fr, F_FR_v, [Mx_FR_k,0,Mz_FR_k]);

D_PIN = VPARAMS.D_PINION / 2000;
e_fl = (FL_k.TR_UPRIGHT-FL_k.TR_RACK)/norm(FL_k.TR_UPRIGHT-FL_k.TR_RACK);
e_fr = (FR_k.TR_UPRIGHT-FR_k.TR_RACK)/norm(FR_k.TR_UPRIGHT-FR_k.TR_RACK);
rk_fl_k = -D_FL.TieRod * e_fl(2);
rk_fr_k = -D_FR.TieRod * e_fr(2);
T_SW_key = -(rk_fl_k + rk_fr_k) * D_PIN;

fprintf('  FZ  [N]   FL=%7.1f  FR=%7.1f  RL=%7.1f  RR=%7.1f\n', FZ_FL_k,FZ_FR_k,FZ_RL_k,FZ_RR_k);
fprintf('  FY  [N]   FL=%7.1f  FR=%7.1f  RL=%7.1f  RR=%7.1f\n', FY_FL_k,FY_FR_k,-o_RL_k(2),-o_RR_k(2));
fprintf('  α  [deg]  FL=%7.2f  FR=%7.2f  RL=%7.2f  RR=%7.2f\n', ...
    rad2deg(aFL_k),rad2deg(aFR_k),rad2deg(aRL_k),rad2deg(aRR_k));
fprintf('  γ  [deg]  FL=%7.2f  FR=%7.2f  RL=%7.2f  RR=%7.2f\n', ...
    FL_Kk.CAMBER, FR_Kk.CAMBER, RL_Kk.CAMBER, RR_Kk.CAMBER);
fprintf('  δ  [deg]  FL=%7.2f  FR=%7.2f\n', FL_Kk.STEER, FR_Kk.STEER);

fprintf('\n  Componentes de suspensión  (FL — exterior)\n');
fprintf('  %-12s %10s  |  %-12s %10s\n','Componente','FL [N]','Componente','FR [N]');
comps = {'TieRod','PUSH','UFW','URW','LFW','LRW'};
for c = comps
    fprintf('  %-12s %10.1f  |  %-12s %10.1f\n', ...
        c{1}, D_FL.(c{1}), c{1}, D_FR.(c{1}));
end
fprintf('\n  T_SW en max ay = %.2f N·m\n', T_SW_key);

%% ── Máximo T_SW: buscar sobre el contorno de equilibrio Mz=0 ─────────────
fprintf('\n══════════════════════════════════════════════════\n');
fprintf('[ MÁXIMO T_SW  sobre la frontera Mz=0 ]\n');

% Para cada punto de equilibrio SW barrido, calcular T_SW
n_eq = sum(valid & eq_ay > 0);
TSW_eq = NaN(1, n_eq);
SW_eq  = SW_pos;
b_eq   = b_pos;

for ie = 1:n_eq
    b_ie  = b_eq(ie);
    sw_ie = SW_eq(ie);
    ay_ie = eq_pos(ie) * g_acc;
    [~, isw] = min(abs(SW_vec - sw_ie));

    dFZ_ie = ay_ie * m_spr / TF_m * (h_roll*kf/(kf+kr) + b_wb/WB_m*h_RC_F);
    c_FL_ie = comp_h_F + (dFZ_ie/WR_F)*MR_F*1000;
    c_FR_ie = comp_h_F - (dFZ_ie/WR_F)*MR_F*1000;
    FZ_FL_ie = max(FZ_F_stat + dFZ_ie, 50);
    FZ_FR_ie = max(FZ_F_stat - dFZ_ie, 50);

    ws_s = warning('off','all');
    [FLi,FRi,~,~,~,~,FLKi,FRKi] = ...
        ETR11_GET_POINTS(sw_ie, c_FL_ie, c_FR_ie, comp_h_R, comp_h_R, LR);
    warning(ws_s);

    Vy_ie = Vx*deg2rad(b_ie);  r_ie = ay_ie/Vx;
    dFL_ie = deg2rad(FLKi.STEER);  dFR_ie = deg2rad(FRKi.STEER);
    gFL_ie = deg2rad(FLKi.CAMBER); gFR_ie = deg2rad(FRKi.CAMBER);
    aFL_ie = ca(dFL_ie - atan2(Vy_ie+a_wb*r_ie, Vx-TF_m/2*r_ie));
    aFR_ie = ca(dFR_ie - atan2(Vy_ie+a_wb*r_ie, Vx+TF_m/2*r_ie));

    ws_s = warning('off','all');
    o_FLi = mfeval(tir,[FZ_FL_ie,0,aFL_ie,gFL_ie,0,Vx],uMode);
    o_FRi = mfeval(tir,[FZ_FR_ie,0,aFR_ie,gFR_ie,0,Vx],uMode);
    warning(ws_s);

    FY_FLi=-o_FLi(2); FY_FRi=-o_FRi(2);
    Mx_FLi=-o_FLi(4); Mx_FRi=-o_FRi(4);
    Mz_FLi=-o_FLi(6); Mz_FRi=-o_FRi(6);

    spFLi=FLi.SPINDLE; spFLi(3)=0; spFLi=sign(spFLi(2))*spFLi/norm(spFLi);
    spFRi=FRi.SPINDLE; spFRi(3)=0; spFRi=sign(spFRi(2))*spFRi/norm(spFRi);
    HPfli=hp_from_wheel(FLi);  HPfri=hp_from_wheel(FRi);
    DFLi=ETR11_DYNAMICS_SOLVER(HPfli,FY_FLi*spFLi+[0,0,FZ_FL_ie],[Mx_FLi,0,Mz_FLi]);
    DFRi=ETR11_DYNAMICS_SOLVER(HPfri,FY_FRi*spFRi+[0,0,FZ_FR_ie],[Mx_FRi,0,Mz_FRi]);
    efli=(FLi.TR_UPRIGHT-FLi.TR_RACK)/norm(FLi.TR_UPRIGHT-FLi.TR_RACK);
    efri=(FRi.TR_UPRIGHT-FRi.TR_RACK)/norm(FRi.TR_UPRIGHT-FRi.TR_RACK);
    TSW_eq(ie) = -((-DFLi.TieRod*efli(2)) + (-DFRi.TieRod*efri(2))) * D_PIN;
end

% Plotear T_SW sobre la frontera
plot(ax, eq_pos, zeros(1,n_eq), 'g-', 'LineWidth', 2, ...
    'DisplayName', 'Frontera equilibrio (Mz=0)');
scatter(ax, eq_pos, zeros(1,n_eq), 40, TSW_eq, 'filled', ...
    'HandleVisibility','off');
colormap(ax, hot(256));
cb = colorbar(ax); cb.Color = 'w';
ylabel(cb,'T_{SW} [N·m]','Color','w');

[T_SW_max, itmax] = max(TSW_eq);
plot(ax, eq_pos(itmax), 0, 'g*','MarkerSize',14,'LineWidth',2, ...
    'DisplayName', sprintf('Max T_{SW} = %.1f N·m', T_SW_max));

fprintf('  T_SW_max = %.2f N·m\n', T_SW_max);
fprintf('  ay       = %.3f g\n',   eq_pos(itmax));
fprintf('  SW       = %.1f°\n',    SW_eq(itmax));
fprintf('  β        = %.2f°\n',    b_eq(itmax));
fprintf('══════════════════════════════════════════════════\n');

clear ETR11_KINEMATICS_ANALYTICAL

%% ── Helper: clamp alpha ────────────────────────────────────────────────────
function a = ca(a)
    a = max(min(a, deg2rad(25)), deg2rad(-25));
end

function HP = hp_from_wheel(W)
    HP.CONTACT_PATCH = W.CONTACT_PATCH;  HP.TR_RACK  = W.TR_RACK;
    HP.TR_UPRIGHT    = W.TR_UPRIGHT;     HP.PUSH_RKR = W.PUSH_RKR;
    HP.PUSH_UW       = W.PUSH_UW;        HP.LW_KN    = W.LW_KN;
    HP.UW_KN         = W.UW_KN;          HP.URW_MC   = W.URW_MC;
    HP.UFW_MC        = W.UFW_MC;         HP.LRW_MC   = W.LRW_MC;
    HP.LFW_MC        = W.LFW_MC;
end
