% TEST: Compara ETR11_KINEMATICS_SOLVER (numérico) vs ETR11_KINEMATICS_ANALYTICAL
% Ejecutar desde la carpeta SOLVER UNIFICADO.
% Si todos los errores son < 1e-6 mm, el solver analítico es correcto.

clc
clear ETR11_KINEMATICS_SOLVER ETR11_KINEMATICS_ANALYTICAL ETR11_GET_POINTS

fprintf('=== TEST SOLVER ANALÍTICO vs NUMÉRICO ===\n\n');

LOADED_RADIUS = 0.203;
tol = 1e-4;   % tolerancia en mm (muy conservador)
all_ok = true;

% Casos de prueba: [steer, FL_comp, FR_comp, RL_comp, RR_comp]
cases = [
     0,    0,    0,    0,    0;   % posición estática
     0,   20,   20,   20,   20;   % compresión máxima
     0,  -20,  -20,  -20,  -20;   % extensión máxima
     0,   20,  -20,   20,  -20;   % roll máximo
    90,    0,    0,    0,    0;   % giro máximo
    45,   15,   -5,   10,    0;   % caso mixto
   -90,  -10,   10,   -5,   15;   % caso mixto inverso
];

fields_3d = {'LW_KN','UW_KN','PUSH_UW','PUSH_RKR','TR_UPRIGHT','TR_RACK', ...
             'DPR_RKR','SPINDLE_CENTER','SPINDLE_INNER','CONTACT_PATCH','KP_FLOOR'};
fields_1d = {'RKR_ANGLE','ARB_ANGLE'};
wheels    = {'FL','FR','RL','RR'};

for i = 1:size(cases, 1)
    st = cases(i,1); cFL = cases(i,2); cFR = cases(i,3);
    cRL = cases(i,4); cRR = cases(i,5);

    % Reset semillas para que ambos empiecen desde cero en cada caso
    clear ETR11_KINEMATICS_SOLVER ETR11_KINEMATICS_ANALYTICAL

    [FL_n, FR_n, RL_n, RR_n] = get_all_numerical(st, cFL, cFR, cRL, cRR, LOADED_RADIUS);
    [FL_a, FR_a, RL_a, RR_a] = get_all_analytical(st, cFL, cFR, cRL, cRR, LOADED_RADIUS);

    W_num = {FL_n, FR_n, RL_n, RR_n};
    W_ana = {FL_a, FR_a, RL_a, RR_a};

    fprintf('Caso %d: steer=%3.0f°  comp=[%3.0f %3.0f %3.0f %3.0f] mm\n', ...
            i, st, cFL, cFR, cRL, cRR);

    case_ok = true;
    for w = 1:4
        Wn = W_num{w};  Wa = W_ana{w};

        for f = 1:length(fields_3d)
            fn = fields_3d{f};
            err = norm(Wn.(fn) - Wa.(fn));
            if err > tol
                fprintf('  ✗ %s.%s  err=%.2e mm\n', wheels{w}, fn, err);
                case_ok = false;  all_ok = false;
            end
        end

        for f = 1:length(fields_1d)
            fn = fields_1d{f};
            err = abs(Wn.(fn) - Wa.(fn));
            if err > tol
                fprintf('  ✗ %s.%s  err=%.2e rad\n', wheels{w}, fn, err);
                case_ok = false;  all_ok = false;
            end
        end
    end

    if case_ok
        fprintf('  ✓ Todos los puntos coinciden (err < %.0e)\n', tol);
    end
end

fprintf('\n');
if all_ok
    fprintf('✓✓ RESULTADO: Solver analítico CORRECTO en todos los casos.\n');
    fprintf('   Puedes sustituir ETR11_KINEMATICS_SOLVER por ETR11_KINEMATICS_ANALYTICAL.\n');
else
    fprintf('✗✗ RESULTADO: Hay discrepancias. Revisar los casos marcados con ✗.\n');
end

%% ── Benchmark de velocidad ───────────────────────────────────────────────────
% METODOLOGÍA: 100 frames secuenciales de un barrido de roll.
% El numérico usa semilla caliente COMPLETA: fzero(RKR), fzero(ARB) y fsolve
% se inicializan con los ángulos del frame anterior (como ocurre en el visualizador).
% El analítico no necesita semilla: solución cerrada pura.
% Ambos parten de estado frío (clear) para el primer frame.

fprintf('\n=== BENCHMARK (100 frames de animación roll, semilla caliente activa) ===\n');

N = 100;
comp_sweep = linspace(-20, 20, N);

% ── Numérico con semilla caliente ──
clear ETR11_KINEMATICS_SOLVER ETR11_KINEMATICS_ANALYTICAL
tic
for k = 1:N
    get_all_numerical(0, -comp_sweep(k), comp_sweep(k), -comp_sweep(k), comp_sweep(k), LOADED_RADIUS);
end
t_num_warm = toc;

% ── Numérico en frío (peor caso: cada frame parte de semilla 0) ──
t_num_cold = 0;
for k = 1:N
    clear ETR11_KINEMATICS_SOLVER
    t0 = tic;
    get_all_numerical(0, -comp_sweep(k), comp_sweep(k), -comp_sweep(k), comp_sweep(k), LOADED_RADIUS);
    t_num_cold = t_num_cold + toc(t0);
end

% ── Analítico ──
clear ETR11_KINEMATICS_SOLVER ETR11_KINEMATICS_ANALYTICAL
tic
for k = 1:N
    get_all_analytical(0, -comp_sweep(k), comp_sweep(k), -comp_sweep(k), comp_sweep(k), LOADED_RADIUS);
end
t_ana = toc;

fprintf('  Numérico (semilla caliente): %.3f s  (%.1f ms/frame)  ~%.0f FPS\n', t_num_warm, t_num_warm/N*1000, N/t_num_warm);
fprintf('  Numérico (frío, peor caso):  %.3f s  (%.1f ms/frame)  ~%.0f FPS\n', t_num_cold, t_num_cold/N*1000, N/t_num_cold);
fprintf('  Analítico:                   %.3f s  (%.1f ms/frame)  ~%.0f FPS\n', t_ana,      t_ana/N*1000,      N/t_ana);
fprintf('  Speedup analítico vs numérico (caliente): %.1fx\n', t_num_warm/t_ana);
fprintf('  Speedup analítico vs numérico (frío):     %.1fx\n', t_num_cold/t_ana);


%% ── Helpers ──────────────────────────────────────────────────────────────────
function [FL, FR, RL, RR] = get_all_numerical(st, cFL, cFR, cRL, cRR, lr)
    persistent HP
    if isempty(HP), HP = build_hardpoints(); end
    FL = ETR11_KINEMATICS_SOLVER(HP.FL, st,  cFL, lr, 'FL');
    FR = ETR11_KINEMATICS_SOLVER(HP.FR, st,  cFR, lr, 'FR');
    RL = ETR11_KINEMATICS_SOLVER(HP.RL, 0,   cRL, lr, 'RL');
    RR = ETR11_KINEMATICS_SOLVER(HP.RR, 0,   cRR, lr, 'RR');
end

function [FL, FR, RL, RR] = get_all_analytical(st, cFL, cFR, cRL, cRR, lr)
    persistent HP
    if isempty(HP), HP = build_hardpoints(); end
    FL = ETR11_KINEMATICS_ANALYTICAL(HP.FL, st,  cFL, lr, 'FL');
    FR = ETR11_KINEMATICS_ANALYTICAL(HP.FR, st,  cFR, lr, 'FR');
    RL = ETR11_KINEMATICS_ANALYTICAL(HP.RL, 0,   cRL, lr, 'RL');
    RR = ETR11_KINEMATICS_ANALYTICAL(HP.RR, 0,   cRR, lr, 'RR');
end

function HP = build_hardpoints()
    % Copia exacta de los hardpoints de ETR11_GET_POINTS
    HP_FL.SPINDLE_CENTER = [-100, -625, 203];
    HP_FL.SPINDLE_INNER  = [-100, -567, 200.5];
    HP_FL.UW_KN          = [-98, -545, 294];
    HP_FL.LW_KN          = [-106,-585, 106];
    HP_FL.URW_MC         = [67, -240, 274];
    HP_FL.UFW_MC         = [-207, -240, 279.5];
    HP_FL.LRW_MC         = [94.5, -175, 136];
    HP_FL.LFW_MC         = [-224.5, -175, 122.5];
    HP_FL.RKR_1          = [-48, -179, 587];
    HP_FL.RKR_2          = [-48, -154.25, 552];
    f_rkr = HP_FL.RKR_1 - HP_FL.RKR_2;
    f_a = 50; f_b = 85.5;
    HP_FL.PUSH_UW        = [-98, -516, 315];
    HP_FL.PUSH_RKR       = (HP_FL.RKR_2 + 0.5*f_rkr) + f_a*[-1,0,0];
    HP_FL.DPR_RKR        = (HP_FL.RKR_2 + 0.5*f_rkr) + f_b*cross(f_a*[-1,0,0], f_rkr)/norm(cross(f_a*[-1,0,0], f_rkr));
    HP_FL.DPR_MC         = HP_FL.DPR_RKR + 175*[1,0,0];
    HP_FL.TR_UPRIGHT     = [-180, -560, 137];
    HP_FL.TR_RACK        = [-162, -176.5, 150.75];
    HP_FL.D_PINION       = 29;
    HP_FL.AR_AXIS        = [-150, -130, 634];
    HP_FL.AR_LINK_RKR    = (HP_FL.RKR_2 + 0.5*f_rkr) + 30*((HP_FL.DPR_RKR-(HP_FL.RKR_2+0.5*f_rkr))/norm(HP_FL.DPR_RKR-(HP_FL.RKR_2+0.5*f_rkr)));
    HP_FL.AR_LINK_ARB    = HP_FL.AR_AXIS - [0,0,60];

    HP_RL.SPINDLE_CENTER = [1435, -625, 203];
    HP_RL.SPINDLE_INNER  = [1435, -567, 200.5];
    HP_RL.UW_KN          = [1435, -540, 294];
    HP_RL.LW_KN          = [1433, -590, 111];
    HP_RL.URW_MC         = [1533, -285, 278.5];
    HP_RL.UFW_MC         = [1283, -285, 288.5];
    HP_RL.LRW_MC         = [1540, -270, 123.5];
    HP_RL.LFW_MC         = [1258, -270, 145];
    HP_RL.RKR_1          = [1385, -259, 481];
    HP_RL.RKR_2          = [1385, -240, 447];
    r_rkr = HP_RL.RKR_1 - HP_RL.RKR_2;
    r_a = 50; r_b = 98.75;
    HP_RL.PUSH_UW        = [1435, -520, 305];
    HP_RL.PUSH_RKR       = (HP_RL.RKR_2 + 0.5*r_rkr) + r_a*[1,0,0];
    HP_RL.DPR_RKR        = (HP_RL.RKR_2 + 0.5*r_rkr) + r_b*cross(r_rkr, r_a*[1,0,0])/norm(cross(r_a*[1,0,0], r_rkr));
    HP_RL.DPR_MC         = HP_RL.DPR_RKR + 175*[-1,0,0];
    HP_RL.TR_UPRIGHT     = [1535, -595, 200];
    HP_RL.TR_RACK        = [1535, -275, 203.25];
    HP_RL.D_PINION       = 0;
    HP_RL.AR_AXIS        = [1500, -200, 455];
    r_aph = (HP_RL.DPR_RKR - HP_RL.PUSH_RKR)/norm(HP_RL.DPR_RKR - HP_RL.PUSH_RKR);
    HP_RL.AR_LINK_RKR    = HP_RL.PUSH_RKR + 40*r_aph;
    HP_RL.AR_LINK_RKR    = [HP_RL.RKR_1(1), HP_RL.AR_LINK_RKR(2), HP_RL.AR_LINK_RKR(3)];
    HP_RL.AR_LINK_ARB    = HP_RL.AR_AXIS + [0,0,60];

    f = [1,-1,1];
    HP_FR = mirror_hp(HP_FL, f); HP_FR.D_PINION = HP_FL.D_PINION;
    HP_RR = mirror_hp(HP_RL, f); HP_RR.D_PINION = 0;

    HP.FL = HP_FL;  HP.FR = HP_FR;
    HP.RL = HP_RL;  HP.RR = HP_RR;
end

function HP_out = mirror_hp(HP_in, f)
    fields = fieldnames(HP_in);
    HP_out = HP_in;
    for i = 1:length(fields)
        v = HP_in.(fields{i});
        if isnumeric(v) && numel(v) == 3
            HP_out.(fields{i}) = v .* f;
        end
    end
end
