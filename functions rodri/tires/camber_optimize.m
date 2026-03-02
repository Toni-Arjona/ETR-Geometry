clc
clear all
close all
long_slip = 0;
car_speed = 13;
camber_interval = -3:0.01:3;
slip_angle_interval = -10:0.01:10;

% Definimos fz_vec para que funcione tu plot final
fz_vec = 100:100:1500; 
idx_fz = 1; 
camber = 0;

for fz = fz_vec
    for k = 1:5
        idx_b = 1;
        fy_vs_slip = zeros(1, length(slip_angle_interval));
        for slip_angle = slip_angle_interval
            [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(fz, long_slip, slip_angle, camber, car_speed);
            fy_vs_slip(idx_b) = Fy;
            idx_b = idx_b + 1;
        end
        
        [fy_slip_max_val, fy_slip_max_idx] = max(fy_vs_slip);
        slip_fy_max = slip_angle_interval(fy_slip_max_idx);
        
        idx_c = 1;
        
        % Uso cam_iter para no romper el bucle
        for cam_iter = camber_interval
           [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(fz, long_slip, slip_fy_max, cam_iter, car_speed);
           fy_vs_camber(idx_c) = Fy;
           idx_c = idx_c + 1;
        end
        
        [fy_camber_max_val, fy_camber_max_idx] = max(fy_vs_camber);
        camber = camber_interval(fy_camber_max_idx);
    end
    
    % Asignación para que cuadre con tu bloque
    current_camber = camber;
    current_slip = slip_fy_max;
    
    % --- TU BLOQUE DE PRESENTACIÓN ---
    res_camber_opt(idx_fz) = current_camber;
    res_slip_opt(idx_fz)   = current_slip;
    
    fprintf('Fz: %d N -> Camber Opt: %.2f deg | Slip Opt: %.2f deg\n', fz, current_camber, current_slip);
    
    idx_fz = idx_fz + 1; 
end

% --- TU PLOT FINAL ---
figure(8)
plot(fz_vec, res_camber_opt, '-o', 'LineWidth', 1.5)
grid on
xlabel('Carga Vertical Fz [N]')
ylabel('Camber Óptimo [deg]')
title('Sensibilidad del Camber Óptimo a la Carga')


for k = 1:length(res_camber_opt)
    [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(fz_vec(k), 0, res_slip_opt(k), res_camber_opt(k), car_speed);
    res_re(k) = Re;
    res_pneumatic_trail(k) = t;
    res_mz(k) = Mz;
    res_mx(k) = Mx;
    res_scrub_offset(k) = res_mx(k)/fz_vec(k);

end

table(fz_vec', res_slip_opt' , res_camber_opt', res_re', res_pneumatic_trail', res_mz', res_scrub_offset', 'VariableNames', {'Fz [N]', 'Slip angle óptimo [deg]', 'Camber óptimo [deg]', 'Effective rolling radius [m]', 'Pneumatic trail [m]', 'Mz [Nm]', 'CoP y dist [m]'})

      
