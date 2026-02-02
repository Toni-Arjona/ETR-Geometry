clc
clear all
close all

long_slip = 0;
car_speed = 13;
camber_interval = -6:0.01:6;
fz = 1500;
slip_angle_interval = -10:0.01:10;
idx_b = 1;
camber = 0;

for k = 1:5
    idx_a = 1;


    fy_vs_slip = zeros(1, length(slip_angle_interval));
    for slip_angle = slip_angle_interval

        [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(fz, long_slip, slip_angle, camber, car_speed);
        fy_vs_slip(idx_b) = Fy;
        idx_b = idx_b + 1;
    end
    
    [fy_slip_max_val, fy_slip_max_idx] = max(fy_vs_slip);
    slip_fy_max(idx_a) = slip_angle_interval(fy_slip_max_idx);

    idx_a = idx_a + 1;
    idx_b = 1; 
    idx_c = 1;
    
    
    for camber = camber_interval
    
       [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(fz, long_slip, slip_fy_max, camber, car_speed);
       fy_vs_camber(idx_c) = Fy;
       idx_c = idx_c + 1;
    
    end
    
    [fy_camber_max_val, fy_camber_max_idx] = max(fy_vs_camber);
    camber = camber_interval(fy_camber_max_idx);
end

plot(camber_interval, fy_vs_camber)
camber
slip_fy_max



