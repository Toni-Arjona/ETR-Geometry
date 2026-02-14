clear all
close all
clc


slip_opt = [-4.2100,   -4.2700,   -4.3700,   -4.5100,   -4.7000,   -4.9300,   -5.2100,   -5.5400,   -5.9200,   -6.3400,   -6.8200,  -7.3700,   -7.9700,   -8.6500 ,  -9.4000];
camber_opt = [-0.5300,   -0.6100,   -0.6900,   -0.7700,   -0.8500,   -0.9300,   -1.0100,   -1.1000,   -1.1900,   -1.2700,   -1.3500,   -1.4400,   -1.5300,   -1.6200,   -1.7100];
Fz_int= 100:100:1500;

slip_int = -12:0.01:0;
car_speed = 30;
long_slip = 0;

mz_matrix = zeros(length(Fz_int), length(slip_int));

for i = 1:length(Fz_int)
    [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(Fz_int(i), long_slip, slip_opt(i), camber_opt(i), car_speed);
    mz_opt(i) = Mz;
           

    for j = 1:length(slip_int)
            [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(Fz_int(i), long_slip, slip_int(j), camber_opt(i), car_speed);
            mz_matrix(i, j) = Mz; 

            
    end            
end

[mz_max_value, mz_max_idx] = min(mz_matrix');

figure(1)
colororder(jet(15))
plot(slip_int, mz_matrix)
hold on
grid on
plot(slip_opt, mz_opt, '.', 'MarkerSize', 10, 'Color', 'y')
plot(slip_int(mz_max_idx), min(mz_matrix'), '.', 'MarkerSize', 10, 'Color', 'w')
xlabel('Slip Angle [deg]');
ylabel('Mz (Self-alignment) [Nm]');
title('Mz vs slip angle');

legend(...
    '100N Fz // -0.53º camber', ...
    '200N Fz // -0.61º camber', ...
    '300N Fz // -0.69º camber', ...
    '400N Fz // -0.77º camber', ...
    '500N Fz // -0.85º camber', ...
    '600N Fz // -0.93º camber', ...
    '700N Fz // -1.01º camber', ...
    '800N Fz // -1.10º camber', ...
    '900N Fz // -1.19º camber', ...
    '1000N Fz // -1.27º camber', ...
    '1100N Fz // -1.35º camber', ...
    '1200N Fz // -1.44º camber', ...
    '1300N Fz // -1.53º camber', ...
    '1400N Fz // -1.62º camber', ...
    '1500N Fz // -1.71º camber', 'Mz at optimum slip', 'Mz max')
figure(2)
plot(slip_int, diff(mz_matrix'))

