clear all
close all

fz_total = 1800; %
track = 1250; %
h_cg = 250; 
camber_ext = -1.71; camber_int = -1.71; %
slip_int = -12:0.01:0; %
long_slip = 0; car_speed = 30; mechanical_trail = 0.004; %

for i = 1:length(slip_int)
    % Inicialización para convergencia de carga
    fz_ext = fz_total/2; 
    fz_int = fz_total/2;
    
    for k = 1:3 % Bucle de convergencia Fy -> Fz
        [~, Fy_e, ~, ~, ~, Mz_e] = mfeval_function(fz_ext, long_slip, slip_int(i), camber_ext, car_speed);
        [~, Fy_i, ~, ~, ~, Mz_i] = mfeval_function(fz_int, long_slip, slip_int(i), camber_int, car_speed);
        
        % Transferencia de carga dinámica
        dfz = (abs(Fy_e + Fy_i) * h_cg) / track;
        fz_ext = (fz_total/2) + dfz;
        fz_int = (fz_total/2) - dfz;
    end

    % Guardar resultados (Rueda Exterior)
    fy_ext_result(i) = Fy_e;
    mz_ext_result(i) = Mz_e;
    mech_ext_mz_result(i) = -Fy_e * mechanical_trail;
    total_ext_mz(i) = mech_ext_mz_result(i) + Mz_e;

    % Guardar resultados (Rueda Interior)
    fy_int_result(i) = Fy_i;
    mz_int_result(i) = -Mz_i;
    mech_int_mz_result(i) = -Fy_i * mechanical_trail;
    total_int_mz(i) = mech_int_mz_result(i) - Mz_i;
end

fy_result = fy_int_result + fy_ext_result;
mz_result = mz_int_result + mz_ext_result;
mech_mz_result = mech_int_mz_result + mech_ext_mz_result;
total_mz = total_int_mz + total_ext_mz;

plot(fy_result, mz_result); hold on; grid on;
plot(fy_result, mech_mz_result);
plot(fy_result, total_mz);
legend('Pneumatic trail Mz', 'Mechanical trail Mz', 'Total trail MZ')     
xlim([0, 3500])
xlabel('Total Fy [N]')
ylabel('Total SA torque [Nm]')