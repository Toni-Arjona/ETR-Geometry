close all

RL_dpr_compr_int = 0:0.1:55;
FL_dpr_compr_int = 0:0.1:55;
RL_contact_patch_z = zeros(1, length(RL_dpr_compr_int));
FL_contact_patch_z = zeros(1, length(FL_dpr_compr_int));

for i = 1:length(RL_dpr_compr_int)

    %[RL, RR] = REAR_KINEMATICS_SOLVER_ETR11_rodri(RL_dpr_compr_int(i), 0);
    [FL, FR] = FRONT_KINEMATICS_SOLVER_ETR11_rodri(0, FL_dpr_compr_int(i), 0);

    %RL_contact_patch_z(i) = RL.CONTACT_PATCH(3);
    FL_contact_patch_z(i) = FL.CONTACT_PATCH(3);

end

%R_motion_ratio = diff(RL_dpr_compr_int) ./ diff(RL_contact_patch_z);
F_motion_ratio = diff(FL_dpr_compr_int) ./ diff(FL_contact_patch_z);

figure(1)
% Se usa 1:end-1 en el eje X para igualar el tamaño reducido por diff()
%plot(RL_dpr_compr_int(1:end-1), R_motion_ratio, 'LineWidth', 1.5, 'DisplayName', 'Rear MR')
%   hold on
plot(FL_dpr_compr_int(1:end-1), F_motion_ratio, 'LineWidth', 1.5, 'DisplayName', 'Front MR')
grid on
legend()
xlabel('Damper Compression (mm)')
ylabel('Motion Ratio (\Delta Damper / \Delta Wheel Z)')
title('Motion Ratio Curve')
