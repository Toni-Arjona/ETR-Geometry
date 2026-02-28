clc
clear all
close all

static_damper_compression = 0; 


%% MOTION RATIO and BUMP STEER
    dpr_compr_int = -25:1:25;

    RL_contact_patch_z = zeros(1, length(dpr_compr_int));
    FL_contact_patch_z = zeros(1, length(dpr_compr_int));

    RL_bump_steer = zeros(1, length(dpr_compr_int));
    FL_bump_steer = zeros(1, length(dpr_compr_int));

    F_ROLL_CENTER_h = zeros(1, length(dpr_compr_int));
    R_ROLL_CENTER_h = zeros(1, length(dpr_compr_int));
    
    for i = 1:length(dpr_compr_int)
        [FL, ~, RL, ~, F_ROLL_CENTER, R_ROLL_CENTER, FL_KINEMATICS, ~, RL_KINEMATICS] = ETR11_GET_POINTS(0, dpr_compr_int(i), dpr_compr_int(i), dpr_compr_int(i), dpr_compr_int(i), 0.199);
     
        FL_contact_patch_z(i) = FL.CONTACT_PATCH(3);    
        RL_contact_patch_z(i) = RL.CONTACT_PATCH(3);

        FL_bump_steer(i) = FL_KINEMATICS.STEER;
        RL_bump_steer(i) = RL_KINEMATICS.STEER;

        F_ROLL_CENTER_h(i) = F_ROLL_CENTER(3);
        R_ROLL_CENTER_h(i) = R_ROLL_CENTER(3);
    
    end
    
    R_motion_ratio = diff(dpr_compr_int) ./ diff(RL_contact_patch_z);
    F_motion_ratio = diff(dpr_compr_int) ./ diff(FL_contact_patch_z);


%% STEER
    steer_int = [0:1:145];

    FL_steer = zeros(1, length(steer_int));
    FR_steer = zeros(1, length(steer_int));
    FL_camber = zeros(1, length(steer_int));
    FR_camber = zeros(1, length(steer_int));
    FL_spindle_height = zeros(1, length(steer_int));
    FR_spindle_height = zeros(1, length(steer_int));
    FL_trail = zeros(1, length(steer_int));
    FR_trail = zeros(1, length(steer_int));
    FL_scrub = zeros(1, length(steer_int));
    FR_scrub = zeros(1, length(steer_int));
    
    for i = 1:length(steer_int)
        [FL, FR, ~, ~, ~, ~, FL_KINEMATICS, FR_KINEMATICS] = ETR11_GET_POINTS(steer_int(i), static_damper_compression, static_damper_compression, static_damper_compression, static_damper_compression, 0.199);
     
        FL_steer(i) = FL_KINEMATICS.STEER;
        FR_steer(i) = FR_KINEMATICS.STEER;
        FL_camber(i) = FL_KINEMATICS.CAMBER;
        FR_camber(i) = FR_KINEMATICS.CAMBER;
        FL_spindle_height(i) =  FL.SPINDLE_CENTER(3);
        FR_spindle_height(i) =  FR.SPINDLE_CENTER(3);
        FL_trail(i) = FL_KINEMATICS.TRAIL;
        FR_trail(i) = FR_KINEMATICS.TRAIL;
        FL_scrub(i) = FL_KINEMATICS.SCRUB;
        FR_scrub(i) = FR_KINEMATICS.SCRUB;
    
    end

    dynamic_toe = FR_steer - FL_steer;
    dynamic_toe_ack = - atand(1535 ./ (1535 ./ tand(FR_steer) + 1250)) + FR_steer;
    ack_pctge = dynamic_toe./dynamic_toe_ack;

    
    FL_jack_rate = - gradient(FL_spindle_height)./gradient(FL_steer);
    FR_jack_rate = - gradient(FR_spindle_height)./gradient(FR_steer);

    steer_ratio_int = gradient(steer_int)./gradient(FR_steer);
    steer_ratio_ext = gradient(steer_int)./gradient(FL_steer);

    %% ACKERMANN Y STEER
    figure(1)
    subplot(2, 1, 1)
    plot(steer_int, FL_steer, 'LineWidth', 1.5, 'DisplayName', 'Front left wheel')
    grid on; hold on
    plot(steer_int, FR_steer, 'LineWidth', 1.5, 'DisplayName', 'Front right wheel')
    legend()
    xlabel('Steering wheel angle [deg]')
    ylabel('Steer angle [deg]')
    title('Steer angle')


    subplot(2, 1, 2)
    plot(steer_int, ack_pctge, 'LineWidth', 1.5)
    grid on
    xlabel('Steering wheel angle [deg]')
    ylabel('Ackermann percentage [deg]')
    title('Ackermann percentage')
    ylim([-2, 2])
    
    % SUSPE (MOTION RATIO, ROLL CENTER, JACK, BUMP STEER)
    figure(2)
    subplot(2,2,1)
    plot(dpr_compr_int(1:end-1), R_motion_ratio, 'LineWidth', 1.5, 'DisplayName', 'Rear MR')
    hold on
    plot(dpr_compr_int(1:end-1), F_motion_ratio, 'LineWidth', 1.5, 'DisplayName', 'Front MR')
    grid on
    legend()
    xlabel('Damper Compression (mm)')
    ylabel('Motion Ratio')
    title('Motion Ratio Curve')

    subplot(2,2,2)
    plot(dpr_compr_int, RL_bump_steer, 'LineWidth', 1.5, 'DisplayName', 'Rear BSteer')
    hold on
    plot(dpr_compr_int, FL_bump_steer, 'LineWidth', 1.5, 'DisplayName', 'Front Bsteer')
    grid on
    legend()
    xlabel('Damper Compression (mm)')
    ylabel('Bump steer [deg]')
    title('Bump steer')

    subplot(2,2,3)
    plot(dpr_compr_int, F_ROLL_CENTER_h - FL_contact_patch_z, 'LineWidth', 1.5, 'DisplayName', 'Front')
    hold on
    plot(dpr_compr_int, R_ROLL_CENTER_h - RL_contact_patch_z, 'LineWidth', 1.5, 'DisplayName', 'Rear')
    grid on
    legend()
    xlabel('Damper Compression (mm)')
    ylabel('Roll center height [deg]')
    title('Roll centers heigths (relative to monocoque)')
    
    subplot(2,2,4)
    plot(steer_int, -(FL_spindle_height - FL_spindle_height(1)), 'LineWidth', 1.5, 'DisplayName', 'Front left wheel')
    hold on
    plot(steer_int, -(FR_spindle_height - FR_spindle_height(1)), 'LineWidth', 1.5, 'DisplayName', 'Front right wheel')
    grid on
    legend()
    ylabel('Jack [m]')
    xlabel('Steer angle [deg]')
    title('Jack')


    % STEER PAREMTERS (TRAIL, SCRUB, CAMBER, JACK RATE)
    figure(3)
    subplot(2, 2, 1)
    plot(steer_int, FL_trail, 'LineWidth', 1.5, 'DisplayName', 'Front left wheel')
    hold on
    plot(steer_int, FR_trail, 'LineWidth', 1.5, 'DisplayName', 'Front right wheel')
    grid on
    legend()
    xlabel('Steering wheel angle [deg]')
    ylabel('Mechanical trail [mm]')
    title('Mchanical trail')

    subplot(2, 2, 2)
    plot(steer_int, FL_jack_rate, 'LineWidth', 1.5, 'DisplayName', 'Front left wheel')
    hold on
    plot(steer_int, FR_jack_rate, 'LineWidth', 1.5, 'DisplayName', 'Front right wheel')
    plot(steer_int, FL_jack_rate + FR_jack_rate, 'LineWidth', 1.5, 'DisplayName', 'Total' )
    grid on
    legend()
    ylabel('Jack rate [mm/deg]')
    xlabel('Steer angle [deg]')
    title('Jack rate')

    subplot(2, 2, 3)
    plot(steer_int, FL_scrub, 'LineWidth', 1.5, 'DisplayName', 'Front left wheel')
    hold on
    plot(steer_int, FR_scrub, 'LineWidth', 1.5, 'DisplayName', 'Front right wheel')
    grid on
    legend()
    xlabel('Steering wheel angle [deg]')
    ylabel('Scrub radius [mm]')
    title('Scrub radius')

    subplot(2, 2, 4)
    plot(steer_int, FL_camber, 'LineWidth', 1.5, 'DisplayName', 'Front left wheel')
    hold on
    plot(steer_int, FR_camber, 'LineWidth', 1.5, 'DisplayName', 'Front right wheel')
    grid on
    legend()
    ylabel('Camber [deg]')
    xlabel('Steer angle [deg]')
    title('Camber')
    
    

% CASO 1
ay = 2*9.81;
v = 20;

% CASO 2
%{
ay = 2*9.81;
v = 20;


% CASO 3
ay = 2.5*9.81;
v = 30;
%}

% SUSPE Y DIMENSIONES
    g = 9.81;
    v = 15;
    wheelbase = 1.535;
    front_track = 1.25;
    rear_track = 1.25;
    
    m = 270; % masa total
    m_sprung = 220; % masa suspendida
    
    cg_balance = 0.55; % porcentaje de wheelbases desde eje delantero
    a = wheelbase*cg_balance;
    b = wheelbase*(1 - cg_balance);
    
    cg_h = 0.26; % altura cg
    
    k_roll_front = 18370.86641; % rigidez dampers delanteros
    k_roll_rear = 18370.86641; % rigidez dampers traseros
    
    h_rollcenter_front = 0.07;
    h_rollcenter_rear = 0.07;
    
    h = cg_h - (h_rollcenter_rear + h_rollcenter_front)/2;


    mu = 1.6;
    

% Aero
    df_coeff = 4;
    drag_coeff = 1.3;
    area = 0.56;
    air_density = 1.225;

    aero_balance = 0.4; % porcentaje wheelbase desde eje delantero
    cop_x = aero_balance*wheelbase;
    cop_z = 0.6; % altura del cop

front_load = m*g*(1 - cg_balance);
rear_load = m*g*cg_balance;

ay = [1.5, 2, 2.5]*9.81;
v = [10, 20, 30];

FL_jacking_torque = zeros(3, length(steer_int));
FR_jacking_torque = zeros(3, length(steer_int));
FL_mechanical_torque = zeros(3, length(steer_int));
FR_mechanical_torque = zeros(3, length(steer_int));
total_left_torque = zeros(3, length(steer_int));
total_right_torque = zeros(3, length(steer_int));
total_jack_torque = zeros(3, length(steer_int));
total_mechanical_torque = zeros(3, length(steer_int));

for i = 1:length(ay)
    F_weight_transfer = ay(i)*m_sprung/front_track * ((h * k_roll_front)/(k_roll_rear + k_roll_front) + (b/wheelbase)*h_rollcenter_front);
    R_weight_transfer = ay(i)*m_sprung/rear_track * ((h * k_roll_rear)/(k_roll_rear + k_roll_front) + (a/wheelbase)*h_rollcenter_rear);
    
    % AERO FORCES
    dForce = 0.5*air_density*area*df_coeff*v(i)^2; % downforce total [N]
    dForce_front = dForce*(1-aero_balance); % downforce eje delantero [N]
    dForce_rear = dForce*aero_balance; % downforce eje trasero [N]
    drag = 0.5*air_density*area*drag_coeff*v(i)^2; % cálculo drag [N]
    drag_front = - drag*cop_z/wheelbase; % carga que aporta el drag del eje delantero [N] (es negativa, por lo que realmente es carga que quita)
    drag_rear = - drag_front; % carga que aporta el drag sobre el eje trasero [N] (es positiva, por lo que aporta carga, exactamente la carga que se quita del eje delantero)
    
    % TOTAL LOAD PER TIRE
    FZ_FL = front_load/2 + F_weight_transfer + (dForce_front + drag_front)/2; % total load on front left [N]
    FZ_FR = front_load/2 - F_weight_transfer + (dForce_front + drag_front)/2;% total load on front right [N]
    FZ_RL = rear_load/2 + R_weight_transfer + (dForce_rear + drag_rear)/2; % total load on rear left [N]
    FZ_RR = rear_load/2 - R_weight_transfer + (dForce_rear + drag_rear)/2; % total load on rear right [N]

    
    FL_jacking_torque(i, :) = (FZ_FL .* (FL_jack_rate * 180/pi) / 1000) ./ steer_ratio_ext; 
    FR_jacking_torque(i, :) = (FZ_FR .* (FR_jack_rate * 180/pi) / 1000) ./ steer_ratio_int; 
    
    FL_mechanical_torque(i, :) = (FZ_FL .* mu .* (FL_trail / 1000)) ./ steer_ratio_ext; 
    FR_mechanical_torque(i, :) = (FZ_FR .* mu .* (FR_trail / 1000)) ./ steer_ratio_int; 
    
    total_left_torque(i, :) = FL_mechanical_torque(i, :) + FL_jacking_torque(i, :);
    total_right_torque(i, :) = FR_mechanical_torque(i, :) + FR_jacking_torque(i, :);
    
    total_jack_torque(i, :) = FL_jacking_torque(i, :) + FR_jacking_torque(i, :);
    total_mechanical_torque(i, :) = FL_mechanical_torque(i, :) + FR_mechanical_torque(i, :);
end


    
   %% TORQUE STEERING
    figure('Name', '1.5g // 10m/s', 'NumberTitle', 'off')
    subplot(2, 2, 1)
    plot(steer_int, steer_ratio_int, 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, steer_ratio_ext, 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    legend()
    ylabel('Steer ratio')
    xlabel('Steer angle [deg]')
    title('Steer ratio')

    subplot(2, 2, 3)
    plot(steer_int, FR_jacking_torque(1, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, FL_jacking_torque(1, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (FL_jacking_torque(1, :) + FR_jacking_torque(1, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-1.5, 2.5])
    legend()
    ylabel('JACKING TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('JACKING TORQUE')

    subplot(2, 2, 4)
    plot(steer_int, FR_mechanical_torque(1, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, FL_mechanical_torque(1, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (FL_mechanical_torque(1, :) + FR_mechanical_torque(1, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-10, 8])
    legend()
    legend()
    ylabel('MECHANICAL TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('MECHANICAL TORQUE')

    subplot(2, 2, 2)
    plot(steer_int, total_right_torque(1, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, total_left_torque(1, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (total_left_torque(1, :) + total_right_torque(1, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-10, 8])
    legend()
    ylabel('TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('TOTAL TORQUE')


    %% TORQUE STEERING
    figure('Name', '2g // 20m/s', 'NumberTitle', 'off')    
    subplot(2, 2, 1)
    plot(steer_int, steer_ratio_int, 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, steer_ratio_ext, 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    legend()
    ylabel('Steer ratio')
    xlabel('Steer angle [deg]')
    title('Steer ratio')

    subplot(2, 2, 3)
    plot(steer_int, FR_jacking_torque(2, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, FL_jacking_torque(2, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (FL_jacking_torque(2, :) + FR_jacking_torque(2, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-1.5, 2.5])
    legend()
    ylabel('JACKING TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('JACKING TORQUE')

    subplot(2, 2, 4)
    plot(steer_int, FR_mechanical_torque(2, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, FL_mechanical_torque(2, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (FL_mechanical_torque(2, :) + FR_mechanical_torque(2, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-10, 8])
    legend()
    legend()
    ylabel('MECHANICAL TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('MECHANICAL TORQUE')

    subplot(2, 2, 2)
    plot(steer_int, total_right_torque(2, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, total_left_torque(2, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (total_left_torque(2, :) + total_right_torque(2, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-10, 8])
    legend()
    ylabel('TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('TOTAL TORQUE')


    %% TORQUE STEERING
    figure('Name', '2.5g // 30m/s', 'NumberTitle', 'off')    
    subplot(2, 2, 1)
    plot(steer_int, steer_ratio_int, 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, steer_ratio_ext, 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    legend()
    ylabel('Steer ratio')
    xlabel('Steer angle [deg]')
    title('Steer ratio')

    subplot(2, 2, 3)
    plot(steer_int, FR_jacking_torque(3, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, FL_jacking_torque(3, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (FL_jacking_torque(3, :) + FR_jacking_torque(3, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-1.5, 2.5])
    legend()
    ylabel('JACKING TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('JACKING TORQUE')

    subplot(2, 2, 4)
    plot(steer_int, FR_mechanical_torque(3, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, FL_mechanical_torque(3, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (FL_mechanical_torque(3, :) + FR_mechanical_torque(3, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-10, 8])
    legend()
    legend()
    ylabel('MECHANICAL TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('MECHANICAL TORQUE')

    subplot(2, 2, 2)
    plot(steer_int, total_right_torque(3, :), 'LineWidth', 1.5,  'DisplayName', 'Front right wheel')
    grid on; hold on
    plot(steer_int, total_left_torque(3, :), 'LineWidth', 1.5 , 'DisplayName', 'Front left wheel')
    plot(steer_int, (total_left_torque(3, :) + total_right_torque(3, :)), 'LineWidth', 1.5, 'DisplayName', 'Total' )
    ylim([-10, 8])
    legend()
    ylabel('TORQUE STEERING WHEEL [NM]')
    xlabel('Steer angle [deg]')
    title('TOTAL TORQUE')