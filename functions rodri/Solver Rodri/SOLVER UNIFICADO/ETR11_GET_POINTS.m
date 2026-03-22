function [FL, FR, RL, RR, F_ROLL_CENTER, R_ROLL_CENTER, FL_KINEMATICS, FR_KINEMATICS, RL_KINEMATICS, RR_KINEMATICS] = ETR11_GET_POINTS(steering_wheel_angle, FL_COMPRESSION, FR_COMPRESSION, RL_COMPRESSION, RR_COMPRESSION, LOADED_RADIUS)
    % [wheel]_[componente 1]_[componente 2] (mah o menoh)
    % HP - Hardpoint
    % F - Front
    % R - Rear/Right
    % U - Upper
    % L - Lower
    % W - Wishbone
    % MC - Monocoque
    % KN - Knuckle
    % RKR - Rocker
    % PUSH - Pushrod
    % DPR - Damper
    % TR - Tie rod
    % KP - Kingpin

    %% CACHÉ DE HARDPOINTS — se computan solo la primera vez, son constantes
    persistent HP_cache
    if isempty(HP_cache)

        %% FRONT HARDPOINTS (LEFT WHEEL)
            % SPINDLE
            HP_FL.SPINDLE_CENTER = [-100, -625, 203];
            HP_FL.SPINDLE_INNER  = [-100, -567, 200.5];

            HP_FL.UW_KN          = [-98, -545, 294];
            HP_FL.LW_KN          = [-106,-585, 106];

            % WISHBONE - MONOCOQUE JOINT
            HP_FL.URW_MC         = [67, -240, 274];
            HP_FL.UFW_MC         = [-207, -240, 279.5];
            HP_FL.LRW_MC         = [94.5, -175, 136];
            HP_FL.LFW_MC         = [-224.5, -175, 122.5];

            % ROCKER AXIS
            HP_FL.RKR_1          = [-48, -179, 587];
            HP_FL.RKR_2          = [-48, -154.25, 552];

            front_rkr_axis = HP_FL.RKR_1 - HP_FL.RKR_2;
            f_a = 50;
            f_b = 85.5;

            % PUSH ROD
            HP_FL.PUSH_UW        = [-98, -516, 315];
            HP_FL.PUSH_RKR       = (HP_FL.RKR_2 + 0.5*front_rkr_axis) + f_a*[-1, 0, 0];

            % DAMPER
            HP_FL.DPR_RKR        = (HP_FL.RKR_2 + 0.5*front_rkr_axis) + f_b*(cross(f_a*[-1, 0, 0], front_rkr_axis))/norm(cross(f_a*[-1, 0, 0], front_rkr_axis));
            HP_FL.DPR_MC         = HP_FL.DPR_RKR + 175*[1, 0, 0];

            % TIE ROD
            HP_FL.TR_UPRIGHT     = [-180, -560, 137];
            HP_FL.TR_RACK        = [-162, -176.5, 150.75];

            % PINION
            HP_FL.D_PINION       = 29;

            % ANTIROLL
            HP_FL.AR_AXIS = [-150, -130, 634];

            f_arb_lever_length   = 60;
            f_ar_rkr_lever_length = 30;

            HP_FL.AR_LINK_RKR = (HP_FL.RKR_2 + 0.5*front_rkr_axis) + f_ar_rkr_lever_length*((HP_FL.DPR_RKR - (HP_FL.RKR_2 + 0.5*front_rkr_axis))/norm(HP_FL.DPR_RKR - (HP_FL.RKR_2 + 0.5*front_rkr_axis)));
            HP_FL.AR_LINK_ARB =  HP_FL.AR_AXIS - [0, 0, f_arb_lever_length];



        %% REAR HARDPOINTS (LEFT WHEEL)
            % SPINDLE
            HP_RL.SPINDLE_CENTER = [1435, -625, 203];
            HP_RL.SPINDLE_INNER  = [1435, -567, 200.5];
           
            HP_RL.UW_KN          = [1435, -540, 294];
            HP_RL.LW_KN          = [1433 ,-590, 111];

            % WISHBONE - MONOCOQUE JOINT
            HP_RL.URW_MC         = [1533, -285, 278.5];
            HP_RL.UFW_MC         = [1283, -285, 288.5];
            HP_RL.LRW_MC         = [1540, -270, 123.5];
            HP_RL.LFW_MC         = [1258, -270, 145];

            % ROCKER AXIS
            HP_RL.RKR_1          = [1385, -259, 481];
            HP_RL.RKR_2          = [1385, -240, 447];

            rear_rkr_axis = HP_RL.RKR_1 - HP_RL.RKR_2;
            r_a = 50;
            r_b = 98.75;

            % PUSH ROD
            HP_RL.PUSH_UW        = [1435, -520, 305];
            HP_RL.PUSH_RKR       = (HP_RL.RKR_2 + 0.5*rear_rkr_axis) + r_a*[1, 0, 0];

            % DAMPER
            HP_RL.DPR_RKR        = (HP_RL.RKR_2 + 0.5*rear_rkr_axis) + r_b*(cross(rear_rkr_axis, r_a*[1, 0, 0]))/norm(cross(r_a*[1, 0, 0], rear_rkr_axis));
            HP_RL.DPR_MC         = HP_RL.DPR_RKR + 175*[-1, 0, 0];

            % TIE ROD
            HP_RL.TR_UPRIGHT     = [1535, -595, 200];
            HP_RL.TR_RACK        = [1535, -275, 203.25];

            % PINION
            HP_RL.D_PINION       = 0;

            % ANTIROLL
            HP_RL.AR_AXIS = [1500, -200, 455];

            r_arb_lever_length   = 60;
            r_ar_rkr_lever_length = 40;

            HP_RL.AR_LINK_RKR = (HP_RL.RKR_2 + 0.5*rear_rkr_axis) + r_ar_rkr_lever_length*((HP_RL.DPR_RKR - (HP_RL.RKR_2 + 0.5*rear_rkr_axis))/norm(HP_RL.DPR_RKR - (HP_RL.RKR_2 + 0.5*rear_rkr_axis)));
            HP_RL.AR_LINK_ARB =  HP_RL.AR_AXIS + [0, 0, r_arb_lever_length];

        % MIRROR
        HP_FR = HP_FL;
        HP_RR = HP_RL;

        names_FL = fieldnames(HP_FL);
        
        for i = 1:length(names_FL)
            HP_FR.(names_FL{i}) = HP_FL.(names_FL{i}) .* [1, -1, 1];
            HP_RR.(names_FL{i}) = HP_RL.(names_FL{i}) .* [1, -1, 1];

        end

        HP_FR.D_PINION       = HP_FL.D_PINION;
        HP_RR.D_PINION       = 0;
        

        HP_cache.FL = HP_FL;
        HP_cache.FR = HP_FR;
        HP_cache.RL = HP_RL;
        HP_cache.RR = HP_RR;
    end

    HP_FL = HP_cache.FL;
    HP_FR = HP_cache.FR;
    HP_RL = HP_cache.RL;
    HP_RR = HP_cache.RR;

    %% LLAMADA SOLVER PARA CADA RUEDA
    FL = ETR11_KINEMATICS_ANALYTICAL(HP_FL, steering_wheel_angle, FL_COMPRESSION, LOADED_RADIUS, 'FL');
    FR = ETR11_KINEMATICS_ANALYTICAL(HP_FR, steering_wheel_angle, FR_COMPRESSION, LOADED_RADIUS, 'FR');
    RL = ETR11_KINEMATICS_ANALYTICAL(HP_RL, 0, RL_COMPRESSION, LOADED_RADIUS, 'RL');
    RR = ETR11_KINEMATICS_ANALYTICAL(HP_RR, 0, RR_COMPRESSION, LOADED_RADIUS, 'RR');

    % LLAMADA FUNCIÓN ROLL CENTERS
    [F_ROLL_CENTER, R_ROLL_CENTER] = roll_centers_calc(FL, FR, RL, RR);

    % LLAMADA FUNCIÓN DATOS
    [FL_KINEMATICS, FR_KINEMATICS, RL_KINEMATICS, RR_KINEMATICS] = solver_kinematics_data(FL, FR, RL, RR);

end


