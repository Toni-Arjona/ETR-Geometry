function [FL, FR, RL, RR, F_ROLL_CENTER, R_ROLL_CENTER] = ETR11_GET_POINTS(steering_wheel_angle, FL_COMPRESSION, FR_COMPRESSION, RL_COMPRESSION, RR_COMPRESSION)
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

    %% FRONT HARDPOINTS (LEFT WHEEL)
        % SPINDLE
        HP_FL.SPINDLE_CENTER = [-100, -625, 203]; 
        HP_FL.SPINDLE_INNER  = [-100, -561.5, 200.5];

        % KUNCKLES
        HP_FL.UW_KN          = [-83, -510, 294]; 
        HP_FL.LW_KN          = [-96.5 ,-571, 111];

        % WISHBONE - MONOCOQUE JOINT
        HP_FL.URW_MC         = [75, -225, 267];
        HP_FL.UFW_MC         = [-210, -225, 281];
        HP_FL.LRW_MC         = [80, -225, 135];
        HP_FL.LFW_MC         = [-210, -225, 123];
        
        % PUSH ROD
        HP_FL.PUSH_UW        = [-83, -489, 319];
        HP_FL.PUSH_RKR       = [-83, -193.5, 558];
        
        % ROCKER AXIS
        HP_FL.RKR_1          = [-18, -186.5, 587];
        HP_FL.RKR_2          = [-18, -164, 552];

        % DAMPER
        HP_FL.DPR_RKR        = [-43, -112, 610];
        HP_FL.DPR_MC         = [167, -112, 610];
        
        % TIE ROD
        HP_FL.TR_UPRIGHT     = [-180, -560, 140];
        HP_FL.TR_RACK        = [-170, -225, 149.4];
        
        % PINION
        HP_FL.D_PINION       = 35;

    %% REAR HARDPOINTS (LEFT WHEEL)
        % Wheel spindle 
        HP_RL.SPINDLE_CENTER = [1435, -625, 203]; 
        HP_RL.SPINDLE_INNER  = [1435, -561.5, 200.5];
               
        % RL Knuckles
        HP_RL.UW_KN          = [1435, -551.5, 294]; 
        HP_RL.LW_KN          = [1435 ,-576, 111];
        
        % RL wishbones joints with monocoque
        HP_RL.URW_MC         = [1545, -255, 281];
        HP_RL.UFW_MC         = [1265, -255, 294];
        HP_RL.LRW_MC         = [1545, -255, 124];
        HP_RL.LFW_MC         = [1250, -255, 151];
        
        % RL Push rod
        HP_RL.PUSH_UW        = [1435, -517, 306.5];
        HP_RL.PUSH_RKR       = [1435, -206, 487];
        
        % RL Rocker axis points
        HP_RL.RKR_1          = [1373, -228, 497];
        HP_RL.RKR_2          = [1373, -205, 463];
        
        % RL Damper
        HP_RL.DPR_RKR        = [1359, -161, 518];
        HP_RL.DPR_MC         = [1149, -161, 518];
        
        % RL Tie rod
        HP_RL.TR_UPRIGHT     = [1517, -560, 140];
        HP_RL.TR_RACK        = [1528, -255, 155];
        
        % Eje trasero fijo 
        HP_RL.D_PINION       = 0;

    % FRONT HARDPOINTS (RIGHT WHEEL). LEFT INVERTIDOS, SOLO TOCAR LOS DE LA RUEDA IZQUIERDA, NO TOCAR ESTO
        HP_FR.SPINDLE_CENTER = HP_FL.SPINDLE_CENTER .* [1, -1, 1]; 
        HP_FR.SPINDLE_INNER  = HP_FL.SPINDLE_INNER  .* [1, -1, 1];
        HP_FR.UW_KN          = HP_FL.UW_KN          .* [1, -1, 1]; 
        HP_FR.LW_KN          = HP_FL.LW_KN          .* [1, -1, 1];
        HP_FR.URW_MC         = HP_FL.URW_MC         .* [1, -1, 1];
        HP_FR.UFW_MC         = HP_FL.UFW_MC         .* [1, -1, 1];
        HP_FR.LRW_MC         = HP_FL.LRW_MC         .* [1, -1, 1];
        HP_FR.LFW_MC         = HP_FL.LFW_MC         .* [1, -1, 1];
        HP_FR.PUSH_UW        = HP_FL.PUSH_UW        .* [1, -1, 1];
        HP_FR.PUSH_RKR       = HP_FL.PUSH_RKR       .* [1, -1, 1];
        HP_FR.RKR_1          = HP_FL.RKR_1          .* [1, -1, 1];
        HP_FR.RKR_2          = HP_FL.RKR_2          .* [1, -1, 1];
        HP_FR.DPR_RKR        = HP_FL.DPR_RKR        .* [1, -1, 1];
        HP_FR.DPR_MC         = HP_FL.DPR_MC         .* [1, -1, 1]; 
        HP_FR.TR_UPRIGHT     = HP_FL.TR_UPRIGHT     .* [1, -1, 1];
        HP_FR.TR_RACK        = HP_FL.TR_RACK        .* [1, -1, 1];
        HP_FR.D_PINION       = HP_FL.D_PINION;

    % REAR HARDPOINTS (RIGHT WHEEL). LEFT INVERTIDOS, SOLO TOCAR LOS DE LA RUEDA IZQUIERDA, NO TOCAR ESTO
        HP_RR.SPINDLE_CENTER = HP_RL.SPINDLE_CENTER .* [1, -1, 1];
        HP_RR.SPINDLE_INNER  = HP_RL.SPINDLE_INNER .* [1, -1, 1];
        HP_RR.UW_KN          = HP_RL.UW_KN .* [1, -1, 1];
        HP_RR.LW_KN          = HP_RL.LW_KN .* [1, -1, 1];
        HP_RR.URW_MC         = HP_RL.URW_MC .* [1, -1, 1];
        HP_RR.UFW_MC         = HP_RL.UFW_MC .* [1, -1, 1];
        HP_RR.LRW_MC         = HP_RL.LRW_MC .* [1, -1, 1];
        HP_RR.LFW_MC         = HP_RL.LFW_MC .* [1, -1, 1];
        HP_RR.PUSH_UW        = HP_RL.PUSH_UW .* [1, -1, 1];
        HP_RR.PUSH_RKR       = HP_RL.PUSH_RKR .* [1, -1, 1];
        HP_RR.RKR_1          = HP_RL.RKR_1 .* [1, -1, 1];
        HP_RR.RKR_2          = HP_RL.RKR_2 .* [1, -1, 1];
        HP_RR.DPR_RKR        = HP_RL.DPR_RKR .* [1, -1, 1];
        HP_RR.DPR_MC         = HP_RL.DPR_MC .* [1, -1, 1];
        HP_RR.TR_UPRIGHT     = HP_RL.TR_UPRIGHT .* [1, -1, 1];
        HP_RR.TR_RACK        = HP_RL.TR_RACK .* [1, -1, 1];
        HP_RR.D_PINION       = 0;


    %% LLAMADA SOLVER PARA CADA RUEDA
    FL = ETR11_KINEMATICS_SOLVER(HP_FL, steering_wheel_angle, FL_COMPRESSION);
    FR = ETR11_KINEMATICS_SOLVER(HP_FR, steering_wheel_angle, FR_COMPRESSION);
    RL = ETR11_KINEMATICS_SOLVER(HP_RL, 0, RL_COMPRESSION);
    RR = ETR11_KINEMATICS_SOLVER(HP_RR, 0, RR_COMPRESSION);

    % LLAMADA FUNCIÓN ROLL CENTERS
    [F_ROLL_CENTER, R_ROLL_CENTER] = roll_centers_calc(FL, FR, RL, RR);

end

