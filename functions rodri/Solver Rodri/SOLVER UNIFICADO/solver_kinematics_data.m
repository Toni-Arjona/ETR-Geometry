function [FL_KINEMATICS, FR_KINEMATICS, RL_KINEMATICS, RR_KINEMATICS] = solver_kinematics_data(FL, FR, RL, RR)
    %% STEER (TOE)
        %% CÁLCULO DE STEER (CORRECCIÓN DE SIGNOS)
        FL_KINEMATICS.STEER =  atan2d(FL.SPINDLE(1),  FL.SPINDLE(2));
        FR_KINEMATICS.STEER =  atan2d(-FR.SPINDLE(1), -FR.SPINDLE(2));
        RL_KINEMATICS.STEER =  atan2d(RL.SPINDLE(1),  RL.SPINDLE(2));
        RR_KINEMATICS.STEER =  atan2d(-RR.SPINDLE(1), -RR.SPINDLE(2));
        
    %% CAMBER
        FL_KINEMATICS.CAMBER = atan2d(FL.SPINDLE(3),  FL.SPINDLE(2));
        FR_KINEMATICS.CAMBER = atan2d(FR.SPINDLE(3), -FR.SPINDLE(2));
        RL_KINEMATICS.CAMBER = atan2d(RL.SPINDLE(3),  RL.SPINDLE(2));
        RR_KINEMATICS.CAMBER = atan2d(RR.SPINDLE(3), -RR.SPINDLE(2));
 
    %% KINGPIN - KPI y Caster
        FL_KINEMATICS.KPI = atan2d(-FL.KP(2), -FL.KP(3));
        FR_KINEMATICS.KPI = atan2d( FR.KP(2), -FR.KP(3));
        RL_KINEMATICS.KPI = atan2d(-RL.KP(2), -RL.KP(3));
        RR_KINEMATICS.KPI = atan2d( RR.KP(2), -RR.KP(3));
        
    %% CASTER (Plano XZ). 
        FL_KINEMATICS.CASTER = atan2d(-FL.KP(1), -FL.KP(3));
        FR_KINEMATICS.CASTER = atan2d(-FR.KP(1), -FR.KP(3));
        RL_KINEMATICS.CASTER = atan2d(-RL.KP(1), -RL.KP(3));
        RR_KINEMATICS.CASTER = atan2d(-RR.KP(1), -RR.KP(3));

    %% SCRUB RADIUS Y MECHANICAL TRAIL
        [FL_KINEMATICS.TRAIL, FL_KINEMATICS.SCRUB] = calc_trail_scrub(FL.SPINDLE, FL.CONTACT_PATCH, FL.KP_FLOOR);
        [FR_KINEMATICS.TRAIL, FR_KINEMATICS.SCRUB] = calc_trail_scrub(FR.SPINDLE, FR.CONTACT_PATCH, FR.KP_FLOOR);
        [RL_KINEMATICS.TRAIL, RL_KINEMATICS.SCRUB] = calc_trail_scrub(RL.SPINDLE, RL.CONTACT_PATCH, RL.KP_FLOOR);
        [RR_KINEMATICS.TRAIL, RR_KINEMATICS.SCRUB] = calc_trail_scrub(RR.SPINDLE, RR.CONTACT_PATCH, RR.KP_FLOOR);
        
      function [TRAIL, SCRUB] = calc_trail_scrub(SPINDLE, CONTACT_PATCH, KP_FLOOR)
        v_kp = KP_FLOOR - CONTACT_PATCH;
    
        inboard_dir = SPINDLE / norm(SPINDLE);
        fwd_dir = -cross(inboard_dir, [0,0,1]) * sign(SPINDLE(2));
    
        TRAIL = dot(v_kp, fwd_dir);
        SCRUB = dot(v_kp, inboard_dir);
      end
    
    %% JACK RATE
        FL_KINEMATICS.JACKING_RATE = deg2rad(dot(cross(FL.KP, (FL.CONTACT_PATCH - FL.LW_KN)), [0,0,1]));
        FR_KINEMATICS.JACKING_RATE = deg2rad(dot(cross(FR.KP, (FR.CONTACT_PATCH - FR.LW_KN)), [0,0,1]));
        RL_KINEMATICS.JACKING_RATE = deg2rad(dot(cross(RL.KP, (RL.CONTACT_PATCH - RL.LW_KN)), [0,0,1]));
        RR_KINEMATICS.JACKING_RATE = deg2rad(dot(cross(RR.KP, (RR.CONTACT_PATCH - RR.LW_KN)), [0,0,1]));

        

end


