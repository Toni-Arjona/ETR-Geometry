function [F_motion_ratio, R_motion_ratio, F_wheel_travel, R_wheel_travel] = motion_ratio(dpr_compr_int, LOADED_RADIUS)

    RL_contact_patch_z = zeros(1, length(dpr_compr_int));
    FL_contact_patch_z = zeros(1, length(dpr_compr_int));
    
    for i = 1:length(dpr_compr_int)
        [FL, ~, RL] = ETR11_GET_POINTS(0, dpr_compr_int(i), dpr_compr_int(i), dpr_compr_int(i), dpr_compr_int(i), LOADED_RADIUS);
     
        FL_contact_patch_z(i) = FL.CONTACT_PATCH(3) 
        RL_contact_patch_z(i) = RL.CONTACT_PATCH(3)
        
      
    end

    FL_contact_patch_z = FL_contact_patch_z - FL_contact_patch_z(1);    
    RL_contact_patch_z = RL_contact_patch_z - RL_contact_patch_z(1);
    
    R_motion_ratio = gradient(dpr_compr_int) ./ gradient(RL_contact_patch_z);
    F_motion_ratio = gradient(dpr_compr_int) ./ gradient(FL_contact_patch_z);
    F_wheel_travel = FL_contact_patch_z;
    R_wheel_travel = RL_contact_patch_z;

end