function Out_Dynamics = ETR11_DYNAMICS_SOLVER(HP, CONTACT_PATCH_FORCE)

    % KNOWN FORCE MOMENT (CONTACT PATCH FORCE)
    contact_patch_moment = cross(HP.CONTACT_PATCH, CONTACT_PATCH_FORCE);

    % FORCES APLICATION POINTS
    push_point = HP.PUSH_RKR;
    tie_point = HP.TR_RACK;
    ufw_point = HP.UFW_MC;
    urw_point = HP.URW_MC;
    lfw_point = HP.LFW_MC;
    lrw_point = HP.LRW_MC;

    % FORCES DIRECTIONS
    push_direction = (HP.PUSH_UW - HP.PUSH_RKR)/norm(HP.PUSH_UW - HP.PUSH_RKR);
    tie_direction = (HP.TR_UPRIGHT - HP.TR_RACK)/norm(HP.TR_UPRIGHT - HP.TR_RACK);
    ufw_direction = (HP.UW_KN - HP.UFW_MC)/norm(HP.UW_KN - HP.UFW_MC);
    urw_direction = (HP.UW_KN - HP.URW_MC)/norm(HP.UW_KN - HP.URW_MC);
    lfw_direction = (HP.LW_KN - HP.LFW_MC)/norm(HP.LW_KN - HP.LFW_MC);
    lrw_direction = (HP.LW_KN - HP.LRW_MC)/norm(HP.LW_KN - HP.LRW_MC);

    % MOMENTS
    push_moment = cross(push_point, push_direction);
    tie_moment = cross(tie_point, tie_direction);
    ufw_moment = cross(ufw_point, ufw_direction);
    urw_moment = cross(urw_point, urw_direction);
    lfw_moment = cross(lfw_point, lfw_direction);
    lrw_moment = cross(lrw_point, lrw_direction);
    
    % TOTAL MATRIXS
    A = [tie_direction, tie_moment; 
        push_direction, push_moment;
        ufw_direction, ufw_moment;
        urw_direction, urw_moment;
        lfw_direction, lfw_moment;
        lrw_direction, lrw_moment]';

    B = [-CONTACT_PATCH_FORCE, -contact_patch_moment]';
    
    % RESOLVER SISTEMA
    F_incognitas = A \ B;

    Out_Dynamics.TieRod = F_incognitas(1);
    Out_Dynamics.PushRod = F_incognitas(2);
    Out_Dynamics.UFW = F_incognitas(3);
    Out_Dynamics.URW = F_incognitas(4);
    Out_Dynamics.LFW = F_incognitas(5);
    Out_Dynamics.LRW = F_incognitas(6);

end



    

