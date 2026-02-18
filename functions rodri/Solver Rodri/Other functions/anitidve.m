 ANTIDIVE (Mantengo tu bloque de comentarios original)
    
    Wheelbase = 1535;
    H_CG = 260;
    X_CG = 0.45*Wheelbase - 100;
    
    FL_UW_V = [FL.URW_MC(1)-FL.UFW_MC(1), FL.URW_MC(3)-FL.UFW_MC(3)];
    FL_LW_V = [FL.LRW_MC(1)-FL.LFW_MC(1), FL.LRW_MC(3)-FL.LFW_MC(3)];
    
    FL_UW_L = @(t) [FL.UFW_MC(1), FL.UFW_MC(3)] + t*FL_UW_V;
    FL_LW_L = @(s) [FL.LFW_MC(1), FL.LFW_MC(3)] + s*FL_LW_V;
    
    x = fsolve(@(x) FL_UW_L(x(1)) - FL_LW_L(x(2)), [0, 0], optimoptions('fsolve', 'Display', 'off'));
    
    FL_SVIC = FL_UW_L(x(1));
    
    FL_V_REAC = FL_SVIC - [FL.CONTACT_PATCH(1), FL.CONTACT_PATCH(3)];
    
    FL_H_ANTIDIVE = FL.CONTACT_PATCH(3) + (X_CG - FL.CONTACT_PATCH(1)) / FL_V_REAC(1) * FL_V_REAC(2);
    
    FRONT_ANTIDIVE = (FL_V_REAC(2) / FL_V_REAC(1)) * (Wheelbase / H_CG) * 0.5;
    
    fprintf('ANTIDIVE: %.2f %%\n', FRONT_ANTIDIVE * 100);
    