function `[Forces] = ETR11_FORCES_SOLVER (points, Fz, slip_angle, slip_ratio, camber, speed)

    [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(Fz, long_slip, slip_angle, camber, car_speed)

    %% VECTORES UNITARIOS FUERZAS
    URW_force_vector = (points.UW_KN - points.URW_MC)/norm(points.UW_KN - points.URW_MC);
    UFW_force_vector = (points.UW_KN - points.UFW_MC)/norm(points.UW_KN - points.UFW_MC);
    LRW_force_vector = (points.LW_KN - points.LRW_MC)/norm(points.LW_KN - points.LRW_MC);
    UFW_force_vector = (points.LW_KN - points.LFW_MC)/norm(points.LW_KN - points.LFW_MC);

    PUSH_force_vector = (points.PUSH_UW - points.PUSH_RKR)/norm(points.PUSH_UW - points.PUSH_RKR);

    TR_force_vector = (points.TR_UPRIGHT - points.TR_RACK)/norm(points.TR_UPRIGHT - points.TR_RACK);

    %% PUNTOS DE APLIACIÓN FUERZAS
    


    
  