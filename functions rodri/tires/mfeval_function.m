function [Fx, Fy, Fz, Mx, My, Mz, kappa, alpha, gamma, phit, Vx, P, Re, rho, two_a, t, mux, muy, omega, Rl, two_b, Mzr, Cx, Cy, Cz, Kya, sigmax, sigmay, dFy_dSA, Kxk] = mfeval_function(Fz, long_slip, slip_angle, camber, car_speed)
    tire_model_data = mfeval.readTIR("C:\Users\rodri\OneDrive\Escritorio\ETECH\GitHub\ETR-Geometry\functions rodri\tires\Hoosier_16_7.5_ETSEIB(4).tir");
    
    pressure = 82737;
    turn_slip = 0;

    inputs = [Fz, long_slip, deg2rad(slip_angle), deg2rad(camber), turn_slip, car_speed, pressure];
    
    outputs = mfeval(tire_model_data, inputs, 211);
    
    % Asignación de las 30 columnas del vector 'outputs' a variables individuales
    
    Fx      = outputs(1);  % longitudinal force
    Fy      = outputs(2);  % lateral force
    Fz      = outputs(3);  % normal force
    Mx      = outputs(4);  % overturning moment
    My      = outputs(5);  % rolling resistance moment
    Mz      = outputs(6);  % self aligning moment
    
    kappa   = outputs(7);  % longitudinal slip
    alpha   = outputs(8);  % side slip angle
    gamma   = outputs(9);  % inclination angle
    phit    = outputs(10); % turn slip
    Vx      = outputs(11); % longitudinal velocity
    P       = outputs(12); % pressure
    
    Re      = outputs(13); % effective rolling radius
    rho     = outputs(14); % tyre deflection
    two_a   = outputs(15); % contact patch length (2a)
    t       = outputs(16); % pneumatic trail
    
    mux     = outputs(17); % longitudinal friction coefficient
    muy     = outputs(18); % lateral friction coefficient
    omega   = outputs(19); % rotational speed
    Rl      = outputs(20); % loaded radius
    two_b   = outputs(21); % contact patch width (2b)
    
    Mzr     = outputs(22); % residual torque
    Cx      = outputs(23); % longitudinal stiffness
    Cy      = outputs(24); % lateral stiffness
    Cz      = outputs(25); % vertical stiffness
    Kya     = outputs(26); % cornering stiffness
    
    sigmax  = outputs(27); % longitudinal relaxation length
    sigmay  = outputs(28); % lateral relaxation length
    dFy_dSA = outputs(29); % Instantaneous cornering stiffness: dFy/dSA
    Kxk     = outputs(30); % slip stiffness

end
