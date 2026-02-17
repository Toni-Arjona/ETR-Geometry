function [F_ROLL_CENTER, R_ROLL_CENTER] = roll_centers_calc(FL, FR, RL, RR)
    % Llamada solver roll center eje delantero
    F_ROLL_CENTER = solve_rc_axle(FL, FR);
    
    % Llamada solver roll center eje trasero
    R_ROLL_CENTER = solve_rc_axle(RL, RR);
end

function RC = solve_rc_axle(L, R)
    %% LEFT IC
        % Plano upper wishbone
        L_UW_PLANE = cross(L.URW_MC - L.UW_KN, L.UFW_MC - L.UW_KN)/norm(cross(L.URW_MC - L.UW_KN, L.UFW_MC - L.UW_KN));
        L_UW_PLANE(4) = -dot(L_UW_PLANE, L.UW_KN)/norm(L_UW_PLANE);
        
        % Plano lower wishbone
        L_LW_PLANE = cross(L.LRW_MC - L.LW_KN, L.LFW_MC - L.LW_KN)/norm(cross(L.LRW_MC - L.LW_KN, L.LFW_MC - L.LW_KN));
        L_LW_PLANE(4) = -dot(L_LW_PLANE, L.LW_KN)/norm(L_LW_PLANE);
        
        % Ecuaciones intersección planos upper y lower wishbone
        L_UW_PLANE_EQ = @(x) L_UW_PLANE(1)*L.CONTACT_PATCH(1) + L_UW_PLANE(2)*x(1) + L_UW_PLANE(3)*x(2) + L_UW_PLANE(4);
        L_LW_PLANE_EQ = @(x) L_LW_PLANE(1)*L.CONTACT_PATCH(1) + L_LW_PLANE(2)*x(1) + L_LW_PLANE(3)*x(2) + L_LW_PLANE(4);
        
        % Solve eje intersección planos wishbones
        x_L = fsolve(@(x) [L_UW_PLANE_EQ(x); L_LW_PLANE_EQ(x)], [0, 0], optimoptions('fsolve', 'Display', 'off'));
        
        % Resultados
        L_IC = [L.CONTACT_PATCH(1), x_L(1), x_L(2)]; % coordenadas IC (instant center)
        L_CPATCH_to_IC_vec = L_IC - L.CONTACT_PATCH; % vector contact patch a IC.
     
    %% RIGHT IC
        % Plano lower wishbone
        R_UW_PLANE = cross(R.URW_MC - R.UW_KN, R.UFW_MC - R.UW_KN)/norm(cross(R.URW_MC - R.UW_KN, R.UFW_MC - R.UW_KN));
        R_UW_PLANE(4) = -dot(R_UW_PLANE, R.UW_KN)/norm(R_UW_PLANE);

        % Plano lower wishbone
        R_LW_PLANE = cross(R.LRW_MC - R.LW_KN, R.LFW_MC - R.LW_KN)/norm(cross(R.LRW_MC - R.LW_KN, R.LFW_MC - R.LW_KN));
        R_LW_PLANE(4) = -dot(R_LW_PLANE, R.LW_KN)/norm(R_LW_PLANE);
        
        % Ecuaciones intersección planos upper y lower wishbone
        R_UW_PLANE_EQ = @(x) R_UW_PLANE(1)*R.CONTACT_PATCH(1) + R_UW_PLANE(2)*x(1) + R_UW_PLANE(3)*x(2) + R_UW_PLANE(4);
        R_LW_PLANE_EQ = @(x) R_LW_PLANE(1)*R.CONTACT_PATCH(1) + R_LW_PLANE(2)*x(1) + R_LW_PLANE(3)*x(2) + R_LW_PLANE(4);

        % Solve eje intersección planos wishbones
        x_R = fsolve(@(x) [R_UW_PLANE_EQ(x); R_LW_PLANE_EQ(x)], [0, 0], optimoptions('fsolve', 'Display', 'off'));
        
        % Resultados
        R_IC = [R.CONTACT_PATCH(1), x_R(1), x_R(2)]; % coordenadas IC (instant center)
        R_CPATCH_to_IC_vec = R_IC - R.CONTACT_PATCH; % vector contact patch a IC.
    
    %% Cálculo Roll center. Intersección rectas de contact patch a IC de cada lado.
        % Proyecciones de las rectas que pasa por el IC y Contact Patch sobre plano YZ
        L_CPATCH_to_IC_line_YZ = @(x) [L.CONTACT_PATCH(2) + x(1)*L_CPATCH_to_IC_vec(2); L.CONTACT_PATCH(3) + x(1)*L_CPATCH_to_IC_vec(3)];
        R_CPATCH_to_IC_line_YZ = @(x) [R.CONTACT_PATCH(2) + x(2)*R_CPATCH_to_IC_vec(2); R.CONTACT_PATCH(3) + x(2)*R_CPATCH_to_IC_vec(3)];
    
        % Cálculo coordenadas [y, z] RC.
        RC_YZ = L_CPATCH_to_IC_line_YZ(fsolve(@(x) L_CPATCH_to_IC_line_YZ(x) - R_CPATCH_to_IC_line_YZ(x), [0; 0], optimoptions('fsolve', 'Display', 'off')));
    
        % Recta que pasa por ambos contact patch
        between_CPATCHS_vec = R.CONTACT_PATCH - L.CONTACT_PATCH;
        between_CPATCHS_line_Y = @(lambda) L.CONTACT_PATCH(2) + lambda*between_CPATCHS_vec(2);

        % Cálculo coordenada x RC (para y = 0, porque es la coordenada y del CG)
        lambda = fzero(@(lambda) between_CPATCHS_line_Y(lambda), 0, optimset('Display', 'off'));
        RC_X = L.CONTACT_PATCH(1) + lambda*between_CPATCHS_vec(1);

        % ROLL CENTER
        RC = [RC_X, RC_YZ(1), RC_YZ(2)];
end