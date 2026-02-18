function [F_ROLL_CENTER, R_ROLL_CENTER] = roll_centers_calc(FL, FR, RL, RR)
    F_ROLL_CENTER = solve_rc_axle(FL, FR);
    R_ROLL_CENTER = solve_rc_axle(RL, RR);
end

function RC = solve_rc_axle(L, R)
    % Llamada función cálculo IC
    [L_IC, L_CPATCH_to_IC_vec] = solve_wheel_ic(L);
    [R_IC, R_CPATCH_to_IC_vec] = solve_wheel_ic(R);

    %% Cálculo Roll center. Intersección rectas de contact patch a IC.
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

function [IC, CPATCH_to_IC_vec] = solve_wheel_ic(W)
    % Plano upper wishbone
    UW_PLANE = cross(W.URW_MC - W.UW_KN, W.UFW_MC - W.UW_KN)/norm(cross(W.URW_MC - W.UW_KN, W.UFW_MC - W.UW_KN));
    UW_PLANE(4) = -dot(UW_PLANE, W.UW_KN)/norm(UW_PLANE);
    
    % Plano lower wishbone
    LW_PLANE = cross(W.LRW_MC - W.LW_KN, W.LFW_MC - W.LW_KN)/norm(cross(W.LRW_MC - W.LW_KN, W.LFW_MC - W.LW_KN));
    LW_PLANE(4) = -dot(LW_PLANE, W.LW_KN)/norm(LW_PLANE);
    
    % Ecuaciones intersección planos upper y lower wishbone
    UW_PLANE_EQ = @(x) UW_PLANE(1)*W.CONTACT_PATCH(1) + UW_PLANE(2)*x(1) + UW_PLANE(3)*x(2) + UW_PLANE(4);
    LW_PLANE_EQ = @(x) LW_PLANE(1)*W.CONTACT_PATCH(1) + LW_PLANE(2)*x(1) + LW_PLANE(3)*x(2) + LW_PLANE(4);
    
    % Solve eje intersección planos wishbones
    x_W = fsolve(@(x) [UW_PLANE_EQ(x); LW_PLANE_EQ(x)], [0, 0], optimoptions('fsolve', 'Display', 'off'));
    
    % Resultados
    IC = [W.CONTACT_PATCH(1), x_W(1), x_W(2)]; % coordenadas IC (instant center)
    CPATCH_to_IC_vec = IC - W.CONTACT_PATCH; % vector contact patch a IC.
end