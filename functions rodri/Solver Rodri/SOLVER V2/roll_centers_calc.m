function [F_ROLL_CENTER, R_ROLL_CENTER] = roll_centers_calc(FL, FR, RL, RR)
    F_ROLL_CENTER = solve_rc_axle(FL, FR);
    R_ROLL_CENTER = solve_rc_axle(RL, RR);
end

%% CÁLCULO ROLL CENTER CON ENTRADA DE IC Y CONTACT PATCH
    function RC = solve_rc_axle(L, R)
        % Llamada función cálculo IC
        [~, L_CPATCH_to_IC_vec] = solve_wheel_ic(L);
        [~, R_CPATCH_to_IC_vec] = solve_wheel_ic(R);

        %% Cálculo Roll center. Intersección rectas de contact patch a IC.
            % Intersección de líneas CP→IC en plano YZ (sistema lineal 2×2, no requiere fsolve)
            A_rc = [L_CPATCH_to_IC_vec(2), -R_CPATCH_to_IC_vec(2); ...
                    L_CPATCH_to_IC_vec(3), -R_CPATCH_to_IC_vec(3)];
            b_rc = [R.CONTACT_PATCH(2) - L.CONTACT_PATCH(2); ...
                    R.CONTACT_PATCH(3) - L.CONTACT_PATCH(3)];
            lam_rc = A_rc \ b_rc;
            RC_YZ = [L.CONTACT_PATCH(2) + lam_rc(1)*L_CPATCH_to_IC_vec(2); ...
                     L.CONTACT_PATCH(3) + lam_rc(1)*L_CPATCH_to_IC_vec(3)];

            % Cálculo coordenada x RC (para y = 0, solución directa)
            between_CPATCHS_vec = R.CONTACT_PATCH - L.CONTACT_PATCH;
            lambda = -L.CONTACT_PATCH(2) / between_CPATCHS_vec(2);
            RC_X = L.CONTACT_PATCH(1) + lambda*between_CPATCHS_vec(1);

        % ROLL CENTER
        RC = [RC_X, RC_YZ(1), RC_YZ(2)];
    end


%% CÁLCULO IC POR RUEDA
function [IC, CPATCH_to_IC_vec] = solve_wheel_ic(W)
        % Plano upper wishbone
        UW_PLANE = cross(W.URW_MC - W.UW_KN, W.UFW_MC - W.UW_KN)/norm(cross(W.URW_MC - W.UW_KN, W.UFW_MC - W.UW_KN));
        UW_PLANE(4) = -dot(UW_PLANE, W.UW_KN);

        % Plano lower wishbone
        LW_PLANE = cross(W.LRW_MC - W.LW_KN, W.LFW_MC - W.LW_KN)/norm(cross(W.LRW_MC - W.LW_KN, W.LFW_MC - W.LW_KN));
        LW_PLANE(4) = -dot(LW_PLANE, W.LW_KN);

        % Intersección de planos wishbones en x = CONTACT_PATCH(1) (sistema lineal 2×2)
        A_ic = [UW_PLANE(2), UW_PLANE(3); LW_PLANE(2), LW_PLANE(3)];
        b_ic = [-UW_PLANE(1)*W.CONTACT_PATCH(1) - UW_PLANE(4); ...
                -LW_PLANE(1)*W.CONTACT_PATCH(1) - LW_PLANE(4)];
        x_W = A_ic \ b_ic;

        % Resultados
        IC = [W.CONTACT_PATCH(1), x_W(1), x_W(2)];
        CPATCH_to_IC_vec = IC - W.CONTACT_PATCH;
end


