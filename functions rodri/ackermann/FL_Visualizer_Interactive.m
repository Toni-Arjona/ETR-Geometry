function FL_Viewer_V2()
    % --- CONFIGURACIÓN DE LA FIGURA ---
    fig = uifigure('Name', 'ETR11 Live Geometry - Full Rocker', 'Position', [100 100 1100 750], 'Color', [0.1 0.1 0.1]);
    ax = uiaxes(fig, 'Position', [250 50 800 650], 'BackgroundColor', [0 0 0], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
    ax.XLim = [-500 400]; ax.YLim = [-800 -100]; ax.ZLim = [-50 750];

    % --- PUNTOS FIJOS DEL CHASIS (Wishbones) ---
    MC_F.URW = [80, -225, 268]; MC_F.UFW = [-210, -225, 277];
    MC_F.LRW = [85, -225, 136]; MC_F.LFW = [-210, -225, 123];

    % --- OBJETOS GRÁFICOS ---
    h.UW = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    h.LW = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    h.PUSH = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    h.DPR = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    
    % Rocker (Tetraedro completo en blanco)
    h.RKR = plot3(ax, 0,0,0, 'w', 'LineWidth', 1.5);
    
    % Amarillos: Tie Rod y Spindle
    h.TR = plot3(ax, 0,0,0, 'y', 'LineWidth', 2);
    h.Spindle = plot3(ax, 0,0,0, 'y', 'LineWidth', 3);
    h.Conn = plot3(ax, 0,0,0, 'y', 'LineWidth', 1.5);
    
    % Puntos (Nodos)
    h.PtsW = plot3(ax, 0,0,0, 'wo', 'MarkerFaceColor', 'w', 'MarkerSize', 5); 
    h.PtsY = plot3(ax, 0,0,0, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 5); 

    % Neumático
    [xc, yc, zc] = cylinder((16*25.4)/2, 40); zc = (zc - 0.5) * (7.5*25.4);
    h.Wheel = surface(ax, xc, yc, zc, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % --- SLIDERS ---
    s_steer = uislider(fig, 'Position', [50 530 150 3], 'Limits', [-140 140], 'Value', 0);
    s_comp = uislider(fig, 'Position', [50 430 150 3], 'Limits', [0 50], 'Value', 0);
    s_steer.ValueChangingFcn = @(src, event) updatePlot(event.Value, s_comp.Value);
    s_comp.ValueChangingFcn = @(src, event) updatePlot(s_steer.Value, event.Value);

    function updatePlot(steer_val, comp_val)
        FL = FL_SOLVER_ETR11_rodri(steer_val, comp_val);
        
        % Sincronizar anclajes fijos con el z_offset dinámico
        z_off = 150.68 - FL.TR_RACK(3); 
        off = [0, 0, z_off];
        cURW = MC_F.URW-off; cUFW = MC_F.UFW-off; cLRW = MC_F.LRW-off; cLFW = MC_F.LFW-off;

        % Wishbones, Pushrod y Damper
        set(h.UW, 'XData', [cUFW(1) FL.UW_KN(1) cURW(1)], 'YData', [cUFW(2) FL.UW_KN(2) cURW(2)], 'ZData', [cUFW(3) FL.UW_KN(3) cURW(3)]);
        set(h.LW, 'XData', [cLFW(1) FL.LW_KN(1) cLRW(1)], 'YData', [cLFW(2) FL.LW_KN(2) cLRW(2)], 'ZData', [cLFW(3) FL.LW_KN(3) cLRW(3)]);
        set(h.PUSH, 'XData', [FL.PUSH_UW(1) FL.PUSH_RKR(1)], 'YData', [FL.PUSH_UW(2) FL.PUSH_RKR(2)], 'ZData', [FL.PUSH_UW(3) FL.PUSH_RKR(3)]);
        set(h.DPR, 'XData', [FL.DPR_RKR(1) FL.DPR_MC(1)], 'YData', [FL.DPR_RKR(2) FL.DPR_MC(2)], 'ZData', [FL.DPR_RKR(3) FL.DPR_MC(3)]);

        % Tetraedro del Rocker (6 aristas)
        % Orden: P1 -> P2 -> P3 -> P1 -> P4 -> P2 -> P4 -> P3
        P1 = FL.RKR_AXIS_1; P2 = FL.RKR_AXIS_2; P3 = FL.DPR_RKR; P4 = FL.PUSH_RKR;
        xR = [P1(1) P2(1) P3(1) P1(1) P4(1) P2(1) P4(1) P3(1)];
        yR = [P1(2) P2(2) P3(2) P1(2) P4(2) P2(2) P4(2) P3(2)];
        zR = [P1(3) P2(3) P3(3) P1(3) P4(3) P2(3) P4(3) P3(3)];
        set(h.RKR, 'XData', xR, 'YData', yR, 'ZData', zR);

        % Amarillos
        set(h.TR, 'XData', [FL.TR_RACK(1) FL.TR_UPRIGHT(1)], 'YData', [FL.TR_RACK(2) FL.TR_UPRIGHT(2)], 'ZData', [FL.TR_RACK(3) FL.TR_UPRIGHT(3)]);
        set(h.Spindle, 'XData', [FL.SPINDLE_CENTER(1) FL.SPINDLE_INNER(1)], 'YData', [FL.SPINDLE_CENTER(2) FL.SPINDLE_INNER(2)], 'ZData', [FL.SPINDLE_CENTER(3) FL.SPINDLE_INNER(3)]);
        set(h.Conn, 'XData', [FL.SPINDLE_INNER(1) FL.TR_UPRIGHT(1)], 'YData', [FL.SPINDLE_INNER(2) FL.TR_UPRIGHT(2)], 'ZData', [FL.SPINDLE_INNER(3) FL.TR_UPRIGHT(3)]);

        % Puntos
        ptsW = [cUFW; cURW; cLFW; cLRW; FL.UW_KN; FL.LW_KN; FL.PUSH_UW; FL.PUSH_RKR; FL.DPR_RKR; FL.DPR_MC; FL.RKR_AXIS_1; FL.RKR_AXIS_2];
        set(h.PtsW, 'XData', ptsW(:,1), 'YData', ptsW(:,2), 'ZData', ptsW(:,3));
        ptsY = [FL.TR_RACK; FL.TR_UPRIGHT; FL.SPINDLE_CENTER; FL.SPINDLE_INNER];
        set(h.PtsY, 'XData', ptsY(:,1), 'YData', ptsY(:,2), 'ZData', ptsY(:,3));

        % Neumático
        s_vec = (FL.SPINDLE_INNER - FL.SPINDLE_CENTER); s_vec = s_vec / norm(s_vec);
        k = [0 0 1]; v = cross(k, s_vec); c = dot(k, s_vec);
        vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        Rmat = eye(3) + vx + vx^2 * (1/(1+c));
        pts = Rmat * [xc(:)'; yc(:)'; zc(:)'];
        set(h.Wheel, 'XData', reshape(pts(1,:), size(xc)) + FL.SPINDLE_CENTER(1), ...
                     'YData', reshape(pts(2,:), size(xc)) + FL.SPINDLE_CENTER(2), ...
                     'ZData', reshape(pts(3,:), size(xc)) + FL.SPINDLE_CENTER(3));
    end
    updatePlot(0, 0);
end