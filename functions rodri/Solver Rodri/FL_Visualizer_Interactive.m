function FL_Viewer_V5()
    fig = uifigure('Name', 'ETR11 Live Geometry - Pure Plot', 'Position', [100 100 1100 750], 'Color', [0.1 0.1 0.1]);
    ax = uiaxes(fig, 'Position', [50 190 1000 530], 'BackgroundColor', [0 0 0], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
    ax.XLim = [-500 400]; ax.YLim = [-800 -100]; ax.ZLim = [0 750];

    % Objetos Gráficos
    h.UW = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    h.LW = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    h.PUSH = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    h.DPR = plot3(ax, 0,0,0, 'w', 'LineWidth', 2);
    h.RKR = plot3(ax, 0,0,0, 'w', 'LineWidth', 1.5);
    
    h.TR = plot3(ax, 0,0,0, 'y', 'LineWidth', 2);
    h.Spindle = plot3(ax, 0,0,0, 'y', 'LineWidth', 3);
    h.Conn = plot3(ax, 0,0,0, 'y', 'LineWidth', 1.5);
    
    h.PtsW = plot3(ax, 0,0,0, 'wo', 'MarkerFaceColor', 'w', 'MarkerSize', 5); 
    h.PtsY = plot3(ax, 0,0,0, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 5); 

    [xc, yc, zc] = cylinder((16*25.4)/2, 40); zc = (zc - 0.5) * (7.5*25.4);
    h.Wheel = surface(ax, xc, yc, zc, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % Sliders
    uilabel(fig, 'Position', [150 150 800 20], 'Text', 'ÁNGULO DE VOLANTE (Steering Angle)', 'FontWeight', 'bold', 'FontColor', 'w', 'HorizontalAlignment', 'center');
    s_steer = uislider(fig, 'Position', [150 130 800 3], 'Limits', [-150 150], 'Value', 0);
    
    uilabel(fig, 'Position', [150 70 800 20], 'Text', 'COMPRESIÓN DEL AMORTIGUADOR (Damper Compression)', 'FontWeight', 'bold', 'FontColor', 'w', 'HorizontalAlignment', 'center');
    s_comp = uislider(fig, 'Position', [150 50 800 3], 'Limits', [0 50], 'Value', 0);

    s_steer.ValueChangingFcn = @(src, event) updatePlot(event.Value, s_comp.Value);
    s_comp.ValueChangingFcn = @(src, event) updatePlot(s_steer.Value, event.Value);

    function updatePlot(steer_val, comp_val)
        FL = FL_SOLVER_ETR11_rodri(steer_val, comp_val);
        
        % Asignación directa pura
        set(h.UW, 'XData', [FL.UFW_MC(1) FL.UW_KN(1) FL.URW_MC(1)], 'YData', [FL.UFW_MC(2) FL.UW_KN(2) FL.URW_MC(2)], 'ZData', [FL.UFW_MC(3) FL.UW_KN(3) FL.URW_MC(3)]);
        set(h.LW, 'XData', [FL.LFW_MC(1) FL.LW_KN(1) FL.LRW_MC(1)], 'YData', [FL.LFW_MC(2) FL.LW_KN(2) FL.LRW_MC(2)], 'ZData', [FL.LFW_MC(3) FL.LW_KN(3) FL.LRW_MC(3)]);
        set(h.PUSH, 'XData', [FL.PUSH_UW(1) FL.PUSH_RKR(1)], 'YData', [FL.PUSH_UW(2) FL.PUSH_RKR(2)], 'ZData', [FL.PUSH_UW(3) FL.PUSH_RKR(3)]);
        set(h.DPR, 'XData', [FL.DPR_RKR(1) FL.DPR_MC(1)], 'YData', [FL.DPR_RKR(2) FL.DPR_MC(2)], 'ZData', [FL.DPR_RKR(3) FL.DPR_MC(3)]);

        P1 = FL.RKR_AXIS_1; P2 = FL.RKR_AXIS_2; P3 = FL.DPR_RKR; P4 = FL.PUSH_RKR;
        set(h.RKR, 'XData', [P1(1) P2(1) P3(1) P1(1) P4(1) P2(1) P4(1) P3(1)], ...
                   'YData', [P1(2) P2(2) P3(2) P1(2) P4(2) P2(2) P4(2) P3(2)], ...
                   'ZData', [P1(3) P2(3) P3(3) P1(3) P4(3) P2(3) P4(3) P3(3)]);

        set(h.TR, 'XData', [FL.TR_RACK(1) FL.TR_UPRIGHT(1)], 'YData', [FL.TR_RACK(2) FL.TR_UPRIGHT(2)], 'ZData', [FL.TR_RACK(3) FL.TR_UPRIGHT(3)]);
        set(h.Spindle, 'XData', [FL.SPINDLE_CENTER(1) FL.SPINDLE_INNER(1)], 'YData', [FL.SPINDLE_CENTER(2) FL.SPINDLE_INNER(2)], 'ZData', [FL.SPINDLE_CENTER(3) FL.SPINDLE_INNER(3)]);
        set(h.Conn, 'XData', [FL.SPINDLE_INNER(1) FL.TR_UPRIGHT(1)], 'YData', [FL.SPINDLE_INNER(2) FL.TR_UPRIGHT(2)], 'ZData', [FL.SPINDLE_INNER(3) FL.TR_UPRIGHT(3)]);

        ptsW = [FL.UFW_MC; FL.URW_MC; FL.LFW_MC; FL.LRW_MC; FL.UW_KN; FL.LW_KN; FL.PUSH_UW; FL.PUSH_RKR; FL.DPR_RKR; FL.DPR_MC; FL.RKR_AXIS_1; FL.RKR_AXIS_2];
        set(h.PtsW, 'XData', ptsW(:,1), 'YData', ptsW(:,2), 'ZData', ptsW(:,3));
        ptsY = [FL.TR_RACK; FL.TR_UPRIGHT; FL.SPINDLE_CENTER; FL.SPINDLE_INNER];
        set(h.PtsY, 'XData', ptsY(:,1), 'YData', ptsY(:,2), 'ZData', ptsY(:,3));

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