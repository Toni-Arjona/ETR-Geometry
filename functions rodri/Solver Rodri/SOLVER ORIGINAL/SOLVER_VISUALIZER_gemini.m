function ETR11_SOLVER_VISUALIZER_gemini()
    fig = uifigure('Name', 'ETR11 Live Geometry - Full Car', 'Position', [100 100 1200 850], 'Color', [0.1 0.1 0.1]);
    ax = uiaxes(fig, 'Position', [50 250 1100 550], 'BackgroundColor', [0 0 0], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
    ax.XLim = [-300 1800]; ax.YLim = [-800 800]; ax.ZLim = [0 750]; 
    
    %% --- INICIALIZACIÓN DE OBJETOS GRÁFICOS ---
    createPlot = @(c, w) plot3(ax, 0,0,0, 'Color', c, 'LineWidth', w);
    createPts  = @(c) plot3(ax, 0,0,0, 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', c, 'MarkerSize', 5);

    % Estructura para almacenar handles de las 4 ruedas
    side = {'FL', 'FR', 'RL', 'RR'};
    for i = 1:4
        s = side{i};
        h.(s).UW = createPlot('w', 2);
        h.(s).LW = createPlot('w', 2);
        h.(s).PUSH = createPlot('w', 2);
        h.(s).DPR = createPlot('w', 2);
        h.(s).RKR = createPlot('w', 1.5);
        h.(s).TR = createPlot('y', 2);
        h.(s).Spindle = createPlot('y', 3);
        h.(s).Conn = createPlot('y', 1.5);
        h.(s).PtsW = createPts('w');
        h.(s).PtsY = createPts('y');
        h.(s).LineCP = plot3(ax, 0,0,0, 'c--', 'LineWidth', 1.5);
        h.(s).PtsCP = createPts('c');
    end

    h.FrontRack = createPlot([0.8 0.8 0.8], 4);
    h.RearRack  = createPlot([0.8 0.8 0.8], 4);
    
    %% --- RUEDAS (CILINDROS) ---
    [xc, yc, zc] = cylinder((16*25.4)/2, 60); 
    zc_m = (zc - 0.5) * (7.5*25.4);
    for i = 1:4
        h.(side{i}).Wheel = surface(ax, xc, yc, zc_m, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
    end
    
    %% --- UI SLIDERS (0-50 para dampers) ---
    lblP = {'FontWeight', 'bold', 'FontColor', 'w', 'HorizontalAlignment', 'center'};
    uilabel(fig, 'Position', [200 210 800 20], 'Text', 'ÁNGULO DE VOLANTE', lblP{:});
    s_st = uislider(fig, 'Position', [200 190 800 3], 'Limits', [-140 140]);
    
    uilabel(fig, 'Position', [150 140 350 20], 'Text', 'COMPRESIÓN FL', lblP{:});
    s_cFL = uislider(fig, 'Position', [150 120 350 3], 'Limits', [-25 25]);
    uilabel(fig, 'Position', [700 140 350 20], 'Text', 'COMPRESIÓN FR', lblP{:});
    s_cFR = uislider(fig, 'Position', [700 120 350 3], 'Limits', [-25 25]);
    
    uilabel(fig, 'Position', [150 70 350 20], 'Text', 'COMPRESIÓN RL', lblP{:});
    s_cRL = uislider(fig, 'Position', [150 50 350 3], 'Limits', [-25 25]);
    uilabel(fig, 'Position', [700 70 350 20], 'Text', 'COMPRESIÓN RR', lblP{:});
    s_cRR = uislider(fig, 'Position', [700 50 350 3], 'Limits', [-25 25]);

    % Callbacks con ValueChangingFcn
    s_st.ValueChangingFcn  = @(src, e) updateFront(e.Value, s_cFL.Value, s_cFR.Value);
    s_cFL.ValueChangingFcn = @(src, e) updateFront(s_st.Value, e.Value, s_cFR.Value);
    s_cFR.ValueChangingFcn = @(src, e) updateFront(s_st.Value, s_cFL.Value, e.Value);
    s_cRL.ValueChangingFcn = @(src, e) updateRear(e.Value, s_cRR.Value);
    s_cRR.ValueChangingFcn = @(src, e) updateRear(s_cRL.Value, e.Value);

    %% --- FUNCIONES DE ACTUALIZACIÓN ---
    function updateFront(st, cFL, cFR)
        [FL, FR] = FRONT_KINEMATICS_SOLVER_ETR11_rodri(st, cFL, cFR);
        updateWheelGraphics('FL', FL);
        updateWheelGraphics('FR', FR);
        set(h.FrontRack, 'XData', [FL.TR_RACK(1) FR.TR_RACK(1)], 'YData', [FL.TR_RACK(2) FR.TR_RACK(2)], 'ZData', [FL.TR_RACK(3) FR.TR_RACK(3)]);
        drawnow limitrate;
    end

    function updateRear(cRL, cRR)
        [RL, RR] = REAR_KINEMATICS_SOLVER_ETR11_rodri(cRL, cRR);
        updateWheelGraphics('RL', RL);
        updateWheelGraphics('RR', RR);
        set(h.RearRack, 'XData', [RL.TR_RACK(1) RR.TR_RACK(1)], 'YData', [RL.TR_RACK(2) RR.TR_RACK(2)], 'ZData', [RL.TR_RACK(3) RR.TR_RACK(3)]);
        drawnow limitrate;
    end

    function updateWheelGraphics(id, data)
        sh = h.(id);
        set(sh.UW, 'XData', [data.UFW_MC(1) data.UW_KN(1) data.URW_MC(1)], 'YData', [data.UFW_MC(2) data.UW_KN(2) data.URW_MC(2)], 'ZData', [data.UFW_MC(3) data.UW_KN(3) data.URW_MC(3)]);
        set(sh.LW, 'XData', [data.LFW_MC(1) data.LW_KN(1) data.LRW_MC(1)], 'YData', [data.LFW_MC(2) data.LW_KN(2) data.LRW_MC(2)], 'ZData', [data.LFW_MC(3) data.LW_KN(3) data.LRW_MC(3)]);
        set(sh.PUSH, 'XData', [data.PUSH_UW(1) data.PUSH_RKR(1)], 'YData', [data.PUSH_UW(2) data.PUSH_RKR(2)], 'ZData', [data.PUSH_UW(3) data.PUSH_RKR(3)]);
        set(sh.DPR, 'XData', [data.DPR_RKR(1) data.DPR_MC(1)], 'YData', [data.DPR_RKR(2) data.DPR_MC(2)], 'ZData', [data.DPR_RKR(3) data.DPR_MC(3)]);
        
        P1 = data.RKR_AXIS_1; P2 = data.RKR_AXIS_2; P3 = data.DPR_RKR; P4 = data.PUSH_RKR;
        set(sh.RKR, 'XData', [P1(1) P2(1) P3(1) P1(1) P4(1) P2(1) P4(1) P3(1)], 'YData', [P1(2) P2(2) P3(2) P1(2) P4(2) P2(2) P4(2) P3(2)], 'ZData', [P1(3) P2(3) P3(3) P1(3) P4(3) P2(3) P4(3) P3(3)]);
        
        set(sh.TR, 'XData', [data.TR_RACK(1) data.TR_UPRIGHT(1)], 'YData', [data.TR_RACK(2) data.TR_UPRIGHT(2)], 'ZData', [data.TR_RACK(3) data.TR_UPRIGHT(3)]);
        set(sh.Spindle, 'XData', [data.SPINDLE_CENTER(1) data.SPINDLE_INNER(1)], 'YData', [data.SPINDLE_CENTER(2) data.SPINDLE_INNER(2)], 'ZData', [data.SPINDLE_CENTER(3) data.SPINDLE_INNER(3)]);
        set(sh.Conn, 'XData', [data.SPINDLE_INNER(1) data.TR_UPRIGHT(1)], 'YData', [data.SPINDLE_INNER(2) data.TR_UPRIGHT(2)], 'ZData', [data.SPINDLE_INNER(3) data.TR_UPRIGHT(3)]);
        
        % Puntos blancos
        ptsW = [data.UFW_MC; data.URW_MC; data.LFW_MC; data.LRW_MC; data.UW_KN; data.LW_KN; data.PUSH_UW; data.PUSH_RKR; data.DPR_RKR; data.DPR_MC; data.RKR_AXIS_1; data.RKR_AXIS_2];
        set(sh.PtsW, 'XData', ptsW(:,1), 'YData', ptsW(:,2), 'ZData', ptsW(:,3));
        
        % Puntos amarillos
        ptsY = [data.TR_RACK; data.TR_UPRIGHT; data.SPINDLE_CENTER; data.SPINDLE_INNER];
        set(sh.PtsY, 'XData', ptsY(:,1), 'YData', ptsY(:,2), 'ZData', ptsY(:,3));
        
        % Parche de contacto
        set(sh.LineCP, 'XData', [data.SPINDLE_CENTER(1) data.CONTACT_PATCH(1)], 'YData', [data.SPINDLE_CENTER(2) data.CONTACT_PATCH(2)], 'ZData', [data.SPINDLE_CENTER(3) data.CONTACT_PATCH(3)]);
        set(sh.PtsCP, 'XData', data.CONTACT_PATCH(1), 'YData', data.CONTACT_PATCH(2), 'ZData', data.CONTACT_PATCH(3));
        
        % Cilindro de la rueda (Rotación de Rodrigues)
        s_vec = (data.SPINDLE_INNER - data.SPINDLE_CENTER); 
        s_vec = s_vec / norm(s_vec);
        k_ref = [0 0 1]; v_rot = cross(k_ref, s_vec); c_rot = dot(k_ref, s_vec);
        vx = [0 -v_rot(3) v_rot(2); v_rot(3) 0 -v_rot(1); -v_rot(2) v_rot(1) 0];
        Rmat = eye(3) + vx + vx^2 * (1/(1+c_rot));
        pts_cyl = Rmat * [xc(:)'; yc(:)'; zc_m(:)'];
        set(sh.Wheel, 'XData', reshape(pts_cyl(1,:), size(xc)) + data.SPINDLE_CENTER(1), ...
                      'YData', reshape(pts_cyl(2,:), size(xc)) + data.SPINDLE_CENTER(2), ...
                      'ZData', reshape(pts_cyl(3,:), size(xc)) + data.SPINDLE_CENTER(3));
    end

    % Render inicial
    updateFront(0, 0, 0); updateRear(0, 0);
end