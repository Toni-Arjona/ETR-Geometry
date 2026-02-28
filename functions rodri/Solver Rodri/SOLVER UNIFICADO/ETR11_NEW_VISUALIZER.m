function ETR11_NEW_VISUALIZER()
    % Se ensancha un poco la figura para dejar espacio a la tabla de datos
    fig = uifigure('Name', 'ETR11 Live Geometry - Full Car', 'Position', [50 50 1350 850], 'Color', [0.1 0.1 0.1]);
    
    % Se ajusta el ancho del gráfico 3D para que deje un bloque a la derecha
    ax = uiaxes(fig, 'Position', [20 250 900 580], 'BackgroundColor', [0 0 0], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
    ax.XLim = [-300 1800]; ax.YLim = [-800 800]; ax.ZLim = [0 750]; 
    
    %% --- INICIALIZACIÓN DE OBJETOS GRÁFICOS ---
    createPlot = @(c, w) plot3(ax, 0,0,0, 'Color', c, 'LineWidth', w);
    createDashed = @(c, w) plot3(ax, 0,0,0, 'Color', c, 'LineWidth', w, 'LineStyle', '--');
    createPts  = @(c) plot3(ax, 0,0,0, 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', c, 'MarkerSize', 5);
    
    side = {'FL', 'FR', 'RL', 'RR'};
    for i = 1:4
        s = side{i};
        h.(s).UW = createPlot('w', 2);
        h.(s).UW_Base = createDashed('w', 1.5); 
        h.(s).LW = createPlot('w', 2);
        h.(s).LW_Base = createDashed('w', 1.5); 
        
        h.(s).KP_Line = createPlot('w', 1.5); 
        h.(s).KP_Floor_Line = createDashed('w', 1.5); 
        h.(s).KP_Floor_Pt = createPts('w'); 
        
        h.(s).PUSH = createPlot('w', 2);
        h.(s).DPR = createPlot('w', 2);
        h.(s).RKR = createPlot('w', 1.5);
        h.(s).TR = createPlot('y', 2);
        h.(s).Spindle = createPlot('y', 3);
        h.(s).Conn = createPlot('y', 1.5);
        h.(s).PtsW = createPts('w');
        h.(s).PtsY = createPts('y');
        
        h.(s).LineCP = plot3(ax, 0,0,0, 'y--', 'LineWidth', 1.5); 
        h.(s).PtsCP = createPts('y'); 
        
        % NUEVO: Eje Y local del neumático en el parche de contacto
        h.(s).TireAxisY = plot3(ax, 0,0,0, 'g--', 'LineWidth', 2);
        
        % TRAYECTORIAS BARRIDO
        h.(s).TrajCP = plot3(ax, 0,0,0, 'r-', 'LineWidth', 1.5); 
    end
    h.FrontRack = createPlot([0.8 0.8 0.8], 4);
    h.RearRack  = createPlot([0.8 0.8 0.8], 4);
    h.F_RC = plot3(ax, 0,0,0, 'Marker', 'x', 'MarkerSize', 10, 'Color', 'r', 'LineWidth', 2);
    h.R_RC = plot3(ax, 0,0,0, 'Marker', 'x', 'MarkerSize', 10, 'Color', 'r', 'LineWidth', 2);
    h.RollAxis = createDashed('r', 1.5); 
    
    [xc, yc, zc] = cylinder((16*25.4)/2, 60); 
    zc_m = (zc - 0.5) * (7.5*25.4);
    for i = 1:4
        h.(side{i}).Wheel = surface(ax, xc, yc, zc_m, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
    end
    
    %% --- TABLA DE DATOS CINEMÁTICOS ---
    cnames = {'FL', 'FR', 'RL', 'RR'};
    rnames = {'Toe / Steer (º)', 'Camber (º)', 'Caster (º)', 'KPI (º)', 'Scrub Rad (mm)', 'Mech Trail (mm)', 'Jacking (mm/º)'};
    % Estilo oscuro para que coincida con el fondo
    h.Table = uitable(fig, 'Position', [940 400 390 205], 'ColumnName', cnames, 'RowName', rnames, ...
                      'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w', ...
                      'RowStriping', 'off');
    
    lblT = {'FontWeight', 'bold', 'FontColor', 'y', 'HorizontalAlignment', 'center'};
    uilabel(fig, 'Position', [940 610 390 20], 'Text', 'VALORES INSTANTÁNEOS', lblT{:});
    %% --- UI SLIDERS (DISTRIBUCIÓN ACTUALIZADA) ---
    lblP = {'FontWeight', 'bold', 'FontColor', 'w', 'HorizontalAlignment', 'center'};
    
    % Fila 1: Dirección
    uilabel(fig, 'Position', [250 220 800 20], 'Text', 'ÁNGULO DE VOLANTE', lblP{:});
    s_st = uislider(fig, 'Position', [250 200 800 3], 'Limits', [-140 140]);
    
    % Fila 2: Ejes (Delantero / Trasero)
    uilabel(fig, 'Position', [150 160 350 20], 'Text', 'COMPRESIÓN EJE DELANTERO', lblP{:});
    s_F = uislider(fig, 'Position', [150 140 350 3], 'Limits', [-25 25]);
    
    uilabel(fig, 'Position', [800 160 350 20], 'Text', 'COMPRESIÓN EJE TRASERO', lblP{:});
    s_R = uislider(fig, 'Position', [800 140 350 3], 'Limits', [-25 25]);
    
    % Fila 3: Lados (Izquierdo / Derecho) - Roll/Balanceo
    uilabel(fig, 'Position', [150 100 350 20], 'Text', 'COMPRESIÓN LADO IZQUIERDO', lblP{:});
    s_Left = uislider(fig, 'Position', [150 80 350 3], 'Limits', [-25 25]);
    
    uilabel(fig, 'Position', [800 100 350 20], 'Text', 'COMPRESIÓN LADO DERECHO', lblP{:});
    s_Right = uislider(fig, 'Position', [800 80 350 3], 'Limits', [-25 25]);
    
    % Fila 4: Global (Heave)
    uilabel(fig, 'Position', [475 40 350 20], 'Text', 'COMPRESIÓN GLOBAL (HEAVE TOTAL)', lblP{:});
    s_All = uislider(fig, 'Position', [475 20 350 3], 'Limits', [-25 25]);
    
    % Callbacks unificados para 6 deslizables
    s_st.ValueChangingFcn    = @(src, e) updateAll(e.Value, s_F.Value, s_R.Value, s_Left.Value, s_Right.Value, s_All.Value);
    s_F.ValueChangingFcn     = @(src, e) updateAll(s_st.Value, e.Value, s_R.Value, s_Left.Value, s_Right.Value, s_All.Value);
    s_R.ValueChangingFcn     = @(src, e) updateAll(s_st.Value, s_F.Value, e.Value, s_Left.Value, s_Right.Value, s_All.Value);
    s_Left.ValueChangingFcn  = @(src, e) updateAll(s_st.Value, s_F.Value, s_R.Value, e.Value, s_Right.Value, s_All.Value);
    s_Right.ValueChangingFcn = @(src, e) updateAll(s_st.Value, s_F.Value, s_R.Value, s_Left.Value, e.Value, s_All.Value);
    s_All.ValueChangingFcn   = @(src, e) updateAll(s_st.Value, s_F.Value, s_R.Value, s_Left.Value, s_Right.Value, e.Value);
    
    function updateAll(st, cF, cR, cLeft, cRight, cAll)
        persistent last_comp
        
        % Se calculan las compresiones individuales combinando los deslizables
        cFL = cF + cLeft  + cAll;
        cFR = cF + cRight + cAll;
        cRL = cR + cLeft  + cAll;
        cRR = cR + cRight + cAll;
        
        % Llamada a la función con el output ampliado
        [FL, FR, RL, RR, F_RC, R_RC, FL_KIN, FR_KIN, RL_KIN, RR_KIN] = ETR11_GET_POINTS(st, cFL, cFR, cRL, cRR, 0.199);
        
        % --- BARRIDO DE TRAYECTORIA ---
        current_comp = [cFL, cFR, cRL, cRR];
        if isempty(last_comp) || any(last_comp ~= current_comp)
            last_comp = current_comp;
            
            % Hacemos 15 puntos de barrido desde el tope izquierdo al derecho (-140 a 140)
            steer_sweep = linspace(-140, 140, 15); 
            
            cp_x = zeros(2, length(steer_sweep)); cp_y = zeros(2, length(steer_sweep)); cp_z = zeros(2, length(steer_sweep));
            
            for j = 1:length(steer_sweep)
                [FL_sw, FR_sw, ~, ~] = ETR11_GET_POINTS(steer_sweep(j), cFL, cFR, cRL, cRR, 0.199);
                
                % Datos Rueda Izquierda (FL)
                cp_x(1,j) = FL_sw.CONTACT_PATCH(1); cp_y(1,j) = FL_sw.CONTACT_PATCH(2); cp_z(1,j) = FL_sw.CONTACT_PATCH(3);
                
                % Datos Rueda Derecha (FR)
                cp_x(2,j) = FR_sw.CONTACT_PATCH(1); cp_y(2,j) = FR_sw.CONTACT_PATCH(2); cp_z(2,j) = FR_sw.CONTACT_PATCH(3);
            end
            
            % Actualizamos el ploteo de las curvas (rojo = CP)
            set(h.FL.TrajCP, 'XData', cp_x(1,:), 'YData', cp_y(1,:), 'ZData', cp_z(1,:));
            set(h.FR.TrajCP, 'XData', cp_x(2,:), 'YData', cp_y(2,:), 'ZData', cp_z(2,:));
        end
        % --- FIN BARRIDO ---
        
        updateWheelGraphics('FL', FL);
        updateWheelGraphics('FR', FR);
        updateWheelGraphics('RL', RL);
        updateWheelGraphics('RR', RR);
        
        set(h.FrontRack, 'XData', [FL.TR_RACK(1) FR.TR_RACK(1)], 'YData', [FL.TR_RACK(2) FR.TR_RACK(2)], 'ZData', [FL.TR_RACK(3) FR.TR_RACK(3)]);
        set(h.RearRack,  'XData', [RL.TR_RACK(1) RR.TR_RACK(1)], 'YData', [RL.TR_RACK(2) RR.TR_RACK(2)], 'ZData', [RL.TR_RACK(3) RR.TR_RACK(3)]);
        set(h.F_RC, 'XData', F_RC(1), 'YData', F_RC(2), 'ZData', F_RC(3));
        set(h.R_RC, 'XData', R_RC(1), 'YData', R_RC(2), 'ZData', R_RC(3));
        
        % Actualización del Roll Axis
        set(h.RollAxis, 'XData', [F_RC(1) R_RC(1)], 'YData', [F_RC(2) R_RC(2)], 'ZData', [F_RC(3) R_RC(3)]);
        
        % Actualización de la Tabla Dinámica (Redondeado a 2 decimales para lectura limpia)
        kin_data = round([
            FL_KIN.STEER, FR_KIN.STEER, RL_KIN.STEER, RR_KIN.STEER;
            FL_KIN.CAMBER, FR_KIN.CAMBER, RL_KIN.CAMBER, RR_KIN.CAMBER;
            FL_KIN.CASTER, FR_KIN.CASTER, RL_KIN.CASTER, RR_KIN.CASTER;
            FL_KIN.KPI, FR_KIN.KPI, RL_KIN.KPI, RR_KIN.KPI;
            FL_KIN.SCRUB, FR_KIN.SCRUB, RL_KIN.SCRUB, RR_KIN.SCRUB;
            FL_KIN.TRAIL, FR_KIN.TRAIL, RL_KIN.TRAIL, RR_KIN.TRAIL;
            FL_KIN.JACKING_RATE, FR_KIN.JACKING_RATE, RL_KIN.JACKING_RATE, RR_KIN.JACKING_RATE
        ], 2);
        
        set(h.Table, 'Data', kin_data);
        
        drawnow limitrate;
    end
    
    function updateWheelGraphics(id, data)
        sh = h.(id);
        
        % Wishbones
        set(sh.UW, 'XData', [data.UFW_MC(1) data.UW_KN(1) data.URW_MC(1)], 'YData', [data.UFW_MC(2) data.UW_KN(2) data.URW_MC(2)], 'ZData', [data.UFW_MC(3) data.UW_KN(3) data.URW_MC(3)]);
        set(sh.LW, 'XData', [data.LFW_MC(1) data.LW_KN(1) data.LRW_MC(1)], 'YData', [data.LFW_MC(2) data.LW_KN(2) data.LRW_MC(2)], 'ZData', [data.LFW_MC(3) data.LW_KN(3) data.LRW_MC(3)]);
        
        % Cierres de los trapecios
        set(sh.UW_Base, 'XData', [data.UFW_MC(1) data.URW_MC(1)], 'YData', [data.UFW_MC(2) data.URW_MC(2)], 'ZData', [data.UFW_MC(3) data.URW_MC(3)]);
        set(sh.LW_Base, 'XData', [data.LFW_MC(1) data.LRW_MC(1)], 'YData', [data.LFW_MC(2) data.LRW_MC(2)], 'ZData', [data.LFW_MC(3) data.LRW_MC(3)]);
        
        % Líneas y puntos de Kingpin
        set(sh.KP_Line, 'XData', [data.UW_KN(1) data.LW_KN(1)], 'YData', [data.UW_KN(2) data.LW_KN(2)], 'ZData', [data.UW_KN(3) data.LW_KN(3)]);
        set(sh.KP_Floor_Line, 'XData', [data.LW_KN(1) data.KP_FLOOR(1)], 'YData', [data.LW_KN(2) data.KP_FLOOR(2)], 'ZData', [data.LW_KN(3) data.KP_FLOOR(3)]);
        set(sh.KP_Floor_Pt, 'XData', data.KP_FLOOR(1), 'YData', data.KP_FLOOR(2), 'ZData', data.KP_FLOOR(3));
        
        set(sh.PUSH, 'XData', [data.PUSH_UW(1) data.PUSH_RKR(1)], 'YData', [data.PUSH_UW(2) data.PUSH_RKR(2)], 'ZData', [data.PUSH_UW(3) data.PUSH_RKR(3)]);
        set(sh.DPR, 'XData', [data.DPR_RKR(1) data.DPR_MC(1)], 'YData', [data.DPR_RKR(2) data.DPR_MC(2)], 'ZData', [data.DPR_RKR(3) data.DPR_MC(3)]);
        
        P1 = data.RKR_AXIS_1; P2 = data.RKR_AXIS_2; P3 = data.DPR_RKR; P4 = data.PUSH_RKR;
        set(sh.RKR, 'XData', [P1(1) P2(1) P3(1) P1(1) P4(1) P2(1) P4(1) P3(1)], 'YData', [P1(2) P2(2) P3(2) P1(2) P4(2) P2(2) P4(2) P3(2)], 'ZData', [P1(3) P2(3) P3(3) P1(3) P4(3) P2(3) P4(3) P3(3)]);
        
        set(sh.TR, 'XData', [data.TR_RACK(1) data.TR_UPRIGHT(1)], 'YData', [data.TR_RACK(2) data.TR_UPRIGHT(2)], 'ZData', [data.TR_RACK(3) data.TR_UPRIGHT(3)]);
        set(sh.Spindle, 'XData', [data.SPINDLE_CENTER(1) data.SPINDLE_INNER(1)], 'YData', [data.SPINDLE_CENTER(2) data.SPINDLE_INNER(2)], 'ZData', [data.SPINDLE_CENTER(3) data.SPINDLE_INNER(3)]);
        set(sh.Conn, 'XData', [data.SPINDLE_INNER(1) data.TR_UPRIGHT(1)], 'YData', [data.SPINDLE_INNER(2) data.TR_UPRIGHT(2)], 'ZData', [data.SPINDLE_INNER(3) data.TR_UPRIGHT(3)]);
        
        ptsW = [data.UFW_MC; data.URW_MC; data.LFW_MC; data.LRW_MC; data.UW_KN; data.LW_KN; data.PUSH_UW; data.PUSH_RKR; data.DPR_RKR; data.DPR_MC; data.RKR_AXIS_1; data.RKR_AXIS_2];
        set(sh.PtsW, 'XData', ptsW(:,1), 'YData', ptsW(:,2), 'ZData', ptsW(:,3));
        ptsY = [data.TR_RACK; data.TR_UPRIGHT; data.SPINDLE_CENTER; data.SPINDLE_INNER];
        set(sh.PtsY, 'XData', ptsY(:,1), 'YData', ptsY(:,2), 'ZData', ptsY(:,3));
        
        set(sh.LineCP, 'XData', [data.SPINDLE_CENTER(1) data.CONTACT_PATCH(1)], 'YData', [data.SPINDLE_CENTER(2) data.CONTACT_PATCH(2)], 'ZData', [data.SPINDLE_CENTER(3) data.CONTACT_PATCH(3)]);
        set(sh.PtsCP, 'XData', data.CONTACT_PATCH(1), 'YData', data.CONTACT_PATCH(2), 'ZData', data.CONTACT_PATCH(3));
        
        % NUEVO: Actualización del eje Y del neumático
        % Definimos la longitud de representación del eje transversal (ej: 100 mm a cada lado)
        ty_length = 100; 
        
        % El eje local Y del neumático está definido por la dirección del spindle
        spindle_dir = data.SPINDLE / norm(data.SPINDLE);
        
        % Calculamos los extremos del segmento que representa el eje Y pasando por el parche de contacto
        TY_pt1 = data.CONTACT_PATCH + ty_length * spindle_dir;
        TY_pt2 = data.CONTACT_PATCH - ty_length * spindle_dir;
        
        set(sh.TireAxisY, 'XData', [TY_pt1(1) TY_pt2(1)], ...
                          'YData', [TY_pt1(2) TY_pt2(2)], ...
                          'ZData', [TY_pt1(3) TY_pt2(3)]);
        
        s_vec = data.SPINDLE;
        k_ref = [0 0 1]; v_rot = cross(k_ref, s_vec); c_rot = dot(k_ref, s_vec);
        vx = [0 -v_rot(3) v_rot(2); v_rot(3) 0 -v_rot(1); -v_rot(2) v_rot(1) 0];
        Rmat = eye(3) + vx + vx^2 * (1/(1+c_rot));
        pts_cyl = Rmat * [xc(:)'; yc(:)'; zc_m(:)'];
        set(sh.Wheel, 'XData', reshape(pts_cyl(1,:), size(xc)) + data.SPINDLE_CENTER(1), ...
                      'YData', reshape(pts_cyl(2,:), size(xc)) + data.SPINDLE_CENTER(2), ...
                      'ZData', reshape(pts_cyl(3,:), size(xc)) + data.SPINDLE_CENTER(3));
    end
    % Llamada inicial para renderizar en reposo
    updateAll(0, 0, 0, 0, 0, 0);
end