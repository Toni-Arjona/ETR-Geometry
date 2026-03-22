function ETR11_NEW_VISUALIZER()
    % Resetear semilla del solver analítico para evitar ruedas en posición
    % residual de una ejecución anterior (e.g. barrido steady-state a SW>145°)
    clear ETR11_KINEMATICS_ANALYTICAL

    % --- PARÁMETROS Y FIGURA ---
    LOADED_RADIUS = 0.203;
    fig = uifigure('Name', 'ETR11 PRO - Full Control', 'Position', [50 50 1600 850], 'Color', [0.1 0.1 0.1]);
    fig.CloseRequestFcn = @(src, e) closeApp(src);
    
    ax = uiaxes(fig, 'Position', [20 280 1060 550], 'BackgroundColor', [0 0 0], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
    ax.XLim = [-300 1800]; ax.YLim = [-800 800]; ax.ZLim = [-150 750];

    % --- TABLA DE PARÁMETROS (panel derecho) ---
    h_tbl = uitextarea(fig, 'Position', [1095 280 490 550], ...
        'FontName', 'Courier New', 'FontSize', 10.5, ...
        'FontColor', [0.9 0.9 0.9], 'BackgroundColor', [0.07 0.07 0.07], ...
        'Editable', 'off', 'Value', {'Calculando...'});
    
    %% --- OBJETOS GRÁFICOS (RENDERIZADO OPTIMIZADO) ---
    h.W_Solid = plot3(ax, nan, nan, nan, 'Color', 'w', 'LineWidth', 2);
    h.W_Dash  = plot3(ax, nan, nan, nan, 'Color', 'w', 'LineWidth', 1.5, 'LineStyle', '--');
    h.Y_Solid = plot3(ax, nan, nan, nan, 'Color', 'y', 'LineWidth', 2);
    h.Y_Dash  = plot3(ax, nan, nan, nan, 'Color', 'y', 'LineWidth', 1.5, 'LineStyle', '--');
    h.C_Solid = plot3(ax, nan, nan, nan, 'Color', 'c', 'LineWidth', 1.5);
    h.G_Dash  = plot3(ax, nan, nan, nan, 'Color', 'g', 'LineWidth', 2, 'LineStyle', '--');
    h.R_Dash  = plot3(ax, nan, nan, nan, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '--');
    
    h.PtsW = plot3(ax, nan, nan, nan, 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'w', 'MarkerSize', 5);
    h.PtsY = plot3(ax, nan, nan, nan, 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'y', 'MarkerSize', 5);
    h.PtsRC = plot3(ax, nan, nan, nan, 'Marker', 'x', 'MarkerSize', 10, 'Color', 'r', 'LineWidth', 2, 'LineStyle', 'none');
    h.Rack = plot3(ax, nan, nan, nan, 'Color', [0.8 0.8 0.8], 'LineWidth', 4);
    
    [xc, yc, zc] = cylinder((16*25.4)/2, 10); 
    zc_m = (zc - 0.5) * (7.5*25.4);
    side = {'FL', 'FR', 'RL', 'RR'};
    for i = 1:4
        h.(side{i}).Wheel = surface(ax, xc, yc, zc_m, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
    end
    
    %% --- INTERFAZ DE USUARIO ---
    lblP = {'FontWeight', 'bold', 'FontColor', 'w', 'HorizontalAlignment', 'center'};
    
    % BOTONES
    uibutton(fig, 'Position', [50 250 80 25], 'Text', 'Heave', 'ButtonPushedFcn', @(~,~) toggleAnim('heave'));
    uibutton(fig, 'Position', [140 250 80 25], 'Text', 'Roll', 'ButtonPushedFcn', @(~,~) toggleAnim('roll'));
    uibutton(fig, 'Position', [250 250 50 25], 'Text', 'Front', 'ButtonPushedFcn', @(~,~) view(ax, [90, 0]));
    uibutton(fig, 'Position', [310 250 50 25], 'Text', 'Side', 'ButtonPushedFcn', @(~,~) view(ax, [0, 0]));
    uibutton(fig, 'Position', [370 250 50 25], 'Text', 'Top', 'ButtonPushedFcn', @(~,~) view(ax, [0, 90]));
    uibutton(fig, 'Position', [430 250 50 25], 'Text', 'ISO', 'ButtonPushedFcn', @(~,~) view(ax, 3));
    
    % GLOBAL / STEER
    uilabel(fig, 'Position', [200 220 200 20], 'Text', 'GLOBAL COMPRESSION', lblP{:});
    s_G = uislider(fig, 'Position', [100 200 400 3], 'Limits', [-25 32], 'ValueChangedFcn', @(~,e) handleGlobal(e.Value));
    e_G = uieditfield(fig, 'numeric', 'Position', [520 190 60 22], 'Value', 0);
    uibutton(fig, 'Position', [590 190 60 22], 'Text', 'Reset', 'ButtonPushedFcn', @(~,~) resetCtrl('G'));
    
    uilabel(fig, 'Position', [800 220 200 20], 'Text', 'STEERING WHEEL ANGLE', lblP{:});
    s_st = uislider(fig, 'Position', [700 200 400 3], 'Limits', [-145 145], 'ValueChangedFcn', @(~,~) triggerUpdate());
    e_st = uieditfield(fig, 'numeric', 'Position', [1120 190 60 22], 'Value', 0);
    uibutton(fig, 'Position', [1190 190 60 22], 'Text', 'Reset', 'ButtonPushedFcn', @(~,~) resetCtrl('st'));
    
    % INDIVIDUALES
    % FL
    uilabel(fig, 'Position', [200 140 200 20], 'Text', 'FL COMPRESSION', lblP{:});
    s_FL = uislider(fig, 'Position', [100 120 400 3], 'Limits', [-25 32], 'ValueChangedFcn', @(~,~) triggerUpdate());
    e_FL = uieditfield(fig, 'numeric', 'Position', [520 110 60 22], 'Value', 0);
    uibutton(fig, 'Position', [590 110 60 22], 'Text', 'Reset', 'ButtonPushedFcn', @(~,~) resetCtrl('FL'));
    % FR
    uilabel(fig, 'Position', [800 140 200 20], 'Text', 'FR COMPRESSION', lblP{:});
    s_FR = uislider(fig, 'Position', [700 120 400 3], 'Limits', [-25 32], 'ValueChangedFcn', @(~,~) triggerUpdate());
    e_FR = uieditfield(fig, 'numeric', 'Position', [1120 110 60 22], 'Value', 0);
    uibutton(fig, 'Position', [1190 110 60 22], 'Text', 'Reset', 'ButtonPushedFcn', @(~,~) resetCtrl('FR'));
    % RL
    uilabel(fig, 'Position', [200 60 200 20], 'Text', 'RL COMPRESSION', lblP{:});
    s_RL = uislider(fig, 'Position', [100 40 400 3], 'Limits', [-25 32], 'ValueChangedFcn', @(~,~) triggerUpdate());
    e_RL = uieditfield(fig, 'numeric', 'Position', [520 30 60 22], 'Value', 0);
    uibutton(fig, 'Position', [590 30 60 22], 'Text', 'Reset', 'ButtonPushedFcn', @(~,~) resetCtrl('RL'));
    % RR
    uilabel(fig, 'Position', [800 60 200 20], 'Text', 'RR COMPRESSION', lblP{:});
    s_RR = uislider(fig, 'Position', [700 40 400 3], 'Limits', [-25 32], 'ValueChangedFcn', @(~,~) triggerUpdate());
    e_RR = uieditfield(fig, 'numeric', 'Position', [1120 30 60 22], 'Value', 0);
    uibutton(fig, 'Position', [1190 30 60 22], 'Text', 'Reset', 'ButtonPushedFcn', @(~,~) resetCtrl('RR'));
    
    % FPS COUNTER
    fps_last_time = tic;
    fps_alpha     = 0.2;   % suavizado exponencial
    fps_val       = 0;
    h_fps = uilabel(fig, 'Position', [1095 255 490 22], ...
        'Text', 'Solver: -- Hz', ...
        'FontName', 'Courier New', 'FontSize', 10, ...
        'FontColor', [0.4 1 0.4], 'BackgroundColor', [0.07 0.07 0.07], ...
        'HorizontalAlignment', 'right');

    % TIMER
    t_start = 0; anim_type = '';
    main_timer = timer('ExecutionMode', 'fixedRate', 'Period', 0.02, 'BusyMode', 'drop', 'TimerFcn', @(~,~) animStep());
    
    %% --- CALLBACKS ---
    function handleGlobal(val)
        e_G.Value = val; s_FL.Value = val; e_FL.Value = val; s_FR.Value = val; e_FR.Value = val;
        s_RL.Value = val; e_RL.Value = val; s_RR.Value = val; e_RR.Value = val;
        triggerUpdate();
    end
    
    function resetCtrl(type)
        switch type
            case 'G', s_G.Value = 0; handleGlobal(0);
            case 'st', s_st.Value = 0; e_st.Value = 0;
            case 'FL', s_FL.Value = 0; e_FL.Value = 0;
            case 'FR', s_FR.Value = 0; e_FR.Value = 0;
            case 'RL', s_RL.Value = 0; e_RL.Value = 0;
            case 'RR', s_RR.Value = 0; e_RR.Value = 0;
        end
        triggerUpdate();
    end
    
    function toggleAnim(type)
        if strcmp(main_timer.Running, 'on')
            stop(main_timer); 
        else
            anim_type = type; t_start = tic; start(main_timer); 
        end
    end
    
    function animStep()
        t = toc(t_start);
        G = s_G.Value;
        
        if strcmp(anim_type, 'heave')
            % Oscilación ajustada a los límites [-25, 32] -> Centro: 3.5, Amplitud: 28.5
            val = 3.5 + 28.5 * sin(2*pi*t); 
            s_G.Value = val; 
            handleGlobal(val);
        else
            amp = min(32 - G, G - (-25));
            delta = amp * sin(2*pi*t);
            
            vL = G - delta;
            vR = G + delta;
            
            s_FL.Value = vL; e_FL.Value = vL;
            s_FR.Value = vR; e_FR.Value = vR;
            s_RL.Value = vL; e_RL.Value = vL;
            s_RR.Value = vR; e_RR.Value = vR;
            
            updateAll(s_st.Value, vL, vR, vL, vR);
        end
    end
    
    function triggerUpdate()
        e_st.Value = s_st.Value; e_FL.Value = s_FL.Value; e_FR.Value = s_FR.Value; 
        e_RL.Value = s_RL.Value; e_RR.Value = s_RR.Value;
        updateAll(s_st.Value, s_FL.Value, s_FR.Value, s_RL.Value, s_RR.Value);
    end
    
    %% --- RENDER ENGINE OPTIMIZADO POR PIEZAS ---
    function updateAll(st, cFL, cFR, cRL, cRR)
        % FPS: mide el tiempo entre llamadas completas (solver + render)
        dt = toc(fps_last_time);
        fps_last_time = tic;
        if dt > 0
            fps_val = fps_alpha * (1/dt) + (1 - fps_alpha) * fps_val;
        end
        h_fps.Text = sprintf('Solver: %.1f Hz', fps_val);

        [FL, FR, RL, RR, F_RC, R_RC, FL_K, FR_K, RL_K, RR_K] =  ETR11_GET_POINTS_v2(st, cFL, cFR, cRL, cRR);
        
        pts_cp = [FL.CONTACT_PATCH; FR.CONTACT_PATCH; RL.CONTACT_PATCH; RR.CONTACT_PATCH];
        coeffs = [pts_cp(:,1:2), ones(4,1)] \ pts_cp(:,3); 
        n_ground = [-coeffs(1), -coeffs(2), 1]; n_ground = n_ground/norm(n_ground);
        v_rot = cross(n_ground, [0,0,1]); s_rot = norm(v_rot); c_rot = dot(n_ground, [0,0,1]);
        R_G = eye(3);
        if s_rot ~= 0
            vx = [0 -v_rot(3) v_rot(2); v_rot(3) 0 -v_rot(1); -v_rot(2) v_rot(1) 0];
            R_G = eye(3) + vx + vx^2 * ((1-c_rot)/s_rot^2);
        end
        cp_c = mean(pts_cp, 1);
        tf = @(P) (P - cp_c) * R_G' + [cp_c(1:2), 0];
        wS.x=[]; wS.y=[]; wS.z=[]; wD=wS; yS=wS; yD=wS; cS=wS; gD=wS; pW=wS; pY=wS;
        wheels = {FL, FR, RL, RR};
        for i = 1:4
            d = wheels{i};
            
            % Eliminado el último segmento para no dibujar el eje con línea continua
            uw = tf([d.UFW_MC; d.UW_KN; d.URW_MC]);
            lw = tf([d.LFW_MC; d.LW_KN; d.LRW_MC]);
            
            % Añadida la conexión entre DPR_RKR y PUSH_RKR al final
            rk = tf([d.RKR_AXIS_1; d.RKR_AXIS_2; d.DPR_RKR; d.RKR_AXIS_1; d.PUSH_RKR; d.RKR_AXIS_2; d.DPR_RKR; d.PUSH_RKR]);
            
            kp = tf([d.UW_KN; d.LW_KN]);
            push = tf([d.PUSH_UW; d.PUSH_RKR]);
            dpr = tf([d.DPR_RKR; d.DPR_MC]);
            
            wS = addSeg(wS, uw, lw, rk, kp, push, dpr);
            
            % Ejes de los wishbones trazados aquí como línea discontinua en wD
            wD = addSeg(wD, tf([d.UFW_MC; d.URW_MC]), tf([d.LFW_MC; d.LRW_MC]), tf([d.LW_KN; d.KP_FLOOR]));
            
            yS = addSeg(yS, tf([d.TR_RACK; d.TR_UPRIGHT; d.SPINDLE_INNER; d.SPINDLE_CENTER; d.SPINDLE_INNER; d.TR_UPRIGHT]));
            yD = addSeg(yD, tf([d.SPINDLE_CENTER; d.CONTACT_PATCH]));
            s_dir = d.SPINDLE / norm(d.SPINDLE);
            gD = addSeg(gD, tf([d.CONTACT_PATCH + 100*s_dir; d.CONTACT_PATCH - 100*s_dir]));
            
            if isfield(d, 'AR_RKR_LINK')
                piv = d.AR_AXIS - [0, 1, 0];
                arb = tf([d.AR_RKR_LINK; d.AR_LINK_ARB; piv; [piv(1), 0, piv(3)]]);
                cS = addSeg(cS, arb(1:2,:), arb(2:3,:), arb(3:4,:));
            end
            
            pW = addPts(pW, tf([d.UFW_MC; d.URW_MC; d.LFW_MC; d.LRW_MC; d.UW_KN; d.LW_KN; d.PUSH_UW; d.PUSH_RKR; d.DPR_RKR; d.DPR_MC]));
            pY = addPts(pY, tf([d.TR_RACK; d.TR_UPRIGHT; d.SPINDLE_CENTER; d.CONTACT_PATCH]));
            updateWheel(h.(side{i}).Wheel, d, tf, R_G, xc, yc, zc_m);
        end
        
        yS = addSeg(yS, tf([FL.TR_RACK; FR.TR_RACK]));
        rAx = tf([F_RC; R_RC]);
        
        set(h.W_Solid, 'XData', wS.x, 'YData', wS.y, 'ZData', wS.z);
        set(h.W_Dash,  'XData', wD.x, 'YData', wD.y, 'ZData', wD.z);
        set(h.Y_Solid, 'XData', yS.x, 'YData', yS.y, 'ZData', yS.z);
        set(h.Y_Dash,  'XData', yD.x, 'YData', yD.y, 'ZData', yD.z);
        set(h.C_Solid, 'XData', cS.x, 'YData', cS.y, 'ZData', cS.z);
        set(h.G_Dash,  'XData', gD.x, 'YData', gD.y, 'ZData', gD.z);
        set(h.R_Dash,  'XData', rAx(:,1), 'YData', rAx(:,2), 'ZData', rAx(:,3));
        set(h.PtsW,    'XData', pW.x, 'YData', pW.y, 'ZData', pW.z);
        set(h.PtsY,    'XData', pY.x, 'YData', pY.y, 'ZData', pY.z);
        set(h.PtsRC,   'XData', rAx(:,1), 'YData', rAx(:,2), 'ZData', rAx(:,3));
        
        % --- Actualizar tabla de parámetros ---
        KS = {FL_K, FR_K, RL_K, RR_K};
        comps = [cFL, cFR, cRL, cRR];
        lbl_w = 10;  col_w = 7;
        hdr = sprintf('%-*s %*s %*s %*s %*s\n', lbl_w,'PARAM', col_w,'FL', col_w,'FR', col_w,'RL', col_w,'RR');
        sep = [repmat('-',1, lbl_w + 4*col_w + 5), newline];

        function v = gf(K, fname, scale)
            if isfield(K, fname), v = K.(fname)*scale; else, v = NaN; end
        end

        rows_txt = {hdr, sep};
        pnames  = {'Comp [mm]','Camber [°]','Toe [°]','KPI [°]','Caster [°]','Trail [mm]','Scrub [mm]'};
        fnames  = {'',         'CAMBER',    'STEER',  'KPI',    'CASTER',    'TRAIL',     'SCRUB'};
        scales  = {1,           1,           1,        1,        1,           1,            1};
        for ri = 1:length(pnames)
            if strcmp(fnames{ri},'')
                vals = comps;
            else
                vals = arrayfun(@(k) gf(KS{k}, fnames{ri}, scales{ri}), 1:4);
            end
            row = sprintf('%-*s', lbl_w, pnames{ri});
            for ci = 1:4
                if isnan(vals(ci)), row = [row, sprintf(' %*s', col_w, 'N/A')];
                else,               row = [row, sprintf(' %+*.2f', col_w, vals(ci))]; end
            end
            rows_txt{end+1} = [row, newline]; %#ok<AGROW>
        end

        % Roll angle (from FL vs FR contact patch Z height difference)
        dz_F = FL.CONTACT_PATCH(3) - FR.CONTACT_PATCH(3);
        dz_R = RL.CONTACT_PATCH(3) - RR.CONTACT_PATCH(3);
        TF_mm = abs(FL.CONTACT_PATCH(2) - FR.CONTACT_PATCH(2));
        TR_mm = abs(RL.CONTACT_PATCH(2) - RR.CONTACT_PATCH(2));
        roll_F = atan2d(dz_F, TF_mm);
        roll_R = atan2d(dz_R, TR_mm);
        rows_txt{end+1} = sep;
        rows_txt{end+1} = sprintf('%-*s %+*.2f° (R)  %+*.2f° (F)\n', lbl_w,'Roll[°]', col_w-2, roll_R, col_w-2, roll_F);

        h_tbl.Value = rows_txt;

        drawnow limitrate;
    end
    
    % --- HELPERS ---
    function s = addSeg(s, varargin)
        for j = 1:length(varargin)
            pts = varargin{j}; 
            s.x = [s.x, pts(:,1)', NaN]; 
            s.y = [s.y, pts(:,2)', NaN]; 
            s.z = [s.z, pts(:,3)', NaN];
        end
    end
    
    function s = addPts(s, pts)
        s.x = [s.x; pts(:,1)]; 
        s.y = [s.y; pts(:,2)]; 
        s.z = [s.z; pts(:,3)]; 
    end
    
    function updateWheel(surfObj, d, tf, R_G, xc, yc, zc_m)
        sp_c = tf(d.SPINDLE_CENTER);
        s_vec = (d.SPINDLE) * R_G';
        k = [0 0 1]; v = cross(k, s_vec); c = dot(k, s_vec);
        if abs(1 + c) < 1e-10
            Rmat = diag([1, -1, -1]); % spindle antiparalelo a Z
        else
            vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
            Rmat = eye(3) + vx + vx^2 * (1/(1+c));
        end
        pts = Rmat * [xc(:)'; yc(:)'; zc_m(:)'];
        set(surfObj, 'XData', reshape(pts(1,:), size(xc)) + sp_c(1), 'YData', reshape(pts(2,:), size(xc)) + sp_c(2), 'ZData', reshape(pts(3,:), size(xc)) + sp_c(3));
    end
    
    function closeApp(src)
        stop(main_timer); 
        delete(main_timer); 
        delete(src); 
    end
    
    % LANZAMIENTO INICIAL
    triggerUpdate();
end