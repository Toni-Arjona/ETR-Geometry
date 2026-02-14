function ackermann_toe_solver()
    close all
    clear all
    clc

    % --- 1. CONFIGURACIÓN VISUAL (DARK MODE) ---
    negro = [0 0 0];
    blanco = [1 1 1];
    gris = [0.15 0.15 0.15];
    cyan = [0 1 1];
    amarillo = [1 1 0];

    fig = figure('Name', 'Ackermann Solver (Toe Integrado)', 'Color', negro, 'Position', [100, 100, 1200, 750]);
    
    % --- 2. ÁREA DE GRÁFICA ---
    ax_plot = axes('Parent', fig, 'Position', [0.08, 0.45, 0.60, 0.50], ...
        'Color', negro, 'XColor', blanco, 'YColor', blanco, ...
        'GridColor', blanco, 'GridAlpha', 0.4, 'LineWidth', 1.2);
    grid(ax_plot, 'on'); hold(ax_plot, 'on');
    
    title(ax_plot, 'Ackermann Integrando Toe en Solver', 'Color', blanco, 'FontSize', 14);
    xlabel(ax_plot, 'Desplazamiento Rack (mm)', 'Color', blanco);
    ylabel(ax_plot, '% Ackermann (Negativo = Anti)', 'Color', blanco);
    
    xlim(ax_plot, [0 50]);
    ylim(ax_plot, [-5 2]); 
    
    % Objetos gráficos
    hLine = plot(ax_plot, 0, 0, 'Color', cyan, 'LineWidth', 2.5);
    yline(ax_plot, 1, '--r', '100% Ackermann', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    yline(ax_plot, 0, '-g', '0% (Paralelo)', 'LineWidth', 1, 'LabelHorizontalAlignment', 'left');

    % --- 3. CONTROLES (SLIDERS) ---
    
    % --> SLIDER TOE OUT (Variable de diseño) <--
    uicontrol('Style', 'text', 'Position', [750, 190, 150, 20], 'String', 'Toe Out Total (deg)', ...
        'BackgroundColor', negro, 'ForegroundColor', amarillo, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
    
    lbl_toe_val = uicontrol('Style', 'text', 'Position', [920, 190, 100, 20], 'String', '0.00°', ...
        'BackgroundColor', negro, 'ForegroundColor', amarillo, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
        
    sld_toe = uicontrol('Style', 'slider', 'Min', -5, 'Max', 5, 'Value', 0, ...
        'Position', [750, 160, 300, 20], 'BackgroundColor', gris);
    addlistener(sld_toe, 'ContinuousValueChange', @recalcular);

    % Slider BX (Mangueta X)
    uicontrol('Style', 'text', 'Position', [50, 190, 150, 20], 'String', 'Mangueta X (bx)', ...
        'BackgroundColor', negro, 'ForegroundColor', blanco, 'HorizontalAlignment', 'left');
    lbl_bx_val = uicontrol('Style', 'text', 'Position', [220, 190, 100, 20], 'String', '0.00 mm', ...
        'BackgroundColor', negro, 'ForegroundColor', cyan, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');     
    sld_bx = uicontrol('Style', 'slider', 'Min', 20, 'Max', 150, 'Value', 70, ...
        'Position', [50, 160, 300, 20], 'BackgroundColor', gris);
    addlistener(sld_bx, 'ContinuousValueChange', @recalcular);

    % Slider BY (Mangueta Y)
    uicontrol('Style', 'text', 'Position', [400, 190, 150, 20], 'String', 'Mangueta Y (by)', ...
        'BackgroundColor', negro, 'ForegroundColor', blanco, 'HorizontalAlignment', 'left'); 
    lbl_by_val = uicontrol('Style', 'text', 'Position', [570, 190, 100, 20], 'String', '0.00 mm', ...
        'BackgroundColor', negro, 'ForegroundColor', cyan, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');    
    sld_by = uicontrol('Style', 'slider', 'Min', -100, 'Max', 100, 'Value', 40, ...
        'Position', [400, 160, 300, 20], 'BackgroundColor', gris);
    addlistener(sld_by, 'ContinuousValueChange', @recalcular);

    % --- 4. PANEL RESULTADOS ---
    pnl_res = uipanel('Parent', fig, 'Position', [0.72, 0.45, 0.25, 0.50], ...
        'Title', 'RESULTADOS SOLVER', 'BackgroundColor', negro, 'ForegroundColor', 'y', ...
        'FontWeight', 'bold', 'FontSize', 12);
    
    txt_ax = uicontrol('Parent', pnl_res, 'Style', 'text', 'Position', [20, 250, 250, 30], ...
        'String', 'AX Calculado: ...', 'BackgroundColor', negro, 'ForegroundColor', blanco, ...
        'HorizontalAlignment', 'left', 'FontSize', 12);
    txt_d = uicontrol('Parent', pnl_res, 'Style', 'text', 'Position', [20, 200, 250, 30], ...
        'String', 'd Ref: ...', 'BackgroundColor', negro, 'ForegroundColor', blanco, ...
        'HorizontalAlignment', 'left', 'FontSize', 12);
    txt_status = uicontrol('Parent', pnl_res, 'Style', 'text', 'Position', [20, 20, 250, 100], ...
        'String', 'Iniciando...', 'BackgroundColor', negro, 'ForegroundColor', 'g', ...
        'HorizontalAlignment', 'left', 'FontSize', 10);

    last_sol = [-45, 50]; 
    recalcular();

    % --- 5. LÓGICA MATEMÁTICA ---
    function recalcular(~, ~)
        % 1. Leer Inputs
        bx_val = get(sld_bx, 'Value');
        by_val = get(sld_by, 'Value');
        toe_total_deg = get(sld_toe, 'Value');
        toe_side = toe_total_deg / 2; % Toe por lado

        % UI Update
        set(lbl_bx_val, 'String', sprintf('%.2f mm', bx_val));
        set(lbl_by_val, 'String', sprintf('%.2f mm', by_val));
        set(lbl_toe_val, 'String', sprintf('%.2f deg', toe_total_deg));

        % Datos Coche
        wheelbase = 1600;
        front_track = 1150; 
        rear_track = 1250; 
        l_rack = 450;
        ay_fixed = front_track/2 - l_rack/2;

        % --- ECUACIONES GEOMÉTRICAS (Tus ecuaciones intactas) ---
        l_brazo = @(rack_disp, ax, ay, bx, by) sqrt(bx^2 + by^2);
        l_tierod = @(rack_disp, ax, ay, bx, by) sqrt((ay - by)^2 + (ax - bx)^2);
        l_ac_ini = @(rack_disp, ax, ay, bx, by) sqrt(ay^2 + ax^2); 
        beta_ini =@(rack_disp, ax, ay, bx, by) atan2d(ax, ay); 
        alpha_ini = @(rack_disp, ax, ay, bx, by) acosd((l_ac_ini(rack_disp, ax, ay, bx, by)^2 + l_brazo(rack_disp, ax, ay, bx, by)^2 - l_tierod(rack_disp, ax, ay, bx, by)^2)/(2*l_brazo(rack_disp, ax, ay, bx, by)*l_ac_ini(rack_disp, ax, ay, bx, by)));
        l_ac = @(rack_disp, ax, ay, bx, by) sqrt((ay + rack_disp)^2 + ax^2); 
        alpha_delta = @(rack_disp, ax, ay, bx, by) alpha_ini(rack_disp, ax, ay, bx, by) - acosd((l_ac(rack_disp, ax, ay, bx, by)^2 + l_brazo(rack_disp, ax, ay, bx, by)^2 - l_tierod(rack_disp, ax, ay, bx, by)^2)/(2*l_brazo(rack_disp, ax, ay, bx, by)*l_ac(rack_disp, ax, ay, bx, by)));
        beta_delta = @(rack_disp, ax, ay, bx, by) beta_ini(rack_disp, ax, ay, bx, by) - atan2d(ax, rack_disp + ay);
        phi_delta = @(rack_disp, ax, ay, bx, by) abs(beta_delta(rack_disp, ax, ay, bx, by) + alpha_delta(rack_disp, ax, ay, bx, by));

        % --- SISTEMA FSOLVE (AHORA INCLUYE EL TOE) ---
        % Objetivo: Que la rueda, DESPUÉS del toe estático, esté en 20.5 y 22.
        
        % x(1) = desplazamiento rack (d)
        % x(2) = posición rack (ax)
        
        % Ecuación 1: Rueda Exterior (Rack +d). Target 20.5.
        % La rueda exterior tiene toe-out (-toe_side) respecto al avance.
        % Angulo Real Exterior = Geometrico - Toe_side
        eq_out = @(d, ax) phi_delta(d, ax, ay_fixed, bx_val, by_val) - toe_side - 20.5;
        
        % Ecuación 2: Rueda Interior (Rack -d). Target 22.
        % La rueda interior tiene toe-out (+toe_side) a favor del giro.
        % Angulo Real Interior = Geometrico + Toe_side
        eq_in = @(d, ax) phi_delta(-d, ax, ay_fixed, bx_val, by_val) + toe_side - 22;

        Sistema = @(x) [eq_out(x(1), x(2)); 
                        eq_in(x(1), x(2))];
        
        opts = optimset('Display', 'off', 'TolFun', 1e-6);
        
        try
            [sol, ~, exitflag] = fsolve(Sistema, last_sol, opts);
            if exitflag <= 0
                set(txt_status, 'String', 'No converge con este Toe', 'ForegroundColor', 'r');
                set(hLine, 'XData', [], 'YData', []); return;
            else
                set(txt_status, 'String', 'Solución OK', 'ForegroundColor', 'g');
            end
            
            last_sol = sol;
            d_res = sol(1);
            ax_res = sol(2);
            set(txt_ax, 'String', sprintf('AX: %.3f mm', ax_res));
            set(txt_d, 'String', sprintf('d Ref: %.3f mm', d_res));

            % --- CÁLCULO DE CURVA ---
            x_vec = 0:0.5:50;
            y_vec = zeros(size(x_vec));
            
            for i = 1:length(x_vec)
                rd = x_vec(i);
                
                % 1. Geometría Pura
                phi_geo_out = phi_delta(rd, ax_res, ay_fixed, bx_val, by_val);
                phi_geo_in  = phi_delta(-rd, ax_res, ay_fixed, bx_val, by_val);
                
                % 2. Ángulos Reales (Incluyendo Toe Static)
                phi_real_in = phi_geo_in + toe_side;   % Interior suma toe out
                phi_real_out = phi_geo_out - toe_side; % Exterior resta toe out
                
                % 3. Dynamic Toe Real
                val_dyn_toe = -phi_real_out + phi_real_in;
                
                % 4. Ackermann Ideal
                if abs(phi_real_in) < 1e-5
                    val_dyn_ack = 0;
                else
                    val_dyn_ack = -atand(wheelbase/(wheelbase/tand(phi_real_in) + rear_track)) + phi_real_in;
                end
                
                % 5. Porcentaje
                if abs(val_dyn_ack) < 1e-9
                    y_vec(i) = 1; 
                else
                    y_vec(i) = real(val_dyn_toe / val_dyn_ack);
                end
            end
            
            set(hLine, 'XData', x_vec, 'YData', y_vec);
            
        catch ME
            set(txt_status, 'String', 'Error Math', 'ForegroundColor', 'r');
        end
    end
end