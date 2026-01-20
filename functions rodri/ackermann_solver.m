function ackermann_anti_fix()
    close all
    clear all
    clc

    % --- 1. CONFIGURACIÓN VISUAL (DARK MODE) ---
    negro = [0 0 0];
    blanco = [1 1 1];
    gris = [0.15 0.15 0.15];
    cyan = [0 1 1];

    fig = figure('Name', 'Ackermann Anti Fix', 'Color', negro, 'Position', [100, 100, 1200, 700]);
    
    % --- 2. ÁREA DE GRÁFICA ---
    ax_plot = axes('Parent', fig, 'Position', [0.08, 0.4, 0.60, 0.55], ...
        'Color', negro, 'XColor', blanco, 'YColor', blanco, ...
        'GridColor', blanco, 'GridAlpha', 0.4, 'LineWidth', 1.2);
    grid(ax_plot, 'on'); hold(ax_plot, 'on');
    
    title(ax_plot, 'Porcentaje Ackermann (Permite Negativos)', 'Color', blanco, 'FontSize', 14);
    xlabel(ax_plot, 'Desplazamiento Rack (mm)', 'Color', blanco, 'FontSize', 11);
    ylabel(ax_plot, '% Ackermann (Negativo = Anti)', 'Color', blanco, 'FontSize', 11);
    
    % LIMITES DEL EJE (AQUÍ ESTABA EL PROBLEMA)
    xlim(ax_plot, [0 50]);
    ylim(ax_plot, [-5 2]); % Abierto hacia abajo para ver Anti-Ackermann
    
    % Objetos gráficos
    hLine = plot(ax_plot, 0, 0, 'Color', cyan, 'LineWidth', 2.5);
    yline(ax_plot, 1, '--r', '100% Ackermann', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    yline(ax_plot, 0, '-g', '0% (Paralelo)', 'LineWidth', 1, 'LabelHorizontalAlignment', 'left');

    % --- 3. CONTROLES (SLIDERS) ---
    
    % Slider BX
    uicontrol('Style', 'text', 'Position', [50, 190, 150, 20], 'String', 'Mangueta X (bx)', ...
        'BackgroundColor', negro, 'ForegroundColor', blanco, 'HorizontalAlignment', 'left');
    lbl_bx_val = uicontrol('Style', 'text', 'Position', [220, 190, 100, 20], 'String', '0.00 mm', ...
        'BackgroundColor', negro, 'ForegroundColor', cyan, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');     
    sld_bx = uicontrol('Style', 'slider', 'Min', 40, 'Max', 150, 'Value', 70, ...
        'Position', [50, 160, 300, 20], 'BackgroundColor', gris);
    addlistener(sld_bx, 'ContinuousValueChange', @recalcular);

    % Slider BY
    uicontrol('Style', 'text', 'Position', [400, 190, 150, 20], 'String', 'Mangueta Y (by)', ...
        'BackgroundColor', negro, 'ForegroundColor', blanco, 'HorizontalAlignment', 'left'); 
    lbl_by_val = uicontrol('Style', 'text', 'Position', [570, 190, 100, 20], 'String', '0.00 mm', ...
        'BackgroundColor', negro, 'ForegroundColor', cyan, 'HorizontalAlignment', 'left', 'FontWeight', 'bold');    
    sld_by = uicontrol('Style', 'slider', 'Min', 10, 'Max', 100, 'Value', 40, ...
        'Position', [400, 160, 300, 20], 'BackgroundColor', gris);
    addlistener(sld_by, 'ContinuousValueChange', @recalcular);

    % --- 4. PANEL RESULTADOS ---
    pnl_res = uipanel('Parent', fig, 'Position', [0.72, 0.4, 0.25, 0.55], ...
        'Title', 'RESULTADOS', 'BackgroundColor', negro, 'ForegroundColor', 'y', ...
        'FontWeight', 'bold', 'FontSize', 12);
    
    txt_ax = uicontrol('Parent', pnl_res, 'Style', 'text', 'Position', [20, 250, 250, 30], ...
        'String', 'AX Calculado: ...', 'BackgroundColor', negro, 'ForegroundColor', blanco, ...
        'HorizontalAlignment', 'left', 'FontSize', 12);
    txt_status = uicontrol('Parent', pnl_res, 'Style', 'text', 'Position', [20, 20, 250, 100], ...
        'String', 'Iniciando...', 'BackgroundColor', negro, 'ForegroundColor', 'g', ...
        'HorizontalAlignment', 'left', 'FontSize', 10);

    last_sol = [-45, 50]; 
    recalcular();

    % --- 5. LÓGICA MATEMÁTICA ---
    function recalcular(~, ~)
        bx_val = get(sld_bx, 'Value');
        by_val = get(sld_by, 'Value');
        set(lbl_bx_val, 'String', sprintf('%.2f mm', bx_val));
        set(lbl_by_val, 'String', sprintf('%.2f mm', by_val));

        wheelbase = 1600;
        front_track = 1150; 
        rear_track = 1250; 
        l_rack = 450;
        ay_fixed = front_track/2 - l_rack/2;

        % --- TUS ECUACIONES EXACTAS ---
        l_brazo = @(rack_disp, ax, ay, bx, by) sqrt(bx^2 + by^2);
        l_tierod = @(rack_disp, ax, ay, bx, by) sqrt((ay - by)^2 + (ax - bx)^2);
        l_ac_ini = @(rack_disp, ax, ay, bx, by) sqrt(ay^2 + ax^2); 
        beta_ini =@(rack_disp, ax, ay, bx, by) atan2d(ax, ay); 
        alpha_ini = @(rack_disp, ax, ay, bx, by) acosd((l_ac_ini(rack_disp, ax, ay, bx, by)^2 + l_brazo(rack_disp, ax, ay, bx, by)^2 - l_tierod(rack_disp, ax, ay, bx, by)^2)/(2*l_brazo(rack_disp, ax, ay, bx, by)*l_ac_ini(rack_disp, ax, ay, bx, by)));
        l_ac = @(rack_disp, ax, ay, bx, by) sqrt((ay + rack_disp)^2 + ax^2); 
        alpha_delta = @(rack_disp, ax, ay, bx, by) alpha_ini(rack_disp, ax, ay, bx, by) - acosd((l_ac(rack_disp, ax, ay, bx, by)^2 + l_brazo(rack_disp, ax, ay, bx, by)^2 - l_tierod(rack_disp, ax, ay, bx, by)^2)/(2*l_brazo(rack_disp, ax, ay, bx, by)*l_ac(rack_disp, ax, ay, bx, by)));
        beta_delta = @(rack_disp, ax, ay, bx, by) beta_ini(rack_disp, ax, ay, bx, by) - atan2d(ax, rack_disp + ay);
        phi_delta = @(rack_disp, ax, ay, bx, by) abs(beta_delta(rack_disp, ax, ay, bx, by) + alpha_delta(rack_disp, ax, ay, bx, by));

        % --- SOLVER ---
        Sistema = @(x) [phi_delta(x(1), x(2), ay_fixed, bx_val, by_val) - 20.5;
                        phi_delta(-x(1), x(2), ay_fixed, bx_val, by_val)- 22];     
        opts = optimset('Display', 'off', 'TolFun', 1e-6);
        
        try
            [sol, ~, exitflag] = fsolve(Sistema, last_sol, opts);
            if exitflag <= 0
                set(txt_status, 'String', 'No converge', 'ForegroundColor', 'r');
                set(hLine, 'XData', [], 'YData', []); return;
            else
                set(txt_status, 'String', 'Solución OK', 'ForegroundColor', 'g');
            end
            
            last_sol = sol;
            ax_res = sol(2);
            set(txt_ax, 'String', sprintf('AX: %.3f mm', ax_res));

            % --- PLOT ---
            x_vec = 0:0.5:50;
            y_vec = zeros(size(x_vec));
            
            for i = 1:length(x_vec)
                rd = x_vec(i);
                
                phi_pos = phi_delta(rd, ax_res, ay_fixed, bx_val, by_val);
                phi_neg = phi_delta(-rd, ax_res, ay_fixed, bx_val, by_val);
                
                val_dyn_toe = -phi_pos + phi_neg;
                val_dyn_ack = -atand(wheelbase/(wheelbase/tand(phi_neg) + rear_track)) + phi_neg;
                
                if rd == 0 || abs(val_dyn_ack) < 1e-9
                    y_vec(i) = 1; 
                else
                    % Calculamos el valor
                    val = val_dyn_toe / val_dyn_ack;
                    % IMPORTANTE: Usamos real() por si la geometría se rompe (complex)
                    % para que no deje de plotear.
                    y_vec(i) = real(val); 
                end
            end
            
            set(hLine, 'XData', x_vec, 'YData', y_vec);
            
        catch ME
            set(txt_status, 'String', 'Error', 'ForegroundColor', 'r');
        end
    end
end