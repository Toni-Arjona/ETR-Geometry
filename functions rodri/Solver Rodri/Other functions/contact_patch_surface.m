function [FL_CONTACT_PATCH_INTERSECTION_PLUS, FL_CONTACT_PATCH_INTERSECTION_MINUS] = contact_patch_surface(TIRE_RADIUS, FL)

    u = [-FL.SPINDLE(2), FL.SPINDLE(1), 0]; % vector radial spindle
    u = u/norm(u); % normalización
   
    v = cross(FL.SPINDLE, u); % vector radial spindle que completa el triedro

    FL_TIRE_SURFACE = @(L, THETA) FL.SPINDLE_CENTER + TIRE_RADIUS*cos(THETA')*u + TIRE_RADIUS*sin(THETA')*v + (L')*FL.SPINDLE; % ecuación global cilindro

    % FUNCIÓN DE THETA PARA CONDICIÓN DE Z = 0
    A = sqrt(u(3)^2 + v(3)^2);
    phi = atan2(v(3), u(3));

    THETA_SOLVER_PLUS = @(L)  phi + acos(-(FL.SPINDLE_CENTER(3) + L.*FL.SPINDLE(3))/(TIRE_RADIUS*A));
    THETA_SOLVER_MINUS = @(L) phi - acos(-(FL.SPINDLE_CENTER(3) + L.*FL.SPINDLE(3))/(TIRE_RADIUS*A));

    L_INT = linspace(-7.5*25.4, 7.5*25.4, 1000); % intervalo de ancho del neumático, analiza solo para la región donde el neumático existe

    % Obtención de los valores de theta para el intervalo de L definido
    THETA_PLUS_FINAL = THETA_SOLVER_PLUS(L_INT);
    THETA_MINUS_FINAL = THETA_SOLVER_MINUS(L_INT);
    
    % Filtro valores complejos
    mask = imag(THETA_PLUS_FINAL) == 0; 
    
    L_real = L_INT(mask);
    T_plus_real  = THETA_PLUS_FINAL(mask);
    T_minus_real = THETA_MINUS_FINAL(mask);
    
    % Cálculo final
    FL_CONTACT_PATCH_INTERSECTION_PLUS  = FL_TIRE_SURFACE(L_real, T_plus_real);
    FL_CONTACT_PATCH_INTERSECTION_MINUS = FL_TIRE_SURFACE(L_real, T_minus_real);

   % Concatenar puntos X e Y cerrando el polígono
   %{
    X_patch = [FL.CONTACT_PATCH_INTERSECTION_PLUS(:,1); flipud(FL.CONTACT_PATCH_INTERSECTION_MINUS(:,1)); FL.CONTACT_PATCH_INTERSECTION_PLUS(1,1)];
    Y_patch = [FL.CONTACT_PATCH_INTERSECTION_PLUS(:,2); flipud(FL.CONTACT_PATCH_INTERSECTION_MINUS(:,2)); FL.CONTACT_PATCH_INTERSECTION_PLUS(1,2)];
    
    % Plotear
    figure;
    plot(X_patch, Y_patch, 'k-', 'LineWidth', 1.5);
    hold on;
    fill(X_patch, Y_patch, 'k', 'FaceAlpha', 0.2); % Sombreado de la huella
    
    axis equal;
    grid on;
    xlabel('X (mm)');
    ylabel('Y (mm)');
    title('Huella de contacto (Plano Z=0)');
   %}
end

   
        
        
   
    