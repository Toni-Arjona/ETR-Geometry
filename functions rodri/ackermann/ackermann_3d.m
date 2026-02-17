clear all
close all

wheelbase = 1535;
track = 1250;

% Puntos geometría. Todos los puntos corresponden a rueda delantera izquierda
% Spindle
spindle_center_ini = [-100, -625, 203]; 
spindle_inner_point = [-100, -564, 201];
spindle_vec_ini = (spindle_inner_point - spindle_center_ini)/ norm(spindle_inner_point - spindle_center_ini); % Vector unitario del spindle, inicial.

% Knuckles
upper_knuckle = [-83, -510, 294]; 
lower_knuckle = [-96.5 ,-571, 111]; % referencia para cálculo Rodrigues
kingpin_vector = (upper_knuckle - lower_knuckle)/norm(lower_knuckle - upper_knuckle); % vector unitario kingpin.

% Wishbones - monocoque joints
upper_rear_joint = [80, -225, 268];
upper_front_joint = [-210, -225, 277];
lower_rear_joint = [85, -225, 136];
lower_front_joint = [-210, -225, 123];

upper_wishbone_axis = (upper_front_joint - upper_rear_joint)/norm(upper_front_joint - upper_rear_joint); % vector dirección eje rotación upper wishbone
lower_wishbone_axis = (lower_front_joint - lower_rear_joint)/norm(lower_front_joint - lower_rear_joint); % vector dirección eje rotación lower wishbone

% Push rod
pushrod_upright_joint = [ -89, -523, 305];
pushrod_rocker_joint = [ -91, -193.5, 558];
l_pushrod = norm(pushrod_rocker_joint - pushrod_upright_joint); % longitud push_rod

% Damper
damper_rocker_joint = [ -51, -112, 610];
damper_monocoque_joint = [ 164, -112, 610];

% Tie rod
tie_rack_joint_ini = [-170, -225, 149.39];
tie_upright_joint_ini = [-180, -560, 140];
l_tie = norm(tie_upright_joint_ini - tie_rack_joint_ini); % longitud tie

% Vector brazo de dirección (para trackear rotación total)
steerarm_vector_ini = tie_upright_joint_ini - lower_knuckle;

% Vector radio desde el punto de pivote (lower knuckle) al centro de rueda. Se usa para el cálculo del jacking geométrico
spindle_radius_vector_ini = spindle_center_ini - lower_knuckle; 

% Formula Rodrigues: rotar un vector v (steerarm_vec), alrededor de un eje k (kingpin_vec), un ángulo th (total steer, contando variaciones en todos los ejes).
rotated_vec = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); 

idx = 1;
ext_steer = zeros(1, 100);
ext_camber = zeros(1, 100);
ext_jacking = zeros(1, 100); 
ext_ratio = zeros(1, 100); 

% Bucle Exterior (Rack desplazándose en una dirección)
for rack_disp = 0:0.5:50
    
    tie_rack_joint = tie_rack_joint_ini + [0, rack_disp, 0]; % nueva posición unión tie-rack.
    
    cost_func = @(th) norm((lower_knuckle + rotated_vec(steerarm_vector_ini, kingpin_vector, th)) - tie_rack_joint) - l_tie; % función a resolver. Saca el ángulo que hacer que se mantenga la longitud del tie.
    
    theta_axis = fzero(cost_func, 0);
    
    spindle_final = rotated_vec(spindle_vec_ini, kingpin_vector, theta_axis); % posición final del vector spindle después de la rotación.
    
    steer_angle = atan2d(spindle_final(1), spindle_final(2)); % proyección del spindle sobre plano XY para obtener steer
    camber_angle = asind(spindle_final(3)); % proyección del spindle sobre plano perpendicular al suelo y dirección de la rueda
    
    ext_steer(idx) = abs(steer_angle);
    ext_camber(idx) = camber_angle;

    ext_ratio(idx) = rad2deg(rack_disp/17.5)/ext_steer(idx);
   
    spindle_radius_final = rotated_vec(spindle_radius_vector_ini, kingpin_vector, theta_axis); % cálculo vector para jacking rotado   
    spindle_center_final = lower_knuckle + spindle_radius_final; % cálculo nueva posición centro de la rueda    
    ext_jacking(idx) = spindle_center_ini(3) - spindle_center_final(3); % jacking. Diferencia de altura entre 

    
    idx = idx + 1;
end

idx = 1;
int_steer = zeros(1, 100);
int_camber = zeros(1, 100);
int_jacking = zeros(1, 100); % Array para jacking interior

% Bucle Interior (Rack desplazándose en dirección opuesta)
for rack_disp = 0:-0.5:-50
    
    tie_rack_joint = tie_rack_joint_ini + [0, rack_disp, 0];
    
    cost_func = @(th) norm((lower_knuckle + rotated_vec(steerarm_vector_ini, kingpin_vector, th)) - tie_rack_joint) - l_tie;
    
    theta_axis = fzero(cost_func, 0);
    
    spindle_final = rotated_vec(spindle_vec_ini, kingpin_vector, theta_axis);
    
    steer_angle = atan2d(spindle_final(1), spindle_final(2));
    camber_angle = asind(spindle_final(3));
    
    int_steer(idx) = abs(steer_angle);
    int_camber(idx) = camber_angle;
    
    % --- CÁLCULO JACKING INT ---
    spindle_radius_final = rotated_vec(spindle_radius_vector_ini, kingpin_vector, theta_axis);
    spindle_center_final = lower_knuckle + spindle_radius_final;
    int_jacking(idx) = spindle_center_ini(3) - spindle_center_final(3);
    
    idx = idx + 1;
end

% Cálculos finales
dynamic_toe = int_steer - ext_steer;
dynamic_toe_ack = - atand(wheelbase ./ (wheelbase ./ tand(int_steer) + track)) + int_steer;
ack_pctge = dynamic_toe./dynamic_toe_ack;

% --- CÁLCULO JACKING TOTAL DEL EJE ---
% El Jacking total es la media del levantamiento de ambos lados
total_axle_jacking = (ext_jacking + int_jacking) / 2;

%% GRÁFICAS
figure('Name', 'Análisis de Dirección');

subplot(3,1,1)
plot(int_steer, ack_pctge * 100, 'LineWidth', 1.5)
grid on; title('Ackermann Percentage'); ylabel('%');

subplot(3,1,2)
plot(rad2deg((0:0.5:50)./17.5), ext_camber, 'r', rad2deg((0:0.5:50)./17.5), int_camber, 'b', 'LineWidth', 1.5)
legend('Ext Wheel', 'Int Wheel');
grid on; title('Camber Change'); ylabel('Deg');

subplot(3,1,3)
% Graficamos el Jacking individual y el total
plot(rad2deg((0:0.5:50)./17.5), ext_jacking, 'r--', 'LineWidth', 1)
hold on
plot(rad2deg((0:0.5:50)./17.5), int_jacking, 'b--', 'LineWidth', 1)
plot(rad2deg((0:0.5:50)./17.5), total_axle_jacking, 'k', 'LineWidth', 2)
legend('Ext Wheel Height', 'Int Wheel Height', 'Total Chassis Heave');
grid on; title('Geometric Jacking (Chassis Lift)'); 
xlabel('Inner Steer Angle [deg]'); ylabel('Vertical Disp [mm]');

figure(2)
plot(ext_steer, ext_ratio)


