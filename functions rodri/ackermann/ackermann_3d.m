clear all
close all

wheelbase = 1535;
track = 1250;

spindle_vec_ini = ([-100, -561.5, 200.5] - [-100, -625, 203])/ norm([-100, -561.5, 200.5] - [-100, -625 , 203]);
upper_knuckle = [-89, -549, 249]; 
lower_knuckle = [-105 ,-576, 111]; % referencia para cálculo Rodrigues

kingpin_vector = (upper_knuckle - lower_knuckle)/norm(lower_knuckle - upper_knuckle); % vector kingpin normalizado

tie_rack_joint_ini = [-180, -225, 146.71];
tie_upright_joint_ini = [-170, -580, 133];

l_tie = norm(tie_upright_joint_ini - tie_rack_joint_ini); % longitud tie

steerarm_vector_ini = tie_upright_joint_ini - lower_knuckle;

rodrigues_rot = @(v, k, th) v*cos(th) + cross(k, v)*sin(th) + k*dot(k, v)*(1 - cos(th)); % v es el vector que rota (steerarm_vec), k el eje (kingpin_vec), y th el ángulo.

idx = 1;
ext_steer = zeros(1, 100);
ext_camber = zeros(1, 100);

for rack_disp = 0:0.5:50
    
    tie_rack_joint = tie_rack_joint_ini + [0, rack_disp, 0];
    
    cost_func = @(th) norm((lower_knuckle + rodrigues_rot(steerarm_vector_ini, kingpin_vector, th)) - tie_rack_joint) - l_tie;
    
    theta_axis = fzero(cost_func, 0);
    
    spindle_final = rodrigues_rot(spindle_vec_ini, kingpin_vector, theta_axis);
    
    steer_angle = atan2d(spindle_final(1), spindle_final(2));
    
    camber_angle = -asind(spindle_final(3));

    ext_steer(idx) = abs(steer_angle);
    ext_camber(idx) = camber_angle;
    idx = idx + 1;

end


idx = 1;
int_steer = zeros(1, 100);
int_camber = zeros(1, 100);

for rack_disp = 0:-0.5:-50
    
    tie_rack_joint = tie_rack_joint_ini + [0, rack_disp, 0];
    
    cost_func = @(th) norm((lower_knuckle + rodrigues_rot(steerarm_vector_ini, kingpin_vector, th)) - tie_rack_joint) - l_tie;
    
    theta_axis = fzero(cost_func, 0);
    
    spindle_final = rodrigues_rot(spindle_vec_ini, kingpin_vector, theta_axis);
    
    steer_angle = atan2d(spindle_final(1), spindle_final(2));
    
    camber_angle = -asind(spindle_final(3));

    int_steer(idx) = abs(steer_angle);
    int_camber(idx) = camber_angle;
    idx = idx + 1;

end

ext_steer;
int_steer;

ext_camber;
int_camber;


dynamic_toe = int_steer - ext_steer;
dynamic_toe_ack = - atand(wheelbase ./ (wheelbase ./ tand(int_steer) + track)) + int_steer;

ack_pctge = dynamic_toe./dynamic_toe_ack

plot(int_steer, ack_pctge)


%% MOSTRAR RESULTADOS
%fprintf('Desplazamiento Rack: %.2f mm\n', rack_disp)
%fprintf('Giro en eje Kingpin: %.4f deg\n', rad2deg(theta_axis))
%fprintf('Steer Real (Suelo):  %.4f deg\n', steer_angle)
%fprintf('Camber Real:         %.4f deg\n', camber_angle)
