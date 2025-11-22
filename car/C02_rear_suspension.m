addpath('functions'); %NO TOCAR

%% Left Rear Local Knuckle Definition
rl_knuckle_upper_connection = v3(   0.00000000,	  85.80820547,	   -3.80436827); %TOCAR
rl_knuckle_lower_connection = v3(   0.00000000,	  -87.78702810,	  -23.65241301); %TOCAR
rl_knuckle_tierod_connection = v3(-130.00000000,	  -82.87594642,	   10.22798363 ); %TOCAR
rl_knuckle_zero = v3(0,0,0); % NO TOCAR
rl_knuckle_X = v3(1,0,0); % NO TOCAR
rl_knuckle_Y = v3(0,1,0); %NO TOCAR
rl_knuckle_Z = v3(0,0,1); %NO TOCAR
rl_knuckle_wheel_normal = v3(0,0,-1); %NO TOCAR
rl_knuckle_wheel_centre = v3(0,0,-70.02522506); %SOLO TOCAR Z
rl_knuckle = solid([rl_knuckle_upper_connection, rl_knuckle_lower_connection, rl_knuckle_tierod_connection, rl_knuckle_zero, rl_knuckle_X, rl_knuckle_Y, rl_knuckle_Z, rl_knuckle_wheel_normal, rl_knuckle_wheel_centre]); %NO TOCAR






%% Left Rear Damper Definition
rl_damper_support = v3(1257, -160, 516.4536); %TOCAR
rl_damper_end = v3(1437, -161, 517.4536); %TOCAR
rl_damper = rod(rl_damper_support, rl_damper_end); %NO TOCAR

%% Left Rear Global Rocker Definition
rl_rocker_arm1 = rl_damper_end;
rl_rocker_arm2 = v3( 1513, -206, 485.4537 ); % Rocker end
rl_rocker_base1 = v3( 1451, -205, 461.4537 );
rl_rocker_base2 = v3( 1451, -228, 495.4537 );
rl_rocker = solid([rl_rocker_base1, rl_rocker_base2, rl_rocker_arm1, rl_rocker_arm2]);

%% Left Rear Global Pushrod Definition
rl_pushrod_rocker_end = rl_rocker_arm2; %NO TOCAR
rl_pushrod_wishbone_end = v3( 1502, -507, 310.4537 ); %TOCAR
rl_pushrod = rod(rl_pushrod_rocker_end, rl_pushrod_wishbone_end); %NO TOCAR

%% Left Rear Global Upper Wishbone Definition
rl_upper_wishbone_front_support = v3( 1330, -257, 276.4536 ); %TOCAR
rl_upper_wishbone_rear_support = v3( 1610, -257, 267.4536 ); %TOCAR
rl_upper_wishbone_end = v3( 1500, -551, 285.4537 ); %TOCAR
rl_upper_wishbone_pushrod_end = rl_pushrod_wishbone_end; %NO TOCAR
rl_upper_wishbone = solid([ rl_upper_wishbone_front_support, rl_upper_wishbone_rear_support, rl_upper_wishbone_end, rl_upper_wishbone_pushrod_end ]); %NO TOCAR

%% Left Rear Global Lower Wishbone Definition
rl_lower_wishbone_front_support = v3( 1315, -255, 146.4536); %TOCAR
rl_lower_wishbone_rear_support = v3( 1610, -255, 123.0000); %TOCAR
rl_lower_wishbone_end = v3( 1500, -576, 112.4537 ); %TOCAR
rl_lower_wishbone = solid([rl_lower_wishbone_front_support, rl_lower_wishbone_rear_support, rl_lower_wishbone_end]); %NO TOCAR

%%
rl_suspension = suspension( rl_damper, rl_rocker, rl_pushrod, rl_upper_wishbone, rl_knuckle, rl_lower_wishbone ); %NO TOCAR
rl_suspension.set_damper_distance(180);
rr_suspension = rl_suspension.mirror_on_plane( v3(0,1,0), 0);
save('car/car_variables/rl_suspension.mat', 'rl_suspension'); %NO TOCAR
save('car/car_variables/rr_suspension.mat', 'rr_suspension');
clear; %NO TOCAR
fprintf("rl_suspension saved at car/car_variables/rl_suspension.mat\n"); %NO TOCAR
fprintf("rr_suspension saved at car/car_variables/rr_suspension.mat\n"); %NO TOCAR