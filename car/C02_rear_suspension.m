

%% Left Rear Local Knuckle Definition
rl_knuckle_upper_connection = v3(   0.00000000,	  83.39326104,	   9.52963865); %TOCAR
rl_knuckle_lower_connection = v3(   0.00000000,	  -90.37095526,	  -8.78011651); %TOCAR
rl_knuckle_tierod_connection = v3(-130.00000000,	  -82.16190324,	  24.94837983); %TOCAR
rl_knuckle_zero = v3(0,0,0); % NO TOCAR
rl_knuckle_X = v3(1,0,0); % NO TOCAR
rl_knuckle_Y = v3(0,1,0); %NO TOCAR
rl_knuckle_Z = v3(0,0,1); %NO TOCAR
rl_knuckle_wheel_normal = v3(0,0,-1); %NO TOCAR
rl_knuckle_wheel_centre = v3(0,0,-56.03570290); %SOLO TOCAR Z
rl_knuckle = solid([rl_knuckle_upper_connection, rl_knuckle_lower_connection, rl_knuckle_tierod_connection, rl_knuckle_zero, rl_knuckle_X, rl_knuckle_Y, rl_knuckle_Z, rl_knuckle_wheel_normal, rl_knuckle_wheel_centre]); %NO TOCAR






%% Left Rear Damper Definition
rl_damper_support = v3(1244, -161, 518); %TOCAR
rl_damper_end = v3(1424.5, -161, 518); %TOCAR
rl_damper = rod(rl_damper_support, rl_damper_end); %NO TOCAR

%% Left Rear Global Rocker Definition
rl_rocker_arm1 = rl_damper_end;
rl_rocker_arm2 = v3( 1500, -206, 487 ); % Rocker end
rl_rocker_base1 = v3( 1438, -205, 463 );
rl_rocker_base2 = v3( 1438, -228, 497 );
rl_rocker = solid([rl_rocker_base1, rl_rocker_base2, rl_rocker_arm1, rl_rocker_arm2]);

%% Left Rear Global Pushrod Definition
rl_pushrod_rocker_end = rl_rocker_arm2; %NO TOCAR
rl_pushrod_wishbone_end = v3( 1500, -517, 316.5 ); %TOCAR
rl_pushrod = rod(rl_pushrod_rocker_end, rl_pushrod_wishbone_end); %NO TOCAR

%% Left Rear Global Upper Wishbone Definition
rl_upper_wishbone_front_support = v3( 1330, -255, 281 ); %TOCAR
rl_upper_wishbone_rear_support = v3( 1610, -255, 268 ); %TOCAR
rl_upper_wishbone_end = v3( 1500, -551.5, 284 ); %TOCAR
rl_upper_wishbone_pushrod_end = rl_pushrod_wishbone_end; %NO TOCAR
rl_upper_wishbone = solid([ rl_upper_wishbone_front_support, rl_upper_wishbone_rear_support, rl_upper_wishbone_end, rl_upper_wishbone_pushrod_end ]); %NO TOCAR

%% Left Rear Global Lower Wishbone Definition
rl_lower_wishbone_front_support = v3( 1315, -255, 149); %TOCAR
rl_lower_wishbone_rear_support = v3( 1610, -255, 123); %TOCAR
rl_lower_wishbone_end = v3( 1500, -576, 111 ); %TOCAR
rl_lower_wishbone = solid([rl_lower_wishbone_front_support, rl_lower_wishbone_rear_support, rl_lower_wishbone_end]); %NO TOCAR

%%
rl_suspension = suspension( rl_damper, rl_rocker, rl_pushrod, rl_upper_wishbone, rl_knuckle, rl_lower_wishbone ); %NO TOCAR
rl_suspension.set_damper_distance(180);
rr_suspension = rl_suspension.mirror_on_plane( v3(0,1,0), 0);
save('car_variables/rl_suspension.mat', 'rl_suspension'); %NO TOCAR
save('car_variables/rr_suspension.mat', 'rr_suspension');
clear; %NO TOCAR
fprintf("rl_suspension saved at car/car_variables/rl_suspension.mat\n"); %NO TOCAR
