
%% Left Front Local Knuckle Definition
fl_knuckle_upper_connection = v3( -20.00000000,	   95.10844307	,   37.66807743); %TOCAR
fl_knuckle_lower_connection = v3( -3.50000000,	  -88.81282952	,   -5.89756830); %TOCAR
fl_knuckle_tierod_connection = v3(  70.00000000,	  -60.38914979	,    3.95608232 ); %TOCAR
fl_knuckle_zero = v3(0,0,0); % NO TOCAR
fl_knuckle_X = v3(1,0,0); % NO TOCAR
fl_knuckle_Y = v3(0,1,0); % NO TOCAR
fl_knuckle_Z = v3(0,0,1); % NO TOCAR
fl_knuckle_wheel_normal = v3(0,0,-1); % NO TOCAR: Direction Outside The Car
fl_knuckle_wheel_centre = v3(0,0,-61.551); % SOLO TOCAR Z
fl_knuckle = solid([fl_knuckle_upper_connection, fl_knuckle_lower_connection, fl_knuckle_tierod_connection, fl_knuckle_zero, fl_knuckle_X, fl_knuckle_Y, fl_knuckle_Z, fl_knuckle_wheel_normal, fl_knuckle_wheel_centre]); %NO TOCAR






%% Left Front Damper Definition
fl_damper_support = v3( 164.0, -112.0, 610.0 ); %TOCAR
fl_damper_end = v3( -51.0, -112.0 , 610.0 ); %TOCAR
fl_damper = rod(fl_damper_support, fl_damper_end); %NO TOCAR

%% Left Front Rocker Definition
fl_rocker_arm1 = fl_damper_end;
fl_rocker_arm2 = v3( -89, -193.5, 558 );
fl_rocker_base1 = v3( -26, -164, 552 );
fl_rocker_base2 = v3( -26, -186.5, 587 );
fl_rocker = solid([fl_rocker_base1, fl_rocker_base2, fl_rocker_arm1, fl_rocker_arm2]);

%% Left Front Global Pushrod Definition
fl_pushrod_rocker_end = fl_rocker_arm2; %NO TOCAR
fl_pushrod_wishbone_end = v3( -89.0, -523.0, 305.0 ); %TOCAR
fl_pushrod = rod(fl_pushrod_rocker_end, fl_pushrod_wishbone_end); %NO TOCAR

%% Left Front Global Upper Wishbone Definition
fl_upper_wishbone_front_support = v3( -210.0 , -225.0 , 277.0); %TOCAR
fl_upper_wishbone_rear_support = v3( 80.0 , -225.0 , 268.0); %TOCAR
fl_upper_wishbone_end = v3(-80, -520 , 294 ); %TOCAR
fl_upper_wishbone_pushrod_end = fl_pushrod_wishbone_end; %NO TOCAR
fl_upper_wishbone = solid([ fl_upper_wishbone_front_support, fl_upper_wishbone_rear_support, fl_upper_wishbone_end, fl_upper_wishbone_pushrod_end ]); %NO TOCAR

%% Left Front Global Lower Wishbone Definition
fl_lower_wishbone_front_support = v3( -210.0 , -225.0, 123.0  ); %TOCAR
fl_lower_wishbone_rear_support = v3( 85, -225, 136); %TOCAR
fl_lower_wishbone_end = v3(-96.5, -571, 112); %TOCAR
fl_lower_wishbone = solid([fl_lower_wishbone_front_support, fl_lower_wishbone_rear_support, fl_lower_wishbone_end]); %NO TOCAR



fl_suspension = suspension( fl_damper, fl_rocker, fl_pushrod, fl_upper_wishbone, fl_knuckle, fl_lower_wishbone ); %NO TOCAR
fl_suspension.set_damper_distance(180);
fr_suspension = fl_suspension.mirror_on_plane( v3(0,1,0), 0);
save('car_variables/fl_suspension.mat', 'fl_suspension'); %NO TOCAR
save('car_variables/fr_suspension.mat', 'fr_suspension');
clear; %NO TOCAR
fprintf("fl_suspension saved at car/car_variables/fl_suspension.mat\n"); %NO TOCAR
fprintf("fr_suspension saved at car/car_variables/fr_suspension.mat\n"); %NO TOCAR