addpath('functions'); %NO TOCAR

%% Left Front Local Knuckle Definition
fl_knuckle_upper_connection = v3( -7.14000000, 75.00000000, 24.00000000 ); %TOCAR
fl_knuckle_lower_connection = v3( -1.00000000, -79.00000000, 22.00000000 ); %TOCAR
fl_knuckle_tierod_connection = v3( 62.83805199, -48.63592517, 23.00000000 ); %TOCAR
fl_knuckle_zero = v3(0,0,0); % NO TOCAR
fl_knuckle_X = v3(1,0,0); % NO TOCAR
fl_knuckle_Y = v3(0,1,0); % NO TOCAR
fl_knuckle_Z = v3(0,0,1); % NO TOCAR
fl_knuckle_wheel_normal = v3(0,0,-1); % NO TOCAR: Direction Outside The Car
fl_knuckle_wheel_centre = v3(0,0,-31.14105609); % SOLO TOCAR Z
fl_knuckle = solid([fl_knuckle_upper_connection, fl_knuckle_lower_connection, fl_knuckle_tierod_connection, fl_knuckle_zero, fl_knuckle_X, fl_knuckle_Y, fl_knuckle_Z, fl_knuckle_wheel_normal, fl_knuckle_wheel_centre]); %NO TOCAR






%% Left Front Damper Definition
fl_damper_support = v3( 130.0, -120.0, 551.0 ); %TOCAR
fl_damper_end = v3( -50.0, -114.0 , 555.0 ); %TOCAR
fl_damper = rod(fl_damper_support, fl_damper_end); %NO TOCAR

%% Left Front Rocker Definition
fl_rocker_arm1 = fl_damper_end;
fl_rocker_arm2 = v3( -90, -196, 503 );
fl_rocker_base1 = v3( -25, -166, 497 );
fl_rocker_base2 = v3( -25, -188, 532 );
fl_rocker = solid([fl_rocker_base1, fl_rocker_base2, fl_rocker_arm1, fl_rocker_arm2]);

%% Left Front Global Pushrod Definition
fl_pushrod_rocker_end = fl_rocker_arm2; %NO TOCAR
fl_pushrod_wishbone_end = v3( -93.0, -511.0, 300.0 ); %TOCAR
fl_pushrod = rod(fl_pushrod_rocker_end, fl_pushrod_wishbone_end); %NO TOCAR

%% Left Front Global Upper Wishbone Definition
fl_upper_wishbone_front_support = v3( -190.0 , -240.0 , 265.0); %TOCAR
fl_upper_wishbone_rear_support = v3( 20.0 , -244.0 , 256.0); %TOCAR
fl_upper_wishbone_end = v3( -105.5 , -549.0 , 285.0 ); %TOCAR
fl_upper_wishbone_pushrod_end = fl_pushrod_wishbone_end; %NO TOCAR
fl_upper_wishbone = solid([ fl_upper_wishbone_front_support, fl_upper_wishbone_rear_support, fl_upper_wishbone_end, fl_upper_wishbone_pushrod_end ]); %NO TOCAR

%% Left Front Global Lower Wishbone Definition
fl_lower_wishbone_front_support = v3( -154.0 , -239.0, 124.0  ); %TOCAR
fl_lower_wishbone_rear_support = v3( 66.0, -244.0 , 129.5 ); %TOCAR
fl_lower_wishbone_end = v3( -108.0, -576.0, 112.0 ); %TOCAR
fl_lower_wishbone = solid([fl_lower_wishbone_front_support, fl_lower_wishbone_rear_support, fl_lower_wishbone_end]); %NO TOCAR





fl_suspension = suspension( fl_damper, fl_rocker, fl_pushrod, fl_upper_wishbone, fl_knuckle, fl_lower_wishbone ); %NO TOCAR
fr_suspension = fl_suspension.mirror_on_plane( v3(0,1,0), 0);
save('car/car_variables/fl_suspension.mat', 'fl_suspension'); %NO TOCAR
save('car/car_variables/fr_suspension.mat', 'fr_suspension');
clear; %NO TOCAR
fprintf("fl_suspension saved at car/car_variables/fl_suspension.mat\n"); %NO TOCAR
fprintf("fr_suspension saved at car/car_variables/fr_suspension.mat\n"); %NO TOCAR