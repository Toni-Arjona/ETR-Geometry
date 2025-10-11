addpath('functions'); %NO TOCAR

% Left Front Damper Definition
fl_damper_support = v3( 7.20000000, -25.00000000, 596.36000000 ); %TOCAR
fl_damper_end = v3( 7.20000000, -233.98672712, 616.96455992 ); %TOCAR
fl_damper = rod(fl_damper_support, fl_damper_end); %NO TOCAR

% Left Front Global Rocker Definition
fl_rocker_centre = v3( 7.20000000, -210.21062969, 540.57937031 ); %TOCAR
fl_rocker_arm1 = fl_damper_end; %NO TOCAR
fl_rocker_arm2 = v3(7.20000000, -257.81582072, 584.83709039 ); %TOCAR
fl_rocker = rocker(fl_rocker_centre, fl_rocker_arm1, fl_rocker_arm2); %NO TOCAR

% Left Front Global Pushrod Definition
fl_pushrod_rocker_end = fl_rocker_arm2; %NO TOCAR
fl_pushrod_wishbone_end = v3( 7.09376568, -562.66290500, 292.40701530 ); %TOCAR
fl_pushrod = rod(fl_pushrod_rocker_end, fl_pushrod_wishbone_end); %NO TOCAR

% Left Front Global Upper Wishbone Definition
fl_upper_wishbone_front_support = v3( -103.99993772, -243.65643182, 250.00000000 ); %TOCAR
fl_upper_wishbone_rear_support = v3( 105.15004960, -247.30715843, 250.00000000 ); %TOCAR
fl_upper_wishbone_end = v3( 7.13100377, -586.60191607, 268.21642199 ); %TOCAR
fl_upper_wishbone_pushrod_end = fl_pushrod_wishbone_end; %NO TOCAR
fl_upper_wishbone = solid([ fl_upper_wishbone_front_support, fl_upper_wishbone_rear_support, fl_upper_wishbone_end, fl_upper_wishbone_pushrod_end ]); %NO TOCAR

% Left Front Local Knuckle Definition
fl_knuckle_upper_connection = v3( -7.14000000, 75.00000000, 24.00000000 ); %TOCAR
fl_knuckle_lower_connection = v3( -1.00000000, -79.00000000, 22.00000000 ); %TOCAR
fl_knuckle_tierod_connection = v3( 62.83805199, -48.63592517, 23.00000000 ); %TOCAR
fl_knuckle_zero = v3(0,0,0); % NO TOCAR
fl_knuckle_X = v3(1,0,0); % NO TOCAR
fl_knuckle_Y = v3(0,1,0); %NO TOCAR
fl_knuckle_Z = v3(0,0,1); %NO TOCAR
fl_knuckle_wheel_normal = v3(0,0,-1); %NO TOCAR
fl_knuckle_wheel_centre = v3(0,0,-31.14105609); %SOLO TOCAR Z
fl_knuckle = solid([fl_knuckle_upper_connection, fl_knuckle_lower_connection, fl_knuckle_tierod_connection, fl_knuckle_zero, fl_knuckle_X, fl_knuckle_Y, fl_knuckle_Z, fl_knuckle_wheel_normal, fl_knuckle_wheel_centre]); %NO TOCAR

% Left Front Global Lower Wishbone Definition
fl_lower_wishbone_front_support = v3( -112.37132168, -242.61317619, 148.01276501 ); %TOCAR
fl_lower_wishbone_rear_support = v3( 109.12614768, -247.52019236, 154.45816341 ); %TOCAR
fl_lower_wishbone_end = v3( 1.80217124, -587.54165026, 114.17477164 ); %TOCAR
fl_lower_wishbone = solid([fl_lower_wishbone_front_support, fl_lower_wishbone_rear_support, fl_lower_wishbone_end]); %NO TOCAR

fl_suspension = suspension( fl_damper, fl_rocker, fl_pushrod, fl_upper_wishbone, fl_knuckle, fl_lower_wishbone ); %NO TOCAR
save('car/car_variables/fl_suspension.mat', 'fl_suspension'); %NO TOCAR
clear; %NO TOCAR
fprintf("fl_suspension saved at car/car_variables/fl_suspension.mat\n"); %NO TOCAR