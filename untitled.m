clear;
clc;
addpath('functions');
format long

% Damper Definition
damper_support = v3( 7.20000000, -25.00000000, 596.36000000 );
damper_end = v3( 7.20000000, -233.98672712, 616.96455992 );
damper = rod(damper_support, damper_end);

% Global Rocker Definition
rocker_centre = v3( 7.20000000, -210.21062969, 540.57937031 );
rocker_arm1 = damper_end;
rocker_arm2 = v3(7.20000000, -257.81582072, 584.83709039 );
rocker = rocker(rocker_centre, rocker_arm1, rocker_arm2);

% Global Pushrod Definition
pushrod_rocker_end = rocker_arm2;
pushrod_wishbone_end = v3( 7.09376568, -562.66290500, 292.40701530 );
pushrod = rod(pushrod_rocker_end, pushrod_wishbone_end);

% Global Upper Wishbone Definition
upper_wishbone_front_support = v3( -103.99993772, -243.65643182, 250.00000000 );
upper_wishbone_rear_support = v3( 105.15004960, -247.30715843, 250.00000000 );
upper_wishbone_end = v3( 7.13100377, -586.60191607, 268.21642199 );
upper_wishbone_pushrod_end = pushrod_wishbone_end;
upper_wishbone = solid([ upper_wishbone_front_support, upper_wishbone_rear_support, upper_wishbone_end, upper_wishbone_pushrod_end ]);

% Local Knuckle Definition
knuckle_upper_connection = v3( -7.14000000, 75.00000000, 24.10000000  );
knuckle_lower_connection = v3( -1.00000000, -79.00000000, 22.00000000 );
knuckle_tierod_connection = v3( 62.83805199, -48.63592517, 23.00000000 );
knuckle_X = v3(0,0,0);
knuckle_Y = v3(0,0,0);
knuckle_Z = v3(0,0,0);
knuckle = solid([knuckle_upper_connection, knuckle_lower_connection, knuckle_tierod_connection, knuckle_X, knuckle_Y, knuckle_Z]);

% Global Lower Wishbone Definition
lower_wishbone_front_support = v3( -112.37132168, -242.61317619, 148.01276501 );
lower_wishbone_rear_support = v3( 109.12614768, -247.52019236, 154.45816341 );
lower_wishbone_end = v3( 1.80217124, -587.54165026, 114.17477164 );
lower_wishbone = solid([lower_wishbone_front_support, lower_wishbone_rear_support, lower_wishbone_end]);



left_front_suspension = suspension(damper, rocker, pushrod, upper_wishbone, knuckle, lower_wishbone);

left_front_suspension.print();
left_front_suspension.set_damper_distance(180);
left_front_suspension.print();




