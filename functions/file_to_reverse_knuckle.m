clear; clc;
%% Front Knuckle
% Global coordinates definition
global_upper_wishbone = v3(-105.5, -549.00 , 285 ); 
global_lower_wishbone = v3(-108, -576, 112);
global_tierod_position = v3(-168, -579, 122);
global_knuckle_centre = v3(-100, -561, 200);
global_wheel_centre = v3(-100, -623, 202);


global_knuckle_direction = (global_wheel_centre - global_knuckle_centre)';
standard_knuckle_direction = v3(0,0,-1);
R = rotation_matrix(global_knuckle_direction, standard_knuckle_direction); % Computation of rotation matrix

% Linear displacement
local_upper_wishbone = global_upper_wishbone - global_knuckle_centre;
local_lower_wishbone = global_lower_wishbone - global_knuckle_centre;
local_tierod_position = global_tierod_position - global_knuckle_centre;
local_knuckle_centre = global_knuckle_centre - global_knuckle_centre;
local_wheel_centre = global_wheel_centre - global_knuckle_centre;

% Aplication of rotation matrix
local_upper_wishbone = local_upper_wishbone.apply_rotation(R);
local_lower_wishbone = local_lower_wishbone.apply_rotation(R);
local_tierod_position = local_tierod_position.apply_rotation(R);
local_wheel_centre = local_wheel_centre.apply_rotation(R);

knf = solid([local_upper_wishbone, local_lower_wishbone, local_tierod_position, local_knuckle_centre, local_wheel_centre]);
knf = knf.mirror_on_plane(v3(0,1,0), 0);
knf = knf.mirror_on_plane(v3(1,0,0), 0);
% Definition of knucke
fprintf("--- Front Knuckle Local Coordinates ---\n");
fprintf("Local Front-Knuckle Upper Wishbone Coordinate:\t");knf.coord(1).print();
fprintf("Local Front-Knuckle Lower Wishbone Coordinate:\t");knf.coord(2).print();
fprintf("Local Front-Knuckle Tierod Coordinate:\t\t\t"); knf.coord(3).print();
fprintf("Local Front-Knuckle CAD Centre Coordinate:\t\t"); knf.coord(4).print();
fprintf("Local Front-Knuckle Wheel Centre Coordinate:\t"); knf.coord(5).print();

knf.alone_plot3d();



%% Rear Knuckle
% Global coordinates definition
global_upper_wishbone = v3(1500, -551.5 , 285.4537 ); 
global_lower_wishbone = v3(1500, -576, 112.4537);
global_tierod_position = v3(1630, -542, 116.4537);
global_knuckle_centre = v3(1500, -550, 199.5743);
global_wheel_centre = v3(1500, -620, 201.4537);


global_knuckle_direction = (global_wheel_centre - global_knuckle_centre)';
standard_knuckle_direction = v3(0,0,-1);
R = rotation_matrix(global_knuckle_direction, standard_knuckle_direction); % Computation of rotation matrix

% Linear displacement
local_upper_wishbone = global_upper_wishbone - global_knuckle_centre;
local_lower_wishbone = global_lower_wishbone - global_knuckle_centre;
local_tierod_position = global_tierod_position - global_knuckle_centre;
local_knuckle_centre = global_knuckle_centre - global_knuckle_centre;
local_wheel_centre = global_wheel_centre - global_knuckle_centre;

% Aplication of rotation matrix
local_upper_wishbone = local_upper_wishbone.apply_rotation(R);
local_lower_wishbone = local_lower_wishbone.apply_rotation(R);
local_tierod_position = local_tierod_position.apply_rotation(R);
local_wheel_centre = local_wheel_centre.apply_rotation(R);

knr = solid([local_upper_wishbone, local_lower_wishbone, local_tierod_position, local_knuckle_centre, local_wheel_centre]);
knr = knr.mirror_on_plane(v3(0,1,0), 0);
knr = knr.mirror_on_plane(v3(1,0,0), 0);
% Definition of knucke
fprintf("--- Rear Knuckle Local Coordinates ---\n");
fprintf("Local Rear-Knuckle Upper Wishbone Coordinate:\t");knr.coord(1).print();
fprintf("Local Rear-Knuckle Lower Wishbone Coordinate:\t");knr.coord(2).print();
fprintf("Local Rear-Knuckle Tierod Coordinate:\t\t\t"); knr.coord(3).print();
fprintf("Local Rear-Knuckle CAD Centre Coordinate:\t\t"); knr.coord(4).print();
fprintf("Local Rear-Knuckle Wheel Centre Coordinate:\t\t"); knr.coord(5).print();

knr.alone_plot3d();

