clear;
clc;
addpath('functions');
format long;


a = v3(1,1,1);
b = v3(1,1,-1);
c = v3(1,-1,1);
d = v3(1,-1,-1);
e = v3(-1,1,1);
f = v3(-1,1,-1);
g = v3(-1,-1,1);
h = v3(-1,-1,-1);
i = v3(0,0,0);
j = v3(0,0,1);
k = v3(0,0,-1);

s = solid([a,b,c,d,e,f,g,h,i,j,k]);
s.print();

fprintf("------------------------------\n");
s.setDirection( 10, 11, 2, 4, v3(1,0,0) );
s.print();

fl_knuckle_upper_connection = v3( -7.14000000, 75.00000000, 24.10000000 ); %TOCAR
fl_knuckle_lower_connection = v3( -1.00000000, -79.00000000, 22.00000000 ); %TOCAR
fl_knuckle_tierod_connection = v3( 62.83805199, -48.63592517, 23.00000000 ); %TOCAR
fl_knuckle_zero = v3(0,0,0); % NO TOCAR
fl_knuckle_X = v3(1,0,0); % NO TOCAR
fl_knuckle_Y = v3(0,1,0); %NO TOCAR
fl_knuckle_Z = v3(0,0,1); %NO TOCAR
fl_knuckle_wheel_normal = v3(0,0,-1); %NO TOCAR
fl_knuckle_wheel_centre = v3(0,0,-31.14105609); %SOLO TOCAR Z
fl_knuckle = solid([fl_knuckle_upper_connection, fl_knuckle_lower_connection, fl_knuckle_tierod_connection, fl_knuckle_zero, fl_knuckle_X, fl_knuckle_Y, fl_knuckle_Z, fl_knuckle_wheel_normal, fl_knuckle_wheel_centre]); %NO TOCAR

direction = v3(1,0,0);
normal = (fl_knuckle.coord(2) - fl_knuckle.coord(1))';
plane_D = -(normal*fl_knuckle.coord(4));
protation_centre = point_plane_projection(fl_knuckle.coord(1), normal, plane_D);
rotation_centre = point_plane_intersection(fl_knuckle.coord(4), fl_knuckle.coord(1), fl_knuckle.coord(2));
pdirection_on_plane = line_plane_intersection(rotation_centre+direction, v3(0,1,0), normal, plane_D);
direction_on_plane = (pdirection_on_plane - rotation_centre)';

fl_knuckle.setDirection(1,2, 4, 9, direction_on_plane);
q = 0;