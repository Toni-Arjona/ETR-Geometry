function [instant_axis, roll_center]= rollcenter_instantaxis(suspl, suspr)
    arguments
    suspl suspension
    suspr suspension
    end
    
    % Plane Geometry
    plane_dir_l_u= ((suspl.u_wishbone.coord(1)-suspl.u_wishbone.coord(2))^(suspl.u_wishbone.coord(1)-suspl.u_wishbone.coord(3)))';
    plane_D_l_u = -(plane_dir_l_u*suspl.u_wishbone.coord(1)); %Left upperwhisbone plane
    
    plane_dir_r_u= ((suspr.u_wishbone.coord(1)-suspr.u_wishbone.coord(2))^(suspr.u_wishbone.coord(1)-suspr.u_wishbone.coord(3)))';
    plane_D_r_u = -(plane_dir_r_u*suspr.u_wishbone.coord(1)); %Right upperwhisbone plane

    plane_dir_l_l= ((suspl.l_wishbone.coord(1)-suspl.l_wishbone.coord(2))^(suspl.l_wishbone.coord(1)-suspl.l_wishbone.coord(3)))';
    plane_D_l_l= -(plane_dir_l_l*suspr.l_wishbone.coord(1)); %Left lowerwhisbone plane

    plane_dir_r_l= ((suspr.l_wishbone.coord(1)-suspr.l_wishbone.coord(2))^(suspr.l_wishbone.coord(1)-suspr.l_wishbone.coord(3)))';
    plane_D_r_l = -(plane_dir_r_l*suspr.l_wishbone.coord(1)); %Right upperwhisbone plane

    front_plane_wheel= -(v3(-1,0,0)*suspl.knuckle.coord(9)); %The 9th coordinate of the knuckle is the wheel center
    
    % Instant axis, upper-/ lowerwhisbone plane intersection
    instant_axis_left= plane_dir_l_u^plane_dir_l_l;% intersection line between 2 planes: v= n1^n2 (n1 and n2 being the normal vectors of the plane)
    instant_axis_right = plane_dir_r_u^plane_dir_r_l;
    instant_axis= [instant_axis_left, instant_axis_right];
    
    % Roll center, point intersection between axis projected
    point_intersection_instant= instant_axis_left*instant_axis_right;
    roll_center= point_plane_projection(point_intersection_instant, v3(-1,0,0), front_plane_wheel);
end