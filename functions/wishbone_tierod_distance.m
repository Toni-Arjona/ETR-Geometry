function d = wishbone_tierod_distance(u_wish_obj,l_wish_obj, knuckle_tierod_coord, clevi_coord)
arguments
    u_wish_obj solid
    l_wish_obj solid
    knuckle_tierod_coord v3
    clevi_coord v3
end

    d1 = line_line_distance_clamped(u_wish_obj.coord(1), u_wish_obj.coord(3), knuckle_tierod_coord, clevi_coord);
    d2 = line_line_distance_clamped(u_wish_obj.coord(2), u_wish_obj.coord(3), knuckle_tierod_coord, clevi_coord);
    d3 = line_line_distance_clamped(l_wish_obj.coord(1), l_wish_obj.coord(3), knuckle_tierod_coord, clevi_coord);
    d4 = line_line_distance_clamped(l_wish_obj.coord(2), l_wish_obj.coord(3), knuckle_tierod_coord, clevi_coord);

    d = min([d1, d2, d3, d4]);
end

