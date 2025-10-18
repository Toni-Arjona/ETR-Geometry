function point = line_plane_intersection( line_point, line_direction, plane_normal, plane_D )
    arguments (Input)
        line_point v3
        line_direction v3
        plane_normal v3
        plane_D double
    end
    arguments (Output)
        point v3
    end

    line_direction = line_direction';

    lambda = -( plane_D + plane_normal*line_point )/( plane_normal*line_direction );

    point = line_point + lambda.*line_direction;

end