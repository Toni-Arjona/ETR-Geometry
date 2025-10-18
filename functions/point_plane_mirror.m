function mirror_point = point_plane_mirror(input_point, plane_direction, plane_D)
    arguments (Input)
        input_point v3
        plane_direction v3
        plane_D double
    end
    arguments (Output)
        mirror_point v3
    end

    plane_direction = plane_direction';

    lambda = -(plane_D + plane_direction*input_point)/(plane_direction*plane_direction);
    mirror_point = input_point + (2*lambda).*plane_direction;

end

