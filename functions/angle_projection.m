function angle = angle_projection(angle_rad, original_plane_direction, new_plane_direction)
    arguments (Input)
        angle_rad double
        original_plane_direction v3
        new_plane_direction v3
    end
    arguments (Output)
        angle double
    end
    if(original_plane_direction.' < 1e-20) %#ok<BDSCA>
        angle = angle_rad;
        return
    end

    original_plane_direction = original_plane_direction';
    new_plane_direction = new_plane_direction';
    angle_between_planes = acos( (original_plane_direction*new_plane_direction)/((original_plane_direction.')*(new_plane_direction.')) );
    angle = atan( tan(angle_rad) * cos(angle_between_planes) );
end
