function direction = line_plane_projection(line_direction, plane_normal)
arguments
    line_direction v3 
    plane_normal v3
end
    line_direction = line_direction';
    plane_normal = plane_normal';
    if( (line_direction ^ plane_normal).' < 1e-17)
        fprintf("Vectores paralelos!\n");
    end

    % https://www.euclideanspace.com/maths/geometry/elements/plane/lineOnPlane/index.htm
    direction = plane_normal ^ (line_direction ^ plane_normal);
end

