function point = point_plane_intersection(input_point, start, finale)
    arguments(Input)
        input_point v3
        start v3
        finale v3
    end
    arguments(Output)
        point v3
    end

    plane_dir = (finale - start)';
    plane_D = -(plane_dir*input_point);
    point = start + (plane_dir .* (-plane_D-(plane_dir*start)));
end