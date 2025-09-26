function projection = point_plane_projection(point, plane_dir, plane_D)
    arguments
        point v3
        plane_dir v3
        plane_D double
    end
    distance = (plane_dir*point + plane_D)/(plane_dir*plane_dir);
    projection = point - distance.*plane_dir;
end

