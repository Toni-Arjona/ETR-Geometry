function coordinate = point_line_projection(point,line_direction, line_origin)
    arguments
        point v3
        line_direction v3
        line_origin v3
    end
    line_direction = line_direction';
    ap = point-line_origin;
    coordinate = line_origin + (ap*line_direction).*line_direction;

end

