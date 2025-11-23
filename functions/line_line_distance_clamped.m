function d = line_line_distance_clamped(line1_point1,line1_point2, line2_point1, line2_point2)
arguments
    line1_point1 v3
    line1_point2 v3
    line2_point1 v3
    line2_point2 v3
end

    [p1,p2] = line_line_closest_positions_clamped(line1_point1,line1_point2, line2_point1, line2_point2);

    d = (p1 - p2).';
end
