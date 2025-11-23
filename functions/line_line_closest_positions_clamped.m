function [p1,p2] = line_line_closest_positions_clamped(line1_point1,line1_point2, line2_point1, line2_point2)
arguments
    line1_point1 v3
    line1_point2 v3
    line2_point1 v3
    line2_point2 v3
end
    % Formula del chat...
    u = line1_point2 - line1_point1;
    v = line2_point2 - line2_point1;
    w = line1_point1 - line2_point1;
    a = u*u;
    b = u*v;
    c = v*v;
    d = u*w;
    e = v*w;
    D = (a*c - b^2);

    s_prima = (b*e - c*d)/D;
    t_prima = (a*e - b*d)/D;

    s = min( max(s_prima, 0), 1 );
    t = min( max(t_prima, 0), 1 );

    p1 = line1_point1 + s.*u;
    p2 = line2_point1 + t.*v;
end

