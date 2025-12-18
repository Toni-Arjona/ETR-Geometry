function d = line_line_distance(point1, direction1, point2, direction2)
arguments
    point1 v3
    direction1 v3 
    point2 v3
    direction2 v3 
end

    direction1 = direction1';
    direction2 = direction2';

    a1 = point2 - point1;
    a2 = direction1 ^ direction2;
    d = (a1 * a2)/a2.';
end

