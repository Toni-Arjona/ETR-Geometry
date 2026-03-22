function th = th_solver(v, k, offset, target, L, theta_prev)

    a = dot(k,v)*k;
    r = v - a;
    s = cross(k,v); 

    q = -target + offset + a;

    B = dot(q, r);                     
    C = dot(q, s);                          
    D = (L^2 - norm(r)^2 - norm(q)^2) / 2;

    ratio = D / sqrt(B^2 + C^2);
    ratio = max(-1.0, min(1.0, ratio));

    th_plus = atan2(C, B) + acos(ratio);
    th_minus = atan2(C, B) - acos(ratio);

    th_plus_error = abs(th_plus - theta_prev);
    th_minus_error = abs(th_minus - theta_prev);

    if th_minus_error >= th_plus_error
        th = th_plus;

    else
        th = th_minus;

    end
end
