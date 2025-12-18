function R = rotation_matrix(direction1, direction2)
arguments
    direction1 v3
    direction2 v3
end
    direction1 = direction1';
    direction2 = direction2';

    v = direction1 ^ direction2; %Rotation axis
    s = v.';
    cos = direction1 * direction2;

    if s ~= 0
        vx = [0 -v.z v.y; v.z 0 -v.x; -v.y v.x 0];
        R = eye(3) + vx + vx^2 * (1/(1+cos));
    else
        R = eye(3); % En caso de que los vectores sean inversos
    end
end

