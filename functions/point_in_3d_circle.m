function coordinate = point_in_3d_circle(rotation_centre,radians,radius,plane_vector1, plane_vector2)
    arguments (Input)
        rotation_centre v3
        radians double % Radians of rotation from plane_vector1 going to plane_vector2
        radius double % Radius of rotation of point
        plane_vector1 v3 % What would be the 'X' in the plane
        plane_vector2 v3 % What would be the 'Y' in the plane
    end
    arguments (Output)
        coordinate v3 % End coordinate
    end
    plane_vector1 = plane_vector1'; % Normalize the vector
    plane_vector2 = plane_vector2'; % Normalize the vector

    coordinate = rotation_centre + plane_vector1.*(radius*cos(radians)) + plane_vector2.*(radius*sin(radians));
end

