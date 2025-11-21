classdef wheel < handle
    %WHEEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=private)
        centre v3
        static_radius double
        radius double
        width double
        normal v3
        resolution {mustBeInteger}
        X
        Y
        Z
    end
    
    methods

        function obj = wheel(radius, width, resolution)
            arguments
                radius double
                width double
                resolution {mustBeInteger}
            end

        obj.centre = v3();
        obj.normal = v3();
        obj.width = width;
        obj.static_radius = radius;
        obj.radius = radius;
        obj.resolution = resolution;

        [X, Y, Z] = cylinder(obj.radius, obj.resolution);
        obj.X = X;
        obj.Y = Y;
        obj.Z = obj.width * Z - obj.width/2;

        end

        function set_centre(obj, centre)
            arguments
                obj wheel
                centre v3
            end
            obj.centre = centre;
        end

        function set_normal(obj, normal)
            arguments
                obj wheel
                normal v3
            end
            obj.normal = normal;
        end

        function set_rolling_radius(obj, rolling_radius)
            arguments
                obj wheel
                rolling_radius double
            end
            obj.radius = rolling_radius;
        end

        function update(obj, centre, normal)
            obj.centre = centre;
            obj.normal = normal;
            R = rotation_matrix(v3(0,0,1), obj.normal);
            X1 = obj.X; Y1 = obj.Y; Z1 = obj.Z;
            xyz = R * [X1(:)'; Y1(:)';Z1(:)'];
            obj.X = reshape(xyz(1,:), size(X1)) + obj.centre.x;
            obj.Y = reshape(xyz(2,:), size(Y1)) + obj.centre.y;
            obj.Z = reshape(xyz(3,:), size(Z1)) + obj.centre.z;
        end

        function plot3d(obj)
            surf(obj.X, obj.Y, obj.Z, 'FaceAlpha',0.7, 'EdgeColor','none', 'FaceColor','k');
        end

    end
end

