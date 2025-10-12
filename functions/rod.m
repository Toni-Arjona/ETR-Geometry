classdef rod < handle
    %ROD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        p1 (1,1) v3
        p2 (1,1) v3
    end

    properties (Access = private)
        length (1,1) double
    end
    
    methods
        function obj = rod(position1,position2)
            arguments
                position1 (1,1) v3
                position2 (1,1) v3
            end
            obj.p1 = position1;
            obj.p2 = position2;
            obj.length = (obj.p2 - obj.p1).';
        end

        function obj = rodL(position1, length)
            arguments
                position1 (1,1) v3
                length (1,1) double
            end
            obj.p1 = position1;
            obj.p2 = position1 + v3(1,0,0).*length;
            obj.lenght = length;
        end
        
        function L = getLength(obj)
            L = obj.length;
        end

        function set_length(obj, length)
            % Set the length of the rod by moving the p2
            obj.p2 = obj.p1 + (obj.p2 - obj.p1)'.*length;
            obj.length = length;
        end

        function set_p2(obj, position)
            temp_direction = (position - obj.p1)';
            obj.p2 = obj.p1 + temp_direction.*obj.length;
        end

        function set_p1(obj, position)
            displacement = position - obj.p1;
            obj.p1 = position;
            obj.p2 = obj.p2 + displacement;
        end

        function mirror_rod = mirror_on_plane(obj, plane_direction, plane_D)
            arguments (Input)
                obj rod
                plane_direction v3
                plane_D double
            end
            arguments (Output)
                mirror_rod rod
            end
            plane_direction = plane_direction';
            mirror_p1 = point_plane_mirror( obj.p1, plane_direction, plane_D );
            mirror_p2 = point_plane_mirror( obj.p2, plane_direction, plane_D );

            mirror_rod = rod( mirror_p1, mirror_p2 );
        end

        function print(obj)
            fprintf("P1: "); obj.p1.print();
            fprintf("P2: "); obj.p2.print();
            fprintf("Distance: %14.8f\n", obj.length);
        end
    end
end

