classdef rocker < handle
    %ROCKER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        rockerCentre (1,1) v3
        arm1 (1,1) v3
        arm2 (1,1) v3
    end

    properties (Access = private)
        rockerPlaneDir (1,1) v3
        rockerPlane_D (1,1) double
        lever_arm1 (1,1) double
        lever_arm2 (1,1) double
        angle
    end

    methods (Access = private)
        function update_private(obj)
            obj.rockerPlaneDir = ((obj.arm1 - obj.rockerCentre)'^(obj.arm2 - obj.rockerCentre)')';
            obj.rockerPlane_D = -obj.rockerPlaneDir*obj.rockerCentre;
            obj.lever_arm1 = (obj.arm1 - obj.rockerCentre).';
            obj.lever_arm2 = (obj.arm2 - obj.rockerCentre).';
            obj.angle = anglev3( (obj.arm1 - obj.rockerCentre)', (obj.arm2 - obj.rockerCentre)' );
        end

        function update_arm2(obj)
            plane_direction_1 = (obj.arm1 - obj.rockerCentre)';
            plane_direction_2 = (obj.rockerPlaneDir ^ plane_direction_1)';
            obj.arm2 = point_in_3d_circle( obj.rockerCentre, obj.angle, obj.lever_arm2, plane_direction_1, plane_direction_2);
        end

        function update_arm1(obj)
            plane_direction_1 = (obj.arm2 - obj.rockerCentre)';
            plane_direction_2 = (plane_direction_1 ^ obj.rockerPlaneDir)';
            obj.arm1 = point_in_3d_circle( obj.rockerCentre, obj.angle, obj.lever_arm1, plane_direction_1, plane_direction_2);
        end
    end
    methods (Access = public)
        function obj = rocker(rockerCentre, arm1, arm2)
            arguments
                rockerCentre (1,1) v3
                arm1 (1,1) v3
                arm2 (1,1) v3
            end
            obj.rockerCentre = rockerCentre;
            obj.arm1 = arm1;
            obj.arm2 = arm2;
            
            obj.rockerPlaneDir = ((arm1 - rockerCentre)'^(arm2 - rockerCentre)')';
            obj.rockerPlane_D = -obj.rockerPlaneDir*rockerCentre;
            obj.lever_arm1 = (arm1 - rockerCentre).';
            obj.lever_arm2 = (arm2 - rockerCentre).';
            obj.angle = anglev3( (obj.arm1 - obj.rockerCentre)', (obj.arm2 - obj.rockerCentre)' );

        end

        function set_arm1_length(obj, new_length)
            arguments 
                obj rocker
                new_length (1,1) double
            end
            temp_dir = (obj.arm1 - obj.rockerCentre)';
            obj.arm1 = obj.rockerCentre + temp_dir.*new_length;
            obj.update_private();
            obj.update_arm2();
        end

        function set_arm2_length(obj, new_length)
            arguments 
                obj rocker
                new_length (1,1) double
            end
            temp_dir = (obj.arm2 - obj.rockerCentre)';
            obj.arm2 = obj.rockerCentre + temp_dir.*new_length;
            obj.update_private();
            obj.update_arm1();
        end

        function set_rocker_angle(obj, new_angle_radians)
            arguments
                obj rocker
                new_angle_radians (1,1) double
            end
            plane_direction_1 = (obj.arm1 - obj.rockerCentre)';
            plane_direction_2 = (obj.rockerPlaneDir ^ plane_direction_1)';
            obj.arm2 = point_in_3d_circle( obj.rockerCentre, new_angle_radians, obj.lever_arm2, plane_direction_1, plane_direction_2 );
            obj.update_private();
        end

        function set_arm1(obj, position)
            arguments
                obj rocker
                position (1,1) v3
            end

            % First we project the position on the rocker plane
            position = point_plane_projection(position, obj.rockerPlaneDir, obj.rockerPlane_D);

            % We move the point to the correct distance from the centre
            position_dir = (position - obj.rockerCentre)';
            obj.arm1 = obj.rockerCentre + position_dir.*obj.lever_arm1;
            obj.update_arm2();
        end

        function set_arm2(obj, position)
            arguments
                obj rocker
                position (1,1) v3
            end

            % First we project the position on the rocker plane
            position = point_plane_projection(position, obj.rockerPlaneDir, obj.rockerPlane_D);

            % We move the point to the correct distance from the centre
            position_dir = (position - obj.rockerCentre)';
            obj.arm2 = obj.rockerCentre + position_dir.*obj.lever_arm2;
            obj.update_arm1();
        end

        function print(obj)
            fprintf("Rocker Centre: "); obj.rockerCentre.print();
            fprintf("Arm 1: "); obj.arm1.print();
            fprintf("Arm 2: "); obj.arm2.print();
            fprintf("\nLever arm 1: %f\tLever arm 2: %f\tAngle: %f\n", obj.lever_arm1, obj.lever_arm2, obj.angle);
        end
    end
end

