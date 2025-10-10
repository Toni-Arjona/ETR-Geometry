classdef suspension < handle
    %SUSPENSION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        damper rod
        rocker rocker
        pushrod rod
        u_wishbone solid
        knuckle solid
        l_wishbone solid
    end

    methods (Access = private)
        function susp_update(obj)
            % Virtually linking up the damper & rocker
            error = 1;
            while error > 1e-6
                new_rocker_position = obj.damper.p2;
                obj.rocker.set_arm1(new_rocker_position);
                obj.damper.set_p2(obj.rocker.arm1);
                error = (obj.rocker.arm1 - obj.damper.p2).';
            end

            % Virtually linking up the rocker & pushrod
            obj.pushrod.set_p1( obj.rocker.arm2 );

            % Virtually linking up the pushrod with the upper wishbone
            error = 1;
            while error > 1e-6
                new_pushrod_end_position = obj.pushrod.p2;
                obj.u_wishbone.setPoint(4, new_pushrod_end_position);
                obj.pushrod.set_p2(obj.u_wishbone.coord(4));
                error = (obj.pushrod.p2 - obj.u_wishbone.coord(4)).';
            end

            % Virtually linking up the upper wishbone with the knuckle
            obj.knuckle.free_move(1, obj.u_wishbone.coord(3));

            % Virtually linking up the knuckle with the lower knuckle
            error = 1;
            while error > 1e-5
                new_lower_knuckle_position = obj.knuckle.coord(2);
                obj.l_wishbone.setPoint(3, new_lower_knuckle_position);
                obj.knuckle.fixed_free_move(1, 2, obj.l_wishbone.coord(3))
                error = (obj.knuckle.coord(2) - obj.l_wishbone.coord(3)).';
            end
        end

        function centre_steering(obj)
            error = 1;
            while error > 5e-3
                rotation_centre = point_plane_intersection( obj.knuckle.coord(4), obj.knuckle.coord(1), obj.knuckle.coord(2) );
                normal = (obj.knuckle.coord(2) - obj.knuckle.coord(1))';
                vector_1 = ( obj.knuckle.coord(4) - rotation_centre)';
                vector_2 = vector_1 ^ normal;

                if( obj.knuckle.coord(5).y - obj.knuckle.coord(4).y < 0 )
                    vector_2 = -vector_2;
                end
                radius = (rotation_centre - obj.knuckle.coord(4)).';
                angle = anglev3( (obj.knuckle.coord(5) - obj.knuckle.coord(4))', v3(-1,0,0) );

                objective_point = point_in_3d_circle(rotation_centre, angle, radius, vector_1, vector_2);
                
                obj.knuckle.setPoint(4, objective_point);
                error = anglev3( (obj.knuckle.coord(5) - obj.knuckle.coord(4))', v3(-1,0,0) );
            end
        end

        function centre = knuckle_rotation_centre(obj)
            arguments (Output)
                centre v3
            end
            centre = point_plane_intersection( obj.knuckle.coord(3), obj.knuckle.coord(1), obj.knuckle.coord(2) );
        end

        function angle = unprojected_steering_angle(obj)
            angle = anglev3( (obj.knuckle.coord(5) - obj.knuckle.coord(4)), v3(-1,0,0) );
        end

        function angle = unprojected_camber(obj)
            angle = anglev3( (obj.knuckle.coord(6)-obj.knuckle.coord(4))', v3(0,0,1) );
        end

    end

    
    methods (Access = public)
        function obj = suspension(damper, rocker,pushrod, u_wishbone, knuckle, l_wishbone)
            obj.damper = damper;
            obj.rocker = rocker;
            obj.pushrod = pushrod;
            obj.u_wishbone = u_wishbone;
            obj.knuckle = knuckle;
            obj.l_wishbone = l_wishbone;
        end
        
        function set_damper_distance(obj, damper_distance)
            obj.damper.set_length(damper_distance);
            obj.susp_update();
            obj.centre_steering();
        end


        function angle = steering_angle(obj)
            arguments (Output)
                angle double
            end
            knukle_direction = (obj.knuckle.coord(1) - obj.knuckle.coord(2))';
            angle = angle_projection( obj.unprojected_steering_angle, knukle_direction, v3(0,0,1) );
            if(obj.knuckle.coord(5).y - obj.knuckle.coord(4).y > 0 )
                angle = -angle;
            end
        end


        function angle = camber_angle(obj)
            arguments (Output)
                angle double
            end

            rotation = obj.steering_angle();
            unprojected_camber = obj.unprojected_camber();
            unporjected_camber_plane = (obj.knuckle.coord(6)-obj.knuckle.coord(4))' ^ v3(0,0,1);
            projection_direction = point_in_3d_circle( v3(0,0,0), rotation, 1, v3(-1,0,0), v3(0,-1,0) );

            angle = angle_projection( unprojected_camber, unporjected_camber_plane, projection_direction );

            if(obj.knuckle.coord(6).y > obj.knuckle.coord(4) )
                angle = -angle;
            end

        end


        function print(obj)
            fprintf("--- Suspension ---\n - Damper:\n"); obj.damper.print();
            fprintf(" - Rocker:\n"); obj.rocker.print();
            fprintf(" - Pushrod:\n"); obj.pushrod.print();
            fprintf(" - U_Wishbone:\n"); obj.u_wishbone.print();
            fprintf(" - Knuckle:\n"); obj.knuckle.print();
            fprintf(" - L_Wishbone:\n"); obj.l_wishbone.print();
        end

    end
end

