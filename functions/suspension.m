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
                obj.u_wishbone.setPoint(4, new_pushrod_end_position, 1, 2);
                obj.pushrod.set_p2(obj.u_wishbone.coord(4));
                error = (obj.pushrod.p2 - obj.u_wishbone.coord(4)).';
            end

            % Virtually linking up the upper wishbone with the knuckle
            obj.knuckle.free_move(1, obj.u_wishbone.coord(3));

            % Virtually linking up the knuckle with the lower knuckle
            error = 1;
            while error > 1e-5
                new_lower_knuckle_position = obj.knuckle.coord(2);
                obj.l_wishbone.setPoint(3, new_lower_knuckle_position, 1, 2);
                obj.knuckle.fixed_free_move(1, 2, obj.l_wishbone.coord(3))
                error = (obj.knuckle.coord(2) - obj.l_wishbone.coord(3)).';
            end
        end

        function centre_wheel(obj)

            % If we are working either on the left or the right side of the car
            if( obj.knuckle.coord(4).y < 0 )
                outside_car_direction = v3(0,-1,0);
            else
                outside_car_direction = v3(0,1,0);
            end
            
            
            rotation_centre = point_plane_intersection(obj.knuckle.coord(4), obj.knuckle.coord(1), obj.knuckle.coord(2));
            normal = (obj.knuckle.coord(2) - obj.knuckle.coord(1))';
            plane_D = -(normal*obj.knuckle.coord(4));

            direction = line_plane_intersection( outside_car_direction, v3(0,0,1), normal, 0 );
            direction = direction';

            obj.knuckle.setDirection( 1, 2, 4, 9, direction );
        end

        function centre = knuckle_rotation_centre(obj)
            arguments (Output)
                centre v3
            end
            centre = point_plane_intersection( obj.knuckle.coord(3), obj.knuckle.coord(1), obj.knuckle.coord(2) );
        end

        function angle = unprojected_steering_angle(obj)
            % If we are working either on the left or the right side of the car
            if( obj.knuckle.coord(4).y < 0 )
                outside_car_direction = v3(0,-1,0);
            else
                outside_car_direction = v3(0, 1,0);
            end

            angle = anglev3( (obj.knuckle.coord(8) - obj.knuckle.coord(4)), outside_car_direction );
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
            obj.centre_wheel();
        end


        function angle = steering_angle(obj)
            arguments (Output)
                angle double
            end
            % If we are working either on the left or the right side of the car
            if( obj.knuckle.coord(4).y < 0 )
                outside_car_direction = v3(0,-1,0);
            else
                outside_car_direction = v3(0, 1,0);
            end
            
            unprojected_steering = obj.unprojected_steering_angle();
            angle_direction = ((obj.knuckle.coord(8) - obj.knuckle.coord(4))' ^ outside_car_direction)';
            angle_direction.z = abs(angle_direction.z);
            angle = angle_projection( unprojected_steering, angle_direction, v3(0,0,1) );
            knuckle_front_direction = (obj.knuckle.coord(5) - obj.knuckle.coord(4))';
            if(knuckle_front_direction*outside_car_direction < 0 )
                angle = -angle;
            end
        end


        function angle = camber_angle(obj)
            arguments (Output)
                angle double
            end

            % If we are working either on the left or the right side of the car
            if( obj.knuckle.coord(4).y < 0 )
                outside_car_direction = v3(0,-1,0);
                camber_plane_correction = -1;
            else
                outside_car_direction = v3(0, 1,0);
                camber_plane_correction = 1;
            end

            rotation = obj.steering_angle();
            unprojected_camber = obj.unprojected_camber();
            unprojected_camber_plane = camber_plane_correction.*((obj.knuckle.coord(6)-obj.knuckle.coord(4))' ^ v3(0,0,1))';
            projection_direction = point_in_3d_circle( v3(0,0,0), rotation, 1, v3(-1,0,0), v3(0,-1,0) );

            angle = angle_projection( unprojected_camber, unprojected_camber_plane, projection_direction );
            knuckle_vertical_direction = (obj.knuckle.coord(6) - obj.knuckle.coord(4))';

            if( outside_car_direction*knuckle_vertical_direction < 0 )
                angle = -angle;
            end

        end


        function mirror_suspension = mirror_on_plane(obj, plane_direction, plane_D)
            arguments (Input)
                obj suspension
                plane_direction v3
                plane_D double
            end
            arguments (Output)
                mirror_suspension suspension
            end
            plane_direction = plane_direction';

            mirror_damper =     obj.damper.mirror_on_plane(     plane_direction, plane_D );
            mirror_rocker =     obj.rocker.mirror_on_plane(     plane_direction, plane_D );
            mirror_pushrod =    obj.pushrod.mirror_on_plane(    plane_direction, plane_D );
            mirror_u_wishbone = obj.u_wishbone.mirror_on_plane( plane_direction, plane_D );
            mirror_knuckle =    obj.knuckle.mirror_on_plane(    plane_direction, plane_D );
            mirror_l_wishbone = obj.l_wishbone.mirror_on_plane( plane_direction, plane_D );
            mirror_suspension = suspension(mirror_damper, mirror_rocker, mirror_pushrod, mirror_u_wishbone, mirror_knuckle, mirror_l_wishbone);
        end


        function set_knuckle(obj, knuckle_tierod_point)
            arguments
                obj suspension
                knuckle_tierod_point v3
            end
            obj.knuckle.setPoint(3, knuckle_tierod_point, 1, 2);
        end

        function set_toe(obj, toe_ang_deg)
            % Positive toe is TOE IN
            arguments
                obj suspension
                toe_ang_deg double
            end
            obj.centre_wheel();
            toe_ang_rad = toe_ang_deg * pi / 180;

            obj.knuckle.rotate( 3, 1, 2, toe_ang_rad);
        end

        function centre_steering(obj)
            obj.centre_wheel();
            obj.set_toe(0);
            error = obj.steering_angle();
            while abs(error) > 1e-3
                obj.set_toe(error*180/pi);
                error = abs(obj.steering_angle());
            end

            obj.centre_wheel();
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

