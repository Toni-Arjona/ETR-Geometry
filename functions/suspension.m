classdef suspension < handle
    %SUSPENSION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        damper rod
        rocker solid
        pushrod rod
        u_wishbone solid
        knuckle solid
        l_wishbone solid
        wheel wheel
        camber_shims_distance double
    end

    methods (Access = private)
        function susp_update(obj)
            % Virtually linking up the damper & rocker
            error = 1;
            while error > 1e-6
                new_rocker_position = obj.damper.p2;
                obj.rocker.setPoint(3, new_rocker_position, 1, 2);
                obj.damper.set_p2( obj.rocker.coord(3) );
                error = (obj.rocker.coord(3) - obj.damper.p2).';
            end

            % Virtually linking up the rocker & pushrod
            obj.pushrod.set_p1( obj.rocker.coord(4) );

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

            %Update wheel
            obj.wheel.update(obj.knuckle.coord(9), (obj.knuckle.coord(8) - obj.knuckle.coord(4))');
        end

        function centre_wheel(obj)

            % If we are working either on the left or the right side of the car
            if( obj.knuckle.coord(4).y < 0 )
                outside_car_direction = v3(0,-1,0);
            else
                outside_car_direction = v3(0,1,0);
            end
            
            normal = (obj.knuckle.coord(2) - obj.knuckle.coord(1))';

            direction = line_plane_intersection( outside_car_direction, v3(0,0,1), normal, 0 );
            direction = direction';

            obj.knuckle.setDirection( 1, 2, 4, 9, direction );
            %Update wheel
            obj.wheel.update(  obj.knuckle.coord(9) ,   (obj.knuckle.coord(8) - obj.knuckle.coord(4))' )
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
            obj.wheel = wheel(16*25.4/2, 7.5*25.4, 15 );
            obj.camber_shims_distance = 0;

            %if( (obj.knuckle.coord(1) - obj.knuckle.coord(2))' * v3(0,0,1) < 0 )
            %    obj.knuckle = obj.knuckle.mirror_on_plane(v3(0,0,1), 0);
            %end

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

            outside_car_direction = v3(0,-1,0); % This defines which way the steering is positive 
            % Right now, left is positive
            
            unprojected_steering = obj.unprojected_steering_angle();
            angle_direction = ((obj.knuckle.coord(8) - obj.knuckle.coord(4))' ^ outside_car_direction)';
            angle_direction.z = abs(angle_direction.z);
            angle = angle_projection( unprojected_steering, angle_direction, v3(0,0,1) );
            knuckle_front_direction = (obj.knuckle.coord(5) - obj.knuckle.coord(4))';
            if(knuckle_front_direction*outside_car_direction < 0 )
                angle = -angle;
            end
        end

        function set_ground_height(obj, height)
            arguments
                obj suspension
                height double
            end
            
            current_damper_len = obj.get_damper_distance();
            max_iterations = 50;
            tolerance = 1e-3; % mm

            for i = 1:max_iterations
                % Update the suspension with the current length
                obj.damper.set_length(current_damper_len);
                obj.susp_update();
                
                % Get the current ground height (Z-coordinate of the contact patch)
                current_height = obj.get_contact_patch().z;
                
                error = current_height- height; % Target - Current
                
                if abs(error) < tolerance
                    % Solution found
                    return; 
                end
                
                % Estimate how much the damper needs to change.
                % This step requires a derivative (Jacobian) or a simple guess.
                % For simplicity, we assume a small linear relationship for the step.
                
                % NOTE: The actual linear rate (spring rate) for the ground height 
                % w.r.t damper length will be the motion ratio squared times the 
                % spring rate, but here we just need a step size that moves us 
                % towards the target.
                
                % Simple step: move the length by a fraction of the height error
                % This is a very basic, non-robust search.
                step_factor = 0.5; % This will need tuning or replacement with a proper method.
                current_damper_len = current_damper_len + step_factor * error;
            end

            warning('Did not converge to the desired ground height in %d iterations.', max_iterations);

        end

        function direction = get_kingpin_direction(obj)
            direction = (obj.knuckle.coord(1) - obj.knuckle.coord(2))';
        end

        function point = get_kingpin_in_ground(obj)
            normal_plane = v3(0,0,1);
            normal_plane_D = -(obj.get_contact_patch()*normal_plane);
            point = line_plane_intersection(obj.knuckle.coord(2), obj.get_kingpin_direction(), normal_plane, normal_plane_D);
        end

        function radius = get_scrub_radius(obj)
            radius = (obj.get_contact_patch() - obj.get_kingpin_in_ground()).';
        end

        function radius = get_scrub_radius_X(obj)
            radius = abs(obj.get_contact_patch().y - obj.get_kingpin_in_ground().y);
        end
        
        function angle = caster_angle(obj)
            arguments
                obj suspension
            end

            rotation_axis = (obj.knuckle.coord(1) - obj.knuckle.coord(2))';
            ground_normal = v3(0,0,1);

            angle = anglev3(rotation_axis, ground_normal);
            angle_plane = (rotation_axis ^ ground_normal)';
            
            outside_direction = (obj.knuckle.coord(8) - obj.knuckle.coord(4))';
            front_direction = (outside_direction ^ ground_normal)';
            plane = (ground_normal ^ front_direction);

            angle = angle_projection( angle, angle_plane, plane );
            angle = angle*sign( obj.knuckle.coord(4).y );
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

            angle = -angle_projection( unprojected_camber, unprojected_camber_plane, projection_direction );
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

            % TODO
            
            wheel_normal = (obj.knuckle.coord(8) - obj.knuckle.coord(4))';

            rotation_centre = obj.knuckle_rotation_centre();
            rotation_normal = (obj.knuckle.coord(2) - obj.knuckle.coord(1))';
            radius = (obj.knuckle.coord(3) - rotation_centre).';
            vector1 = (obj.knuckle.coord(3) - rotation_centre)';
            vector2 = rotation_normal ^ vector1;

            point = point_in_3d_circle(rotation_centre, toe_ang_rad*sign(-obj.knuckle.coord(4).y), radius, vector1, vector2);
            obj.knuckle.setPoint(3, point, 1, 2);

            %rotation_centre = point_plane_intersection(obj.knuckle.coord(4), obj.knuckle.coord(1), obj.knuckle.coord(2));
            %normal = (obj.knuckle.coord(2) - obj.knuckle.coord(1))';
            %plane_D = -(normal*obj.knuckle.coord(4));

            %direction = line_plane_intersection( outside_car_direction, v3(0,0,1), normal, 0 );
            %direction = direction';

            %obj.knuckle.setDirection( 1, 2, 4, 9, direction );
            %Update wheel
            %obj.wheel.set_centre( obj.knuckle.coord(9) );
            %obj.wheel.set_normal( (obj.knuckle.coord(8) - obj.knuckle.coord(4))' );





            %obj.knuckle.rotate( 3, 1, 2, toe_ang_rad);
        end

        function centre_steering(obj)
            obj.centre_wheel();
            obj.set_toe(0);
            %error = obj.steering_angle();
            %while abs(error) > 1e-3
            %    obj.set_toe(error*180/pi);
            %    error = obj.steering_angle();
            %end
        end


        function print(obj)
            fprintf("--- Suspension ---\n - Damper:\n"); obj.damper.print();
            fprintf(" - Rocker:\n"); obj.rocker.print();
            fprintf(" - Pushrod:\n"); obj.pushrod.print();
            fprintf(" - U_Wishbone:\n"); obj.u_wishbone.print();
            fprintf(" - Knuckle:\n"); obj.knuckle.print();
            fprintf(" - L_Wishbone:\n"); obj.l_wishbone.print();
        end

        function wheel_print(obj)
            fprintf("DamperD:  %5.2f\n", obj.damper.getLength());
            fprintf("PushrodD: %5.2f\n", obj.pushrod.getLength());
            fprintf("Steering: %6.2f\n", obj.steering_angle()*180/pi);
            fprintf("Camber:    %4.2f\n", obj.camber_angle()*180/pi);

        end

        function distance = get_damper_distance(obj)
            distance = obj.damper.getLength();
        end
        function distance = get_pushrod_distance(obj)
            distance = obj.pushrod.getLength();
        end

        function point = get_contact_patch(obj)
            arguments (Output)
                point v3
            end

            ground_direction = v3(0,0,1);
            wheel_normal_direction = (obj.knuckle.coord(8) - obj.knuckle.coord(4))';
            reverse_ground = -ground_direction;

            to_ground_direction = line_plane_projection(reverse_ground, wheel_normal_direction);
            point = obj.knuckle.coord(9) + to_ground_direction.*obj.wheel.get_radius();
        end

        function plot3d(obj,ax)

            obj.damper.plot3d(ax,'b')
            obj.rocker.plot3d(ax,'k');
            obj.pushrod.plot3d(ax,'k');
            obj.u_wishbone.plot3d(ax,'k');
            obj.l_wishbone.plot3d(ax,'k');
            %obj.knuckle.plot3d('r');

            % Knuckle
            plot3dline(ax,obj.knuckle.coord(1), obj.knuckle.coord(2), 'r');
            plot3dline(ax,obj.knuckle.coord(3), obj.knuckle_rotation_centre(), 'r');
            plot3dline(ax,obj.knuckle.coord(9), obj.knuckle.coord(4), 'r');
            plot3dline(ax,obj.get_contact_patch(), obj.knuckle.coord(9), 'r');
            obj.get_kingpin_in_ground().plot3d(ax, 'r');

            %Wheel
            obj.wheel.update(obj.knuckle.coord(9), (obj.knuckle.coord(8) - obj.knuckle.coord(4))');
            obj.wheel.plot3d(ax);
        end

        function alone_plot3d(obj)
            figure
            hold on

            obj.plot3d();

            grid on
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            axis equal
            view(3);
        end

        function r = knuckle_radius(obj)
            p = point_line_projection(obj.knuckle.coord(3), (obj.knuckle.coord(2) - obj.knuckle.coord(1))', obj.knuckle.coord(2));
            r = (obj.knuckle.coord(3) - p);
        end

    end
end

