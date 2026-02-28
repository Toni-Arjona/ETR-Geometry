classdef car < handle
    %CAR 
    %   Using suspension and steering
    
    properties (Access=public)
        fl_susp suspension
        fr_susp suspension
        rl_susp suspension
        rr_susp suspension
        f_steer steering
        r_steer steering
        fl_tierod double
        fr_tierod double
        rl_tierod double
        rr_tierod double
        steering_wheel_deg double
        AnnotationHandle handle = [];
        fl_label handle = [];
        fr_label handle = [];
        rl_label handle = [];
        rr_label handle = [];
    end

    methods (Access=private)
     %% Update Functions
        function update_front(obj)
            arguments
                obj car
            end


            % Iterate to make the front wheels converge to static
            error = 1;
            while abs(error) > 1e-6
                direction = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi())';
                place = obj.f_steer.left_clevi() + direction.*obj.fl_tierod;
                obj.fl_susp.set_knuckle(place);
                error = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi()).' - obj.fl_tierod;
            end

            error = 1;
            while abs(error) > 1e-6
                direction = (obj.fr_susp.knuckle.coord(3) - obj.f_steer.right_clevi())';
                place = obj.f_steer.right_clevi() + direction.*obj.fr_tierod;
                obj.fr_susp.set_knuckle(place);
                error = (obj.fr_susp.knuckle.coord(3) - obj.f_steer.right_clevi()).' - obj.fr_tierod;
            end
        end

        function update_rear(obj)
            arguments
                obj car
            end

            % Iterate to make the front wheels converge to static
            error = 1;
            while abs(error) > 1e-6
                direction = (obj.rl_susp.knuckle.coord(3) - obj.r_steer.left_clevi())';
                place = obj.r_steer.left_clevi() + direction.*obj.rl_tierod;
                obj.rl_susp.set_knuckle(place);
                error = (obj.rl_susp.knuckle.coord(3) - obj.r_steer.left_clevi()).' - obj.rl_tierod;
            end

            error = 1;
            while abs(error) > 1e-6
                direction = (obj.rr_susp.knuckle.coord(3) - obj.r_steer.right_clevi())';
                place = obj.r_steer.right_clevi() + direction.*obj.rr_tierod;
                obj.rr_susp.set_knuckle(place);
                error = (obj.rr_susp.knuckle.coord(3) - obj.r_steer.right_clevi()).' - obj.rr_tierod;
            end
        end

        %% For ui
        function common_callback(obj, fig, ax)
            if ~isempty(obj.AnnotationHandle) && isvalid(obj.AnnotationHandle)
                delete(obj.AnnotationHandle);
            end

            cla(ax, "reset");
            axis_set(ax);
            obj.plot3dpoints(ax);
            obj.update_labels();
            %obj.plot3d_text(fig);
        end

        function steering_callback(obj, fig, ax, angle_deg, str_lbl)
            obj.set_steering_wheel(angle_deg);
            obj.common_callback(fig, ax)
            update_label(str_lbl, "Steering Wheel Angle:", angle_deg)
        end

        function update_labels(obj)
            obj.fl_label.Text = sprintf("FL\nSteer:%3.2f\nCamber:%3.2f\nCaster:%3.2f\nHeight:%3.2f\nDamper:%3.2f\nS. Radius:%3.2f\nTierod:%3.2f\nT. Rad:%.1f\nW-T-D:%5.2f\n", ...
                obj.fl_steering_rad()*180/pi, ...
                obj.fl_susp.camber_angle()*180/pi, ...
                obj.fl_susp.caster_angle()*180/pi, ...
                obj.fl_susp.get_contact_patch().z, ...
                obj.fl_susp.get_damper_distance(), ...
                obj.fl_susp.get_scrub_radius(), ...
                obj.fl_tierod, ...
                obj.fl_virtual_turning_radius()/1000,...
                obj.fl_wishbone_tierod_distance());

            obj.fr_label.Text = sprintf("FR\nSteer:%3.2f\nCamber:%3.2f\nCaster:%3.2f\nHeight:%3.2f\nDamper:%3.2f\nS. Radius:%3.2f\nTierod:%3.2f\nT. Rad:%.1f\nW-T-D:%5.2f\n", ...
                obj.fr_steering_rad()*180/pi, ...
                obj.fr_susp.camber_angle()*180/pi, ...
                obj.fr_susp.caster_angle()*180/pi, ...
                obj.fr_susp.get_contact_patch().z, ...
                obj.fr_susp.get_damper_distance(), ...
                obj.fr_susp.get_scrub_radius(), ...
                obj.fr_tierod, ...
                obj.fr_virtual_turning_radius()/1000,...
                obj.fr_wishbone_tierod_distance());

            obj.rl_label.Text = sprintf("RL\nSteer:%3.2f\nCamber:%3.2f\nCaster:%3.2f\nHeight:%3.2f\nDamper:%3.2f\nS. Radius:%3.2f\nTierod:%3.2f\nW-T-D:%5.2f\n", ...
                obj.rl_steering_rad()*180/pi, ...
                obj.rl_susp.camber_angle()*180/pi, ...
                obj.rl_susp.caster_angle()*180/pi, ...
                obj.rl_susp.get_contact_patch().z, ...
                obj.rl_susp.get_damper_distance(), ...
                obj.rl_susp.get_scrub_radius(), ...
                obj.rl_tierod,...
                obj.rl_wishbone_tierod_distance());

            obj.rr_label.Text = sprintf("RR\nSteer:%3.2f\nCamber:%3.2f\nCaster:%3.2f\nHeight:%3.2f\nDamper:%3.2f\nS. Radius:%3.2f\nTierod:%3.2f\nW-T-D:%5.2f\n", ...
                obj.rr_steering_rad()*180/pi, ...
                obj.rr_susp.camber_angle()*180/pi, ...
                obj.rr_susp.caster_angle()*180/pi, ...
                obj.rr_susp.get_contact_patch().z, ...
                obj.rr_susp.get_damper_distance(), ...
                obj.rr_susp.get_scrub_radius(), ...
                obj.rr_tierod, ...
                obj.rr_wishbone_tierod_distance());
        end       

        function fl_callback(obj, fig, ax, height)
            obj.fl_height(height); % PERRO
            obj.common_callback(fig, ax);
            obj.update_labels();
        end
        function fr_callback(obj, fig, ax, height)
            obj.fr_height(height); % PERRO
            obj.common_callback(fig, ax);
            obj.update_labels();
        end
        function rl_callback(obj, fig, ax, height)
            obj.rl_height(height); % PERRO
            obj.common_callback(fig, ax);
            obj.update_labels();
        end
        function rr_callback(obj, fig, ax, height)
            obj.rr_height(height); % PERRO
            obj.common_callback(fig, ax);
            obj.update_labels();
        end

        function f_toe_callback(obj, fig, ax, steering_sld, value, toe_sld, str_lbl)
            obj.set_toe_front(value);
            obj.common_callback(fig, ax);
            steering_sld.Value = 0;
            toe_sld.Value = value;
            obj.set_steering_wheel(0);
            update_label(str_lbl, "Steering Wheel Angle:", 0);
        end
        function r_toe_callback(obj, fig, ax, value, toe_sld)
            obj.set_toe_rear(value);
            obj.common_callback(fig, ax);
            toe_sld.Value = value;
        end

        function h_callback(obj, fig, ax, fl, fr, rl, rr)
            obj.fl_callback(fig, ax, 0);
            obj.fr_callback(fig, ax, 0);
            obj.rl_callback(fig, ax, 0);
            obj.rr_callback(fig, ax, 0);
            fl.Value = 0;
            fr.Value = 0;
            rl.Value = 0;
            rr.Value = 0;
        end

    end
    methods (Access=public)
        function obj = car(fl_susp, fr_susp, rl_susp, rr_susp, f_steer, r_steer)
            arguments (Input)
                fl_susp suspension
                fr_susp suspension
                rl_susp suspension
                rr_susp suspension
                f_steer steering
                r_steer steering
            end
            obj.fl_susp = fl_susp;
            obj.fr_susp = fr_susp;
            obj.rl_susp = rl_susp;
            obj.rr_susp = rr_susp;
            obj.f_steer = f_steer;
            obj.r_steer = r_steer;

            obj.fl_tierod = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi()).';
            obj.fr_tierod = (obj.fr_susp.knuckle.coord(3) - obj.f_steer.right_clevi()).';
            obj.rl_tierod = (obj.rl_susp.knuckle.coord(3) - obj.r_steer.left_clevi()).';
            obj.rr_tierod = (obj.rr_susp.knuckle.coord(3) - obj.r_steer.right_clevi()).';
            obj.steering_wheel_deg = 0;
        end


        function centre_steering(obj)
            obj.f_steer.centre_steering();
            obj.r_steer.centre_steering();
            obj.fl_susp.centre_steering();
            obj.fr_susp.centre_steering();
            obj.rl_susp.centre_steering();
            obj.rr_susp.centre_steering();
        end

        
        %% Angulos de Steering
        function angle = fl_steering_rad(obj)
            arguments (Input)
                obj car
            end
            arguments (Output)
                angle double
            end
            angle = obj.fl_susp.steering_angle();
        end

        function angle = fr_steering_rad(obj)
            arguments (Input)
                obj car
            end
            arguments (Output)
                angle double
            end
            angle = obj.fr_susp.steering_angle();
        end

        function angle = rl_steering_rad(obj)
            arguments (Input)
                obj car
            end
            arguments (Output)
                angle double
            end
            angle = obj.rl_susp.steering_angle();
        end

        function angle = rr_steering_rad(obj)
            arguments (Input)
                obj car
            end
            arguments (Output)
                angle double
            end
            angle = obj.rr_susp.steering_angle();
        end

        %% Set steering wheel
        function set_steering_wheel(obj, steering_wheel_deg)
            % Left steering is positive
            arguments
                obj car
                steering_wheel_deg double
            end
            obj.steering_wheel_deg = steering_wheel_deg;
            obj.f_steer.set_steering(steering_wheel_deg*pi/180);
            obj.update_front();
        end

       
        %% Set toe
        function set_toe_front(obj, toe_deg)
            arguments
                obj car
                toe_deg double 
            end
            obj.centre_steering();
            obj.fl_susp.set_toe(toe_deg);
            obj.fr_susp.set_toe(toe_deg);

            % Updating the tierods length
            obj.fl_tierod = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi()).';
            obj.fr_tierod = (obj.fr_susp.knuckle.coord(3) - obj.f_steer.right_clevi()).';
            obj.update_front();
        end
        function set_front_toe(obj, toe_deg)
            obj.set_toe_front(toe_deg);
        end

        function set_toe_rear(obj, toe_deg)
            arguments
                obj car
                toe_deg double 
            end
            obj.rl_susp.set_toe(toe_deg);
            obj.rr_susp.set_toe(toe_deg);

            % Updating the tierods length
            obj.rl_tierod = (obj.rl_susp.knuckle.coord(3) - obj.r_steer.left_clevi()).';
            obj.rr_tierod = (obj.rr_susp.knuckle.coord(3) - obj.r_steer.right_clevi()).';
            obj.update_rear();
        end
        function set_rear_toe(obj, toe_deg)
            obj.set_toe_rear(toe_deg);
        end

        %% Set Dampers
        function set_front_dampers(obj, distance)
            arguments
                obj car
                distance double 
            end
            if(distance ~= obj.fl_susp.get_damper_distance())
                obj.fl_susp.set_damper_distance(distance);
                obj.fr_susp.set_damper_distance(distance);
                obj.update_front();
            end
        end
        
        function set_rear_dampers(obj, distance)
            arguments
                obj car
                distance double
            end
            if(distance ~= obj.rl_susp.get_damper_distance())
                obj.rl_susp.set_damper_distance(distance);
                obj.rr_susp.set_damper_distance(distance);
                obj.update_rear();
            end
        end

        %% Set Wheel Heights
        function fl_height(obj, height)
            obj.fl_susp.set_ground_height(height);
            obj.update_front();
        end
        function fr_height(obj, height)
            obj.fr_susp.set_ground_height(height);
            obj.update_front();
        end
        function rl_height(obj, height)
            obj.rl_susp.set_ground_height(height);
            obj.update_rear();
        end
        function rr_height(obj, height)
            obj.rr_susp.set_ground_height(height);
            obj.update_rear();
        end

        %% Wishbone distances
        function distance = fl_wishbone_tierod_distance(obj)
            distance = wishbone_tierod_distance(obj.fl_susp.u_wishbone, obj.fl_susp.l_wishbone, obj.fl_susp.knuckle.coord(3), obj.f_steer.left_clevi());
        end
        function distance = fr_wishbone_tierod_distance(obj)
            distance = wishbone_tierod_distance(obj.fr_susp.u_wishbone, obj.fr_susp.l_wishbone, obj.fr_susp.knuckle.coord(3), obj.f_steer.right_clevi());
        end
        function distance = rl_wishbone_tierod_distance(obj)
            distance = wishbone_tierod_distance(obj.rl_susp.u_wishbone, obj.rl_susp.l_wishbone, obj.rl_susp.knuckle.coord(3), obj.r_steer.left_clevi());
        end
        function distance = rr_wishbone_tierod_distance(obj)
            distance = wishbone_tierod_distance(obj.rr_susp.u_wishbone, obj.rr_susp.l_wishbone, obj.rr_susp.knuckle.coord(3), obj.r_steer.right_clevi());
        end        
        %% Print functions
        function print(obj)
            arguments
                obj car
            end

            fprintf("+- Front Left ------+- Front Right -----+\n")
            fprintf("| DamperD:  %5.2f\t| DamperD:  %5.2f\t|\n", obj.fl_susp.get_damper_distance(), obj.fr_susp.get_damper_distance());
            fprintf("| PushrodD: %5.2f\t| PushrodD: %5.2f\t|\n", obj.fl_susp.get_pushrod_distance(), obj.fr_susp.get_pushrod_distance());
            fprintf("| Steering: %6.2f\t| Steering: %6.2f\t|\n", obj.fl_steering_rad()*180/pi, obj.fr_steering_rad()*180/pi);
            fprintf("| Camber:    %4.2f\t| Camber:    %4.2f\t|\n", obj.fl_susp.camber_angle()*180/pi, obj.fr_susp.camber_angle()*180/pi );
            fprintf("| TierodD:  %5.2f\t| TierodD:  %5.2f\t|\n", obj.fl_tierod, obj.fr_tierod);

            fprintf("+- Rear Left -------+- Rear Right ------+\n")
            fprintf("| DamperD:  %5.2f\t| DamperD:  %5.2f\t|\n", obj.rl_susp.get_damper_distance(), obj.rr_susp.get_damper_distance());
            fprintf("| PushrodD: %5.2f\t| PushrodD: %5.2f\t|\n", obj.rl_susp.get_pushrod_distance(), obj.rr_susp.get_pushrod_distance());
            fprintf("| Steering: %6.2f\t| Steering: %6.2f\t|\n", obj.rl_steering_rad()*180/pi, obj.rr_steering_rad()*180/pi);
            fprintf("| Camber:    %4.2f\t| Camber:    %4.2f\t|\n", obj.rl_susp.camber_angle()*180/pi, obj.rr_susp.camber_angle()*180/pi );
            fprintf("| TierodD:  %5.2f\t| TierodD:  %5.2f\t|\n", obj.rl_tierod, obj.rr_tierod);


            fprintf("+-------------------+-------------------+\n");
        end

        function print_extended(obj)
            arguments
                obj car
            end

            fprintf("+- Front Left ------+- Front Right -----+\n")
            fprintf("| DamperD:  %5.2f\t| DamperD:  %5.2f\t|\n", obj.fl_susp.get_damper_distance(), obj.fr_susp.get_damper_distance());
            fprintf("| PushrodD: %5.2f\t| PushrodD: %5.2f\t|\n", obj.fl_susp.get_pushrod_distance(), obj.fr_susp.get_pushrod_distance());
            fprintf("| Steering: %6.2f\t| Steering: %6.2f\t|\n", obj.fl_steering_rad()*180/pi, obj.fr_steering_rad()*180/pi);
            fprintf("| Camber:    %4.2f\t| Camber:    %4.2f\t|\n", obj.fl_susp.camber_angle()*180/pi, obj.fr_susp.camber_angle()*180/pi );
            fprintf("| TierodD:  %5.2f\t| TierodD:  %5.2f\t|\n", obj.fl_tierod, obj.fr_tierod);
            fprintf("| RackDis:   %5.2f\t| RackDis:   %5.2f\t|\n", obj.steering_wheel_deg*pi/180*obj.f_steer.pinion_diameter/2, obj.steering_wheel_deg*pi/180*obj.f_steer.pinion_diameter/2);

            fprintf("+- Rear Left -------+- Rear Right ------+\n")
            fprintf("| DamperD:  %5.2f\t| DamperD:  %5.2f\t|\n", obj.rl_susp.get_damper_distance(), obj.rr_susp.get_damper_distance());
            fprintf("| PushrodD: %5.2f\t| PushrodD: %5.2f\t|\n", obj.rl_susp.get_pushrod_distance(), obj.rr_susp.get_pushrod_distance());
            fprintf("| Steering: %6.2f\t| Steering: %6.2f\t|\n", obj.rl_steering_rad()*180/pi, obj.rr_steering_rad()*180/pi);
            fprintf("| Camber:    %4.2f\t| Camber:    %4.2f\t|\n", obj.rl_susp.camber_angle()*180/pi, obj.rr_susp.camber_angle()*180/pi );
            fprintf("| TierodD:  %5.2f\t| TierodD:  %5.2f\t|\n", obj.rl_tierod, obj.rr_tierod);


            fprintf("+-------------------+-------------------+\n");

        end

        function extended_print(obj)
            obj.print_extended()
        end

        function f = wheelbase(obj)
            centre_front = obj.fl_susp.get_contact_patch() + (obj.fl_susp.get_contact_patch() - obj.fr_susp.get_contact_patch()).*0.5;
            centre_back = obj.rl_susp.get_contact_patch() + (obj.rl_susp.get_contact_patch() - obj.rr_susp.get_contact_patch()).*0.5;
            f = abs(centre_front.x - centre_back.x);
        end

        function r = fl_virtual_turning_radius(obj)
            front_track = (obj.fl_susp.get_contact_patch().y);
            wheel_base = obj.wheelbase();
            r = front_track + abs(wheel_base/tan((obj.fl_steering_rad)))*sign(-obj.fl_steering_rad);
        end

        function r = fr_virtual_turning_radius(obj)
            front_track = (obj.fr_susp.get_contact_patch().y);
            wheel_base = obj.wheelbase();
            r = front_track + abs(wheel_base/tan((obj.fr_steering_rad)))*sign(-obj.fr_steering_rad);
        end

        function plot3dpoints(obj,ax)
            obj.fl_susp.plot3d(ax);
            obj.fr_susp.plot3d(ax);
            obj.rl_susp.plot3d(ax);
            obj.rr_susp.plot3d(ax);
            obj.f_steer.plot3d(ax,'b');
            obj.r_steer.plot3d(ax,'b');

            % Tierods
            plot3dline(ax,obj.f_steer.left_clevi(), obj.fl_susp.knuckle.coord(3), 'r');
            plot3dline(ax,obj.f_steer.right_clevi(), obj.fr_susp.knuckle.coord(3), 'r');
            plot3dline(ax,obj.r_steer.left_clevi(), obj.rl_susp.knuckle.coord(3), 'r');
            plot3dline(ax,obj.r_steer.right_clevi(), obj.rr_susp.knuckle.coord(3), 'r');
        end

        function plot3d_text(obj, fig)
            t = "---------------";
            text_string = sprintf("Steering Wheel Angle: %8.2f", obj.steering_wheel_deg);

            text_string = [text_string t];

            fl = sprintf("FL Tierod Wishbone MIN distance: %6.2f", obj.fl_wishbone_tierod_distance());
            fr = sprintf("FR Tierod Wishbone MIN distance: %6.2f", obj.fr_wishbone_tierod_distance());
            rl = sprintf("RL Tierod Wishbone MIN distance: %6.2f", obj.rl_wishbone_tierod_distance());
            rr = sprintf("RR Tierod Wishbone MIN distance: %6.2f", obj.rr_wishbone_tierod_distance());


            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Tierod distance: %6.2f", obj.fl_tierod);
            fr = sprintf("FR Tierod distance: %6.2f", obj.fr_tierod);
            rl = sprintf("RL Tierod distance: %6.2f", obj.rl_tierod);
            rr = sprintf("RR Tierod distance: %6.2f", obj.rr_tierod);   

            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Pushrod distance: %6.2f", obj.fl_susp.get_pushrod_distance());
            fr = sprintf("FR Pushrod distance: %6.2f", obj.fr_susp.get_pushrod_distance());
            rl = sprintf("RL Pushrod distance: %6.2f", obj.rl_susp.get_pushrod_distance());
            rr = sprintf("RR Pushrod distance: %6.2f", obj.rr_susp.get_pushrod_distance());   

            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Damper distance: %6.2f", obj.fl_susp.get_damper_distance());
            fr = sprintf("FR Damper distance: %6.2f", obj.fr_susp.get_damper_distance());
            rl = sprintf("RL Damper distance: %6.2f", obj.rl_susp.get_damper_distance());
            rr = sprintf("RR Damper distance: %6.2f", obj.rr_susp.get_damper_distance());

            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Steering: %6.2f", obj.fl_steering_rad()*180/pi);
            fr = sprintf("FR Steering: %6.2f", obj.fr_steering_rad()*180/pi);
            rl = sprintf("RL Steering: %6.2f", obj.rl_steering_rad()*180/pi);
            rr = sprintf("RR Steering: %6.2f", obj.rr_steering_rad()*180/pi);

            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Camber: %6.2f", obj.fl_susp.camber_angle()*180/pi);
            fr = sprintf("FR Camber: %6.2f", obj.fr_susp.camber_angle()*180/pi);
            rl = sprintf("RL Camber: %6.2f", obj.rl_susp.camber_angle()*180/pi);
            rr = sprintf("RR Camber: %6.2f", obj.rr_susp.camber_angle()*180/pi);

            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Caster: %6.2f", obj.fl_susp.caster_angle()*180/pi);
            fr = sprintf("FR Caster: %6.2f", obj.fr_susp.caster_angle()*180/pi);
            rl = sprintf("RL Caster: %6.2f", obj.rl_susp.caster_angle()*180/pi);
            rr = sprintf("RR Caster: %6.2f", obj.rr_susp.caster_angle()*180/pi);

            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Scrub Radius: %6.2f", obj.fl_susp.get_scrub_radius_X());
            fr = sprintf("FR Scrub Radius: %6.2f", obj.fr_susp.get_scrub_radius_X());
            rl = sprintf("RL Scrub Radius: %6.2f", obj.rl_susp.get_scrub_radius_X());
            rr = sprintf("RR Scrub Radius: %6.2f", obj.rr_susp.get_scrub_radius_X());

            text_string = [text_string fl fr rl rr t];

            fl = sprintf("FL Height: %6.2f", obj.fl_susp.get_contact_patch().z);
            fr = sprintf("FR Height: %6.2f", obj.fr_susp.get_contact_patch().z);
            rl = sprintf("RL Height: %6.2f", obj.rl_susp.get_contact_patch().z);
            rr = sprintf("RR Height: %6.2f", obj.rr_susp.get_contact_patch().z);

            text_string = [text_string fl fr rl rr];



            h_annot = annotation(fig,'textbox', [0.01 0.01 0.3 0.99], ...
                'String', text_string, ...
                'HorizontalAlignment', 'left', ...
                'VerticalAlignment', 'top', ...
                'FontSize', 10, ...
                'Tag','CarDataAnnotation',...
                'FontWeight', 'normal', ...
                'Color', 'black', ...
                'EdgeColor', 'none', ...         % Crucial: removes the border
                'BackgroundColor', 'none', ...
                'FontName', 'Courier');

            obj.AnnotationHandle = h_annot;

        end

        function prepare(obj)
            %obj.set_front_dampers(180);
            %obj.set_rear_dampers(180);
            obj.fl_susp.set_ground_height(0);
            obj.fr_susp.set_ground_height(0);
            obj.rl_susp.set_ground_height(0);
            obj.rr_susp.set_ground_height(0);
            obj.centre_steering();
            obj.set_toe_rear(0);
            obj.set_toe_front(0);
        end

        %% Interactive 3d model

        function ui_plot3d(obj)
            obj.prepare();
            f = uifigure("Name","UI Plot");

            ax = uiaxes(f);
            axis_set(ax);

            % Slider Steering
            str_lbl = uilabel(f, "Text","Steering Wheel Angle Slider", 'Position',[20,50,200,30]);
            update_label(str_lbl, "Steering Wheel Angle: ", 0);
            steering_sld = uislider(f, "Limits",[-obj.f_steer.max_to_side,obj.f_steer.max_to_side], "Value",0, 'Position',[20,35,200,3]);
            steering_sld.ValueChangedFcn = @(source, event) steering_callback(obj, f, ax, event.Value*(-1), str_lbl);


            % Slider Height
            obj.fl_label = uilabel(f, 'Text','FL W H', 'Position',[400,120,100,200]);
            fl_sld = uislider(f, 'Limits', [-30,30], 'Value', 0,'Orientation','vertical','Position',[400, 50, 3, 100]);
            fl_sld.ValueChangedFcn = @(source, event) fl_callback(obj, f, ax, event.Value);

            obj.fr_label = uilabel(f, 'Text','FR W H', 'Position',[500,120,100,200]);
            fr_sld = uislider(f, 'Limits', [-30,30], 'Value', 0,'Orientation','vertical','Position',[500, 50, 3, 100]);
            fr_sld.ValueChangedFcn = @(source, event) fr_callback(obj, f, ax, event.Value);

            obj.rl_label = uilabel(f, 'Text','RL W H', 'Position',[600,120,100,200]);
            rl_sld = uislider(f, 'Limits', [-30,30], 'Value', 0,'Orientation','vertical','Position',[600, 50, 3, 100]);
            rl_sld.ValueChangedFcn = @(source, event) rl_callback(obj, f, ax, event.Value);

            obj.rr_label = uilabel(f, 'Text','RR W H', 'Position',[700,120,100,200]);
            rr_sld = uislider(f, 'Limits', [-30,30], 'Value', 0,'Orientation','vertical','Position',[700, 50, 3, 100]);
            rr_sld.ValueChangedFcn = @(source, event) rr_callback(obj, f, ax, event.Value);


            % Toe Slider
            uilabel(f, 'Text','Front Toe', 'Position',[20,400,100,30]);
            ftoe_sld = uislider(f, 'Limits', [-3,3], 'Value', 0, 'Position', [20,400,100,3]);
            ftoe_sld.ValueChangedFcn = @(source, event) f_toe_callback(obj, f, ax, steering_sld, event.Value);
            
            uilabel(f, 'Text','Rear Toe', 'Position',[20,350,100,30]);
            rtoe_sld = uislider(f, 'Limits', [-3,3], 'Value', 0, 'Position', [20,350,100,3]);
            rtoe_sld.ValueChangedFcn = @(source, event) r_toe_callback(obj, f, ax, event.Value);
            
            % Reset toe buttons
            f_btn = uibutton(f, 'Text','Reset Front Toe', 'Position',[400, 400, 200,30]);
            r_btn = uibutton(f, 'Text','Reset Rear Toe', 'Position',[400, 350, 200,30]);
            f_btn.ButtonPushedFcn = @(src, event) f_toe_callback(obj, f, ax, steering_sld, 0, ftoe_sld, str_lbl);
            r_btn.ButtonPushedFcn = @(src, event) r_toe_callback(obj, f, ax, 0, rtoe_sld);

            % Reset height buttons
            h_btn = uibutton(f, 'Text','Reset Heights', 'Position',[600,350,200,80]);
            h_btn.ButtonPushedFcn = @(src, event) h_callback(obj, f, ax, fl_sld, fr_sld, rl_sld, rr_sld);


            obj.common_callback(f, ax);

        end
        %% Graphing functions

        function steering_graph(obj)
            obj.prepare();
            
            steering_inputs = -obj.f_steer.max_to_side:1:obj.f_steer.max_to_side;
            left = zeros(1,length(steering_inputs));
            right = zeros(1,length(steering_inputs));
            left_camber = zeros(1,length(steering_inputs));
            right_camber = zeros(1,length(steering_inputs));
            for i=1:length(steering_inputs)
                obj.set_steering_wheel(steering_inputs(i))
                left(i) = obj.fl_steering_rad*180/pi;
                right(i) = obj.fr_steering_rad*180/pi;
                left_camber(i) = obj.fl_susp.camber_angle()*180/pi;
                right_camber(i) = obj.fr_susp.camber_angle()*180/pi;
            end
            graph_maker_double(steering_inputs, left, right, "Steering", "Left Wheel", "Right Wheel", "Steering Wheel Angle", "Tire Angle", "b-", "r-");
            graph_maker_double(steering_inputs, left_camber, right_camber, "Camber vs Steering", "Left Wheel", "Right Wheel", "Steering Wheel Angle", "Camber Angle", "b-", "r-");
        end

        function suspe_graph(obj)
            obj.prepare();
            damper_inputs = 160:1:210;
            ground_heights = -30:1:30;
            front_height = zeros(1,length(damper_inputs));
            rear_height = zeros(1,length(damper_inputs));
            front_camber = zeros(1,length(damper_inputs));
            rear_camber = zeros(1,length(damper_inputs));
            front_bump = zeros(1,length(damper_inputs));
            rear_bump = zeros(1,length(damper_inputs));
            front_caster = zeros(1,length(damper_inputs));
            rear_caster = zeros(1,length(damper_inputs));

            for i=1:length(damper_inputs)
                %obj.set_front_dampers(damper_inputs(i));
                %obj.set_rear_dampers(damper_inputs(i));
                obj.fl_susp.set_ground_height(ground_heights(i));
                obj.fr_susp.set_ground_height(ground_heights(i));
                obj.rl_susp.set_ground_height(ground_heights(i));
                obj.rr_susp.set_ground_height(ground_heights(i));

                front_height(i) = obj.fl_susp.get_contact_patch().z;
                rear_height(i) = obj.rl_susp.get_contact_patch().z;
                front_camber(i) = obj.fl_susp.camber_angle()*180/pi;
                rear_camber(i) = obj.rl_susp.camber_angle()*180/pi;
                front_bump(i) = obj.fl_steering_rad()*180/pi;
                rear_bump(i) = obj.rl_steering_rad()*180/pi;
                front_caster(i) = obj.fl_susp.caster_angle()*180/pi;
                rear_caster(i) = obj.rl_susp.caster_angle()*180/pi;
            end

            graph_maker_double(damper_inputs, front_height, rear_height, "Damper Distance vs Wheel Height", "Front Height", "Rear Height", "Damper Point to Point Distance", "Wheel Height", 'b-', 'r-');

            graph_maker(front_height, front_camber, "Front Wheel Height vs Front Camber Angle", "Wheel Height", "Camber Angle", 'b-');
            graph_maker(rear_height, rear_camber, "Rear Wheel Height vs Rear Camber Angle", "Wheel Height", "Camber Angle", 'r-');
            graph_maker_double(damper_inputs, front_camber, rear_camber, "Damper Distance vs Camber Angle", "Front Camber", 'Rear Camber', 'Damper Point to Point Distance', 'Camber Angle', 'b-', 'r-');

            graph_maker(front_height, front_bump, "Front Wheel Height vs Front Bump Steer", "Wheel Height", "Steering Angle", 'b-');
            graph_maker(rear_height, front_bump, "Rear Wheel Height vs Rear Bump Steer", "Wheel Height", "Steering Angle", 'r-');
            graph_maker_double(damper_inputs, front_bump, rear_bump, "Damper Distance vs Bump Steer", "Front Bump Steer", 'Rear Bump Steer', 'Damper Point to Point Distance', 'Steering Angle', 'b-', 'r-');


            graph_maker(front_height, front_caster, "Front Wheel Height vs Front Caster", "Wheel Height", "Caster Angle", 'b-');
            graph_maker(rear_height, rear_caster, "Rear Wheel Height vs Rear Caster", "Wheel Height", "Caster Angle", 'r-');
            graph_maker_double(damper_inputs, front_caster, rear_caster, "Damper Distance vs Caster Angle", "Front Caster", 'Rear Caster', 'Damper Point to Point Distance', 'Caster Angle', 'b-', 'r-');
        end




        function graph(obj)
            obj.steering_graph();
            obj.suspe_graph();

        end

    end
end

