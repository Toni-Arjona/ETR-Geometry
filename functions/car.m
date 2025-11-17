classdef car < handle
    %CAR 
    %   Using suspension and steering
    
    properties (Access=private)
        fl_susp suspension
        fr_susp suspension
        rl_susp suspension
        rr_susp suspension
        f_steer steering
        r_steer steering
        f_tierod double
        r_tierod double
        steering_wheel_deg double
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
                place = obj.f_steer.left_clevi() + direction.*obj.f_tierod;
                obj.fl_susp.set_knuckle(place);
                error = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi()).' - obj.f_tierod;
            end

            error = 1;
            while abs(error) > 1e-6
                direction = (obj.fr_susp.knuckle.coord(3) - obj.f_steer.right_clevi())';
                place = obj.f_steer.right_clevi() + direction.*obj.f_tierod;
                obj.fr_susp.set_knuckle(place);
                error = (obj.fr_susp.knuckle.coord(3) - obj.f_steer.right_clevi()).' - obj.f_tierod;
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
                place = obj.r_steer.left_clevi() + direction.*obj.r_tierod;
                obj.rl_susp.set_knuckle(place);
                error = (obj.rl_susp.knuckle.coord(3) - obj.r_steer.left_clevi()).' - obj.r_tierod;
            end

            error = 1;
            while abs(error) > 1e-6
                direction = (obj.rr_susp.knuckle.coord(3) - obj.r_steer.right_clevi())';
                place = obj.r_steer.right_clevi() + direction.*obj.r_tierod;
                obj.rr_susp.set_knuckle(place);
                error = (obj.rr_susp.knuckle.coord(3) - obj.r_steer.right_clevi()).' - obj.r_tierod;
            end
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

            obj.f_tierod = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi()).';
            obj.r_tierod = (obj.rl_susp.knuckle.coord(3) - obj.r_steer.left_clevi()).';
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
            obj.f_tierod = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi()).';
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
            obj.r_tierod = (obj.rl_susp.knuckle.coord(3) - obj.r_steer.left_clevi()).';
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
            fprintf("| TierodD:  %5.2f\t| TierodD:  %5.2f\t|\n", obj.f_tierod, obj.f_tierod);

            fprintf("+- Rear Left -------+- Rear Right ------+\n")
            fprintf("| DamperD:  %5.2f\t| DamperD:  %5.2f\t|\n", obj.rl_susp.get_damper_distance(), obj.rr_susp.get_damper_distance());
            fprintf("| PushrodD: %5.2f\t| PushrodD: %5.2f\t|\n", obj.rl_susp.get_pushrod_distance(), obj.rr_susp.get_pushrod_distance());
            fprintf("| Steering: %6.2f\t| Steering: %6.2f\t|\n", obj.rl_steering_rad()*180/pi, obj.rr_steering_rad()*180/pi);
            fprintf("| Camber:    %4.2f\t| Camber:    %4.2f\t|\n", obj.rl_susp.camber_angle()*180/pi, obj.rr_susp.camber_angle()*180/pi );
            fprintf("| TierodD:  %5.2f\t| TierodD:  %5.2f\t|\n", obj.r_tierod, obj.r_tierod);


            fprintf("+-------------------+-------------------+\n");
        end
    end
end


