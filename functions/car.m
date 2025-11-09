classdef car < handle
    %CAR 
    %   Using suspension and steering
    
    properties
        fl_susp suspension
        fr_susp suspension
        rl_susp suspension
        rr_susp suspension
        f_steer steering
        r_steer steering
        f_tierod double
        r_tierod double
    end
    
    methods
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

            obj.f_steer.set_steering(steering_wheel_deg*pi/180);
            
            % Iterate to make the front wheels converge
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

        %% Set toe

        function set_toe_front(obj, toe_deg)
            arguments
                obj car
                toe_deg double 
            end
            obj.fl_susp.set_toe(toe_deg);
            obj.fr_susp.set_toe(toe_deg);

            % Updating the tierods length
            obj.f_tierod = (obj.fl_susp.knuckle.coord(3) - obj.f_steer.left_clevi()).';
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
        end
        function set_rear_toe(obj, toe_deg)
            obj.set_toe_rear(toe_deg);
        end


        end
    end
end

