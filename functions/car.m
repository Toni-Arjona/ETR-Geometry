classdef car
    %CAR 
    %   Using suspension and steering
    
    properties
        fl_susp
        fr_susp
        rl_susp
        rr_susp
        steer
    end
    
    methods
        function obj = car(fl_susp, fr_susp, rl_susp, rr_susp, steer)
            arguments (Input)
                fl_susp suspension
                fr_susp suspension
                rl_susp suspension
                rr_susp suspension
                steer steering
            end
            obj.fl_susp = fl_susp;
            obj.fr_susp = fr_susp;
            obj.rl_susp = rl_susp;
            obj.rr_susp = rr_susp;
            obj.steer = steer;
        end


        function centre_steering(obj)
            obj.steer.centre_steering();
            obj.fl_susp.centre_steering();
            obj.fr_susp.centre_steering();
            obj.rl_susp.centre_steering();
            obj.rr_susp.centre_steering();
        end

        function set_front_toe(obj, toe_ang_deg)
            % Positive toe is TOE IN
            arguments
                obj car
                toe_ang_deg double
            end

            



        end

        
        function set_steering(obj,steering_pinion_angle_radians)
            arguments
                obj car
                steering_pinion_angle_radians double
            end
            obj.steer.set_steering(steering_pinion_angle_radians);




        end
    end
end

