classdef steering < handle
    properties
        rack_centre (1,1) v3
        rack_right_direction (1,1) v3
        pinion_diameter (1,1) double
        rack_centre_distance (1,1) double
        front_pinion (1,1)
    end
    
    methods (Access = public)
        function obj = steering(rack_centre, pinion_diameter, rack_centre_distance, front_pinion)
            arguments 
                rack_centre (1,1) v3
                pinion_diameter (1,1) double
                rack_centre_distance (1,1) double
                front_pinion (1,1)
            end
            obj.rack_centre = point_plane_projection(rack_centre, v3(0,1,0), 0);

            obj.rack_right_direction = v3(0,1,0);
            obj.pinion_diameter = abs(pinion_diameter);
            obj.rack_centre_distance = abs(rack_centre_distance);  
            obj.front_pinion = front_pinion;
        end
        
        function point = left_clevi(obj)
            arguments (Input)
                obj steering
            end
            arguments (Output)
                point v3
            end
            point = obj.rack_centre - obj.rack_right_direction.*obj.rack_centre_distance;
        end

        function point = right_clevi(obj)
            arguments (Input)
                obj steering
            end
            arguments (Output)
                point v3
            end
            point = obj.rack_centre + obj.rack_right_direction.*obj.rack_centre_distance;
        end
        
        function centre_steering(obj)
            arguments
                obj steering
            end
            obj.rack_centre = point_plane_projection(obj.rack_centre, v3(0,1,0), 0);
        end

        function steer(obj, steering_pinion_angle_radians) % Takes into consideration the previous steering
            arguments (Input)
                obj steering
                steering_pinion_angle_radians double
            end
            % The rack displacement will be the same as the arc length of the pinion
            arc_displacement = steering_pinion_angle_radians*obj.pinion_diameter/2;

            if(obj.front_pinion == true)
                obj.rack_centre.y = obj.rack_centre.y + arc_displacement;
            else
                obj.rack_centre.y = obj.rack_centre.y - arc_displacement;
            end
        end

        function set_steering(obj, steering_pinion_angle_radians) % Absolute steering
            arguments (Input)
                obj steering
                steering_pinion_angle_radians double
            end
            % The rack displacement will be the same as the arc length of the pinion
            arc_displacement = steering_pinion_angle_radians*obj.pinion_diameter/2;

            if(obj.front_pinion == true)
                obj.rack_centre.y = arc_displacement;
            else
                obj.rack_centre.y = -arc_displacement;
            end
        end

    end
end

