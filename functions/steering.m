classdef steering < handle
    properties
        rack_centre (1,1) v3
        rack_direction (1,1) v3
        pinion_diameter (1,1) double
        
    end
    
    methods
        function obj = steering(inputArg1,inputArg2)
            %STEERING Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

