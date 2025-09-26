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
    
    methods
        function obj = suspension(damper, rocker,pushrod, u_wishbone, knuckle, l_wishbone)
            obj.damper = damper;
            obj.rocker = rocker;
            obj.pushrod = pushrod;
            obj.u_wishbone = u_wishbone;
            obj.knuckle = knuckle;
            obj.l_wishbone = l_wishbone;
        end

        function update(obj)
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
        
        function set_damper_distance(obj, damper_distance)
            obj.damper.set_length(damper_distance);
            obj.update();
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

