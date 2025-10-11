classdef v3
    properties
        x (1,1) double {mustBeFinite} %X-value
        y (1,1) double {mustBeFinite} %Y-value
        z (1,1) double {mustBeFinite} %Z-value
    end
    methods

        % Constructor
        function obj = v3(x,y,z)
            if nargin > 0
                obj.x = x;
                obj.y = y;
                obj.z = z;
            end
        end

        function print(obj)
            fprintf("{%14.8f\t%14.8f\t%14.8f}\n", obj.x, obj.y, obj.z);
        end

        function mod = modulus(obj)
            arguments (Input)
                obj (1,1) v3
            end
            arguments (Output)
                mod (1,1) double
            end
            mod = sqrt( obj.x*obj.x + obj.y*obj.y + obj.z*obj.z );
        end

        function c = plus(a,b) % a+b -> sum
            c = v3( a.x+b.x, a.y+b.y, a.z+b.z );
        end

        function c = minus(a,b) %a-b -> subtraction
            c = v3(a.x-b.x, a.y-b.y, a.z-b.z);
        end

        function c = times(a,b) %a.*b -> vector scaling
           if(isa(a,'double') & isa(b,'v3'))
               c = v3(a*b.x, a*b.y, a*b.z);
           elseif(isa(a,'v3') & isa(b,'double'))
               c = v3(a.x*b, a.y*b, a.z*b);
           else
               c = v3(inf, inf, inf);
               fprintf('Invalid dataTypes for the operator: .* \n');
           end
        end

        function c = rdivide(a,b) %a./b -> vector division
           if(isa(a,'v3') & isa(b,'double'))
               c = v3(a.x*b, a.y*b, a.z*b);
           else
               c = v3(inf, inf, inf);
               fprintf('Invalid dataTypes for the operator: ./ \n');
           end
        end

        function c = power(a,b) %a.^b -> elementwise power
           if(isa(a,'v3') & isa(b,'double'))
               c = v3(a.x^b, a.y^b, a.z^b);
           else
               c = v3(inf, inf, inf);
               fprintf('Invalid dataTypes for the operator: .^ \n');
           end
        end

        function c = mtimes(a,b) % a*b -> scalar product
            c = a.x*b.x + a.y*b.y + a.z*b.z;
        end

        function c = mpower(a,b) %a^b -> vectorial
            c = v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
        end

        function c = or(a,b) % a | b -> scalar component
            c = (a*b)/(a.modulus());
        end

        function c = uminus(obj) %-obj -> reverse
            c = v3(-obj.x, -obj.y, -obj.z);
        end

        function c = ctranspose(obj) %obj' -> normalize
            mod = obj.modulus();
            if mod <= 1e-8
                fprintf("Vector modulus calculated to 0. Vector geometry may be incorrectly interpreted.\n")
            end
            c = v3(obj.x/mod, obj.y/mod, obj.z/mod);
        end

        function c = transpose(obj) %obj.' -> modulus
            c = obj.modulus();
        end

        function c = normalize(obj)
            c = obj';
        end

        function c = rotation(obj, matrix)
            c = matrix*[obj.x; obj.y; obj.z];
        end

        function c = eq(a,b) % obj1 == obj2 -> equal
            if(isa(a,'v3') && isa(b,'v3') )
                c = anglev3(a,b)<=1e-10;
            else
                fprintf("Invalid arguments for the comparison.\n");
            end
        end

        function c = ne(a,b) % obj1 ~= obj2 -> not equal
            if(isa(a,'v3') && isa(b,'v3') )
                c = anglev3(a,b)>=1e-10;
            else
                fprintf("Invalid arguments for the comparison.\n");
            end
        end

    end
end


