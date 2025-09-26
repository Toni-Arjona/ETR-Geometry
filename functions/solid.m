classdef solid < handle
    %SOLID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        staticPoints (1,:) v3 %No-Rotation & No-translation coordinates of points of the solid
    end

    properties (Access = public)
        coord (1,:) v3 %Free-moving coordinates of the solid
    end

    methods (Access = private)
        function vector = first3PointsNormalPlane(obj)
            helper1 = (obj.staticPoints(2)-obj.staticPoints(1))';
            helper2 = (obj.staticPoints(3)-obj.staticPoints(1))';
            vector = (helper1^helper2)';
        end

        function vector = first3CoordNormalPlane(obj)
            helper1 = (obj.coord(2)-obj.coord(1))';
            helper2 = (obj.coord(3)-obj.coord(1))';
            vector = (helper1^helper2)';
        end

        function matrix = matrixFirst3Points(obj)
            a1 = obj.staticPoints(1)';
            a2 = obj.staticPoints(2)';
            a3 = obj.staticPoints(3)';
            matrix = [  a1.x, a1.y, a1.z
                        a2.x, a2.y, a2.z
                        a3.x, a3.y, a3.z];
            matrix = transpose(matrix);
        end

        function matrix = matrixFirst3Coord(obj)
            a1 = obj.coord(1)';
            a2 = obj.coord(2)';
            a3 = obj.coord(3)';
            matrix = [  a1.x, a1.y, a1.z
                        a2.x, a2.y, a2.z
                        a3.x, a3.y, a3.z];
            matrix = transpose(matrix);
        end
    end
    
    methods (Access = public)
        function obj = solid(points3DVect)
            if(length(points3DVect)>=3)
                if(isa(points3DVect(1), 'v3'))
                    obj.staticPoints = points3DVect;
                    obj.coord = obj.staticPoints; 
                else
                    fprintf('Unvalid datatype for the variable points3DVector. It should contain a vector of v3 points.\n');
                end
            else
                fprintf('Length of vectors is not correct. They may be under 3. If the solid you are trying to generate has 2 points then it should be made using the class rod.\');
            end
        end
        
        function angle_rad = rotationFromStatic(obj)
            angle_rad = anglev3(obj.first3PointsNormalPlane(), obj.first3CoordNormalPlane());
        end

        function matrix = rotationMatrixFromStatic(obj)
            m = (obj.matrixFirst3Points\obj.matrixFirst3Coord);
            matrix = inv(m);
        end

        function setPoint(obj, index, coord)

            % Determinamos sobre que puntos vamos a rotar
            if(index == 1)
                i = 2;
                j = 3;
            elseif(index == 2)
                i = 1;
                j = 3;
            elseif(index == 3)
                i = 1;
                j = 2;
            else
                i = 1;
                j = 2;
            end

            previous_position = obj.coord(index); % Save the previous coordinate for later
            rotation_plane_dir = (obj.coord(j)-obj.coord(i))'; % Encontramos vector director del plano del punto
            rotation_plane_D = -(rotation_plane_dir*obj.coord(index)); % D del plano en representacion Ax + By + Cz + D = 0
            rotation_centre = obj.coord(i) + (rotation_plane_dir .* (-rotation_plane_D-(rotation_plane_dir*obj.coord(i))/(rotation_plane_dir*rotation_plane_dir) )); % Centro de rotación del punto
            rotation_radius = ((obj.coord(j) - obj.coord(i))^(obj.coord(i) - obj.coord(index))).'/((obj.coord(j)-obj.coord(i)).'); % Radio de rotación del punto
            temp_dist = ((rotation_plane_dir*coord) + rotation_plane_D)/rotation_plane_dir.modulus(); % Distancia de la coordenada dada al plano de rotación
            coord = coord - rotation_plane_dir.*temp_dist; % Se proyecta la coordenada input sobre el plano de rotación
            temp_dir = (coord - rotation_centre)'; % Dirección desde centro de rotación al punto proyectado 
            obj.coord(index) = rotation_centre + temp_dir.*rotation_radius; % Se acerca el punto proyectado hasta el radio correcto
            
            % Se rotan los demás puntos en consecuencia
            rotation_angle = anglev3((obj.coord(index)-rotation_centre)', (previous_position-rotation_centre)' ); % We get the rotation angle from the movement of the assigned point
            if((previous_position-rotation_centre)' == (obj.coord(index)-rotation_centre)' | (previous_position-rotation_centre)' == -(obj.coord(index)-rotation_centre)') % Parallel axis
                rotation_direction = rotation_plane_dir; % This is to check for edgecases when the revolution is 180º, since the orientation of the revolution doesn't matter
            else
                rotation_direction = ((previous_position-rotation_centre)'^(obj.coord(index)-rotation_centre)')'; % It may be different from the plane direction since we always assume a right-hand-rule rotation.
            end
            if(rotation_plane_dir == -rotation_direction)
                rotation_angle = -rotation_angle;
            end
            for n = 1:(length(obj.coord)-2)
                k = 2+n; % Indice del punto que rotamos
                if k == index
                    continue % Si ya hemos editado el índice de input no lo vamos a volver a editar
                end
                % The same steps as before for each point
                rotation_plane_D = -(rotation_plane_dir*obj.coord(k));
                rotation_centre = obj.coord(i) + (rotation_plane_dir .* (-rotation_plane_D-(rotation_plane_dir*obj.coord(i))/(rotation_plane_dir*rotation_plane_dir) ));
                rotation_radius = ((obj.coord(j) - obj.coord(i))^(obj.coord(i) - obj.coord(k))).'/((obj.coord(j)-obj.coord(i)).');

                % Apply rotation around axis of rotation
                zero_angle_direction = (obj.coord(k)-rotation_centre)';
                pi_halfs_direction = rotation_plane_dir^zero_angle_direction;
                obj.coord(k) = point_in_3d_circle(rotation_centre, rotation_angle, rotation_radius, zero_angle_direction, pi_halfs_direction);

            end
        end

        function free_move(obj, index, coord)
            displacement = coord - obj.coord(index);
            obj.coord(index) = coord;
            for i = 1:length(obj.coord)
                if i == index
                    continue
                end
                obj.coord(i) = obj.coord(i) + displacement;
            end
        end

        function fixed_free_move(obj, fixed_index, free_index, coord)
            rotation_vector_1 = (obj.coord(free_index) - obj.coord(fixed_index))';
            rotation_vector_2 = (coord - obj.coord(fixed_index))';
            rotation_angle = anglev3( rotation_vector_1, rotation_vector_2 );
            %rotation_vector_2 = rotation_direction ^ rotation_vector_1;
            rotation_direction = (rotation_vector_2 ^ rotation_vector_1)';
            rotation_vector_2 = rotation_vector_1 ^ rotation_direction;

            
            for i = 1:length(obj.coord)
                if i == fixed_index
                    continue
                end

                rotation_centre = point_line_projection( obj.coord(i), rotation_direction, obj.coord(fixed_index) );
                rotation_radius = (obj.coord(i) - rotation_centre).';
                if(obj.coord(i) == rotation_centre | obj.coord(i) == -rotation_centre)
                    continue
                end
                rotation_vector_1 = (obj.coord(i) - rotation_centre)';
                if( rotation_vector_1 == rotation_direction | rotation_vector_1 == -rotation_direction)
                    continue
                end
                rotation_vector_2 = (rotation_vector_1 ^ rotation_direction)';
                obj.coord(i) = point_in_3d_circle(rotation_centre, rotation_angle, rotation_radius, rotation_vector_1, rotation_vector_2);
            end
        end

        function print(obj)
            for i = 1:length(obj.coord)
                obj.coord(i).print();
            end
        end
    end
end

