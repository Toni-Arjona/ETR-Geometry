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

        function setPoint(obj, index, coord, index_hardpoint_1, index_hardpoint_2)

            % Determinamos sobre que puntos vamos a rotar
            i = index_hardpoint_1;
            j = index_hardpoint_2;

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
            if((previous_position-rotation_centre)' == (obj.coord(index)-rotation_centre)' | (previous_position-rotation_centre)' == -(obj.coord(index)-rotation_centre)')%#ok<BDSCA>
                rotation_direction = rotation_plane_dir; % This is to check for edgecases when the revolution is 180º, since the orientation of the revolution doesn't matter
            else
                rotation_direction = ((previous_position-rotation_centre)'^(obj.coord(index)-rotation_centre)')'; % It may be different from the plane direction since we always assume a right-hand-rule rotation.
            end
            if(rotation_plane_dir == -rotation_direction) %#ok
                rotation_angle = -rotation_angle;
            end
            for n = 1:(length(obj.coord))
                k = n; % Indice del punto que rotamos
                if k == index || k == index_hardpoint_1 || k == index_hardpoint_2
                    continue % Si ya hemos editado el índice de input no lo vamos a volver a editar
                end
                % The same steps as before for each point
                rotation_plane_D = -(rotation_plane_dir*obj.coord(k));
                rotation_centre = obj.coord(i) + (rotation_plane_dir .* (-rotation_plane_D-(rotation_plane_dir*obj.coord(i))/(rotation_plane_dir*rotation_plane_dir) ));
                rotation_radius = ((obj.coord(j) - obj.coord(i))^(obj.coord(i) - obj.coord(k))).'/((obj.coord(j)-obj.coord(i)).');

                % Apply rotation around axis of rotation
                if( (obj.coord(k) - rotation_centre).' <= 1e-10 ) %#ok<BDSCA>
                    continue
                end
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
                if( rotation_vector_1 == rotation_direction | rotation_vector_1 == -rotation_direction) %#ok<BDSCI>
                    continue
                end
                rotation_vector_2 = (rotation_vector_1 ^ rotation_direction)';
                obj.coord(i) = point_in_3d_circle(rotation_centre, rotation_angle, rotation_radius, rotation_vector_1, rotation_vector_2);
            end
        end

        function setDirection(obj, index_hardpoint_1, index_hardpoint_2, index_direction_start, index_direction_end, direction)
            arguments (Input)
                obj solid
                index_hardpoint_1 {mustBeInteger}
                index_hardpoint_2 {mustBeInteger}
                index_direction_start {mustBeInteger}
                index_direction_end {mustBeInteger}
                direction v3
            end
            direction = direction';

            rotation_normal = (obj.coord(index_hardpoint_2) - obj.coord(index_hardpoint_1))';
            start_plane_D = -( rotation_normal*obj.coord(index_direction_start) );
            start_plane_centre = point_plane_projection( obj.coord(index_hardpoint_1), rotation_normal, start_plane_D );
            end_plane_D = -( rotation_normal*obj.coord(index_direction_end) );

            end_projected_on_start_plane = point_plane_projection( obj.coord(index_direction_end), rotation_normal, start_plane_D );
            start_end_direction_on_start_plane = (end_projected_on_start_plane - obj.coord(index_direction_start))';
            end_rotation_radius = (end_projected_on_start_plane - start_plane_centre).';
            angle = anglev3(start_end_direction_on_start_plane, direction);

            while angle > 1e-6
                plane_vector_1 = (end_projected_on_start_plane - start_plane_centre)';
                plane_vector_2 = (rotation_normal ^ plane_vector_1)';
                if(plane_vector_2*direction < 0)
                    plane_vector_2 = -plane_vector_2; % Reverse in case the angle has to be in reverse
                end
                ideal_point = point_in_3d_circle(start_plane_centre, angle, end_rotation_radius, plane_vector_1, plane_vector_2);
                ideal_point = point_plane_projection(ideal_point, rotation_normal, end_plane_D);
                obj.setPoint(index_direction_end, ideal_point, index_hardpoint_1, index_hardpoint_2);
    
                end_projected_on_start_plane = point_plane_projection( obj.coord(index_direction_end), rotation_normal, start_plane_D );
                start_end_direction_on_start_plane = (end_projected_on_start_plane - obj.coord(index_direction_start))';
                angle = anglev3((obj.coord(index_direction_end) - obj.coord(index_direction_start))', direction);
                angle = angle_projection(angle, (obj.coord(index_direction_end) - obj.coord(index_direction_start))' ^ direction, rotation_normal);
            end

        end

        function rotate(obj, index_to_rotate, index_hardpoint_1, index_hardpoint_2, angle_rad)
            % From 1 to 2, right hand rotation rule applies
            arguments
                obj solid
                index_to_rotate {mustBeInteger}
                index_hardpoint_1 {mustBeInteger} 
                index_hardpoint_2 {mustBeInteger}
                angle_rad double
            end

            rotation_normal = (obj.coord(index_hardpoint_2) - obj.coord(index_hardpoint_1))';
            start_plane_D = -( rotation_normal*obj.coord(index_to_rotate) );
            rotation_centre = point_plane_projection( obj.coord(index_hardpoint_1), rotation_normal, start_plane_D );
            
            rotation_radius = (obj.coord(index_to_rotate) - rotation_centre).';
            zero_angle_vector = (obj.coord(index_to_rotate) - rotation_centre)';
            pi_2_angle_vector = (rotation_normal ^ zero_angle_vector)';

            desired_point = point_in_3d_circle(rotation_centre, angle_rad, rotation_radius, zero_angle_vector, pi_2_angle_vector);

            obj.setPoint(index_to_rotate, desired_point, index_hardpoint_1, index_hardpoint_2);
        end

        function mirror_solid = mirror_on_plane(obj, plane_direction, plane_D)
            arguments (Input)
                obj solid
                plane_direction v3
                plane_D double
            end
            arguments (Output)
                mirror_solid solid
            end
            plane_direction = plane_direction';

            mirror_solid_vector = zeros(1, length(obj.coord));
            for i = 1:length(obj.coord)
                mirror_solid_vector(i) = point_plane_mirror( obj.coord(i), plane_direction, plane_D );
            end
            mirror_solid = solid(mirror_solid_vector);
        end



        function print(obj)
            for i = 1:length(obj.coord)
                obj.coord(i).print();
            end
            if(length(obj.coord) == 3 | length(obj.coord) == 4)
                for i = 1:length(obj.coord)
                    for j = i:length(obj.coord)
                        if(i==j)
                            continue
                        end
                        fprintf("Length %1d-%1d: %14.8f\n", i,j, (obj.coord(i)-obj.coord(j)).')
                    end
                end
            end
        end
    end
end

