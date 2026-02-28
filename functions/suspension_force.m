function FORCE_VECTOR = suspension_force(force, moments, force_coord, susp, clevi_point)
arguments
    force v3
    moments v3
    force_coord v3
    susp suspension
    clevi_point v3
end
    num_of_points = 21;
    zero_vector = v3(0,0,0);
    FORCE_VECTOR = zeros(1, num_of_points, like=zero_vector);

    %% Knuckle
    u =[   (susp.l_wishbone.coord(3) - susp.l_wishbone.coord(1))' % Lower Front
           (susp.l_wishbone.coord(3) - susp.l_wishbone.coord(2))' % Lower Rear
           (susp.u_wishbone.coord(3) - susp.u_wishbone.coord(1))' % Upper Front
           (susp.u_wishbone.coord(3) - susp.u_wishbone.coord(2))' % Upper Rear
           (susp.pushrod.p2 - susp.pushrod.p1)'                   % Pushrod
           (susp.knuckle.coord(3) - clevi_point)'                 % Tie-Rod
    ];
    M = [susp.l_wishbone.coord(3) ^ u(1)
         susp.l_wishbone.coord(3) ^ u(2)
         susp.u_wishbone.coord(3) ^ u(3)
         susp.u_wishbone.coord(3) ^ u(4)
         susp.u_wishbone.coord(4) ^ u(5)
         susp.knuckle.coord(3) ^ u(6)
    ];
    ux = [u(1).x, u(2).x, u(3).x, u(4).x, u(5).x, u(6).x];
    uy = [u(1).y, u(2).y, u(3).y, u(4).y, u(5).y, u(6).y];
    uz = [u(1).z, u(2).z, u(3).z, u(4).z, u(5).z, u(6).z];
    Mx = [M(1).x, M(2).x, M(3).x, M(4).x, M(5).x, M(6).x];
    My = [M(1).y, M(2).y, M(3).y, M(4).y, M(5).y, M(6).y];
    Mz = [M(1).z, M(2).z, M(3).z, M(4).z, M(5).z, M(6).z];
    A = [   ux
            uy
            uz
            Mx
            My
            Mz];
    
    total_moment = (force_coord ^ force + moments.*1000);
    b = [  -force.x
           -force.y
           -force.z
           -total_moment.x
           -total_moment.y
           -total_moment.z
        ];

    knuckle_forces = inv(A) * b;
    FORCE_VECTOR(1) = -u(1) .* knuckle_forces(1);
    FORCE_VECTOR(2) = -u(2) .* knuckle_forces(2);
    FORCE_VECTOR(3) = -(FORCE_VECTOR(1) + FORCE_VECTOR(2));
    FORCE_VECTOR(4) = -u(3) .* knuckle_forces(3);
    FORCE_VECTOR(5) = -u(4) .* knuckle_forces(4);
    FORCE_VECTOR(6) = -(FORCE_VECTOR(3) + FORCE_VECTOR(4));
    FORCE_VECTOR(7) = u(5) .* knuckle_forces(5);
    FORCE_VECTOR(8) = -FORCE_VECTOR(7);
    FORCE_VECTOR(11) = u(6) .* knuckle_forces(6);
    FORCE_VECTOR(12) = FORCE_VECTOR(11);

    %% Rocker
    %TODO

%    u = [   (susp.pushrod.p1 - susp.pushrod.p2)'
%            (susp.damper.p2 - susp.damper.p1)'
%            v3(1,0,0)
%            v3(0,1,0)
%            v3(0,0,1)
%            v3(1,0,0)
%            v3(0,1,0)
%            v3(0,0,1)];
%    M = [   susp.rocker.coord(4) ^ u(1)
%            susp.rocker.coord(3) ^ u(2)
%            susp.rocker.coord(1) ^ u(3)
%            susp.rocker.coord(1) ^ u(4)
%            susp.rocker.coord(1) ^ u(5)
%            susp.rocker.coord(2) ^ u(6)
%            susp.rocker.coord(2) ^ u(7)
%            susp.rocker.coord(2) ^ u(8)];
%    ux = [u(1).x, u(2).x, u(3).x, u(4).x, u(5).x, u(6).x, u(7).x, u(8).x];
%    uy = [u(1).y, u(2).y, u(3).y, u(4).y, u(5).y, u(6).y, u(7).y, u(8).y];
%    uz = [u(1).z, u(2).z, u(3).z, u(4).z, u(5).z, u(6).z, u(7).z, u(8).z];
%    Mx = [M(1).x, M(2).x, M(3).x, M(4).x, M(5).x, M(6).x, M(7).x, M(8).x];
%    My = [M(1).y, M(2).y, M(3).y, M(4).y, M(5).y, M(6).y, M(7).y, M(8).y];
%    Mz = [M(1).z, M(2).z, M(3).z, M(4).z, M(5).z, M(6).z, M(7).z, M(8).z];
%    %
%
%    A = [   ux
%            uy
%            uz
%            Mx
%            My
%            Mz];

%    force_coord = susp.damper.p2;
%    force = FORCE_VECTOR(7);
%    total_moment = (force_coord ^ force + moments.*1000);
%    b = [  -force.x
%           -force.y
%           -force.z
%           -total_moment.x
%           -total_moment.y
%           -total_moment.z
%        ];

%    rocker_forces = inv(A) * b;

    


end