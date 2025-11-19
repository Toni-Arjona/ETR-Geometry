clf
figure
hold on
axis equal
grid on
view(3)
xlabel('X')
ylabel('Y')
zlabel('Z')

% Example coordinates (units arbitrary)
% Chassis points
U1 = [ 0  0  0.3];   % Upper arm inner joint 1
U2 = [ 0  0.6 0.3];  % Upper arm inner joint 2
L1 = [ 0  0  0.0];   % Lower arm inner joint 1
L2 = [ 0  0.6 0.0];  % Lower arm inner joint 2

% Wheel upright points
Uhub = [0.7 0.3 0.35];  % Upper hub joint
Lhub = [0.7 0.3 -0.05]; % Lower hub joint

% Damper top and bottom
Dtop = [0.2 0.3 0.5];
Dbot = [0.5 0.3 0.1];

% Draw arms as lines
plot3([U1(1) Uhub(1)], [U1(2) Uhub(2)], [U1(3) Uhub(3)], 'k', 'LineWidth', 2);
plot3([U2(1) Uhub(1)], [U2(2) Uhub(2)], [U2(3) Uhub(3)], 'k', 'LineWidth', 2);

plot3([L1(1) Lhub(1)], [L1(2) Lhub(2)], [L1(3) Lhub(3)], 'k', 'LineWidth', 2);
plot3([L2(1) Lhub(1)], [L2(2) Lhub(2)], [L2(3) Lhub(3)], 'k', 'LineWidth', 2);

% Draw a cylinder for the damper
drawCylinderBetweenPoints(Dtop, Dbot, 0.03, 30, [0.9 0 0]);   % red damper body
drawCylinderBetweenPoints(Dtop, Dbot, 0.02, 30, [0.8 0.8 0.8]); % inner rod

% Optional: small cylinder for the wheel hub
drawCylinderBetweenPoints(Uhub, Lhub, 0.04, 20, [0.2 0.2 0.6]);

title('Simple 3D suspension sketch')