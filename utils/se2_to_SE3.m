function T = se2_to_SE3(pose2d)
% SE2_TO_SE3 Converts [x, y, theta] planar pose to full 4x4 SE(3) matrix

x = pose2d(1);
y = pose2d(2);
theta = pose2d(3);

R = [cos(theta), -sin(theta), 0;
     sin(theta),  cos(theta), 0;
           0,           0,    1];

t = [x; y; 0];  % Planar motion = Z=0

T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = t;
end
