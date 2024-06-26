%% Lidar Range Test by Eric Foss
% Test script to develop range finder measurement dynamics. Uses rotation
% matrixes to determine height of Lander Junior based off LiDAR-lite V3HP.
% Defines body axis lidar orientation idk yet UPDATE


%Tait-Bryan Angles: z -> y' -> x'' yaw pitch roll sequence 
phi = -15; %rotation about x'' : roll
theta = 15; %rotation about y' : pitch
psi = 0; %rotation about z : yaw

sphi = sind(phi); cphi = cosd(phi);
stheta = sind(theta); ctheta = cosd(theta);
spsi = sind(psi); cpsi = cosd(psi);

% Rotation Matrices
Rx = [1 0 0;
      0 cphi -sphi;
      0 sphi cphi];

Ry = [ctheta 0 stheta;
      0 1 0;
      -stheta 0 ctheta];

Rz = [cpsi -spsi 0;
      spsi cpsi 0;
      0 0 1];

R = Rz*Ry*Rx; %full rotation matrix 
% INTRINSIC ROTATIONS: z -> y' -> x''
% EXTRINSIC ROTATIONS: x -> y -> z


% Lidar Measurement
zhat = [0; 0; 1]; %lidar orientation

m = 5; %measurement from lidar (lidar magnitude)

L = m*L_hat;






