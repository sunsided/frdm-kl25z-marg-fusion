clc;

m10 = 46681;
m11 = 45998;
m12 = -262;

m20 = -1721;
m21 = 3720;
m22 = 65408;

norm_m1 = norm([m10 m11 m12]/65535)
norm_m2 = norm([m20 m21 m22]/65535)

m0 = cross([m10 m11 m12]/65535, [m20 m21 m22]/65535) * 65535;

norm_m0 = norm(m0/65535)

m00 = m0(1)/norm_m0
m01 = m0(2)/norm_m0
m02 = m0(3)/norm_m0

% Calculate DCM

DCM = [m00 m01 m02;
       m10 m11 m12;
       m20 m21 m22]

DCM = DCM / 65535

close all; 
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
axis square;

figure;
quiver3(0, 0, 0, DCM(1,1), DCM(1,2), DCM(1,3), 'r'); hold on;
quiver3(0, 0, 0, DCM(2,1), DCM(2,2), DCM(2,3), 'g', 'LineWidth', 3); hold on;
quiver3(0, 0, 0, DCM(3,1), DCM(3,1), DCM(3,3), 'b', 'LineWidth', 3); hold on;
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
axis square;

% Calculate quaterneion
   
quat = quaternionFromRotation(DCM)
%quat = [quat(2:4) quat(1)]
quatAsFix = quat*65535

% Calculate angle from quaternion

R = DCM;
pitch = -asind(R(3, 1));
roll  =  atan2d(R(3, 2), R(3, 3));
yaw   =  atan2d(R(2, 1), R(1, 1));

angleFromDCM = [roll pitch yaw]

% Directly convert quaternion to euler angles

[roll pitch yaw] = quaternionToEuler(quat);
angleFromQuat = [roll pitch yaw]

% Test vector rotation

qx = quaternionRotateVector(quat, [1 0 0])
dx = (DCM * [1 0 0]')'

qy = quaternionRotateVector(quat, [0 1 0])
dy = (DCM * [0 1 0]')'

qz = quaternionRotateVector(quat, [0 0 1])
dz = (DCM * [0 0 1]')'