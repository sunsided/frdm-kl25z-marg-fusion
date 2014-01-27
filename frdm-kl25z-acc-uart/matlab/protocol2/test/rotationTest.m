clc;

m10 = -28095;
m11 = -58763;
m12 = 7254;

m20 = -877;
m21 = -8081;
m22 = -65030;

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

%close all; 
figure;
quiver3(0, 0, 0, 10, 0, 0, '-.r', 'LineWidth', 1); hold on;
quiver3(0, 0, 0, 0, 10, 0, '-.g', 'LineWidth', 1); hold on;
quiver3(0, 0, 0, 0, 0, 10, '-.b', 'LineWidth', 1); hold on;

DCMd = DCM';
quiver3(0, 0, 0, DCMd(1,1), DCMd(1,2), DCMd(1,3), ':r', 'LineWidth', 3); hold on;
quiver3(0, 0, 0, DCMd(2,1), DCMd(2,2), DCMd(2,3), 'g', 'LineWidth', 3); hold on;
quiver3(0, 0, 0, DCMd(3,1), DCMd(3,2), DCMd(3,3), 'b', 'LineWidth', 3); hold on;
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
xlabel('x');
ylabel('y');
zlabel('z');
axis square;

text(DCM(1,1), DCM(1,2), DCM(1,3), 'DCM1', 'Color', 'r');
text(DCM(2,1), DCM(2,2), DCM(2,3), 'DCM2', 'Color', 'g');
text(DCM(3,1), DCM(3,2), DCM(3,3), 'DCM3', 'Color', 'b');

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