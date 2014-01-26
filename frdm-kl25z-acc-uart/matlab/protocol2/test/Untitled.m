clc;

% hoch, 0 roll
acc(1) = -64639;
acc(2) = -1279;
acc(3) = -10731;

% regulär nord
acc(1) = -2133;
acc(2) = 5900;
acc(3) = -65236;

% hoch, 90° roll
acc(1) = -64762;
acc(2) = 5402;
acc(3) = -8473;

% regulär west
acc(1) = -3447;
acc(2) = -3138;
acc(3) = -65371;

% nord, 90°roll
acc(1) = -4487;
acc(2) = 65189;
acc(3) = -5027;

% west, 90°roll
acc(1) = -8035;
acc(2) = 64190;
acc(3) = -10490;

% nord, 45 pitch, 0 roll
acc(1) = -37802;
acc(2) = -3431;
acc(3) = -53425;

% nord, 45 pitch, 90 roll
acc(1) = -44845;
acc(2) = 47403;
acc(3) = -6078;

% nord, ~80 pitch, 90 roll
acc(1) = -63511;
acc(2) = 16048;
acc(3) = -1951;

% nord, ~80 pitch, -90 roll
acc(1) = -61374;
acc(2) = -22968;
acc(3) = -860;

x = acc/65535;

pitch = -asind(x(1));
roll  = atan2d(x(2), x(3));

int32([pitch, roll]*pi/180*65535)

figure;
quiver3(0, 0, 0, x(1), x(2), x(3), 'm', 'LineWidth', 3); hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 1); hold on;
quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 1); hold on;
quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 1); hold on;
title(sprintf('pitch: %5.2f\nroll %5.2f', pitch, roll));
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
zlim([-1.2 1.2]);
xlabel('x');
ylabel('y');
zlabel('z');
axis square;