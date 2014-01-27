a = [
     -690
     8843
     -66280
     ]/65535;
 
m = [
     13699
     8843
     -66280
     ]/65535;

clc;
 
pitch = -asind(a(1));
roll  = atan2d(a(2), a(3));
yaw   = NaN;
 
%% Plotify!

close all; figure('Name', 'LOL', 'NumberTitle', 'off');

O = [0 0 0];
X = [1 0 0];
Y = [0 1 0];
Z = [0 0 1];

subplot(3,3,1);
quiver3(O(1), O(2), O(3), X(1), X(2), X(3), 'r', 'LineWidth', 2); hold on;
quiver3(O(1), O(2), O(3), Y(1), Y(2), Y(3), 'g', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), Z(1), Z(2), Z(3), 'b', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), a(1), a(2), a(3), ':m', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), m(1), m(2), m(3), ':c', 'LineWidth', 2);
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
view(0, 90);
xlabel('x');
ylabel('y');
zlabel('z');
title('X/Y plane');
axis square;

subplot(3,3,4);
quiver3(O(1), O(2), O(3), X(1), X(2), X(3), 'r', 'LineWidth', 2); hold on;
quiver3(O(1), O(2), O(3), Y(1), Y(2), Y(3), 'g', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), Z(1), Z(2), Z(3), 'b', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), a(1), a(2), a(3), ':m', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), m(1), m(2), m(3), ':c', 'LineWidth', 2);
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
view(0, 0);
xlabel('x');
ylabel('y');
zlabel('z');
title('X/Z plane');
axis square;

subplot(3,3,7);
quiver3(O(1), O(2), O(3), X(1), X(2), X(3), 'r', 'LineWidth', 2); hold on;
quiver3(O(1), O(2), O(3), Y(1), Y(2), Y(3), 'g', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), Z(1), Z(2), Z(3), 'b', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), a(1), a(2), a(3), ':m', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), m(1), m(2), m(3), ':c', 'LineWidth', 2);
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
view(90, 0);
xlabel('x');
ylabel('y');
zlabel('z');
title('Y/Z plane');
axis square;

subplot(3,3, [2:3, 5:6, 8:9]);
quiver3(O(1), O(2), O(3), X(1), X(2), X(3), 'r', 'LineWidth', 2); hold on;
quiver3(O(1), O(2), O(3), Y(1), Y(2), Y(3), 'g', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), Z(1), Z(2), Z(3), 'b', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), a(1), a(2), a(3), ':m', 'LineWidth', 2);
quiver3(O(1), O(2), O(3), m(1), m(2), m(3), ':c', 'LineWidth', 2);
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
title(sprintf('roll: %6.2f, pitch: %6.2f, yaw: %6.2f', roll, pitch, yaw));
xlabel('x');
ylabel('y');
zlabel('z');
axis square;
rotate3d;