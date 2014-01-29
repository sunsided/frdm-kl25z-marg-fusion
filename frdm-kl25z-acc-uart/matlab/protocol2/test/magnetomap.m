function yawpitchrolltest

close all; clc;

% pitch ~ 0, roll ~ 0

a = [
     3089
     -3936
     66126
     ]/65535;
 
m = [
     14478
     516
     -29803
     ]/65535;

 displayify(a, m);
 
% pitch ~ 0, roll ~ 45

a = [
     -2964
     -46082
     47952
     ]/65535;
 
m = [
     16315
     -20296
     -23855
     ]/65535;

displayify(a, m);
 
% pitch ~ 0, roll ~ 90
 
a = [
     -5649
     -63817
     16256
     ]/65535;
 
m = [
     16970
     -31706
     -10052
     ]/65535;
     
displayify(a, m); 
 
end

function displayify(a, m)

    %% Calculate roll and pitch

    pitch =  asind(a(1))
    roll  =  atan2d(a(2), -a(3))

    %% Calculate yaw

    Xh = ( m(1)*cosd(pitch) + m(2)*sind(roll)*sind(pitch) + m(3)*cosd(roll)*cosd(pitch) );
    Yh = (                    m(2)*cosd(roll)             + m(3)*sind(roll)             );

    sin_yaw = Yh/sqrt(Xh^2 + Yh^2);
    cos_yaw = Xh/sqrt(Xh^2 + Yh^2);
    yaw_from_sin = asind(sin_yaw)
    yaw_from_cos = acosd(cos_yaw)

    mx =  cosd(pitch) * sin_yaw;
    my =  cosd(roll) * cos_yaw + sind(roll)*sind(pitch)*sin_yaw;
    mz = -sind(roll) * cos_yaw + cosd(roll)*sind(pitch)*sin_yaw;

    yaw   = NaN;

    %% Plotify!

    %close all; 
    figure('Name', 'LOL', 'NumberTitle', 'off');

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
    quiver3(O(1), O(2), O(3), Xh, 0, 0, 'k', 'LineWidth', 1);
    quiver3(O(1), O(2), O(3), 0, Yh, 0, 'k', 'LineWidth', 1);
    quiver3(O(1), O(2), O(3), mx, my, mz, 'c', 'LineWidth', 2);
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
    quiver3(O(1), O(2), O(3), mx, my, mz, 'c', 'LineWidth', 2);
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
    quiver3(O(1), O(2), O(3), mx, my, mz, 'c', 'LineWidth', 2);
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
    quiver3(O(1), O(2), O(3), Xh, 0, 0, 'k', 'LineWidth', 1);
    quiver3(O(1), O(2), O(3), 0, Yh, 0, 'k', 'LineWidth', 1);
    quiver3(O(1), O(2), O(3), mx, my, mz, 'c', 'LineWidth', 2);
    xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
    title(sprintf('roll: %6.2f, pitch: %6.2f, yaw: %6.2f', roll, pitch, yaw));
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis square;
    rotate3d;

end