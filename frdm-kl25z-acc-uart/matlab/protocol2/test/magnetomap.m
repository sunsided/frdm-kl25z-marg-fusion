function magnetomap()

close all; clc;

test_roll();
%test_pitch();

end

function test_roll()

% pitch ~ 0, roll ~ 0

a = [
     8030
     28
     65320
     ]/65535;

m = [
     21345
     1185
     -26376
     ]/65535;

 displayify(a, m);
 
% pitch ~ 0, roll ~ 22

a = [
     -1887
     29423
     60568
     ]/65535;

m = [
     16131
     -13104
     -27591
     ]/65535;

displayify(a, m);
 
% pitch ~ 0, roll ~ 45
 
a = [
     -2622
     47484
     44099
     ]/65535;

m = [
     15735
     -22826
     -23088
     ]/65535;
     
displayify(a, m); 
 
% pitch ~ 0, roll ~ 67
 
a = [
     3207
     61746
     27114
     ]/65535;

m = [
     17513
     -29641
     -14595
     ]/65535;

displayify(a, m); 

% pitch ~ 0, roll ~ 90
 
a = [
     -3969
     67779
     843
     ]/65535;

m = [
     14602
     -36754
     -294
     ]/65535;
     
displayify(a, m); 

end

function test_pitch()

% pitch ~ 0, roll ~ 0

a = [
     1441
     8474
     62331
     ]/65535;

m = [
     17858
     -2741
     -28490
     ]/65535;

 displayify(a, m); 

% pitch ~ 22, roll ~ 0

a = [
     -29137
     -2877
     57672
     ]/65535;

m = [
     2619
     -199
     -32504
     ]/65535;

 displayify(a, m); 

% pitch ~ 45, roll ~ 0

a = [
     -45658
     7701
     46096
     ]/65535;

m = [
     -10932
     -1746
     -29686
     ]/65535;

 displayify(a, m); 

% pitch ~ 67, roll ~ 0

a = [
     -62751
     -1532
     24064
     ]/65535;

m = [
     -23777
     905
     -20517
     ]/65535;

 displayify(a, m); 

% pitch ~ 90, roll ~ 0

a = [
     -65527
     1089
     -4511
     ]/65535;

m = [
     -31328
     -1190
     -8712
     ]/65535;

displayify(a, m); 

end

function displayify(a, m)

    disp('=============================================');
   
    % Fix axes
    a(2) = -a(2);
    a(3) = -a(3);
    
    % Fix axes
    m(2) = m(2);
    m(3) = m(3);
    
    % Calculate roll and pitch
    
    a = a/norm(a)
    m = m/norm(m)
    
    pitch = -asind(a(1));
    roll  = -atan2d(a(2), -a(3));

    pitchroll = [ pitch, roll ]
    
    % Calculate yaw

    Xh = ( m(1)*cosd(pitch) + m(2)*sind(roll)*sind(pitch) + m(3)*cosd(roll)*cosd(pitch) );
    Yh = (                    m(2)*cosd(roll)             + m(3)*sind(roll)             );

    XhYh = [Xh, Yh]
    
    sin_yaw = Yh/sqrt(Xh^2 + Yh^2);
    cos_yaw = Xh/sqrt(Xh^2 + Yh^2);
    
    yaw_from_sin_cos = [ asind(sin_yaw), acosd(cos_yaw) ]
    
    mx =  cosd(pitch) * sin_yaw;
    my =  cosd(roll) * cos_yaw + sind(roll)*sind(pitch)*sin_yaw;
    mz = -sind(roll) * cos_yaw + cosd(roll)*sind(pitch)*sin_yaw;

    mxmymz = [mx my mz]
    
    x = cross(m, a)';
    x = x/norm(x)
        
    yaw   = atan2d(Yh, Xh)

    %% Plotify!

    plot_title = sprintf('roll: %6.2f, pitch: %6.2f, yaw: %6.2f', roll, pitch, yaw);
    
    %close all; 
    figure('Name', plot_title, 'NumberTitle', 'off', 'Units', 'normalized','OuterPosition', [0 0 1 1]);

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
    quiver3(O(1), O(2), O(3), x(1), x(2), x(3), 'g:', 'LineWidth', 1);
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
    quiver3(O(1), O(2), O(3), x(1), x(2), x(3), 'g:', 'LineWidth', 1);
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
    quiver3(O(1), O(2), O(3), x(1), x(2), x(3), 'g:', 'LineWidth', 1);
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
    quiver3(O(1), O(2), O(3), x(1), x(2), x(3), 'g:', 'LineWidth', 1);
    xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
    title(plot_title);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis square;
    rotate3d;

end