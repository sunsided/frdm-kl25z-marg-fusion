%home;

% Measured acceleration
acc = [1;
       1;
       1];

% Calculated roll and pitch and scale by pi to get factors
% instead of angles
rp = rollpitch(acc)
rpfactors = rp * 2 / pi;

% get slope and intercept for horizon
slope       = 0; % rpfactors(1);
intercept   = -rpfactors(2)

roll = rp(1);
R = [cos(roll), -sin(roll), 0;
     sin(roll), cos(roll), 0;
     0, 0, 1]
T = [1, 0, 0;
     0, 1, intercept;
     0, 0, 1];
 A =T*R;
     
size = 2;
topLeft     = A*[-size;  size; 1]
topRight    = A*[ size;  size; 1]
bottomLeft  = A*[-size; -size; 1]
bottomRight = A*[ size; -size; 1]

%topLeft     = topLeft/norm(topLeft);
%topRight    = topRight/norm(topRight);
%bottomLeft  = bottomLeft/norm(bottomLeft);
%bottomRight = bottomRight/norm(bottomRight);
middleLeft  = 0.5*(topLeft+bottomLeft);
middleRight  = 0.5*(topRight+bottomRight);

% close all;

groundHandle = fill(...
    [middleLeft(1) middleRight(1) bottomRight(1) bottomLeft(1)], ...
    [middleLeft(2) middleRight(2) bottomRight(2) bottomLeft(2)], ...
    [0.41176 0.27059 0.17647] ...
    );
hold on;
skyHandle = fill( ...
    [middleLeft(1) topLeft(1) topRight(1) middleRight(1)], ...
    [middleLeft(2) topLeft(2) topRight(2) middleRight(2)], ...
    [0.094118 0.48627 0.68235] ...
    );

axis square;
xlim([-1 1]);
ylim([-1 1]);
