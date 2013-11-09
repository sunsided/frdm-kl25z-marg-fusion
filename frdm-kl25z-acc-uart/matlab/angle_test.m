clear; home;

% The measured acceleration vector XYZ in g
acc = [0;
       -1;
       1];
       
% Get normalized component vectors
n = norm(acc);
x =  [1; 0; 0] * acc(1) / n;
y =  [0; 1; 0] * acc(2) / n;
z =  [0; 0; 1] * acc(3) / n;

% Reference vectors
xref = [1; 0; 0];
yref = [0; 1; 0];
zref = [0; 0; 1];

% Get x-z component angle
xz       = (x+z)/norm(x+z);
xz_cross = cross(x, zref);
xangle   = xz_cross(2);
roll     = asin(xangle)*180/pi

% Get y-z component angle
yz       = (y+z)/norm(y+z);
zy_cross = cross(zref, y);
yangle   = zy_cross(1);
pitch    = asin(yangle)*180/pi
