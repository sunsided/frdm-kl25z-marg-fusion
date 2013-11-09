function [roll, pitch] = rollpitch(acc)
    % ROLLPITCH Calculates roll and pitch angles from measured acceleration
    %
    %   Parameters
    %   acc = 3D vector of component accelerations [x; y; z]
    %
    %   Return values
    %   roll and pitch in radians

    % Get normalized component vectors
    invn = 1/norm(acc);
    x =  [1; 0; 0] * acc(1) * invn;
    y =  [0; 1; 0] * acc(2) * invn;

    % Reference vector
    zref = [0; 0; 1];

    % Get x-z component angle
    xz_cross = cross(x, zref);
    xangle   = xz_cross(2);
    roll     = asin(xangle);

    % Get y-z component angle
    zy_cross = cross(zref, y);
    yangle   = zy_cross(1);
    pitch    = asin(yangle);

end